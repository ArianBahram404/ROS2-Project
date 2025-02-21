import time
from my_py_pkg.lexium_cobot import CobotControl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from message.srv import UrForwardKinematics, UrInverseKinematics
from math import radians, degrees

def parse_joints(joint_string):
    """
    Parse a string representing joint values (j1,j2,j3,j4,j5,j6) into a list of floats.
    """
    return [float(v) for v in joint_string.split(',')]

def convert_joints_real(real_joints, to_simulation=True):
    """
    Converts joint values between real robot and simulation.

    Parameters:
    - real_joints: list of joint values from the real robot (in degrees)
    - to_simulation: True if converting from real to simulation, False otherwise

    Returns:
    - Converted joint values as a list.
    """
    if len(real_joints) != 6:
        raise ValueError(f"Expected 6 joints, but got {len(real_joints)}. Input: {real_joints}")
    
    # Convert to radians or degrees depending on the direction
    if to_simulation:
        sim_joints = [radians(j) for j in real_joints]  # Convert to radians
    else:
        sim_joints = [degrees(j) for j in real_joints]  # Convert to degrees

    # Apply joint-specific conversions for joints 1 to 6
    sim_joints[0] = sim_joints[0]  # Joint 1 stays the same
    sim_joints[1] = -sim_joints[1]  # Reverse direction for joint 2
    sim_joints[2] = -sim_joints[2]  # Reverse direction for joint 3
    sim_joints[3] = -sim_joints[3]  # Reverse direction for joint 4
    sim_joints[4] = sim_joints[4] - 3.14  # Offset for joint 5
    sim_joints[5] = sim_joints[5]  # Joint 6 stays the same

    return sim_joints

def convert_joints(sim_joints):
    """
    Convert joint values from simulation to real robot.

    Args:
        sim_joints (list): List of joint values in simulation (radians).

    Returns:
        list: Converted joint values for the real robot (degrees).
    """
    # Convert radians to degrees for all joints
    real_joints = [degrees(j) for j in sim_joints]

    # Apply joint-specific conversions
    real_joints[0] = real_joints[0]  # Joint 1: No additional conversion
    real_joints[1] = -real_joints[1]  # Joint 2: Reverse direction
    real_joints[2] = -real_joints[2]  # Joint 3: Reverse direction
    real_joints[3] = -real_joints[3]  # Joint 4: Reverse direction
    real_joints[4] = degrees(sim_joints[4] + 3.14)  # Joint 5: Offset by +3.14 (radians) and convert to degrees
    real_joints[5] = real_joints[5]  # Joint 6: No additional conversion

    return real_joints

class ComUnifyCobot(Node):
    def __init__(self):
        super().__init__('com_unify_cobot')

        # Initialize robot connection
        self.robot_ip = "192.168.88.82"
        self.cobot_control = CobotControl(i_sIpAddress=self.robot_ip)

        # Attempt to retrieve current joint positions
        try:
            self.current_joint_states = self.cobot_control.get_joints_pose()
            if self.current_joint_states is None:
                raise RuntimeError("Failed to retrieve joint positions: No data returned.")
            self.get_logger().info(f"Retrieved joint positions: {self.current_joint_states}")
        except Exception as e:
            self.get_logger().error(f"Error connecting to robot or retrieving joint positions: {str(e)}")
            raise

        sim_joints = [19.594, 114.040, 120.666, 31.729, -91.374, 0.179]
        sim_joints_end = [19.594, 114.040, 120.666, 31.729, -91.374, 0.179]

        # Parse joints
        self.start_joints = parse_joints(",".join(map(str, sim_joints)))
        self.end_joints = parse_joints(",".join(map(str, sim_joints_end)))

        self.forward_client = self.create_client(UrForwardKinematics, 'ur_forward_k')
        self.inverse_client = self.create_client(UrInverseKinematics, 'ur_inverse_k')
        self.joint_publisher = self.create_publisher(Float64MultiArray, '/ur_right_joint_group_pos_controller/commands', 10)

        self.forward_client.wait_for_service()
        self.inverse_client.wait_for_service()

    def call_forward_kinematics(self, joints):
        self.get_logger().info(f"Calling Forward Kinematics with joints: {joints}")
        req = UrForwardKinematics.Request()
        req.reference_joints = [Float64MultiArray(data=joints)]
        req.ur_type = "LexiumCobot"
        future = self.forward_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().reference_pose:
            pose = future.result().reference_pose[0]
            return pose
        else:
            raise RuntimeError("Forward Kinematics failed")

    def call_inverse_kinematics(self, pose):
        self.get_logger().info(f"Calling Inverse Kinematics for pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                               f"qx={pose.orientation.x}, qy={pose.orientation.y}, qz={pose.orientation.z}, qw={pose.orientation.w}")
        req = UrInverseKinematics.Request()
        req.reference_pose = [pose]
        req.desired_config = [1]
        req.ur_type = "LexiumCobot"
        future = self.inverse_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().complete_solution:
            return future.result().complete_solution[0].joint_matrix[0].data
        else:
            raise RuntimeError("Inverse Kinematics failed")

    def move_robot(self, joints):
        joint_msg = Float64MultiArray()
        joint_msg.data = joints
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f"Moved robot to joints: {joints}")

    def execute_combined_trajectory(self):
        sim_joints = convert_joints_real(self.start_joints, to_simulation=True)
        self.get_logger().info(f"Converted to simulation joints: {sim_joints}")

        sim_joints_end = convert_joints(self.end_joints)
        self.get_logger().info(f"Converted to simulation joints: {sim_joints_end}")

        self.start_pose = self.call_forward_kinematics(sim_joints)
        self.end_pose = self.call_forward_kinematics(sim_joints_end)

        trajectory1 = self.generate_trajectory(self.start_pose, self.end_pose, num_points=50)

        self.execute_trajectory(trajectory1)

    def generate_trajectory(self, start_pose, end_pose, num_points=100):
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            interp_pose = Pose()
            interp_pose.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
            interp_pose.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
            interp_pose.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)
            interp_pose.orientation = start_pose.orientation  # Keep the orientation fixed
            trajectory.append(interp_pose)
        return trajectory

    def execute_trajectory(self, trajectory):
        for idx, pose in enumerate(trajectory):
            joints = self.call_inverse_kinematics(pose)
            self.move_robot(joints)
            time.sleep(0.1)  # Slow down for visualization

def main(args=None):
    rclpy.init(args=args)
    node = ComUnifyCobot()
    try:
        node.execute_combined_trajectory()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
