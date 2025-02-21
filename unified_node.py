import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from message.srv import UrForwardKinematics, UrInverseKinematics
from math import radians, degrees
import time  # Import time for adding delays

def convert_joints(real_joints, to_simulation=True):
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

class UnifiedNode(Node):
    def __init__(self):
        super().__init__('unified_node')
        
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
            self.get_logger().info(f"Forward Kinematics Result: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                                   f"qx={pose.orientation.x}, qy={pose.orientation.y}, qz={pose.orientation.z}, qw={pose.orientation.w}")
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
            joints = future.result().complete_solution[0].joint_matrix[0].data
            return joints
        else:
            raise RuntimeError("Inverse Kinematics failed")

    def move_robot(self, joints):
        joint_msg = Float64MultiArray()
        joint_msg.data = joints
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f"Moved robot to joints: {joints}")

    def generate_trajectory(self, start_pose, end_pose, num_points=100):  # Increased num_points for smoother motion
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
            self.get_logger().info(f"Trajectory Point {idx}: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                                   f"qx={pose.orientation.x}, qy={pose.orientation.y}, qz={pose.orientation.z}, qw={pose.orientation.w}")
            
            # Call inverse kinematics for each trajectory point
            joints = self.call_inverse_kinematics(pose)
            self.get_logger().info(f"Inverse Kinematics Joints: {joints}")
            
            # Move the robot to the calculated joint positions
            self.move_robot(joints)
            
            # Add a delay to slow down execution
            time.sleep(0.1)  # Adjust the delay as needed (e.g., 0.5 for a slower motion)

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedNode()
    
    try:
        # Step 1: Real robot joint poses (in degrees)
        real_joints = [19.594, 114.040, 120.666, 31.729, -91.374, 0.179]

        # Step 2: Convert to simulation joint values
        sim_joints = convert_joints(real_joints, to_simulation=True)
        node.get_logger().info(f"Converted to simulation joints: {sim_joints}")

        # Step 3: Use forward kinematics to calculate the Cartesian pose
        start_pose = node.call_forward_kinematics(sim_joints)

        # Step 4: Define the end pose (10 cm along X-axis)
        end_pose = Pose()
        end_pose.position.x = start_pose.position.x + 0.1  # Move 10 cm along X
        end_pose.position.y = start_pose.position.y
        end_pose.position.z = start_pose.position.z
        end_pose.orientation = start_pose.orientation  # Keep the same orientation

        # Step 5: Generate a trajectory
        trajectory = node.generate_trajectory(start_pose, end_pose, num_points=100)  # Increased num_points

        # Step 6: Execute the trajectory with delays
        node.execute_trajectory(trajectory)

    except Exception as e:
        node.get_logger().error(str(e))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
