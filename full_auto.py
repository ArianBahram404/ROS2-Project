import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState
from message.srv import UrInverseKinematics, UrForwardKinematics
from tf2_ros import TransformBroadcaster
import time
import copy
import math

class RobotMotionNode(Node):
    def __init__(self):
        super().__init__('robot_motion_node')

        # Initialize joint state storage
        self.current_right_arm_positions = None
        self.current_left_arm_positions = None

        # Subscribers for joint states
        self.right_joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.right_joint_state_callback,
            10
        )

        self.left_joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.left_joint_state_callback,
            10
        )

        # Publishers for joint commands
        self.right_arm_publisher = self.create_publisher(
            Float64MultiArray,
            '/ur_right_joint_group_pos_controller/commands',
            10
        )

        self.left_arm_publisher = self.create_publisher(
            Float64MultiArray,
            '/ur_left_joint_group_pos_controller/commands',
            10
        )

        # IK service client
        self.ik_client = self.create_client(UrInverseKinematics, 'ur_inverse_k')

        # Forward kinematics service client
        self.fk_client = self.create_client(UrForwardKinematics, 'ur_forward_k')

        # Wait for the IK and FK services to be available
        self.wait_for_services()

        self.req_ik = UrInverseKinematics.Request()
        self.req_fk = UrForwardKinematics.Request()

        # Set initial and final poses
        self.initial_pose = Pose()
        self.final_pose = Pose()
        self.set_initial_final_poses()

        # TF broadcaster setup
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

        # Ensure initial joint positions are received before starting motion
        time.sleep(5)  # Wait a bit before starting to check for joint positions
        self.ensure_joint_positions()

    def wait_for_services(self):
        """Wait for the IK and FK services to be available before proceeding."""
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')
        
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting again...')
        
        self.get_logger().info('IK and FK services are now available.')

    def right_joint_state_callback(self, msg):
        self.current_right_arm_positions = msg.position
        self.get_logger().debug(f"Updated right arm positions: {self.current_right_arm_positions}")

    def left_joint_state_callback(self, msg):
        self.current_left_arm_positions = msg.position
        self.get_logger().debug(f"Updated left arm positions: {self.current_left_arm_positions}")

    def ensure_joint_positions(self):
        """Wait until the initial joint positions are received."""
        self.get_logger().info("Waiting for initial joint positions...")
        retries = 0
        while (self.current_right_arm_positions is None or self.current_left_arm_positions is None) and retries < 20:
            self.get_logger().info(f"Waiting for current arm positions... (attempt {retries + 1}/20)")
            rclpy.spin_once(self, timeout_sec=1.0)
            retries += 1
        
        if self.current_right_arm_positions is None:
            self.get_logger().error("Failed to retrieve current right arm positions.")
        
        if self.current_left_arm_positions is None:
            self.get_logger().error("Failed to retrieve current left arm positions.")
        
        if self.current_right_arm_positions is None or self.current_left_arm_positions is None:
            self.get_logger().error("Unable to retrieve initial joint positions. Using default positions.")
            # Set default safe positions if joint states are unavailable
            self.current_right_arm_positions = [0.0] * 6
            self.current_left_arm_positions = [0.0] * 6

    def get_current_right_arm_position(self):
        return self.current_right_arm_positions

    def get_current_left_arm_position(self):
        return self.current_left_arm_positions

    def set_initial_final_poses(self):
        # Set initial pose
        self.initial_pose.position.x = 0.5171880524124829
        self.initial_pose.position.y = 0.061082323646202544
        self.initial_pose.position.z = 0.11278279389084409
        self.initial_pose.orientation.x = -0.12294103597275131
        self.initial_pose.orientation.y = 0.6736173458378836
        self.initial_pose.orientation.z = 0.11490571715587151
        self.initial_pose.orientation.w = 0.7196678742483693

        # Set final pose (example movement along X axis)
        self.final_pose.position.x = self.initial_pose.position.x + 0.1  # Move 10cm along X
        self.final_pose.position.y = self.initial_pose.position.y
        self.final_pose.position.z = self.initial_pose.position.z
        self.final_pose.orientation = self.initial_pose.orientation  # Keep orientation the same

    def interpolate_poses(self, num_points):
        # Generate a sequence of poses between initial and final positions
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            interp_pose = Pose()
            interp_pose.position.x = self.initial_pose.position.x + t * (self.final_pose.position.x - self.initial_pose.position.x)
            interp_pose.position.y = self.initial_pose.position.y + t * (self.final_pose.position.y - self.initial_pose.position.y)
            interp_pose.position.z = self.initial_pose.position.z + t * (self.final_pose.position.z - self.initial_pose.position.z)
            interp_pose.orientation = self.initial_pose.orientation  # Fixed orientation
            points.append(copy.deepcopy(interp_pose))
        return points

    def perform_motion(self):
        # Perform motion by interpolating poses and calling IK service
        interpolated_poses = self.interpolate_poses(num_points=1000) 
        for pose in interpolated_poses:
            self.call_ik_service(pose)
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait for IK service response

    def call_ik_service(self, pose):
        # Prepare and send a request to the IK service
        self.req_ik.reference_pose = [pose]
        self.req_ik.desired_config = [1]
        self.req_ik.ur_type = "LexiumCobot"
        self.req_ik.check_q6 = False
        self.req_ik.verbose = True
        self.req_ik.last_joints = list(self.get_current_right_arm_position() or [0.0]*6)  # Use current position or zeros

        future = self.ik_client.call_async(self.req_ik)
        future.add_done_callback(self.ik_callback)


    def ik_callback(self, future):
        try:
            response = future.result()
        
            # Check if the service call was successful
            if response.success:
                # Log the primary solution
                if response.solution:
                    pose = response.solution[0]  # Assuming you want the first solution
                    # self.get_logger().info(f"Received primary IK solution with joint values: {pose.joints}")
            
                # Log the complete solution
                if response.complete_solution:
                    self.get_logger().info(f"Received complete IK solution set.")
                    joint_matrix = response.complete_solution[0]
                    for joints in joint_matrix.joint_matrix:
                        self.get_logger().info(f"Joint values: {joints.data}")
            
                # Additional information
                self.get_logger().info(f"Move q6: {response.move_q6}")
                self.get_logger().info(f"Max Error: {response.max_error}")
            else:
                self.get_logger().info('Service call returned no valid solution')
        except Exception as e:
            self.get_logger().error(f'Service call failed with error: {str(e)}')

    def send_joint_command(self, right_joint_pose=None, left_joint_pose=None):
        if right_joint_pose is None:
            right_joint_pose = self.get_current_right_arm_position()
            if right_joint_pose is None:
                self.get_logger().error("Cannot send command: Right arm joint positions unavailable.")
                return

        if left_joint_pose is None:
            left_joint_pose = self.get_current_left_arm_position()
            if left_joint_pose is None:
                self.get_logger().error("Cannot send command: Left arm joint positions unavailable.")
                return

        # Ensure joint poses are lists of floats
        right_joint_pose = list(map(float, right_joint_pose))
        left_joint_pose = list(map(float, left_joint_pose))

        # Send to the robot's joint command topic
        self.right_arm_publisher.publish(Float64MultiArray(data=right_joint_pose))
        self.left_arm_publisher.publish(Float64MultiArray(data=left_joint_pose))

        self.get_logger().info(f"Sent right arm joint command: {right_joint_pose}")
        self.get_logger().info(f"Sent left arm joint command: {left_joint_pose}")

    def convert_simulation_to_real(self, joint_angles):
        # Convert the joint angles from simulation (radians) to real robot (degrees)
        real_angles = []
        real_angles.append(joint_angles[1] * 180.0 / math.pi)  # Joint 1: No conversion, just degrees
        real_angles.append(-joint_angles[2] * 180.0 / math.pi)  # Joint 2: Reverse sign and convert
        real_angles.append(-joint_angles[3] * 180.0 / math.pi)  # Joint 3: Reverse sign and convert
        real_angles.append(-joint_angles[4] * 180.0 / math.pi)  # Joint 4: Reverse sign and convert
        real_angles.append((joint_angles[5] - 3.14) * 180.0 / math.pi)  # Joint 5: Subtract 3.14 and convert
        real_angles.append(joint_angles[6] * 180.0 / math.pi)  # Joint 6: No conversion, just degrees
        return real_angles

    def broadcast_transform(self):
        t = TransformStamped()

        # Frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "ur_right_base"  # Parent frame
        t.child_frame_id = "real_robot"  # Child frame

        # Example transformation from simulation to real robot
        t.transform.translation.x = -0.103623
        t.transform.translation.y = -0.366327
        t.transform.translation.z = 0.135886

        # Orientation (quaternion)
        t.transform.rotation.x = 9.17133134e-01
        t.transform.rotation.y =  3.98581002e-01
        t.transform.rotation.z = 2.44060474e-17
        t.transform.rotation.w = 5.61582078e-17

        # Log message to confirm broadcasting
        self.get_logger().info(f'Broadcasting transform from simulated_robot to real_robot with position '
                               f'({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}) '
                               f'and orientation ({t.transform.rotation.x}, {t.transform.rotation.y}, '
                               f'{t.transform.rotation.z}, {t.transform.rotation.w})')

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    robot_motion_node = RobotMotionNode()

    try:
        robot_motion_node.perform_motion()
    except Exception as e:
        robot_motion_node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        robot_motion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
