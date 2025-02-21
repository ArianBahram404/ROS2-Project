import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from message.srv import UrInverseKinematics
import copy
import time

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

        # Wait for the IK service to be available before proceeding
        self.wait_for_ik_service()

        self.req = UrInverseKinematics.Request()

        # Set initial and final poses
        self.initial_pose = Pose()
        self.final_pose = Pose()
        self.set_initial_final_poses()

        # Ensure initial joint positions are received before starting motion
        time.sleep(5)  # Wait a bit before starting to check for joint positions
        self.ensure_joint_positions()

    def wait_for_ik_service(self):
        """Wait for the IK service to be available before proceeding."""
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
        self.get_logger().info('IK service is now available.')

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

        #Set final pose (example movement along X axis)
        self.final_pose.position.x = self.initial_pose.position.x + 0.1  # Move 10cm along X
        self.final_pose.position.y = self.initial_pose.position.y
        self.final_pose.position.z = self.initial_pose.position.z
        self.final_pose.orientation = self.initial_pose.orientation  # Keep orientation the same

        #Set final pose (position of the channel under the end effector)
        #self.final_pose.position.x = -0.082639  # X position of the channel
        #self.final_pose.position.y = -0.364479   # Y position of the channel
        #self.final_pose.position.z = 0.101014  # Z height of the channel
        #self.final_pose.orientation.x = 0.398581002
        #self.final_pose.orientation.y = -0.917133134
        #self.final_pose.orientation.z = -5.61582078e-17
        #self.final_pose.orientation.w = 2.44060474e-17

        # Define Point 1 (initial position near the channel)
        #self.point_1 = Pose()
        #self.point_1.position.x = -0.082639  # X position of the channel
        #self.point_1.position.y = -0.364479  # Y position of the channel
        #self.point_1.position.z = 0.101014  # Z height of the channel
    
        # Define Point 2 (Move along X axis for example)
        #self.point_2 = Pose()
        #self.point_2.position.x = self.point_1.position.x -0.062639  # Move 2 cm along the X-axis
        #self.point_2.position.y = self.point_1.position.y -0.364479  # Keep Y the same
        #self.point_2.position.z = self.point_1.position.z + 0.101014 

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
        # Step 1: Move to Point 1
        #self.get_logger().info("Moving to Point 1 (Initial channel position)")
        #self.move_to_target(self.point_1)  # Move to Point 1
    
        # Wait until the robot reaches Point 1 before proceeding
        #self.wait_until_reached(self.point_1)

        # Update the initial pose to the final pose of Point 1
        #self.initial_pose = self.point_1
    
        # Step 2: Move to Point 2 from Point 1
        #self.get_logger().info("Moving to Point 2 (Next position on the channel)")
        #self.move_to_target(self.point_2)  # Move to Point 2

        # Wait until the robot reaches Point 2
        #self.wait_until_reached(self.point_2)

    #def move_to_target(self, target_pose):
        #"""Moves the robot to the specified target pose."""
        #self.get_logger().info(f"Moving to target: X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z}")

        # Set the final pose to the target
        #self.final_pose = target_pose

        # Perform motion by interpolating poses and calling IK service
        interpolated_poses = self.interpolate_poses(num_points=1000) 
        for pose in interpolated_poses:
            self.call_ik_service(pose)
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait for IK service response

    #def wait_until_reached(self, target_pose):
        #"""Wait until the robot reaches the target pose."""
        # This function waits for the robot to reach a certain target by comparing joint positions
        #self.get_logger().info(f"Waiting until the robot reaches the target pose at X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z}")
        # Add logic to compare the current joint states with the target position
        # For now, let's use a sleep to simulate waiting, but ideally, you would compare joint states
        #time.sleep(1)  # You may adjust this based on how long the movement takes

    def call_ik_service(self, pose):
        # Prepare and send a request to the IK service
        self.req.reference_pose = [pose]
        self.req.desired_config = [1]
        self.req.ur_type = "LexiumCobot"
        self.req.check_q6 = False
        self.req.verbose = True
        self.req.last_joints = list(self.get_current_right_arm_position() or [0.0]*6)  # Use current position or zeros

        future = self.ik_client.call_async(self.req)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            response = future.result()
            if response.success and response.complete_solution:
                self.get_logger().info("Received complete IK solution set.")

                joint_matrix_obj = response.complete_solution[0]

                if joint_matrix_obj.joint_matrix:
                    first_solution = joint_matrix_obj.joint_matrix[0]
                    joint_values = first_solution.data
                    if len(joint_values) == 6:
                        self.send_joint_command(right_joint_pose=joint_values)
                    else:
                         self.get_logger().error("The first solution does not have exactly 6 joint values.")
            else:
                self.get_logger().error("IK service returned no valid solution.")
        except Exception as e:
            self.get_logger().error(f'Service call failed with error: {str(e)}')

    def send_joint_command(self, right_joint_pose=None, left_joint_pose=None):
        print(right_joint_pose)
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
        #right_joint_pose = [0.341979814, -1.99037348, -2.106018995, -0.553775518, -4.734777, 0.0031241394]

        # Create and publish messages
        right_msg = Float64MultiArray()
        right_msg.data = right_joint_pose
        self.right_arm_publisher.publish(right_msg)
        self.get_logger().info(f"Sent right arm joint command: {right_joint_pose}")

        left_msg = Float64MultiArray()
        left_msg.data = left_joint_pose
        self.left_arm_publisher.publish(left_msg)
        self.get_logger().info(f"Sent left arm joint command: {left_joint_pose}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionNode()
    node.perform_motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
