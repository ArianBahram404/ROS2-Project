import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from message.srv import UrInverseKinematics


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # Declare and get parameters
        self.declare_parameter('start_pose', '0.5171880524124829,0.061082323646202544,0.11278279389084409,-0.12294103597275131,0.6736173458378836,0.11490571715587151,0.7196678742483693')
        self.declare_parameter('end_pose', '0.286002217552701, -2.152271025246447, -1.8016201861997523, -0.6951237143815714, 1.5519221660752662, -0.05275498487169215')
        self.declare_parameter('velocity', 0.1)
        self.declare_parameter('robot_mode', 'simulation')

        start_pose_str = self.get_parameter('start_pose').value
        end_pose_str = self.get_parameter('end_pose').value
        self.velocity = self.get_parameter('velocity').value
        self.robot_mode = self.get_parameter('robot_mode').value

        # Parse poses
        self.start_pose = self.parse_pose(start_pose_str)
        self.end_pose = self.parse_pose(end_pose_str)

        # Set up IK service client
        self.ik_client = self.create_client(UrInverseKinematics, 'ur_inverse_k')
        self.wait_for_service(self.ik_client, 'Inverse Kinematics')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(Float64MultiArray, 
                                                             '/joint_group_pos_controller/commands', 10)

        # Timer to start execution
        self.create_timer(1.0, self.execute_trajectory)

    def wait_for_service(self, client, service_name):
        """Wait for a service to be available."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')
        self.get_logger().info(f'{service_name} service is available.')

    def parse_pose(self, pose_str):
        """Parse a pose string into a Pose object with fixed values."""
        components = [float(x) for x in pose_str.split(',')]
        pose = Pose()
        pose.position.x = 0.5171880524124829  
        pose.position.y = 0.061082323646202544
        pose.position.z = 0.11278279389084409
        pose.orientation.x = -0.12294103597275131
        pose.orientation.y = 0.6736173458378836
        pose.orientation.z = 0.11490571715587151
        pose.orientation.w = 0.7196678742483693
        return pose

    def execute_trajectory(self):
        """Execute trajectory in simulation or real robot mode."""
        self.get_logger().info(f"Starting trajectory execution in {self.robot_mode} mode.")

        # Interpolate poses
        num_points = int(1.0 / self.velocity)
        interpolated_poses = self.interpolate_poses(self.start_pose, self.end_pose, num_points)

        # Execute trajectory
        for pose in interpolated_poses:
            joint_command = self.get_joint_command(pose)
            if joint_command:
                self.send_joint_command(joint_command)

        self.get_logger().info("Trajectory execution completed.")

    def interpolate_poses(self, start_pose, end_pose, num_points):
        """Generate interpolated poses with fixed orientation."""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            interp_pose = Pose()
            interp_pose.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
            interp_pose.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
            interp_pose.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)

            # Fixed orientation (uses start_pose orientation throughout)
            interp_pose.orientation = start_pose.orientation
            points.append(interp_pose)

            # Log the details of each trajectory point
            self.get_logger().info(
                f"Trajectory Point {i + 1}/{num_points} -> "
                f"Position: ({interp_pose.position.x:.6f}, {interp_pose.position.y:.6f}, {interp_pose.position.z:.6f}), "
                f"Orientation: ({interp_pose.orientation.x:.6f}, {interp_pose.orientation.y:.6f}, "
                f"{interp_pose.orientation.z:.6f}, {interp_pose.orientation.w:.6f})"
            )

        return points

    def get_joint_command(self, pose):
        """Call IK service to get joint commands."""
        req = UrInverseKinematics.Request()
        req.reference_pose = [pose]
        req.desired_config = [1]  # Example config
        req.ur_type = "LexiumCobot"
        req.check_q6 = False
        req.verbose = True

        future = self.ik_client.call_async(req)
        rclpy.spin_once(self, timeout_sec=1.0)
        if future.done():
            try:
                response = future.result()
                if response.success and response.complete_solution:
                    return response.complete_solution[0].joint_matrix[0].data
            except Exception as e:
                self.get_logger().error(f"Failed to get joint command: {e}")
        return None

    def send_joint_command(self, joint_command):
        """Send joint command to the robot."""
        msg = Float64MultiArray()
        msg.data = joint_command
        self.joint_command_publisher.publish(msg)
        self.get_logger().info(f"Sent joint command: {joint_command}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
