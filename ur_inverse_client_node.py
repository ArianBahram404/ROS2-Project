import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from message.srv import UrInverseKinematics
from std_msgs.msg import Float64MultiArray
import random

class URClientNode(Node):

    def __init__(self):
        super().__init__('ur_client')

        # Client
        self.client_ = self.create_client(UrInverseKinematics, 'ur_inverse_k')
       
        # Subscription
        self.subscription_ = self.create_subscription(
            Pose, 
            'cartesian_space_pose',
            self.self_joint_callback,
            10
        )

        # Timer
        self.timer_ = self.create_timer(1.0, self.publish_cartesian_space)

        # Publisher
        self.cartesian_space_publisher_ = self.create_publisher(Pose, 'cartesian_space_pose', 10)

        # Ensure the service is available
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = UrInverseKinematics.Request()

    def future_callback(self, future):
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



    def publish_cartesian_space(self):
        pose = Pose()
        pose.position.x = -0.10613032699986842
        pose.position.y = -0.2314410204841174
        pose.position.z = 0.5574625539824385
        pose.orientation.x = 0.40272647211756013
        pose.orientation.y = 0.37683273171138876
        pose.orientation.z = -0.5694427410228845
        pose.orientation.w = 0.6095436372098559
        self.cartesian_space_publisher_.publish(pose)
        self.get_logger().info(
            'Publishing Cartesian Space Pose: x=%f, y=%f, z=%f, ox=%f, oy=%f, oz=%f, ow=%f' %
            (pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )

    def self_joint_callback(self, msg):
        self.get_logger().info('Received self joint state')
        self.send_request(msg)

    def send_request(self, pose):
        self.get_logger().info(
            'Sending service request with Pose: x=%f, y=%f, z=%f, ox=%f, oy=%f, oz=%f, ow=%f' %
            (pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )

        self.req.reference_pose.append(pose)  # Assuming that we want to use the received pose as the reference
        self.req.desired_config = [1]  # desired configuration
        #float64_array = Float64MultiArray()
        self.req.last_joints = [-1.569999817071685, -0.940001019294809, -1.7999999310673938, -1.5699997550724019, 1.63799981338739, 2.3613805044675473e-07]
        #self.req.reference_joints.append(float64_array)
        self.req.ur_type = "LexiumCobot"
        self.req.check_q6 = False
        self.req.verbose = True
        # Wait for the service to complete


        self.future = self.client_.call_async(self.req)

        self.future.add_done_callback(self.future_callback)


def main(args=None):
    rclpy.init(args=args)
    node = URClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
