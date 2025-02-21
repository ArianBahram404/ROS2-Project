import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from message.srv import UrForwardKinematics
from std_msgs.msg import Float64MultiArray
import random

class URClientNode(Node):

    def __init__(self):
        super().__init__('ur_client')

        # Client
        self.client_ = self.create_client(UrForwardKinematics, 'ur_forward_k')
       
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
        self.req = UrForwardKinematics.Request()

    def future_callback(self, future):
            try:
                response = future.result()
                if response and response.reference_pose:
                    pose = response.reference_pose[0]
                    self.get_logger().info(f"Received Pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                                       f"ox={pose.orientation.x}, oy={pose.orientation.y}, oz={pose.orientation.z}, ow={pose.orientation.w}")
                else:
                    self.get_logger().info('Service call returned no data')
            except Exception as e:
                self.get_logger().error(f'Service call failed with error: {str(e)}')



    def publish_cartesian_space(self):
        pose = Pose()
        pose.position.x = 0.1
        pose.position.y = 0.1
        pose.position.z = 0.5
        pose.orientation.x = 0.9
        pose.orientation.y = 0.1
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
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

        #self.req.reference_pose.append(pose)  # Assuming that we want to use the received pose as the reference
        #self.req.desired_config = [0, 0, 0, 0, 0, 0]  # desired configuration
        float64_array = Float64MultiArray()
        float64_array.data = [0.341979814, -1.99037348, -2.106018995, -0.553775518, -4.734777, 0.0031241394]
        self.req.reference_joints.append(float64_array)
        self.req.ur_type = "LexiumCobot"
        #self.req.check_q6 = False
        #self.req.verbose = True
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
