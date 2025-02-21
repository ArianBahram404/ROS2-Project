import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class TFPublisher(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()

        # Frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "ur_right_base"  # Parent frame
        t.child_frame_id = "real_robot"  # Child frame

        t.transform.translation.x = -0.103623
        t.transform.translation.y = -0.366327
        t.transform.translation.z = 0.135886

        # Orientation (quaternion) from simulated_robot to real_robot
        t.transform.rotation.x = 9.17133134e-01
        t.transform.rotation.y =  3.98581002e-01
        t.transform.rotation.z = 2.44060474e-17
        t.transform.rotation.w = 5.61582078e-17

        # Log message to confirm broadcasting
        self.get_logger().info(f'Broadcasting transform from simulated_robot to real_robot with position '
                               f'({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}) '
                               f'and orientation ({t.transform.rotation.x}, {t.transform.rotation.y}, '
                               f'{t.transform.rotation.z}, {t.transform.rotation.w})')

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
