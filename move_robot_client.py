import rclpy
from rclpy.node import Node
from message.srv import MoveRobot
from std_msgs.msg import Float64MultiArray
import numpy
class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')
        self.client = self.create_client(MoveRobot, 'move_robot')

    def send_movement_request(self, joint_positions):
        request = MoveRobot.Request()
        print(joint_positions[0][0])
        multi_array  = Float64MultiArray()

        for i in range(len(joint_positions)):
            print(i)
            multi_array.data = []
            multi_array.data.append(joint_positions[i][0])
            multi_array.data.append(joint_positions[i][1])
            multi_array.data.append(joint_positions[i][2])
            multi_array.data.append(joint_positions[i][3])
            multi_array.data.append(joint_positions[i][4])
            multi_array.data.append(joint_positions[i][5])
            request.positions.append(multi_array)
            print(request.positions)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClient()
    

    joint_positions = [[2.0, 1.0, 2.0, 1.0, 2.0, 1.0], [1.2, 2.0, 1.1, 1.5, 1.6, 1.5]]  # Example joint positions
   
    future = node.send_movement_request(joint_positions)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        response = future.result()
        if response.success:
            print('Robot movement successful')
        else:
            print('Robot movement failed')
    else:
        print('Service call failed')

if __name__ == '__main__':
    main()
