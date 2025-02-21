import rclpy
from rclpy.node import Node
from message.srv import MoveRobot
from std_msgs.msg import Float64MultiArray

class MoveRobotServer(Node):
    def __init__(self):
        super().__init__('move_robot_server')
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)
        self.joint_positions_publisher = self.create_publisher(Float64MultiArray, 'ur_left_joint_group_pos_controller/commands', 10)

    def move_robot_callback(self, request, response):
        print(request.positions)

        # Process the request and move the robot accordingly
        success = self.move_robot(request.positions)

        # Set response.success based on whether the movement was successful
        response.success = success
        return response

    def move_robot(self, positions):
        # Publish the desired joint positions to the topic
        multi_array  = Float64MultiArray()

        joint_positions = [[2.0, 1.0, 2.0, 1.0, 2.0, 1.0], [1.2, 2.0, 1.1, 1.5, 1.6, 1.5]]  # Example joint positions
        for i in range(len(joint_positions)):
            print(i)
            multi_array.data = []
            multi_array.data.append(joint_positions[i][0])
            multi_array.data.append(joint_positions[i][1])
            multi_array.data.append(joint_positions[i][2])
            multi_array.data.append(joint_positions[i][3])
            multi_array.data.append(joint_positions[i][4])
            multi_array.data.append(joint_positions[i][5])

            self.joint_positions_publisher.publish(multi_array)

        # Print a message for demonstration purposes
        print("Published desired joint positions:", positions)

        # Assuming the movement was successful, return True
        return True  

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
