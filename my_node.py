import rclpy
from rclpy.node import Node
import csv
import json
from ament_index_python.packages import get_package_share_directory

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started.')

        # Get the path to the package's share directory
        package_share_dir = get_package_share_directory('my_py_pkg')

        # Define the paths to the CSV and JSON files
        csv_file_path = package_share_dir + '/data/traj.csv'
        json_file_path = package_share_dir + '/data/traj.json'

        # Read and process the CSV file
        self.read_csv(csv_file_path)

        # Read and process the JSON file
        self.read_json(json_file_path)

    def read_csv(self, file_path):
        self.get_logger().info(f'Reading CSV file: {file_path}')
        with open(file_path, mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                self.get_logger().info(f'CSV Row: {row}')

    def read_json(self, file_path):
        self.get_logger().info(f'Reading JSON file: {file_path}')
        with open(file_path, 'r') as file:
            data = json.load(file)
            self.get_logger().info(f'JSON Data: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()