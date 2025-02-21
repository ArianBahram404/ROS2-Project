# Separate script to connect to the robot and get the joint positions
from my_py_pkg.lexium_cobot import CobotControl

def main():
    robot_ip = "192.168.88.82"
    robot = CobotControl(i_sIpAddress=robot_ip)
    try:
        joint_positions = robot.get_joints_pose()
        print("Current Joint Positions:", joint_positions)
    except Exception as e:
        print("Failed to retrieve joint positions due to:", str(e))

# Initialize the CobotControl object with the robot's IP address
robot_ip = "192.168.88.82"
robot = CobotControl(i_sIpAddress=robot_ip)

# Try to get the current joint positions
try:
    joint_positions = robot.get_joints_pose()
    if joint_positions is None:
        print("Failed to retrieve joint positions: No data returned.")
    else:
        print("Current Joint Positions:", joint_positions)
except ConnectionError as ce:
    print("Connection Error retrieving joint positions:", str(ce))
except ValueError as ve:
    print("Data Error:", str(ve))
except Exception as e:
    print("General Error retrieving joint positions:", str(e))

if __name__ == "__main__":
    main()