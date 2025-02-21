import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from message.srv import UrForwardKinematics, UrInverseKinematics
from math import radians, degrees
import time
from my_py_pkg.lexium_cobot import CobotControl

def parse_joints(joint_string):
    """
    Parse a string representing joint values (j1,j2,j3,j4,j5,j6) into a list of floats.
    """
    return [float(v) for v in joint_string.split(',')]

def convert_joints_real(real_joints, to_simulation=True):
    """
    Converts joint values between real robot and simulation.

    Parameters:
    - real_joints: list of joint values from the real robot (in degrees)
    - to_simulation: True if converting from real to simulation, False otherwise

    Returns:
    - Converted joint values as a list.
    """
    if len(real_joints) != 6:
        raise ValueError(f"Expected 6 joints, but got {len(real_joints)}. Input: {real_joints}")
    
    # Convert to radians or degrees depending on the direction
    if to_simulation:
        sim_joints = [radians(j) for j in real_joints]  # Convert to radians
    else:
        sim_joints = [degrees(j) for j in real_joints]  # Convert to degrees

    # Apply joint-specific conversions for joints 1 to 6
    sim_joints[0] = sim_joints[0]  # Joint 1 stays the same
    sim_joints[1] = -sim_joints[1]  # Reverse direction for joint 2
    sim_joints[2] = -sim_joints[2]  # Reverse direction for joint 3
    sim_joints[3] = -sim_joints[3]  # Reverse direction for joint 4
    sim_joints[4] = sim_joints[4] - 3.14  # Offset for joint 5
    sim_joints[5] = sim_joints[5]  # Joint 6 stays the same

    return sim_joints

def convert_joints(sim_joints):
    """
    Convert joint values from simulation to real robot.

    Args:
        sim_joints (list): List of joint values in simulation (radians).

    Returns:
        list: Converted joint values for the real robot (degrees).
    """
    # Convert radians to degrees for all joints
    real_joints = [degrees(j) for j in sim_joints]

    # Apply joint-specific conversions
    real_joints[0] = real_joints[0]  # Joint 1: No additional conversion
    real_joints[1] = -real_joints[1]  # Joint 2: Reverse direction
    real_joints[2] = -real_joints[2]  # Joint 3: Reverse direction
    real_joints[3] = -real_joints[3]  # Joint 4: Reverse direction
    real_joints[4] = degrees(sim_joints[4] + 3.14)  # Joint 5: Offset by +3.14 (radians) and convert to degrees
    real_joints[5] = real_joints[5]  # Joint 6: No additional conversion

    return real_joints

def pad_joints(joints, target_length=6):
    """Ensure joint list has the required length by padding with zeros."""
    return joints + [0.0] * (target_length - len(joints))

class UnifiedRobotController(Node):
    def __init__(self):
        super().__init__('unified_robot_controller')

        # sim_joints = self.get_parameter('/start_joints').get_parameter_value().string_value
        # sim_joints_end = self.get_parameter('/end_joints').get_parameter_value().string_value

        # Replace with direct initialization
        sim_joints = [174.503, 114.303, 150.554, -27.123, -74.720, 104.692]
        sim_joints_end = [-18.62, 97.42, 60.53, 30.00, -45.0, 10.0]

        # Parse joints
        self.start_joints = parse_joints(",".join(map(str, sim_joints)))
        self.end_joints = parse_joints(",".join(map(str, sim_joints_end)))

        # ROS 2 service clients and publishers
        self.forward_client = self.create_client(UrForwardKinematics, 'ur_forward_k')
        self.inverse_client = self.create_client(UrInverseKinematics, 'ur_inverse_k')
        self.joint_publisher = self.create_publisher(Float64MultiArray, '/ur_right_joint_group_pos_controller/commands', 10)

        # Subscriber to /joint_states
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.current_joint_states = None  # Store the latest joint states
        self.joint_states_received = False  # Boolean flag for joint states

        self.forward_client.wait_for_service()
        self.inverse_client.wait_for_service()

        # Connection to the robot
        self.robot_ip = "192.168.88.82"
        self.cobot_control = None
        self.real_joint_positions = []
        self.connect_to_robot()

    def connect_to_robot(self):
        """Attempt to establish a connection with the robot."""
        for attempt in range(3):
            try:
                self.get_logger().info(f"Connecting to the robot at {self.robot_ip} (Attempt {attempt + 1}/3)...")
                self.cobot_control = CobotControl(i_sIpAddress=self.robot_ip)
                self.real_joint_positions = self.cobot_control.get_joints_pose()
                if not self.real_joint_positions:
                    raise RuntimeError("Failed to retrieve joint positions from the robot.")
                self.get_logger().info(f"Connected successfully. Real robot joint positions: {self.real_joint_positions}")
                return
            except Exception as e:
                self.get_logger().error(f"Connection failed: {str(e)}")
                time.sleep(2)  # Wait before retrying
        raise RuntimeError("Failed to connect to the robot after 3 attempts.")

    def joint_states_callback(self, msg):
        """
        Callback to update current joint states.
        """
        self.current_joint_states = msg.position  # Capture the joint positions
        self.joint_states_received = True  # Set flag to True when joint states are received

    def call_forward_kinematics(self, joints):
        self.get_logger().info(f"Calling Forward Kinematics with joints: {joints}")
        req = UrForwardKinematics.Request()
        req.reference_joints = [Float64MultiArray(data=joints)]
        req.ur_type = "LexiumCobot"
        future = self.forward_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().reference_pose:
            pose = future.result().reference_pose[0]
            return pose
        else:
            raise RuntimeError("Forward Kinematics failed")

    def call_inverse_kinematics(self, pose):
        self.get_logger().info(f"Calling Inverse Kinematics for pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}, "
                               f"qx={pose.orientation.x}, qy={pose.orientation.y}, qz={pose.orientation.z}, qw={pose.orientation.w}")
        req = UrInverseKinematics.Request()
        req.reference_pose = [pose]
        req.desired_config = [1]
        req.ur_type = "LexiumCobot"
        future = self.inverse_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().complete_solution:
            return future.result().complete_solution[0].joint_matrix[0].data
        else:
            raise RuntimeError("Inverse Kinematics failed")

    def move_robot(self, joints):
        """
        Sends joint commands to both the simulation and the real robot, ensuring correct execution.
        Exits early if the real robot is not ready or faults.
        """

        # -------------------------------------------------------------
        # 1) Publish commands to Simulation
        # -------------------------------------------------------------
        joint_msg = Float64MultiArray()
        joint_msg.data = joints
        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f"Publishing joint commands (simulation): {joints}")

        # -------------------------------------------------------------
        # 2) Send commands to Real Robot (via Modbus)
        # -------------------------------------------------------------
        try:
            # ----------- Coil/bit definitions (example addresses) -----------
            COIL_RESET_FAULT    = 53   # e.g. "Reset Fault"
            COIL_DISABLE_MOTION = 55   # If #55 truly disables the drive
            COIL_MOTION_ENABLE  = 50   # e.g. "Enable servo power"
            COIL_REMOTE_MODE    = 54   # e.g. "Remote / external control"
            COIL_START_MOVE     = 14   # e.g. "Start motion"

            DI_FAULT_ACTIVE     = 50   # "Robot fault" input bit
            DI_ROBOT_READY      = 126  # "Robot ready" input bit
            DI_MOVE_IN_PROGRESS = 14   # If the same #14 bit shows "movement in progress"
            #   (Often different bits are used for "start" vs "is-moving," so check your manual)

            # ------------- Step A: Reset any existing faults ----------------
            self.get_logger().info("[INFO] Resetting any active fault...")
            self.cobot_control.client.write_single_coil(COIL_RESET_FAULT, True)
            #time.sleep(0.3)
            self.cobot_control.client.write_single_coil(COIL_RESET_FAULT, False)
            #time.sleep(0.5)

            # (Optional) If your hardware requires a “disable motion” pulse right after fault reset
            self.get_logger().info("[INFO] Disabling motion (if required by your robot docs)...")
            self.cobot_control.client.write_single_coil(COIL_DISABLE_MOTION, True)
            #time.sleep(0.2)
            self.cobot_control.client.write_single_coil(COIL_DISABLE_MOTION, False)
            #time.sleep(0.5)

            # ------------- Step B: Enable motion & set remote mode ----------
            self.get_logger().info("[INFO] Enabling motion (coil 50)...")
            self.cobot_control.client.write_single_coil(COIL_MOTION_ENABLE, True)
            #time.sleep(0.2)
            self.cobot_control.client.write_single_coil(COIL_MOTION_ENABLE, False)
            #time.sleep(0.2)

            self.get_logger().info("[INFO] Setting robot to remote mode (coil 54)...")
            self.cobot_control.client.write_single_coil(COIL_REMOTE_MODE, True)
            #time.sleep(0.2)
            self.cobot_control.client.write_single_coil(COIL_REMOTE_MODE, False)
            #time.sleep(5.0)  # wait so the drive actually transitions into remote mode

            # ------------- Step C: Check if fault is still active -----------
            self.get_logger().info("[INFO] Checking if fault is active...")
            fault_status = self.cobot_control.client.read_discrete_inputs(DI_FAULT_ACTIVE, 1)
            if fault_status and fault_status[0]:
                self.get_logger().error("[ERROR] Robot has an active fault. Reset required.")
                return

            # ------------- Step D: Check if robot is ready ------------------
            self.get_logger().info("[INFO] Checking if robot is ready...")
            max_attempts = 5
            for attempt in range(max_attempts):
                ready_status = self.cobot_control.client.read_discrete_inputs(DI_ROBOT_READY, 1)
                self.get_logger().info(f"[DEBUG] Robot ready status (bit {DI_ROBOT_READY}): {ready_status}")
                if not ready_status or (ready_status[0] is False):
                    self.get_logger().info("[INFO] Robot is ready.")
                    break
                self.get_logger().info(f"[INFO] Waiting for robot to become ready... Attempt {attempt + 1}/{max_attempts}")
                #time.sleep(1.0)
            else:
                self.get_logger().error("[ERROR] Robot did not become ready after multiple attempts.")
                return

            # ------------- Step E: Write Joint Positions --------------------
            self.get_logger().info("[INFO] Writing joint positions to Modbus registers...")
            for i, joint_value in enumerate(joints):
                register_address = 132 + (i * 2)  # Per your robot doc or your existing code
                scaled_value     = int(joint_value * 1000)
                # handle negative by rolling over:
                if scaled_value < 0:
                    scaled_value += 65536
                # clamp to 16-bit:
                clamped_value   = max(0, min(scaled_value, 65535))
                self.cobot_control.client.write_single_register(register_address, clamped_value)

            # ------------- Step F: Start movement (pulse start coil) --------
            self.get_logger().info("[INFO] Activating movement via coil 14...")
            self.cobot_control.client.write_single_coil(COIL_START_MOVE, True)
            #time.sleep(0.2)
            self.cobot_control.client.write_single_coil(COIL_START_MOVE, False)

            # ------------- Step G: Check if movement actually started -------
            for attempt in range(6):
                #time.sleep(0.5)
                movement_status = self.cobot_control.client.read_discrete_inputs(DI_MOVE_IN_PROGRESS, 1)
                self.get_logger().info(f"[DEBUG] Attempt {attempt+1}: movement_status = {movement_status}")
                if movement_status or movement_status[0]:
                    self.get_logger().info("[INFO] Robot is moving.")
                    break

            # ------------- Step H: Wait for target reached or done ----------
            # If your robot sets the same "ready" bit after motion completes or a separate "in-position" bit, poll it:
            INPOS_REGISTER = 458  # Address for the INPOS bit from the Modbus Address Table
            self.get_logger().info("[INFO] Waiting for robot to reach target position...")

            max_attempts = 20  # Increased attempts to allow more time
            for attempt in range(max_attempts):
                #time.sleep(0.5)
                # Read the INPOS register (single input register)
                inpos_status = self.cobot_control.client.read_input_registers(INPOS_REGISTER, 1)
                self.get_logger().info(f"[DEBUG] Attempt {attempt+1}: INPOS status = {inpos_status}")

                if inpos_status or inpos_status[0]:  # Check if INPOS is True (1)
                    self.get_logger().info("[INFO] Robot acknowledges target position reached.")
                    return

            # If the loop completes without success, log an error
            self.get_logger().error("[ERROR] Robot did NOT acknowledge reaching the target position. Manual intervention required.")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to check target position status: {e}")



    def generate_trajectory(self, start_joints, end_joints, num_points=100):
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            interp_joints = [
                start_joints[j] + t * (end_joints[j] - start_joints[j]) for j in range(len(start_joints))
            ]
            trajectory.append(interp_joints)
        return trajectory

    def execute_combined_trajectory(self):
        # Wait for joint states to be received
        while not self.joint_states_received:
            self.get_logger().info("Waiting for joint states...")
            rclpy.spin_once(self)

        # Convert current joint states to a list
        current_joints = list(self.current_joint_states)

        # Log actual joint lengths for debugging
        self.get_logger().info(f"Received current joints: {current_joints}, Length: {len(current_joints)}")
        self.get_logger().info(f"Start joints: {self.start_joints}, Length: {len(self.start_joints)}")
        self.get_logger().info(f"End joints: {self.end_joints}, Length: {len(self.end_joints)}")


        # Pad joint lists to ensure they are of equal length
        current_joints = (current_joints[:6] + [0.0] * 6)[:6]
        self.start_joints = (self.start_joints[:6] + [0.0] * 6)[:6]
        self.end_joints = (self.end_joints[:6] + [0.0] * 6)[:6]

        self.get_logger().info(f"Current joints: {current_joints}")
        self.get_logger().info(f"Start joints: {self.start_joints}")
        self.get_logger().info(f"End joints: {self.end_joints}")

        # Validate joint lengths
        if len(current_joints) != 6 or len(self.start_joints) != 6 or len(self.end_joints) != 6:
            self.get_logger().error("Mismatch in joint dimensions. Ensure all joint lists have six elements.")
            raise ValueError("Joint state arrays must all have six elements.")

       
        current_joints_sim = convert_joints_real(current_joints, to_simulation=True)
        start_joints_sim = convert_joints_real(self.start_joints, to_simulation=True)
        end_joints_sim = convert_joints_real(self.end_joints, to_simulation=True)

        self.get_logger().info(f"Converted current joints to simulation: {current_joints_sim}")
        self.get_logger().info(f"Converted start joints to simulation: {start_joints_sim}")
        self.get_logger().info(f"Converted end joints to simulation: {end_joints_sim}")

        self.get_logger().info(f"Original current joints: {current_joints}")
        current_joints_sim = convert_joints_real(current_joints, to_simulation=True)
        self.get_logger().info(f"Converted current joints to simulation: {current_joints_sim}")

        self.get_logger().info(f"Original start joints: {self.start_joints}")
        start_joints_sim = convert_joints_real(self.start_joints, to_simulation=True)
        self.get_logger().info(f"Converted start joints to simulation: {start_joints_sim}")

        self.get_logger().info(f"Original end joints: {self.end_joints}")
        end_joints_sim = convert_joints_real(self.end_joints, to_simulation=True)
        self.get_logger().info(f"Converted end joints to simulation: {end_joints_sim}")

        trajectory1 = self.generate_trajectory(current_joints_sim, start_joints_sim, num_points=50)
        trajectory2 = self.generate_trajectory(start_joints_sim, end_joints_sim, num_points=50)

        # Combine both trajectories
        combined_trajectory = trajectory1 + trajectory2
        self.get_logger().info(f"Combined trajectory generated with {len(combined_trajectory)} points.")
        # Execute the combined trajectory
        for i, joints in enumerate(combined_trajectory):
            self.move_robot(joints)
            self.get_logger().info(f"Executing trajectory point {i + 1}/{len(combined_trajectory)}: {joints}")
            time.sleep(0.1)  # Adjust for smoother movement

        self.get_logger().info("Completed execution of combined trajectory.")

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedRobotController()
    try:
         node.execute_combined_trajectory()
    except Exception as e:
         node.get_logger().error(str(e))
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
