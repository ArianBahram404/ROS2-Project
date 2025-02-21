import time
import struct
from pyModbusTCP.client import ModbusClient

class CobotControl:
    """
    Class for controlling the Lexium Cobot using Modbus communication.
    Adjust coil and register addresses to match your robot's manual.
    """

    def __init__(self, i_sIpAddress="192.168.88.82", i_sPort=6502):
        """
        Initialize Modbus connection to the cobot.
        
        :param i_sIpAddress: IP address of the cobot's Modbus server
        :param i_sPort:     Port number of the cobot's Modbus server
        """
        # 1) Create the Modbus client with auto_open=False (we'll open manually).
        self.client = ModbusClient(host=i_sIpAddress, port=i_sPort, auto_open=False)

        # 2) Attempt to open the socket
        print(f"[INFO] Connecting to Modbus server at {i_sIpAddress}:{i_sPort}...")
        connected = self.client.open()
        print("[DEBUG] client.open() returned:", connected)
        if not connected:
            raise RuntimeError(f"[ERROR] Failed to open Modbus connection to {i_sIpAddress}:{i_sPort}")
        
        # 3) Confirm it's open
        if not self.client.is_open:
            raise RuntimeError("[ERROR] Modbus connection reported closed after open() call.")

        print("[INFO] CobotControl: Connection established successfully!\n")

    # -------------------------------------------------------------------------
    # 1. Fault Reset + Enable Sequence
    # -------------------------------------------------------------------------
    def reset_fault_and_enable(self):
        """
        Resets any fault (coil #53), enables motion (coil #50),
        and sets remote mode (coil #54).
        Also checks that the robot is out of fault and 'ready' (DO50=0 => no fault, DO125=1 => ready).
        Adjust addresses per your manual!
        """
        # --- Example coil definitions we write ---
        COIL_RESET_FAULT   = 53
        COIL_ENABLE_MOTION = 50
        COIL_REMOTE_MODE   = 54

        # --- Example discrete inputs (read) ---
        DI_FAULT_ACTIVE    = 50   # If DO50 means "fault present" in the manual
        DI_ROBOT_READY     = 126  # If DO126 means "robot ready" or "no fault + servo on"

        print("[INFO] Resetting fault (coil 53)...")
        self.client.write_single_coil(COIL_RESET_FAULT, True)
        #time.sleep(0.2)
        self.client.write_single_coil(COIL_RESET_FAULT, False)
        #time.sleep(0.5)

        print("[INFO] Enabling motion (coil 50)...")
        self.client.write_single_coil(COIL_ENABLE_MOTION, True)
        #time.sleep(0.2)
        self.client.write_single_coil(COIL_ENABLE_MOTION, False)
        #time.sleep(0.5)

        print("[INFO] Setting remote mode (coil 54)...")
        self.client.write_single_coil(COIL_REMOTE_MODE, True)
        #time.sleep(0.2)
        self.client.write_single_coil(COIL_REMOTE_MODE, False)
        #time.sleep(1.0)

        # Check if fault is still active
        fault_status = self.client.read_discrete_inputs(DI_FAULT_ACTIVE, 1)
        if fault_status and fault_status[0] is True:
            print("[ERROR] Robot fault is still active. Reset did not clear.")
            return False

        # Check if robot is ready
        ready_status = self.client.read_discrete_inputs(DI_ROBOT_READY, 1)
        if not ready_status or (ready_status[0] is False):
            print("[ERROR] Robot is NOT ready after reset/enable.")
            return False

        print("[INFO] Robot fault cleared, motion enabled, remote mode set, and 'ready' bit is on.\n")
        return True

    # -------------------------------------------------------------------------
    # 2. Writing Joint Positions as 32-bit Floats
    # -------------------------------------------------------------------------
    def write_joints_pose(self, joint_positions):
        """
        Writes 6 joint positions (floats) into consecutive 32-bit registers (two 16-bit words each).
        Adjust the base addresses (132, 133, etc.) to match your manual for 'target position'.
        """
        if len(joint_positions) != 6:
            raise ValueError("Expected exactly 6 joint values.")

        for i, pos in enumerate(joint_positions):
            # Convert float -> two 16-bit words
            word1, word2 = self.float_to_words(pos)

            # Example: If Joint 1 uses registers 132 & 133, Joint 2 uses 134 & 135, etc.
            reg_base = 132 + (i * 2)

            success1 = self.client.write_single_register(reg_base,   word1)
            success2 = self.client.write_single_register(reg_base+1, word2)

            print(f"[INFO] Writing Joint {i+1} => {pos:.3f} -> Registers [{reg_base}, {reg_base+1}] | "
                  f"Success: {success1}, {success2}")

    def float_to_words(self, float_value):
        """
        Converts a float into two 16-bit words (big-endian).
        """
        data = struct.pack('!f', float_value)    # float -> 4 bytes big-endian
        int_value = struct.unpack('!I', data)[0] # 4 bytes -> unsigned int
        high_word = (int_value >> 16) & 0xFFFF
        low_word  = int_value & 0xFFFF
        return high_word, low_word

    # -------------------------------------------------------------------------
    # 3. Starting and Monitoring Movement
    # -------------------------------------------------------------------------
    def start_movement(self):
        """
        Pulses the 'start move' coil (example #48). The robot must already be enabled & ready.
        """
        START_MOVE_COIL = 14  # adjust if your manual says coil #14 or #48
        print("[INFO] Starting movement (coil 48)...")
        success_on = self.client.write_single_coil(START_MOVE_COIL, True)
        #time.sleep(0.2)
        success_off = self.client.write_single_coil(START_MOVE_COIL, False)
        #time.sleep(0.3)
        print(f"[DEBUG] Movement command coil {START_MOVE_COIL} set On={success_on}, Off={success_off}\n")

    def target_Achieved(self):
        """
        Reads a discrete input that indicates the target position is reached (example #15).
        Returns bool: True if the bit is set, else False.
        """
        TARGET_DONE_BIT = 15
        result = self.client.read_discrete_inputs(TARGET_DONE_BIT, 1)
        print(f"[DEBUG] target_Achieved() => bit {TARGET_DONE_BIT} = {result}")
        return bool(result and result[0])

    def is_cobot_moving(self) -> bool:
        """
        Example function that reads discrete input #14 to see if robot is moving.
        Only if your hardware sets DO14=1 while in motion. Adjust as needed.
        """
        MOVING_BIT = 14
        result = self.client.read_discrete_inputs(MOVING_BIT, 1)
        return bool(result and result[0])

    # -------------------------------------------------------------------------
    # 4. Example 'Execute Spline' or Multi-Point Path
    # -------------------------------------------------------------------------
    def execute_spline(self, i_JointA, i_JointB, i_JointC):
        """
        Moves the robot through three predefined 6-DOF joint positions in sequence: A -> B -> C.
        This is just an example usage pattern. Adjust as you see fit.
        """
        print("[INFO] Executing spline motion with positions A, B, C:")
        print(" A:", i_JointA)
        print(" B:", i_JointB)
        print(" C:", i_JointC)

        # Basic validation
        for j in [i_JointA, i_JointB, i_JointC]:
            if not (isinstance(j, list) and len(j) == 6):
                raise ValueError("All joint inputs must be lists of 6 floats.")

        # 1) Make sure robot is fault-free, enabled, remote mode
        ok = self.reset_fault_and_enable()
        if not ok:
            print("[ERROR] Robot not enabled. Aborting spline.")
            return

        # 2) Simple state machine to move through A->B->C
        xEndSpline = False
        uiLSM = 0
        while not xEndSpline:
            if uiLSM == 0:
                self.write_joints_pose(i_JointA)
                self.start_movement()
                uiLSM = 1
                print("[STATE] Sent position A => waiting for target_Achieved")
            elif uiLSM == 1 and self.target_Achieved():
                uiLSM = 2
                print("[STATE] Position A reached!\n")

            elif uiLSM == 2:
                self.write_joints_pose(i_JointB)
                self.start_movement()
                uiLSM = 3
                print("[STATE] Sent position B => waiting for target_Achieved")
            elif uiLSM == 3 and self.target_Achieved():
                uiLSM = 4
                print("[STATE] Position B reached!\n")

            elif uiLSM == 4:
                self.write_joints_pose(i_JointC)
                self.start_movement()
                uiLSM = 5
                print("[STATE] Sent position C => waiting for target_Achieved")
            elif uiLSM == 5 and self.target_Achieved():
                uiLSM = 6
                print("[STATE] Position C reached!\n")

            elif uiLSM == 6:
                xEndSpline = True

            time.sleep(0.2)

        print("[INFO] Spline execution complete!\n")

    # -------------------------------------------------------------------------
    # 5. Reading Actual Joint Positions
    # -------------------------------------------------------------------------
    def get_joints_pose(self):
        """
        Returns a list of actual joint positions with a retry mechanism.
        Adjust addresses [382..393] per your manual for 'actual positions'.
        Each pair is a 32-bit float. Example uses (382, 384, 386, 388, 390, 392).
        """
        joint_positions = [0.0] * 6
        register_addresses = [382, 384, 386, 388, 390, 392]

        for i, addr in enumerate(register_addresses):
            success = False
            for attempt in range(3):
                words = self.client.read_input_registers(addr, 2)
                if words and len(words) == 2:
                    joint_positions[i] = self.__words_to_float(words[0], words[1])
                    success = True
                    break
                else:
                    print(f"[Attempt {attempt+1}] Could not read joint {i+1} from register {addr}")
            if not success:
                print(f"[ERROR] Joint {i+1} read failed after 3 tries; using last known value.")

        return joint_positions

    def __words_to_float(self, word1, word2):
        """
        Converts two 16-bit words into a float (big-endian).
        """
        combined_int = (word1 << 16) | (word2 & 0xFFFF)
        return struct.unpack('!f', struct.pack('!I', combined_int))[0]
