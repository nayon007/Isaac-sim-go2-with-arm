import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class TrotPublisher(Node):
    def __init__(self):
        super().__init__('trot_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/robot/joint_commands', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time

        # --- Tuned Parameters for Forward Trotting ---
        hip_offset = 0.0
        hip_amp = np.deg2rad(3)              # Hip swing (small)
        hip_forward_bias = np.deg2rad(1.5)   # Forward bias to help foot placement

        thigh_offset = np.deg2rad(42)        # Crouched neutral for stability
        thigh_amp = -np.deg2rad(10)          # NEGATIVE for forward walking
        calf_offset = np.deg2rad(-82)        # Crouched
        calf_amp = -np.deg2rad(15)           # NEGATIVE for forward walking

        freq = 1.3  # step frequency in Hz

        # --- Joint indices (for Go2) ---
        FL_hip, FL_thigh, FL_calf = 0, 1, 2
        FR_hip, FR_thigh, FR_calf = 3, 4, 5
        RL_hip, RL_thigh, RL_calf = 13, 14, 15
        RR_hip, RR_thigh, RR_calf = 16, 17, 18

        # --- Trotting phase for diagonal legs ---
        phase = -2 * np.pi * freq * t
        groupA = np.sin(phase)            # FL, RR
        groupB = np.sin(phase + np.pi)    # FR, RL

        cmd = [0.0] * 19

        # FL leg (group A)
        cmd[FL_hip]   = hip_offset + hip_amp * groupA + hip_forward_bias
        cmd[FL_thigh] = thigh_offset + thigh_amp * groupA
        cmd[FL_calf]  = calf_offset + calf_amp * (-groupA)
        # FR leg (group B)
        cmd[FR_hip]   = hip_offset + hip_amp * groupB - hip_forward_bias
        cmd[FR_thigh] = thigh_offset + thigh_amp * groupB
        cmd[FR_calf]  = calf_offset + calf_amp * (-groupB)
        # RL leg (group B)
        cmd[RL_hip]   = hip_offset + hip_amp * groupB - hip_forward_bias
        cmd[RL_thigh] = thigh_offset + thigh_amp * groupB
        cmd[RL_calf]  = calf_offset + calf_amp * (-groupB)
        # RR leg (group A)
        cmd[RR_hip]   = hip_offset + hip_amp * groupA + hip_forward_bias
        cmd[RR_thigh] = thigh_offset + thigh_amp * groupA
        cmd[RR_calf]  = calf_offset + calf_amp * (-groupA)

        # Arm (7-DOF): keep at zero for now
        # cmd[6:13] = [0.0] * 7

        msg = Float64MultiArray()
        msg.data = cmd
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

