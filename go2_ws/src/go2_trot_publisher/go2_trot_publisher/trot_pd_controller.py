import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time

class TrotPDController(Node):
    def __init__(self):
        super().__init__('trot_pd_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/robot/joint_commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.start_time = time.time()
        self.current_positions = [0.0]*19  # 19 DOF: 12 legs + 7 arm
        self.joint_order = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7_1',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

    def joint_state_callback(self, msg):
        joint_pos_dict = dict(zip(msg.name, msg.position))
        self.current_positions = [joint_pos_dict.get(name, 0.0) for name in self.joint_order]

    def timer_callback(self):
        t = time.time() - self.start_time

        # --- Stable Trot Parameters (Tune for your robot) ---
        hip_offset = 0.0
        hip_amp = -np.deg2rad(3)
        hip_forward_bias = np.deg2rad(3)

        thigh_offset = np.deg2rad(40)     # stands taller
        thigh_amp = -np.deg2rad(7)         # smaller amplitude
        calf_offset = np.deg2rad(-100)     # stands taller
        calf_amp = -np.deg2rad(11)         # smaller amplitude
        freq = 0.8                        # slower for stability

        FL_hip, FL_thigh, FL_calf = 0, 1, 2
        FR_hip, FR_thigh, FR_calf = 3, 4, 5
        RL_hip, RL_thigh, RL_calf = 13, 14, 15
        RR_hip, RR_thigh, RR_calf = 16, 17, 18

        phase = 2 * np.pi * freq * t
        groupA = np.sin(phase)            # FL, RR
        groupB = np.sin(phase + np.pi)    # FR, RL

        q_ref = [0.0]*19
        # FL leg (group A)
        q_ref[FL_hip]   = hip_offset + hip_amp * groupA + hip_forward_bias
        q_ref[FL_thigh] = thigh_offset + thigh_amp * groupA
        q_ref[FL_calf]  = calf_offset + calf_amp * (-groupA)
        # FR leg (group B)
        q_ref[FR_hip]   = hip_offset + hip_amp * groupB - hip_forward_bias
        q_ref[FR_thigh] = thigh_offset + thigh_amp * groupB
        q_ref[FR_calf]  = calf_offset + calf_amp * (-groupB)
        # RL leg (group B)
        q_ref[RL_hip]   = hip_offset + hip_amp * groupB - hip_forward_bias
        q_ref[RL_thigh] = thigh_offset + thigh_amp * groupB
        q_ref[RL_calf]  = calf_offset + calf_amp * (-groupB)
        # RR leg (group A)
        q_ref[RR_hip]   = hip_offset + hip_amp * groupA + hip_forward_bias
        q_ref[RR_thigh] = thigh_offset + thigh_amp * groupA
        q_ref[RR_calf]  = calf_offset + calf_amp * (-groupA)
        # Arm (7-DOF): keep at zero for now

        # --- PD Controller Parameters ---
        Kp = 1.2  # Lower for softer/smoother steps, increase if sluggish
        Kd = 0.0  # Not used

        cmd = [0.0]*19
        for i in range(19):
            cmd[i] = Kp * (q_ref[i] - self.current_positions[i])

        msg = Float64MultiArray()
        msg.data = cmd
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrotPDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
