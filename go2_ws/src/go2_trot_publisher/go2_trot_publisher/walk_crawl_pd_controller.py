import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time

class WalkCrawlPDController(Node):
    def __init__(self):
        super().__init__('walk_crawl_pd_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/robot/joint_commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.start_time = time.time()
        self.current_positions = [0.0]*19
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

        freq = 0.30  # slow
        hip_offset = 0.0
        hip_amp = np.deg2rad(2)
        thigh_offset = np.deg2rad(40)
        thigh_amp = np.deg2rad(7)
        calf_offset = np.deg2rad(-90)
        calf_amp = np.deg2rad(7)

        # LEG INDICES
        FL_hip, FL_thigh, FL_calf = 0, 1, 2
        FR_hip, FR_thigh, FR_calf = 3, 4, 5
        RL_hip, RL_thigh, RL_calf = 13, 14, 15
        RR_hip, RR_thigh, RR_calf = 16, 17, 18

        # Phase offsets (FORWARD walking: legs swing one at a time, not in diagonal pairs)
        phases = {
            'FL': 0.0,
            'FR': 0.5 * np.pi,
            'RL': np.pi,
            'RR': 1.5 * np.pi
        }

        q_ref = [0.0]*19

        # FL leg
        q_ref[FL_hip]   = hip_offset + hip_amp * np.sin(2*np.pi*freq*t + phases['FL'])
        q_ref[FL_thigh] = thigh_offset + thigh_amp * np.sin(2*np.pi*freq*t + phases['FL'])
        q_ref[FL_calf]  = calf_offset + calf_amp * -np.sin(2*np.pi*freq*t + phases['FL'])
        # FR leg
        q_ref[FR_hip]   = hip_offset + hip_amp * np.sin(2*np.pi*freq*t + phases['FR'])
        q_ref[FR_thigh] = thigh_offset + thigh_amp * np.sin(2*np.pi*freq*t + phases['FR'])
        q_ref[FR_calf]  = calf_offset + calf_amp * -np.sin(2*np.pi*freq*t + phases['FR'])
        # RL leg
        q_ref[RL_hip]   = hip_offset + hip_amp * np.sin(2*np.pi*freq*t + phases['RL'])
        q_ref[RL_thigh] = thigh_offset + thigh_amp * np.sin(2*np.pi*freq*t + phases['RL'])
        q_ref[RL_calf]  = calf_offset + calf_amp * -np.sin(2*np.pi*freq*t + phases['RL'])
        # RR leg
        q_ref[RR_hip]   = hip_offset + hip_amp * np.sin(2*np.pi*freq*t + phases['RR'])
        q_ref[RR_thigh] = thigh_offset + thigh_amp * np.sin(2*np.pi*freq*t + phases['RR'])
        q_ref[RR_calf]  = calf_offset + calf_amp * -np.sin(2*np.pi*freq*t + phases['RR'])

        # Arm (7-DOF): keep at zero for now

        Kp = 1.2
        cmd = [Kp * (q_ref[i] - self.current_positions[i]) for i in range(19)]

        msg = Float64MultiArray()
        msg.data = cmd
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WalkCrawlPDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
