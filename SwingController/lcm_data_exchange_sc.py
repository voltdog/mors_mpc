import lcm
import numpy as np
from mors_msgs.phase_signal_msg import phase_signal_msg
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.robot_cmd_msg import robot_cmd_msg
from mors_msgs.gait_params_msg import gait_params_msg
from mors_msgs.foot_cmd_msg import foot_cmd_msg

R1 = 0
L1 = 1
R2 = 2
L2 = 3

class LCMDataExchange():
    def __init__(self,
                 gait_params_cnannel = "GAIT_PARAMS",
                 gait_phase_channel = "GAIT_PHASE",
                 robot_state_channel = "ROBOT_STATE",
                 robot_cmd_channel = "ROBOT_CMD",
                 foot_cmd_channel = "FOOT_CMD",
                 ) -> None:
        self.gait_params_cnannel = gait_params_cnannel
        self.gait_phase_channel = gait_phase_channel
        self.robot_state_channel = robot_state_channel
        self.robot_cmd_channel = robot_cmd_channel
        self.foot_cmd_channel = foot_cmd_channel

        self.t_st = 0.4
        self.t_sw = 0.25
        self.stride_height = 0.05

        self.gait_phase = [0]*4
        self.gait_phi = [0]*4

        self.ref_body_height = 0.0
        self.ref_body_yaw_vel = 0.0
        self.ref_body_vel = [0.0, 0.0, 0.0]

        self.base_pos = [0.0]*3
        self.base_orientation = [0.0]*3
        self.base_ang_vel = [0.0]*3
        self.base_lin_vel = [0.0]*3

        self.r1_pos = [0.0]*3
        self.l1_pos = [0.0]*3
        self.r2_pos = [0.0]*3
        self.l2_pos = [0.0]*3

        self.phase_sig_msg = phase_signal_msg()
        self.rbt_state_msg = robot_state_msg()
        self.rbt_cmd_msg = robot_cmd_msg()
        self.gait_prms_msg = gait_params_msg()
        self.foot_cmd_msg = foot_cmd_msg()
        self.lc_cmd = lcm.LCM()
        

    def robot_state_callback(self, channel, data : robot_state_msg):
        state_msg = robot_state_msg.decode(data)
        self.base_pos = state_msg.body.position[:]
        self.base_lin_vel = state_msg.body.lin_vel[:]
        self.base_orientation = state_msg.body.orientation[:]
        self.base_ang_vel = state_msg.body.ang_vel[:]
        self.r1_pos = state_msg.legs.r1_pos[:]
        self.l1_pos = state_msg.legs.l1_pos[:]
        self.r2_pos = state_msg.legs.r2_pos[:]
        self.l2_pos = state_msg.legs.l2_pos[:]


    def robot_state_thread(self):
        print("RobotState thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.robot_state_channel, self.robot_state_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    def robot_cmd_callback(self, channel, data : robot_cmd_msg):
        msg = robot_cmd_msg.decode(data)
        self.ref_body_height = msg.cmd_pose[2]
        self.ref_body_yaw_vel = msg.cmd_vel[5]
        self.ref_body_vel = msg.cmd_vel[0:3]


    def robot_cmd_thread(self):
        print("RobotCmd thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.robot_cmd_channel, self.robot_cmd_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    def gait_params_callback(self, channel, data : gait_params_msg):
        msg = gait_params_msg.decode(data)
        self.t_st = msg.t_st
        self.t_sw = msg.t_sw
        self.stride_height = msg.stride_height

    def gait_params_thread(self):
        print("GaitParams thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.gait_params_cnannel, self.gait_params_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass
    
    def gait_phase_callback(self, channel, data : phase_signal_msg):
        msg = phase_signal_msg.decode(data)
        self.gait_phase = msg.phase[:]
        self.gait_phi = msg.phi

    def gait_phase_thread(self):
        print("GaitPhase thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.gait_phase_channel, self.gait_phase_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    
    def get_gait_params(self):
        return self.t_st, self.t_sw, self.stride_height
    
    def get_gait_phase(self):
        return self.gait_phase, self.gait_phi
    
    def get_body_state(self):
        return self.base_pos, self.base_orientation, self.base_lin_vel, self.base_ang_vel
    
    def get_legs_pos(self):
        return self.r1_pos, self.l1_pos, self.r2_pos, self.l2_pos
    
    def get_robot_cmd(self):
        return self.ref_body_height, self.ref_body_yaw_vel, self.ref_body_vel
    
    def set_kpkd(self, kp, kd):
        self.foot_cmd_msg.Kp = kp[:]
        self.foot_cmd_msg.Kd = kd[:]

    def send_foot_cmd(self, x, dx, ddx):
        self.foot_cmd_msg.r1_pos = x[R1][:]
        self.foot_cmd_msg.l1_pos = x[L1][:]
        self.foot_cmd_msg.r2_pos = x[R2][:]
        self.foot_cmd_msg.l2_pos = x[L2][:]

        self.foot_cmd_msg.r1_vel = dx[R1][:]
        self.foot_cmd_msg.l1_vel = dx[L1][:]
        self.foot_cmd_msg.r2_vel = dx[R2][:]
        self.foot_cmd_msg.l2_vel = dx[L2][:]

        self.foot_cmd_msg.r1_acc = ddx[R1][:]
        self.foot_cmd_msg.l1_acc = ddx[L1][:]
        self.foot_cmd_msg.r2_acc = ddx[R2][:]
        self.foot_cmd_msg.l2_acc = ddx[L2][:]

        self.lc_cmd.publish(self.foot_cmd_channel, self.foot_cmd_msg.encode())