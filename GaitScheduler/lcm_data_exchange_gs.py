import lcm
import numpy as np
from transforms3d.euler import euler2mat

from mors_msgs.phase_signal_msg import phase_signal_msg
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.gait_params_msg import gait_params_msg
from mors_msgs.robot_cmd_msg import robot_cmd_msg

class LCMDataExchange():
    def __init__(self,
                 gait_params_cnannel = "GAIT_PARAMS",
                 gait_phase_channel = "GAIT_PHASE",
                 robot_state_channel = "ROBOT_STATE",
                 robot_cmd_channel = "ROBOT_CMD",
                 robot_ref_channel = "ROBOT_REF",
                 ) -> None:
        self.gait_params_cnannel = gait_params_cnannel
        self.gait_phase_channel = gait_phase_channel
        self.robot_state_channel = robot_state_channel
        self.robot_cmd_channel = robot_cmd_channel
        self.robot_ref_channel = robot_ref_channel

        self.gait_type = [np.pi, 0, 0, np.pi]
        self.t_st = 0.4
        self.t_sw = 0.25
        self.standing = True
        self.stride_height = 0.0

        self.gait_phase = [0]*4
        self.gait_phi = [0]*4

        self.ref_pos = [0.0]*3
        self.ref_orientation = [0.0]*3
        self.ref_lin_vel = [0.0]*3
        self.ref_ang_vel = [0.0]*3

        self.leg_contact_state = [0]*4
        self.body_pos = [0,0,0]
        self.body_rpy = [0.0]*3
        self.foot_pos_local = np.array([0.0]*4, dtype=np.float64)


        self.phase_sig_msg = phase_signal_msg()
        self.rbt_state_msg = robot_state_msg()
        self.gait_prms_msg = gait_params_msg()
        self.robot_ref_msg = robot_cmd_msg()
        self.robot_cmd_msg = robot_cmd_msg()
        self.lc_cmd = lcm.LCM()
        

    def robot_state_callback(self, channel, data : robot_state_msg):
        state_msg = robot_state_msg.decode(data)
        self.leg_contact_state = state_msg.legs.contact_states[:]
        self.body_rpy = state_msg.body.orientation[:]
        self.body_pos = state_msg.body.position[:]

        self.R_body = euler2mat(self.body_rpy[0], self.body_rpy[1], self.body_rpy[2])

        self.foot_pos_local = np.array([self.R_body @ np.array(state_msg.legs.r1_pos, dtype=np.float64).reshape(3,),
                                        self.R_body @ np.array(state_msg.legs.l1_pos, dtype=np.float64).reshape(3,),
                                        self.R_body @ np.array(state_msg.legs.r2_pos, dtype=np.float64).reshape(3,),
                                        self.R_body @ np.array(state_msg.legs.l2_pos, dtype=np.float64).reshape(3,)])


    def robot_state_thread(self):
        print("RobotState thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.robot_state_channel, self.robot_state_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

    def gait_params_callback(self, channel, data : gait_params_msg):
        msg = gait_params_msg.decode(data)
        self.gait_type = msg.gait_type[:]
        self.t_st = msg.t_st
        self.t_sw = msg.t_sw
        self.standing = msg.standing
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

    def robot_ref_callback(self, channel, data : robot_cmd_msg):
        msg = robot_cmd_msg.decode(data)
        self.ref_pos = msg.cmd_pose[:3]
        self.ref_orientation = msg.cmd_pose[3:]
        self.ref_lin_vel = msg.cmd_vel[:3]
        self.ref_ang_vel = msg.cmd_vel[3:]


    def robot_ref_thread(self):
        print("GaitParams thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.robot_ref_channel, self.robot_ref_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def send_gait_phase(self, phase, phi):
        self.phase_sig_msg.phase = phase[:]
        self.phase_sig_msg.phi = phi[:]

        self.lc_cmd.publish(self.gait_phase_channel, self.phase_sig_msg.encode())

    def send_robot_cmd(self, pos, orientation, lin_vel, ang_vel):
        self.robot_cmd_msg.cmd_pose[:3] = pos[:]
        self.robot_cmd_msg.cmd_pose[3:] = orientation[:]
        self.robot_cmd_msg.cmd_vel[:3] = lin_vel[:]
        self.robot_cmd_msg.cmd_vel[3:] = ang_vel[:]

        self.lc_cmd.publish(self.robot_cmd_channel, self.robot_cmd_msg.encode())


    def get_leg_contacts(self):
        return self.leg_contact_state
    
    def get_leg_pos_local(self):
        return self.foot_pos_local
    
    def get_body_state(self):
        return self.body_pos, self.body_rpy
    
    def get_gait_params(self):
        return self.gait_type, self.t_st, self.t_sw, self.standing
    
    def get_robot_ref(self):
        return self.ref_pos, self.ref_orientation, self.ref_lin_vel, self.ref_ang_vel