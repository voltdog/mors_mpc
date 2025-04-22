import numpy as np

NOADAPT = 0
HEIGHT_ADAPT = 1
INCL_ADAPT = 1

SWING = 0
STANCE = 1
LATE = 2

R1 = 0
L1 = 1
R2 = 2
L2 = 3

X = 0
Y = 1
Z = 2

class low_pass_filter():
    def __init__(self, cutoff_freq, dt, n_size):
        self.reconfigure_filter(cutoff_freq, dt)
        self.y = np.zeros((n_size, ), dtype=np.float64)
        self.pre_y = np.zeros((n_size, ), dtype=np.float64)
        self.pre_x = np.zeros((n_size, ), dtype=np.float64)

    def update(self, x):
        self.y += (x - self.pre_y) * self.alpha
        self.pre_y = self.y
        return self.y

    def reconfigure_filter(self, cutoff_freq, dt):
        self.alpha = (2 * np.pi * dt * cutoff_freq) / (2 * np.pi * dt * cutoff_freq + 1)



class CommandShaper():
    def __init__(self, dt, c_freq=1.0):
        #low pass filter for ref values
        self.c_freq = c_freq
        self.dt = dt
        self.lpf_lin_vel = low_pass_filter(self.c_freq, dt, 3)
        self.lpf_ang_vel = low_pass_filter(self.c_freq, dt, 1)

        ref_z_pos = 0.0
        ref_body_height = 0.21

        self.pre_phase_signal = [STANCE]*4
        self.foot_pos_global_just_stance = np.array([[0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0]], dtype=np.float64)
        self.foot_pos_local_just_stance = np.array([[0, 0, ref_z_pos-ref_body_height],
                                 [0, 0, ref_z_pos-ref_body_height],
                                 [0, 0, ref_z_pos-ref_body_height],
                                 [0, 0, ref_z_pos-ref_body_height]], dtype=np.float64)

        self.x_ref = np.array([0.0, 0.0, 0.0, # orientation 
                  0.0, 0.0, 0.2, # position
                  0.0, 0.0, -0.0, # angular velocity
                  0.0, 0.0, 0.0, # linear velocity
                  -9.81]) 
        self.body_adapt_mode = INCL_ADAPT

        self.ref_yaw_pos = 0
        self.ref_x_pos = 0
        self.ref_y_pos = 0
        
    def set_body_apadtation_mode(self, mode):
        self.body_adapt_mode = mode

    def step(self, phase_signal, foot_pos_global, foot_pos_local, ref_body_vel, ref_body_yaw_vel, ref_body_height):
        # convert velocity to body frame
        # ref_body_vel = R_body @ np.array(ref_body_vel)

        # low pass filter for ref velocity
        ref_body_vel_filtered = self.lpf_lin_vel.update(ref_body_vel)
        ref_body_yaw_vel_filtered = self.lpf_ang_vel.update(ref_body_yaw_vel)[0]

        # body height adaptation 
        for i in range(4):
            if (self.pre_phase_signal[i] == SWING and phase_signal[i] == STANCE) or (self.pre_phase_signal[i] == LATE and phase_signal[i] == STANCE):
                # np.copyto(self.foot_pos_global_just_stance[i], self.foot_pos_global_just_stance[i])
                self.foot_pos_global_just_stance[i] = foot_pos_global[i][:]
                self.foot_pos_local_just_stance[i] = foot_pos_local[i][:]
                # if i == R1:
                #     print(f"{self.foot_pos_global_just_stance[R1]} | {ref_body_height}")
        ref_z_pos = np.mean(self.foot_pos_global_just_stance[:,Z]) + ref_body_height + 0.03 #ref_body_height#
        
        virtual_leg1_pos = (self.foot_pos_local_just_stance[R1] + self.foot_pos_local_just_stance[L1])/2
        virtual_leg2_pos = (self.foot_pos_local_just_stance[R2] + self.foot_pos_local_just_stance[L2])/2
        virtual_a = -(virtual_leg1_pos[Z] - virtual_leg2_pos[Z])
        virtual_b = np.abs(virtual_leg1_pos[X] - virtual_leg2_pos[X])
        virtual_c = np.sqrt(virtual_a**2 + virtual_b**2)
        if virtual_c != 0:
            ref_pitch_pos = np.arcsin(virtual_a/virtual_c)
        else:
            ref_pitch_pos = 0.0

        # form reference vector for stance controller
        ref_pitch_vel = 0.0
        self.ref_yaw_pos += ref_body_yaw_vel_filtered*self.dt
        self.ref_x_pos += ref_body_vel_filtered[X]*self.dt
        self.ref_y_pos += ref_body_vel_filtered[Y]*self.dt
        self.x_ref[0] = 0.0
        self.x_ref[1] = 0.0#ref_pitch_pos
        self.x_ref[2] = self.ref_yaw_pos
        self.x_ref[3] = self.ref_x_pos
        self.x_ref[4] = self.ref_y_pos
        self.x_ref[5] = ref_z_pos
        self.x_ref[6] = 0.0
        self.x_ref[7] = ref_pitch_vel
        self.x_ref[8] = ref_body_yaw_vel_filtered
        self.x_ref[9] = ref_body_vel_filtered[X]
        self.x_ref[10] = ref_body_vel_filtered[Y]
        self.x_ref[11] = 0.0

        self.pre_phase_signal = phase_signal.copy()

        return self.x_ref, ref_body_vel_filtered, ref_body_yaw_vel_filtered