import numpy as np

from foot_step_planner import FootStepPlanner
from swing_traj_generator import SwingTrajectoryGenerator
from scipy import linalg

R1 = 0
L1 = 1
R2 = 2
L2 = 3

X = 0
Y = 1
Z = 2

SWING = 0
STANCE = 1
LATE = 2

class SwingLegController():
    def __init__(self,  timestep,
                        bx, 
                        by, 
                        l1, 
                        interleave_x, 
                        interleave_y,
                        dz_near_ground,
                        k1_fsp,
                        k2_fsp,
                        ):
        self.dz_near_ground = dz_near_ground

        self.pre_phase_signal = [STANCE]*4

        self.p_start = [[0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]]
        self.p_finish = [[0.2, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]]
        self.p_rise = [[(self.p_finish[R1][X]-self.p_start[R1][X])/2, (self.p_finish[R1][Y]-self.p_start[R1][Y])/2, (self.p_finish[R1][Z]-self.p_start[R1][Z])/2 + 0.05],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]]
        self.d_p_start = [[0.0, 0.0, 1.0],
                    [0.0, 0.0, 1.0],
                    [0.0, 0.0, 1.0],
                    [0.0, 0.0, 1.0]]
        
        self.cnt = [-1]*4
        self.it_swing = [0.0]*4

        self.step_planner = [0]*4
        for i in range(4):
            self.step_planner[i] = FootStepPlanner()
            self.step_planner[i].set_coefficients(k1_fsp, k2_fsp)
        self.step_planner[R1].set_robot_params(np.array([ bx+l1+interleave_x[R1], -(by+interleave_y[R1]), 0.0], dtype=np.float64))
        self.step_planner[L1].set_robot_params(np.array([ bx+l1+interleave_x[L1],  (by+interleave_y[L1]), 0.0], dtype=np.float64))
        self.step_planner[R2].set_robot_params(np.array([-(bx+l1+interleave_x[R2]), -(by+interleave_y[R2]), 0.0], dtype=np.float64))
        self.step_planner[L2].set_robot_params(np.array([-(bx+l1+interleave_x[L2]),  (by+interleave_y[L2]), 0.0], dtype=np.float64))

        sim_freq = 1/timestep
        self.swing_traj_gen = SwingTrajectoryGenerator(sim_freq)
        

    def set_gait_params(self, t_sw, t_st, ref_stride_height):
        self.t_sw = t_sw
        self.t_st = t_st
        self.ref_stride_height = ref_stride_height
        self.swing_traj_gen.set_parameters(self.t_sw, self.dz_near_ground)

    def step(self, phase_signal, phi_cur, ref_body_height, ref_body_yaw_vel, 
             ref_body_vel, base_pos, base_lin_vel, base_rpy_rate, R_body,
             foot_pos_global):
        
        #    foot step planner
        for i in range(4):
            if phase_signal[i] == SWING:
                
                if self.pre_phase_signal[i] == STANCE and phase_signal[i] == SWING:
                    self.p_start[i] = foot_pos_global[i].tolist()
                    self.cnt[i] = -1
                    self.swing_traj_gen.reset_offsets()

                self.p_finish[i] = self.step_planner[i].step( body_pos=base_pos,
                                                R_body=R_body,
                                                body_lin_vel=base_lin_vel,
                                                body_ang_vel=base_rpy_rate,
                                                body_lin_vel_cmd=ref_body_vel,
                                                body_yaw_vel_cmd=ref_body_yaw_vel,
                                                body_height_cmd=ref_body_height,
                                                Tst=self.t_st).tolist()
                # if i == R1:
                #     print(f"{base_pos[Z]:.4f} | {self.p_finish[i][2]:.4f}")
                max_rize_z = self.step_planner[i].get_hip_location()[Z] + ref_body_height - 0.03
                # p_rise_z = self.p_finish[i][Z] + self.ref_stride_height
                p_rise_z = self.p_start[i][Z] + self.ref_stride_height
                if p_rise_z > max_rize_z:
                    p_rise_z = max_rize_z
        
                self.p_rise[i] = [self.p_start[i][X] + (self.p_finish[i][X]-self.p_start[i][X])/2, 
                            self.p_start[i][Y] + (self.p_finish[i][Y]-self.p_start[i][Y])/2, 
                            p_rise_z]
                self.it_swing[i] = phi_cur[i] * self.t_sw
                self.cnt[i] += 1
            else:
                self.p_start[i]  = [0.0]*3
                self.p_rise[i]   = [0.0]*3
                self.p_finish[i] = [0.0]*3
                self.cnt[i] = -1

        # swing leg trajectory generator  
        self.swing_traj_gen.set_parameters(self.t_sw, self.dz_near_ground)
        self.swing_traj_gen.set_points(p_start=self.p_start,
                            p_rise=self.p_rise,
                            p_finish=self.p_finish,
                            d_p_start=self.d_p_start) 
        x_ref_global, d_p_ref, dd_p_ref = self.swing_traj_gen.step(self.it_swing, self.cnt, phase_signal)

        # convert from global to local coordinates
        inv_R_body = linalg.inv(R_body)
        x_ref_local_R1 = (inv_R_body @ (np.array(x_ref_global[ :3]) - np.array(base_pos))).reshape(3,1)
        dx_ref_R1 = (inv_R_body @ np.array(d_p_ref[:3])).reshape(3,1)
        ddx_ref_R1 = (inv_R_body @ np.array(dd_p_ref[:3])).reshape(3,1)

        x_ref_local_L1 = (inv_R_body @ (np.array(x_ref_global[3:6]) - np.array(base_pos))).reshape(3,1)
        dx_ref_L1 = (inv_R_body @ np.array(d_p_ref[3:6])).reshape(3,1)
        ddx_ref_L1 = (inv_R_body @ np.array(dd_p_ref[3:6])).reshape(3,1)

        x_ref_local_R2 = (inv_R_body @ (np.array(x_ref_global[6:9]) - np.array(base_pos))).reshape(3,1)
        dx_ref_R2 = (inv_R_body @ np.array(d_p_ref[6:9])).reshape(3,1)
        ddx_ref_R2 = (inv_R_body @ np.array(dd_p_ref[6:9])).reshape(3,1)

        x_ref_local_L2 = (inv_R_body @ (np.array(x_ref_global[9: ]) - np.array(base_pos))).reshape(3,1)
        dx_ref_L2 = (inv_R_body @ np.array(d_p_ref[9: ])).reshape(3,1)
        ddx_ref_L2 = (inv_R_body @ np.array(dd_p_ref[9: ])).reshape(3,1)

        self.pre_phase_signal = phase_signal[:]

        return (x_ref_local_R1, x_ref_local_L1, x_ref_local_R2, x_ref_local_L2),  \
                (dx_ref_R1, dx_ref_R2, dx_ref_L1, dx_ref_L2),                     \
                (ddx_ref_R1, ddx_ref_L1, ddx_ref_R2, ddx_ref_L2)