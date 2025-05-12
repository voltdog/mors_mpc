import numpy as np

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

class SwingTrajectoryGenerator():
    def __init__(self, 
                 sim_freq=200,
                 ) -> None:
        self.inc = 1/sim_freq
        self.sim_freq = sim_freq
        self.dz_near_ground = 0.0

        self.t_zr = [0]*4
        self.t_zd = [0]*4
        self.t_xsw = [0]*4
        self.t_ysw = [0]*4
        self.a_zr = [0]*6
        self.a_zd = [0]*6
        self.a_xsw = [0]*6
        self.a_ysw = [0]*6
        self.it_offset = [[0]*4]*4

        self.pre_p_rise = [0]*4
        self.pre_p_desc = [0]*4
        self.pre_px_swing = [0]*4
        self.pre_px_stance = [0]*4
        self.pre_py_swing = [0]*4
        self.pre_py_stance = [0]*4
        self.p_start = [[0]*3]*4
        self.p_rise = [[0]*3]*4
        self.p_finish = [[0]*3]*4

        self.p_x=[0]*4
        self.p_y=[0]*4
        self.p_z=[0]*4
        self.p_ref = [0]*12
        self.d_p_ref = [0]*12
        self.dd_p_ref = [0]*12

        self.tf = [0]*4

    def __calc_a(self, p0, pf, tf, d_p0=0, d_pf=0, dd_p0=0, dd_pf=0):
        a0 = p0
        a1 = d_p0
        a2 = dd_p0/2
        if tf != 0:
            a3 = (20*pf - 20*p0 - (8*d_pf + 12*d_p0)*tf - (3*dd_p0 - dd_pf)*tf**2) / (2 * tf**3)
            a4 = (30*p0 - 30*pf + (14*d_pf + 16*d_p0)*tf + (3*dd_p0 - 2*dd_pf)*tf**2) / (2 * tf**4)
            a5 = (12*pf - 12*p0 - (6*d_pf + 6*d_p0)*tf - (dd_p0 - dd_pf)*tf**2) / (2 * tf**5)
        else:
            a3 = 0.0
            a4 = 0.0
            a5 = 0.0

        return [a0, a1, a2, a3, a4, a5]
    
    def __calc_spline(self, a, t):
        t2 = t**2
        t3 = t**3
        t4 = t**4
        t5 = t**5

        p = a[0] + a[1]*t + a[2]*t2 + a[3]*t3 + a[4]*t4 + a[5]*t5
        dp = a[1] + 2*a[2]*t + 3*a[3]*t2 + 4*a[4]*t3 + 5*a[5]*t4
        ddp = 2*a[2] + 6*a[3]*t + 12*a[4]*t2 + 20*a[5]*t3

        return p, dp, ddp
    
    def __map_z_rising(self, leg_num, it, p_start, p_rise, d_p_start, tf, cnt):
        self.a_zr[leg_num] = self.__calc_a(p_start[Z], p_rise[Z], tf+self.inc, d_p_start[Z], 0.0, 0.0, 0.0)

        self.t_zr[leg_num] = it - self.it_offset[leg_num][Z]
        p_zr, d_p_zr, dd_p_zr = self.__calc_spline(self.a_zr[leg_num], self.t_zr[leg_num])

        return p_zr, d_p_zr, dd_p_zr
    
    def __map_z_descending(self, leg_num, it, p_rise, p_finish, tf, cnt, dp_finish):
        self.a_zd[leg_num] = self.__calc_a(p_rise[Z], p_finish[Z], tf, 0.0, dp_finish, 0.0, 0.0)
        
        self.t_zd[leg_num] = it - self.it_offset[leg_num][Z] - tf/4
        p_zd, d_p_zd, dd_p_zd = self.__calc_spline(self.a_zd[leg_num], self.t_zd[leg_num])

        return p_zd, d_p_zd, dd_p_zd
    
    def __map_xy_swing(self, it, p_start, p_finish, d_p_start, tf):
        a = self.__calc_a(p_start, p_finish, tf, d_p_start, 0.0, 0.0, 0.0)
        p_xsw, d_p_xsw, dd_p_xsw = self.__calc_spline(a, it)

        return p_xsw, d_p_xsw, dd_p_xsw

    def set_parameters(self, t_swing, dz_near_ground):
        self.tf = t_swing
        self.cnt_stride = int(t_swing * self.sim_freq)
        self.dz_near_ground = dz_near_ground

    def set_points(self, p_start, p_rise, p_finish, d_p_start):
        self.p_start = p_start
        self.p_rise = p_rise
        self.p_finish = p_finish
        self.d_p_start = d_p_start

    def reset_offsets(self):
        for i in range(4):
            for j in range(4):
                self.it_offset[i][j] = 0.0

    def step(self, it, cnt, leg_phase):
    
        for i in range(4):
            if leg_phase[i] == SWING:
                p_x, d_p_x, dd_p_x = self.__map_xy_swing(
                                                      it=it[i], 
                                                      p_start=self.p_start[i][X], 
                                                      p_finish=self.p_finish[i][X], 
                                                      d_p_start=self.d_p_start[i][X],
                                                      tf=self.tf, 
                                                      )
                p_y, d_p_y, dd_p_y = self.__map_xy_swing(
                                                      it=it[i], 
                                                      p_start=self.p_start[i][Y], 
                                                      p_finish=self.p_finish[i][Y], 
                                                      d_p_start=self.d_p_start[i][Y],
                                                      tf=self.tf, 
                                                      )

                if 0 <= it[i] < self.tf/4:
                    p_z, d_p_z, dd_p_z = self.__map_z_rising(leg_num=i, 
                                                          it=it[i], 
                                                          p_start=self.p_start[i], 
                                                          p_rise=self.p_rise[i], 
                                                          d_p_start=self.d_p_start[i],
                                                          tf=self.tf/4, cnt=cnt[i])
                else:
                    p_z, d_p_z, dd_p_z = self.__map_z_descending(leg_num=i, 
                                                              it=it[i], 
                                                              p_rise=self.p_rise[i], 
                                                              p_finish=self.p_finish[i], 
                                                              dp_finish=self.dz_near_ground,
                                                              tf=3*self.tf/4, 
                                                              cnt=cnt[i])

                self.p_ref[3*i] = p_x
                self.p_ref[3*i+1] = p_y
                self.p_ref[3*i+2] = p_z

                self.d_p_ref[3*i] = d_p_x
                self.d_p_ref[3*i+1] = d_p_y
                self.d_p_ref[3*i+2] = d_p_z

                self.dd_p_ref[3*i] = dd_p_x
                self.dd_p_ref[3*i+1] = dd_p_y
                self.dd_p_ref[3*i+2] = dd_p_z
            elif leg_phase[i] == LATE:
                if self.dz_near_ground < -0.1:
                    self.p_ref[3*i+2] += (self.dz_near_ground*self.inc)
                    self.d_p_ref[3*i+2] = self.dz_near_ground
                else:
                    self.p_ref[3*i+2] += (-0.1*self.inc)
                    self.d_p_ref[3*i+2] = -0.1
            # elif leg_phase[i] == STANCE:
            #     self.p_ref[3*i] = 0.0
            #     self.p_ref[3*i+1] = 0.0
            #     self.p_ref[3*i+2] = 0.0

            #     self.d_p_ref[3*i] = 0.0
            #     self.d_p_ref[3*i+1] = 0.0
            #     self.d_p_ref[3*i+2] = 0.0

            #     self.dd_p_ref[3*i] = 0.0
            #     self.dd_p_ref[3*i+1] = 0.0
            #     self.dd_p_ref[3*i+2] = 0.0

        return self.p_ref, self.d_p_ref, self.dd_p_ref