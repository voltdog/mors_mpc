import numpy as np
from scipy.integrate import ode

WALK = 0
TROT = 1
PACE = 2
GALLOP = 3
PRONKING = 4

class HopfGaitScheduler(object):
    def __init__(self):
        self.r = [0]*4
        self.w = [0]*4
        self.sum_con = [[0,0],[0,0],[0,0],[0,0]]
        self.x = [0]*4
        self.y = [0]*4

        # gait type params
        self.phi_walk   = [0, 0.5*np.pi, np.pi, 1.5*np.pi] # walk gait
        self.phi_trot   = [np.pi, 0, 0, np.pi] # trot gait
        self.phi_pace   = [np.pi, 0, np.pi, 0] # pace gait
        self.phi_gallop = [0, 0, np.pi, np.pi] # gallop gait
        self.phi_pronking = [0, 0, 0, 0] # pronking gait

        self.xy_init = [0.12, -0.01, 0.0001, 0.0001, 0.0001, 0.0001, 0.0, 0.0]

        self.t0 = 0.0

    def set_static_params(self, ampl=1, alpha=100, lamb=1, a=1, x_offset=0.084, y_offset=0.0):
        self.mu = np.sqrt(ampl)
        self.alpha = alpha
        self.lamb = lamb
        self.a = a
        
        self.x_offset = x_offset
        self.y_offset = y_offset

    def set_gait_params(self, w_sw=0.0, w_st=0.0, phi=TROT):
        self.w_sw = w_sw
        self.w_st = w_st

        if phi == WALK:
            self.phi = self.phi_walk
        elif phi == TROT:
            self.phi = self.phi_trot
        elif phi == PACE:
            self.phi = self.phi_pace
        elif phi == GALLOP:
            self.phi = self.phi_gallop
        elif phi == PRONKING:
            self.phi = self.phi_pronking
        else:
            self.phi = phi

    def hopf_osc(self, t, xy):
        self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3] = xy

        for i in range(4):
            self.r[i] = np.sqrt(self.x[i]**2 + self.y[i]**2)
            self.w[i] = (self.w_sw/(np.exp(-self.a*self.y[i])+1)) + (self.w_st/(np.exp(self.a*self.y[i])+1))
            self.sum_con[i][0] = 0
            self.sum_con[i][1] = 0
            for j in range(4):
                self.sum_con[i][0] += self.lamb*(np.cos(self.phi[i]-self.phi[j])*self.x[j] - np.sin(self.phi[i]-self.phi[j])*self.y[j])
                self.sum_con[i][1] += self.lamb*(np.sin(self.phi[i]-self.phi[j])*self.x[j] + np.cos(self.phi[i]-self.phi[j])*self.y[j])

        
        return [self.alpha*(self.mu-self.r[0]**2)*self.x[0]-self.w[0]*self.y[0]+self.sum_con[0][0], 
                self.alpha*(self.mu-self.r[0]**2)*self.y[0]+self.w[0]*self.x[0]+self.sum_con[0][1],
                self.alpha*(self.mu-self.r[1]**2)*self.x[1]-self.w[1]*self.y[1]+self.sum_con[1][0], 
                self.alpha*(self.mu-self.r[1]**2)*self.y[1]+self.w[1]*self.x[1]+self.sum_con[1][1],
                self.alpha*(self.mu-self.r[2]**2)*self.x[2]-self.w[2]*self.y[2]+self.sum_con[2][0], 
                self.alpha*(self.mu-self.r[2]**2)*self.y[2]+self.w[2]*self.x[2]+self.sum_con[2][1],
                self.alpha*(self.mu-self.r[3]**2)*self.x[3]-self.w[3]*self.y[3]+self.sum_con[3][0], 
                self.alpha*(self.mu-self.r[3]**2)*self.y[3]+self.w[3]*self.x[3]+self.sum_con[3][1]]

    def init_integrator(self, dt):
        self.de = ode(self.hopf_osc)
        self.de.set_integrator('lsoda')
        self.de.set_initial_value(self.xy_init, self.t0)
        self.dt = dt # direiative time

    def step(self):
        self.de.integrate(self.de.t + self.dt)

        self.phi_out1 = np.arctan2(self.de.y[1], self.de.y[0])/np.pi
        self.phi_out2 = np.arctan2(self.de.y[3], self.de.y[2])/np.pi
        self.phi_out3 = np.arctan2(self.de.y[5], self.de.y[4])/np.pi
        self.phi_out4 = np.arctan2(self.de.y[7], self.de.y[6])/np.pi

        return [self.phi_out1, self.phi_out2, self.phi_out3, self.phi_out4]
