import numpy as np

class ForwardKinematics():
    def __init__(self):
        self.l1 = 0.0595
        self.l2 = 0.13
        self.l3 = 0.1485

        self.d1 = 0.02
        self.d2 = 0.06
        self.d3 = 0.012

        self.bx = 0.106#0.1055
        self.by = 0.067

    def fkine_R1(self, theta):
        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        px = self.l1 + self.l2*np.sin(q2) + self.l3*np.sin(q2 + q3)
        py = -self.d1*np.cos(q1) - self.d2*np.cos(q1) - self.d3*np.cos(q1) + self.l2*np.sin(q1)*np.cos(q2) + self.l3*np.sin(q1)*np.cos(q2 + q3)
        pz = -self.d1*np.sin(q1) - self.d2*np.sin(q1) - self.d3*np.sin(q1) - self.l2*np.cos(q1)*np.cos(q2) - self.l3*np.cos(q2 + q3)*np.cos(q1)

        return np.array([px, py, pz])

    def fkine_L1(self, theta):
        q1 = theta[3]
        q2 = theta[4]
        q3 = theta[5]

        px = self.l1 - self.l2*np.sin(q2) - self.l3*np.sin(q2 + q3)
        py = self.d1*np.cos(q1) + self.d2*np.cos(q1) + self.d3*np.cos(q1) + self.l2*np.sin(q1)*np.cos(q2) + self.l3*np.sin(q1)*np.cos(q2 + q3)
        pz = self.d1*np.sin(q1) + self.d2*np.sin(q1) + self.d3*np.sin(q1) - self.l2*np.cos(q1)*np.cos(q2) - self.l3*np.cos(q2 + q3)*np.cos(q1)

        return np.array([px, py, pz])

    def fkine_R2(self, theta):
        q1 = theta[6]
        q2 = theta[7]
        q3 = theta[8]

        px = -self.l1 + self.l2*np.sin(q2) + self.l3*np.sin(q2 + q3)
        py = -self.d1*np.cos(q1) - self.d2*np.cos(q1) - self.d3*np.cos(q1) - self.l2*np.sin(q1)*np.cos(q2) - self.l3*np.sin(q1)*np.cos(q2 + q3)
        pz = self.d1*np.sin(q1) + self.d2*np.sin(q1) + self.d3*np.sin(q1) - self.l2*np.cos(q1)*np.cos(q2) - self.l3*np.cos(q2 + q3)*np.cos(q1)

        return np.array([px, py, pz])

    def fkine_L2(self, theta):
        q1 = theta[9]
        q2 = theta[10]
        q3 = theta[11]

        px = -self.l1 - self.l2*np.sin(q2) - self.l3*np.sin(q2 + q3)
        py = self.d1*np.cos(q1) + self.d2*np.cos(q1) + self.d3*np.cos(q1) - self.l2*np.sin(q1)*np.cos(q2) - self.l3*np.sin(q1)*np.cos(q2 + q3)
        pz = -self.d1*np.sin(q1) - self.d2*np.sin(q1) - self.d3*np.sin(q1) - self.l2*np.cos(q1)*np.cos(q2) - self.l3*np.cos(q2 + q3)*np.cos(q1)

        return np.array([px, py, pz])

    def solve(self, theta):
        X_R1 = (self.fkine_R1(theta) + [ self.bx,-self.by,0.0]) 
        X_L1 = (self.fkine_L1(theta) + [ self.bx, self.by,0.0]) 
        X_R2 = (self.fkine_R2(theta) + [-self.bx,-self.by,0.0]) 
        X_L2 = (self.fkine_L2(theta) + [-self.bx, self.by,0.0]) 

        return X_R1, X_L1, X_R2, X_L2