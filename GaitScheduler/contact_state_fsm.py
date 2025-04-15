
SWING = 0
STANCE = 1
LATE = 2

class ContactStateFSM():
    def __init__(self, start_td_detecting):
        self.start_td_detecting = start_td_detecting
        self.state = [STANCE]*4
        self.phi_pre = [0.0]*4

    def step(self, contact_flag, phi):
        for i in range(4):
            if self.state[i] == SWING:
                if contact_flag[i] == True and phi[i] > self.start_td_detecting:
                    self.state[i] = STANCE
                if phi[i] < 0 and self.phi_pre[i] > 0:
                    self.state[i] = LATE
            elif self.state[i] == STANCE:
                if phi[i] >= 0 and self.phi_pre[i] < 0:
                    self.state[i] = SWING
            elif self.state[i] == LATE:
                if contact_flag[i] == True:
                    self.state[i] = STANCE

            self.phi_pre[i] = phi[i]

        return self.state