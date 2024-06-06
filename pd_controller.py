import numpy as np
# A simple PD controller for the 8 joints of the robot


class PD:

    def __init__(self):
        self.kp_pos = 25
        self.kd_pos = 1
        self.kp_force = 25
        self.kd_force = 1
        self.pos_ma = 0
        self.pos_ma_window = 1
        self.force_ma = 0
        self.force_ma_window = 1

    def run_pd(self, leg, des, act, vel):
        # choose the proper controller based on the feedback variable
        if leg.control_type == 0:
            return self.pd_pos(des, act, vel)
        elif leg.control_type == 1:
            return self.pd_force(des, act, vel)

    # pd control for force
    def pd_force(self, force_des, force_curr, vel_curr):
        force_ma = self.force_ma*(self.force_ma_window - 1) + force_curr/self.force_ma_window
        return np.multiply(self.kp_force,
                           (force_des - force_curr)) - np.multiply(
                               self.kd_force, vel_curr)

    # pd control for position
    def pd_pos(self, pos_des, pos_curr, vel_curr):
        pos_ma = self.pos_ma*(self.pos_ma_window - 1)/self.pos_ma_window + pos_curr/self.pos_ma_window
        return self.kp_pos * (pos_des - pos_curr) - self.kd_pos * vel_curr
