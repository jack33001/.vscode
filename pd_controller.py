import numpy as np
# A simple PD controller for the 8 joints of the robot


class PD:
    
    def __init__(self):
        self.kp_pos = 75
        self.kd_pos = 0
        self.pos_ma = 0
        self.pos_ma_window = 1

        self.kp_force = 50

    def run_pd(self, leg, des, act, vel):
        # choose the proper controller based on the feedback variable
        if leg.control_type == 0:
            return self.pd_pos(des, act, vel)
        elif leg.control_type == 1:
            return self.pd_force(des,act)

    # the mujoco actuators used are motors, so we need to do no pd control here
    def pd_force(self, force_des,force_curr):
        return self.kp_force * (force_des - force_curr)#force_des

    # pd control for position
    def pd_pos(self, pos_des, pos_curr, vel_curr):
        pos_ma = self.pos_ma*(self.pos_ma_window - 1)/self.pos_ma_window + pos_curr/self.pos_ma_window
        return self.kp_pos * (pos_des - pos_curr) - self.kd_pos * vel_curr
