import numpy as np


class gait_scheduler:
    # A high level controller for coordinating the motion of the four legs. Commands are given to this object by some sort of motion planning object
    # (not implemented in this thesis), and this object gives commands to instances of the "leg_controller" object.
    def __init__(self,legs):
        self.time = 0
        self.v_des = 1.5
        self.body_l = .3
        self.stride_l = .57 * self.body_l # scaling MIT's value to our body length
        self.t_st = self.stride_l/self.v_des  
        self.t_sw = .3#.22
        self.t_air = (self.t_sw-self.t_st)/2
        self.t_cyc = self.t_air + self.t_st
        self.k_gpg = .3
        print(f"Stance: {self.t_st}, Flight: {self.t_sw}, Ratio {self.t_st/self.t_sw}")
            # fixed time variables
        self.t_init = .5
            # bounding
        self.front_t_last_step = 0
        self.rear_t_last_step = 0
        self.phase_offset = self.t_st + self.t_air
            # walking
        self.fr_t_last_step = 0
        self.fl_t_last_step = 0
        self.rr_t_last_step = 0
        self.rl_t_last_step = 0
            # test controller values
        self.triggered = False

        # set leg values
        for leg in legs:
            leg.t_flight = self.t_sw
            leg.Tst = self.t_st
            leg.T = self.t_cyc
            leg.v_des = self.v_des

    # signal for a step after a fixed amount of time
    def fixed_time_bound(self, fr_leg, fl_leg, rr_leg, rl_leg):
        # pair the legs by front and back
        front = [fr_leg, fl_leg]
        rear = [rr_leg, rl_leg]
        # update the timer for each pair of legs
        self.front_t_elapsed = self.time - self.front_t_last_step
        self.rear_t_elapsed = self.time - self.rear_t_last_step

        # if one leg makes the transition condition, transition both
        # front legs
        for leg in front:
            # init --> stance
            if leg.state == 0 and self.front_t_elapsed > self.t_init:
                for leg in front:
                    leg.transition_signal = True
                self.front_t_last_step = self.time
                break
            # flight --> stance
            elif leg.state == 1 and (self.front_t_elapsed > self.t_air) and (leg.contact == True):
                for leg in front:
                    leg.transition_signal = True
                    self.t_st = self.stride_l/self.v_des - self.k_gpg*(self.t_st + self.front_t_elapsed - self.t_cyc/2)
                self.front_t_last_step = self.time
                break
            # stance --> flight
            elif leg.state == 2 and ((self.front_t_elapsed > self.t_st) or leg.contact == False):
                for leg in front:
                    leg.transition_signal = True
                self.front_t_last_step = self.time
                break

        # rear legs
        for leg in rear:
            # init --> stance
            if leg.state == 0 and self.rear_t_elapsed > self.t_init + self.phase_offset:
                for leg in rear:
                    leg.transition_signal = True
                self.rear_t_last_step = self.time
                break
            # flight --> stance
            elif leg.state == 1 and (self.rear_t_elapsed > self.t_air) and (leg.contact == True):
                for leg in rear:
                    leg.transition_signal = True
                    self.t_st = self.stride_l/self.v_des - self.k_gpg*(self.t_st + self.rear_t_elapsed - self.t_cyc/2)
                self.rear_t_last_step = self.time
                break
            # stance --> flight
            elif leg.state == 2 and ((self.rear_t_elapsed > self.t_st) or leg.contact == False):
                for leg in rear:
                    leg.transition_signal = True
                self.rear_t_last_step = self.time
                break

    # test controller - just triggers a single transition
    def one_transition(self, fr_leg, fl_leg, rr_leg, rl_leg):
        if self.time > .5:
            if not self.triggered:
                fr_leg.transition_signal = True
                fl_leg.transition_signal = True
                rr_leg.transition_signal = True
                rl_leg.transition_signal = True
                self.triggered = True