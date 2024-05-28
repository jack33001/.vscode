import numpy as np

class gait_scheduler:
# A high level controller for coordinating the motion of the four legs. Commands are given to this object by some sort of motion planning object
# (not implemented in this thesis), and this object gives commands to instances of the "leg_controller" object.
    def __init__(self):
        self.time = 0
        self.state = 0
        # fixed time variables
        self.t_switch = .25
        self.t_elapsed = 0
        self.t_last_step = 0

    # signal for a step after a fixed amount of time
    def fixed_time_walk(self,fr_leg,fl_leg,rr_leg,rl_leg):
        self.t_elapsed = self.time - self.t_last_step
        # if the desired time increment has happened, switch the steps!
        if (self.time % self.t_switch) < self.t_elapsed:
            self.t_last_step = self.time
            # step the fr foot
            if self.state == 0:
                fr_leg.transition_signal = True
                self.state = 1
            # step the fl foot
            elif self.state == 1:
                fl_leg.transition_signal = True
                self.state = 2
            # step the rr foot
            elif self.state == 2:
                rr_leg.transition_signal = True
                self.state = 3
            # step the rl foot
            elif self.state == 3:
                rl_leg.transition_signal = True
                self.state = 0