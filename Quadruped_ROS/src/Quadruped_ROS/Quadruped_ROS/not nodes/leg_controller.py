import numpy as np
from scipy.optimize import fsolve
from scipy.special import comb


class leg_controller:
    # A mid-level controller for control of each leg. Commands are given to this selfect by instances of the "gait_scheduler" selfect, and this selfect
    # gives comands to the "pd" selfect.
    def __init__(self, name):
        self.name = name
        self.curr_pos = np.array([np.pi / 4, np.pi / 4 * -2])
        self.curr_vel = np.array([0, 0])

        # all/most controllers
        self.v_des = 1  # desired velocity
        self.contact = 0  # boolean, is the foot touching the ground?
        self.pitch = 0  # the robot's pitch
        self.pitch_vel = 0  # the robot's pich angular velocity
        self.roll = 0  # the robot's roll
        self.roll_vel = 0  # the robot's roll angular velocity
        self.yaw = 0  # the robot's yaw
        self.yaw_vel = 0  # the robot's yaw velocity
        self.forw_vel = 0  # the robot's forward velocity
        self.foot_forces = 0  # forces on the foot
        self.l1 = .15  # thigh length
        self.l2 = .15  # shin length
        self.body_length = .30  # body length
        self.body_width = .8  # body width
        self.m = 8  # mass of the whole robot
        self.control_type = 0  # force or mass feedback?
        self.state = 0  # state machine state
        self.nr_iterlim = 15  # newton-rhapson iteration limit
        self.bez_res = 100  # bezier resolution
        self.transition_signal = False  # signal from the gait scheduler to transition between states
        self.avg_force = np.zeros(2)  # moving average of the joint forces
        self.contact_thresh = 1  # threshold for contact detection
        self.contact = False  # is the foot in contact with the ground?
        self.front_or_rear = 1  # is the leg on the front or rear of the robot? 1 for front, -1 for rear

        # init state
        self.init_pos = np.array([0, .2])  # foot's init state position

        # flight state
        self.liftoff_time = 1  # time the foot last left the ground
        self.t_flight = .2  # time the foot has been in the flight state
        self.bezier_x = np.array([
            -0.2, -0.259, -0.275, -0.384, 0.261, -0.017, 0.248, 0.267, 0.259,
            0.2
        ])  # MIT values
        self.bezier_y = np.array(
            [0.5, 0.45, 0.406, 0.065, 1.031, -0.095, 0.545, 0.374, 0.45,
             0.5])  # MIT values
        self.bezier_x = self.bezier_x * ((self.l1 + self.l2) / .5)  # scale points to fit our robot's leg length
        self.bezier_y = self.bezier_y * ((self.init_pos[1]) / .5)  # scale points to fit our robot's ride height
        self.bezier_y = self.bezier_y   # change the coordinate system from MIT's to mine
        self.bezier_pts = np.array([self.bezier_x, self.bezier_y])
        self.base_bezier = self.bezier(self.bezier_pts[0, :],
                                       self.bezier_pts[1, :])
        self.flight_traj_x = self.base_bezier[0]
        self.flight_traj_y = self.base_bezier[1]
        self.flight_traj_idx = np.zeros_like(self.flight_traj_x)

        # stance state
        self.touchdown_time = 0  # time the foot last touched the ground
        self.Tst = .15  # length of the last stance phase
        self.Tsw = .22  # pulled from cheetah paper
        self.Tair = .05  # time both feet were in the air
        self.T = self.Tst + self.Tsw + self.Tair  # time of a total cycle
        self.ax = 0  # x force amplitude - patrick's thesis says this is user selected, not sure why?
        self.yhip_des = 20  # desired robot hip ride height
        self.thetad_des = 0  # desired pitch oscillation speed
        self.k_stab = 1  # gait pattern stabilizer gain
        self.kpy_hip = 0  #.5  # ride height p gain
        self.kdy_hip = 0  #.5  # ride height d gain
        self.kdx_v = 0  #.5  # velocity p gain
        self.kp_th = 0  #.5  # pitch oscillation p gain
        self.kd_th = 0  #.5  # pitch oscillation d gain
        self.curr_force = np.zeros(
            2)  # current environmental forces on the joints

    def run_controller(self, t):
        # handle transitions
        # init --> stance
        if self.state == 0:
            if self.transition_signal:  #t > .5:#self.transition_signal:
                self.transitions(0, t)
        # flight --> stance
        elif self.state == 1:
            if self.transition_signal:
                self.transitions(1, t)
        # stance -- > flight
        elif self.state == 2:
            if self.transition_signal:  #not self.contact:  #self.transition_signal:
                self.transitions(2, t)

        # run the appropriate controller
        if self.state == 0:
            return self.init(t)
        elif self.state == 1:
            return self.flight(t)
        elif self.state == 2:
            return self.stance(t)

    def transitions(self, transition, t):
        self.transition_signal = False
        # transition init --> flight
        if transition == 0:
            # switch the state to stance
            self.state = 2
            # update the touchdown time
            self.touchdown_time = t
            # switch to force feedback
            self.control_type = 1
            # apply gait pattern stabilization
            self.Tst = self.body_length / self.v_des - self.k_stab * (
                self.Tst + self.Tair - self.T / 2)
        # transition flight --> stance
        elif transition == 1:
            # switch the state to stance
            self.state = 2
            # update the touchdown time
            self.touchdown_time = t
            # switch to force feedback
            self.control_type = 1
            # apply gait pattern stabilization
            self.Tst = self.body_length / self.v_des - self.k_stab * (
                self.Tst + self.Tair - self.T / 2)
        # transition stance --> flight
        elif transition == 2:
            # ----- Flags and value updates -----
            # switch to position feedback
            self.control_type = 0
            # drop the contact flag
            self.contact = False
            # switch the state to flight
            self.state = 1
            # update the liftoff time
            self.liftoff_time = t

            # ----- Trajectory Planning -----
            foot_pos = self.joint_to_cartesian(self.curr_pos[0],
                                               self.curr_pos[1])
            foot_vel = self.joint_to_cartesian(self.curr_vel[0],
                                               self.curr_vel[1])
            # generate the base bezier curve
            bezier_base = self.bezier(self.bezier_x, self.bezier_y)
            # generate the correction polynomial (really just a line, but MIT calls it a polynomial for some reason)
            correction_x = np.linspace(foot_pos[0] - bezier_base[0, 0], 0)
            correction_y = np.linspace(foot_pos[1] - bezier_base[1, 1], 0)
            correction_x = np.pad(correction_x,
                                  (0, self.bez_res - np.size(correction_x)),
                                  'constant',
                                  constant_values=(0, 0))
            correction_y = np.pad(correction_y,
                                  (0, self.bez_res - np.size(correction_y)),
                                  'constant',
                                  constant_values=(0, 0))
            bezier_correction = np.array([correction_x, correction_y])
            # combine the generated trajectories
            self.flight_traj_x = bezier_base[0] + bezier_correction[0]
            self.flight_traj_y = bezier_base[1] + bezier_correction[1]
            # phase-warp the trajectory (scale s nonlinearly to match touchdown velocity)
            bezier_x_dot = np.gradient(
                bezier_base[0]
            ) * self.bez_res / self.t_flight  # second part is unit conversion to seconds!!
            self.flight_traj_idx = self.bezier(
                np.array([
                    0, 1 / 3 * bezier_x_dot[0]**(-1) * foot_vel[0],
                    1 - 1 / 3 * bezier_x_dot[-1]**(-1) * self.v_des, 1
                ]), np.array([0, 0, 0, 0]))
            self.flight_traj_idx = self.flight_traj_idx[0]
            print(f"Transition Position: {foot_pos} Transition length: {np.linalg.norm(foot_pos)}")

    # init state for the leg - just stands there
    def init(self, t):
        # return the joint positions corresponding to the init x and y
        # coords
        joints_des = self.cartesian_to_joint(self.curr_pos, self.init_pos[0],
                                             self.init_pos[1])
        return joints_des

    # flight state for the leg - position control along a bezier curve
    def flight(self, t):
        # update the position along the bezier trajectory
        # calculate the proper index to fetch from the phase-warped bezier s
        flight_percent = (t - self.liftoff_time) / self.t_flight
        flight_percent_idx = round(self.bez_res * flight_percent)
        # make sure the index is within the bounds of the trajectory length
        if flight_percent_idx > (np.size(self.flight_traj_x) - 1):
            flight_percent_idx = np.size(self.flight_traj_x) - 1
            # get the phase-warped s value and make sure it's within the bounds of the trajectory length
        flight_s = round(self.flight_traj_idx[flight_percent_idx] *
                         self.bez_res)
        if flight_s > (np.size(self.flight_traj_x) - 1):
            flight_s = np.size(self.flight_traj_x) - 1
            # get the x and y positions from the bezier trajectory
        x = self.flight_traj_x[flight_s]
        y = self.flight_traj_y[flight_s]

        # convert bezier (x,y) to joints (th1,th2)
        return self.cartesian_to_joint(self.curr_pos, x, y)

    # stance state for the leg - impulse control
    def stance(self, t):
        #unpack/calculate necessary state info
        foot_pos = self.joint_to_cartesian(self.curr_pos[0], self.curr_pos[1])
        theta = self.pitch
        thetad = self.pitch_vel
        xd = self.forw_vel
        yhip = 5
        ydhip = 5
        tbar = t - self.touchdown_time

        #calculate force profile amplitude
        ay = .25 * self.m * 9.81 * np.pi * self.T / self.Tst
        #calculate stance feedforward values
        fx_ff = self.ax * np.sin(np.pi * tbar / self.Tst)
        fy_ff = ay * np.sin(np.pi * tbar / self.Tst)

        #calculate the hip height stabilizing feedback force
        tbar = t - self.touchdown_time
        sst = tbar / self.Tst
        fx_hip = self.gfb(sst) * sst * (-self.kpy_hip *
                                        (yhip - self.yhip_des) - self.kdy_hip *
                                        (ydhip))
        fy_hip = 0
        #calculate the velocity stabilizing feedback force
        fx_v = -self.kdx_v * (xd - self.v_des)
        fy_v = 0
        #calculate the pitch stabilizing feedback force
        fx_th = 0
        fy_th = 0  #1/(-foot_pos[0]*self.front_or_rear) * (self.kp_th*theta + self.kd_th*(thetad - self.thetad_des))

        #Calculate the final output of the controller
        fx_des = (fx_ff + fx_hip + fx_v + fx_th) / 2
        fy_des = (fy_ff + fy_hip + fy_v + fy_th) / 2
        #print(f"x: {fx_des}     y: {fy_des}")

        joint_forces = self.force_map(np.flip(np.array([fx_des, fy_des])))

        return joint_forces


# ----------------------------------------------------- Helper Functions ------------------------------------------------------------
# Inverse Kinematics:
# Inverse Kinematics - Jacobian

    def jacobian(self, th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        j_11 = self.l1 * np.cos(th1) + self.l2 * np.cos(th1 + th2)
        j_12 = self.l2 * np.cos(th1 + th2)
        j_21 = -self.l1 * np.sin(th1) - self.l2 * np.sin(th1 + th2)
        j_22 = -self.l2 * np.sin(th1 + th2)
        J = np.array([[j_11, j_12], [j_21, j_22]])
        return J

    # Inverse Kinematics - F(th1,th2)
    def f(self, th_vector):
        # unpack the joint angles
        th1 = th_vector[0]
        th2 = th_vector[1]

        # restrict the joint angles to the range of -pi to pi
        if abs(th1) > np.pi:
            th1 = th1 % np.pi
        elif abs(th2) > np.pi:
            th2 = th2 % np.pi
        # restrict the knee to positive angles (knees always digigrade)
        if th2 < 0:
            th2 = 0

        # calculate the forward kinematics
        f = self.joint_to_cartesian(th1, th2) - np.array(
            [self.x_inv, self.y_inv])

        return f

    # Inverse Kinematics - Solution
    def cartesian_to_joint(self, th_guess, x_inv, y_inv):
        # unpack the cartesian points and account for the origin/orientation transform
        self.x_inv = x_inv  #-x_inv
        self.y_inv = y_inv  #-y_inv + self.l1 + self.l2

        # check if the desired output is outside of the joint space
        if np.linalg.norm([x_inv,y_inv]) > (self.l1 + self.l2):
            #print(
            #    f"Inverse kinematics are asking the leg to go somewhere it can't reach! Inputs: {x_inv}, {y_inv}"
            #)
            return self.curr_pos

        # solve the inverse kinematics problem
        solution = fsolve(func=self.f,
                          x0=th_guess,
                          fprime=self.jacobian,
                          full_output=False,
                          xtol=1e-5)
        solution[0] = solution[0]
        solution[1] = solution[1]
        return solution

    # Forward kinematics - joint space to cartesian space
    def joint_to_cartesian(self, th1, th2):

        # calculate x and y positions
        x = self.l2 * np.sin(th1 + th2) + self.l1 * np.sin(th1)
        y = self.l2 * np.cos(th1 + th2) + self.l1 * np.cos(th1)

        return np.array([x, y])

    # grf to joint torques - grf space to joint space
    def force_map(self, F):
        transformed_curr_pos = self.curr_pos - np.array([np.pi / 2, 0])
        # calculate the Jacobian
        j = self.jacobian(transformed_curr_pos)

        # Transform from grf space to joint space
        torque = -np.dot(np.transpose(j), F)

        return torque

    # bezier curve generator
    def bernstein_poly(self, i, n, t):
        return comb(n, i) * (t**(n - i)) * (1 - t)**i

    def bezier(self, xPoints, yPoints):

        nTimes = self.bez_res
        nPoints = len(xPoints)

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([
            self.bernstein_poly(i, nPoints - 1, t) for i in range(0, nPoints)
        ])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return np.array([xvals, yvals])

    # Gain modifier
    def gfb(self, sst):
        if 0 <= sst < .2:
            return 5 * sst
        elif .2 <= sst < .8:
            return 1
        elif .8 <= 1:
            return 5 - 5 * sst
        else:
            return 0

    # Leg contact detection - compares the current force to the average force and returns True if the difference is greater than a threshold
    def detect_contact(self):
        if abs(
                abs(self.avg_force[0]) + abs(self.avg_force[1]) -
            (abs(self.curr_force[0]) +
             abs(self.curr_force[1]))) > self.contact_thresh:
            self.contact = True
        self.contact = False

    # Liftoff detection - checks if the hip is decelerating to decide if it's in the air
    def detect_liftoff(self):
        # calculate the hip and the foot's vertical velocities
        foot_vel = self.joint_to_cartesian(self.curr_vel[0], self.curr_vel[1])
        if self.name == "Front Right":
            yd_hip = self.body_length * np.sin(
                self.pitch_vel) + self.body_width * np.sin(self.roll_vel)
        elif self.name == "Front Left":
            yd_hip = self.body_length * np.sin(
                self.pitch_vel) - self.body_width * np.sin(self.roll_vel)
        elif self.name == "Rear Right":
            yd_hip = -self.body_length * np.sin(
                self.pitch_vel) + self.body_width * np.sin(self.roll_vel)
        elif self.name == "Rear Left":
            yd_hip = -self.body_length * np.sin(
                self.pitch_vel) - self.body_width * np.sin(self.roll_vel)
        
        # if the hip is falling and the foot is pushing down, we've lost contact
        if (yd_hip < 0) and (foot_vel[1] < 0):
            self.contact = False
        else:
            pass
