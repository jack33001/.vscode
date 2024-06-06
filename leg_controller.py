import numpy as np
from scipy.optimize import fsolve
from scipy.special import comb


class leg_controller:
    # A mid-level controller for control of each leg. Commands are given to this selfect by instances of the "gait_scheduler" selfect, and this selfect
    # gives comands to the "pd" selfect.
    def __init__(self):
        self.curr_pos = np.array([np.pi / 4, np.pi / 4 * -2])
        self.curr_vel = 0
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
        self.l1 = 15  # thigh length
        self.l2 = 15  # shin length
        self.L = 30  # body length
        self.m = 20  # mass of the whole robot
        self.control_type = 0  # force or mass feedback?
        self.state = 0  # state machine state
        self.out1 = 0  # controller output 1
        self.out2 = 0  # controller output 2
        self.nr_err = .001  # newton-rhapson error
        self.nr_iterlim = 15  # newton-rhapson iteration limit
        self.bez_res = 100  # bezier resolution
        self.transition_signal = False  # signal from the gait scheduler to transition between states
        self.avg_force = np.zeros(2)  # moving average of the joint forces
        self.contact_thresh = 1  # threshold for contact detection
        self.contact = False  # is the foot in contact with the ground?
        # init state
        self.init_pos = np.array([-2, 8])  # foot's init state position
        # flight state
        self.liftoff_time = 1  # time the foot last left the ground
        self.t_flight = .2  # time the foot has been in the flight state
        bezier_x = np.array([-0.2, -0.259, -0.275, -0.384, 0.261, -0.017, 0.248, 0.267, 0.259, 0.2])*10
        bezier_y = np.array([0.5, 0.45, 0.406, 0.065, 1.031, -0.095, 0.545, 0.374, 0.45, 0.5])*10 + self.init_pos[1]
        self.bezier_pts = np.array([bezier_x, bezier_y])
        [self.flight_traj_x,
         self.flight_traj_y] = self.bezier(self.bezier_pts[0, :],
                                           self.bezier_pts[1, :])
        # stance state
        self.touchdown_time = 0  # time the foot last touched the ground
        self.Tst = .1  # length of the last stance phase
        self.Tsw = .22  # pulled from cheetah paper
        self.Tair = .05  # time both feet were in the air
        self.T = self.Tst + self.Tsw + self.Tair  # time of a total cycle
        self.ax = .1  # x force amplitude - patrick's thesis says this is user selected, not sure why?
        self.yhip_des = 15  # desired robot hip ride height
        self.thetad_des = 0  # desired pitch oscillation speed
        self.k_stab = 1  # gait pattern stabilizer gain
        self.kpy_hip = 1  # ride height p gain
        self.kdy_hip = 1  # ride height d gain
        self.kdx_v = 1  # velocity p gain
        self.kp_th = 1  # pitch oscillation p gain
        self.kd_th = 1  # pitch oscillation d gain
        self.curr_force = np.zeros(2)  # current environmental forces on the joints

    def run_controller(self, t):
        # init state
        if self.state == 0:
            # transition out of the stance state
            if self.transition_signal:
                self.transition_signal = False
                self.transitions(0,t)
            return self.init(t)
        # flight state
        elif self.state == 1:
            if t >= self.liftoff_time + self.t_flight:
                self.transitions(1, t)
            return self.flight(t)
        # stance state
        elif self.state == 2:
            if self.transition_signal:
                self.transition_signal = False
                self.transitions(2,t)
            return self.stance(t)

    def transitions(self, transition, t):
        # transition init --> flight
        if transition == 0:
            # switch the state to flight
            self.state = 1
            # generate the bezier curve
            #[self.flight_traj_x,self.flight_traj_y] = self.bezier(self.bezier_pts[0,:],self.bezier_pts[1,:])
            # update liftoff time
            self.liftoff_time = t
        # transition flight --> stance
        elif transition == 1:
            # switch the state to stance
            self.state = 2
            # update the touchdown time
            self.touchdown_time = t
            # switch to force feedback
            self.control_type = 1
        # transition stance --> flight
        elif transition == 2:
            # switch to position feedback
            self.control_type = 0
            # drop the contact flag
            self.contact = False
            # switch the state to flight
            self.state = 0

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
        flight_percent = (t - self.liftoff_time) / self.t_flight
        flight_idx = round(self.bez_res * flight_percent)
        if flight_idx > (np.size(self.flight_traj_x) - 1):
            flight_idx = np.size(self.flight_traj_x) - 1
        x = self.flight_traj_x[flight_idx]
        y = self.flight_traj_y[flight_idx]

        # convert bezier (x,y) to joints (th1,th2)
        return self.cartesian_to_joint(self.curr_pos, x, y)

    # stance state for the leg - impulse control
    def stance(self, t):
        #unpack/calculate necessary state info
        theta = self.pitch
        thetad = self.pitch_vel
        xd = self.forw_vel
        yhip = 5
        ydhip = 5

        #calculate force profile amplitudes
        ay = .25 * self.m * 9.81 * np.pi * self.T / self.Tst
        #calculate stance feedforward values
        fx_ff = ay * np.sin(np.pi * t / self.Tst)
        fy_ff = self.ax * np.sin(np.pi * t / self.Tst)
        fx_ff = 1
        fy_ff = 1

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
        #fx_th = 0
        #fy_th = 1/(x-x_foot) * (self.kp_th*theta + self.kd_th*(thetad - self.thetad_des))

        #gait pattern stabilizer
        self.Tst = self.L / self.v_des - self.k_stab * (self.Tst + self.Tair -
                                                        self.T / 2)

        #Calculate the final output of the controller
        fx_des = fx_ff  # + fx_hip + fx_v+ fx_th
        fy_des = fy_ff  # + fy_hip + fy_v+ fy_th

        fx_des = 0
        fy_des = 1
        joint_forces = self.force_map(np.array([fx_des, fy_des]))

        return joint_forces


# ----------------------------------------------------- Helper Functions ------------------------------------------------------------
# Inverse Kinematics:
# Inverse Kinematics - Jacobian

    def jacobian(self, th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        j_11 = -self.l2 * np.sin(th1) - self.l2 * np.sin(th1 + th2)
        j_12 = -self.l1 * np.sin(th1 + th2)
        j_21 = self.l2 * np.cos(th1) + self.l2 * np.cos(th1 + th2)
        j_22 = self.l1 * np.cos(th1 + th2)
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
        f_11 = self.l1 * np.cos(th1) + self.l2 * np.cos(th1 + th2) - self.x_inv
        f_21 = self.l1 * np.sin(th1) + self.l2 * np.sin(th1 + th2) - self.y_inv
        f = np.array([f_11, f_21])

        return f

    # Inverse Kinematics - Solution
    def cartesian_to_joint(self, th_guess, x_inv, y_inv):
        # unpack the cartesian points and account for the origin/orientation transform
        self.x_inv = -x_inv
        self.y_inv = -y_inv + self.l1 + self.l2

        # check if the desired output is outside of the joint space
        if np.sqrt(self.x_inv**2 +
                   (self.y_inv - self.l1 - self.l2)**2) > (self.l1 + self.l2):
            th1 = np.pi / 4
            th2 = -np.pi / 2
            print(
                "Inverse kinematics are asking the leg to go somewhere it can't reach!"
            )
            return np.array([th1, th2])

        # solve the inverse kinematics problem
        #print(f'IK th1th2: {th_guess}    IK xy: {self.x_inv,self.y_inv}')
        solution = fsolve(func=self.f,
                          x0=th_guess,
                          #fprime=self.jacobian,
                          full_output=False,
                          xtol=1e-5)
        solution[0] = solution[0] % np.pi - np.pi / 2
        solution[1] = solution[1] % np.pi
        return solution

    # Forward kinematics - joint space to cartesian space
    def joint_to_cartesian(self, th1, th2):

        # calculate x and y positions
        y = self.l2 * np.cos(th1 + th2) + self.l1 * np.cos(th1)
        x = self.l2 * np.sin(th1 + th2) + self.l1 * np.sin(th1)

        # shift the origin down to the leg's zero point
        y = self.l1 + self.l2 - y

        return np.array([x, y])

    # grf to joint torques - grf space to joint space
    def force_map(self, F):
        transformed_curr_pos = self.curr_pos - np.array([np.pi / 2,0])
        # calculate the Jacobian
        j = self.jacobian(transformed_curr_pos)

        # Transform from grf space to joint space
        torque = -np.dot(np.transpose(j),F)

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
        else:
            self.contact = False
