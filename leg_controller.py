import numpy as np
from scipy.optimize import fsolve
from scipy.special import comb

class leg_controller:
# A mid-level controller for control of each leg. Commands are given to this object by instances of the "gait_scheduler" object, and this object
# gives comands to the "pd" object.
    def __init__(self):
        self.curr_pos = np.array([np.pi/4 , np.pi/4*-2])
        self.curr_vel = 0
        # all/most controllers
        self.v_des = 1                              # desired velocity
        self.contact = 0                            # boolean, is the foot touching the ground?
        self.body_state = 0                         # current state of the body
        self.foot_forces = 0                        # forces on the foot
        self.l1 = 15                                # thigh length
        self.l2 = 15                                # shin length
        self.L = 30                                 # body length
        self.m = 20                                 # mass of the whole robot
        self.control_type = 0                       # force or mass feedback?
        self.state = 0                              # state machine state
        self.out1 = 0                               # controller output 1
        self.out2 = 0                               # controller output 2
        self.nr_err = .001                          # newton-rhapson error
        self.nr_iterlim = 15                        # newton-rhapson iteration limit
        self.bez_res = 100                          # bezier resolution
        self.transition_signal = False              # signal from the gait scheduler to transition between states
        # init state
        self.init_pos = np.array([-5,8])             # foot's init state position
        # flight state
        self.liftoff_time = 1                       # time the foot last left the ground
        self.t_flight = .2                          # time the foot has been in the flight state
        self.bezier_pts = np.array([[-self.init_pos[0],0,self.init_pos[0]],[self.init_pos[1],-5,self.init_pos[1]]])
        [self.flight_traj_x,self.flight_traj_y] = self.bezier(self.bezier_pts[0,:],self.bezier_pts[1,:])
        # stance state
        self.touchdown_time = 0                     # time the foot last touched the ground
        self.Tst = .2                               # length of the last stance phase
        self.Tsw = .22                              # pulled from cheetah paper
        self.Tair = .2                              # time both feet were in the air
        self.T = self.Tst + self.Tsw + self.Tair    # time of a total cycle
        self.ax = .1                                # x force amplitude - patrick's thesis says this is user selected, not sure why?
        self.yhip_des = 15                          # desired robot hip ride height
        self.thetad_des = 0                         # desired pitch oscillation speed
        self.k_stab = 1                             # gait pattern stabilizer gain
        self.kpy_hip = 1                            # ride height p gain
        self.kdy_hip = 1                            # ride height d gain
        self.kdx_v = 1                              # velocity p gain
        self.kp_th = 1                              # pitch oscillation p gain
        self.kd_th = 1                              # pitch oscillation d gain

    def run_controller(self,t):
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
                self.state=0
            return  self.flight(t)
        # stance state
        elif self.state == 2:
            return self.stance(t)
        
    def transitions(self,transition,t):
        # transition init --> flight
        if transition == 0:
            # switch the state
            self.state = 1
            # generate the bezier curve
            #[self.flight_traj_x,self.flight_traj_y] = self.bezier(self.bezier_pts[0,:],self.bezier_pts[1,:])
            # update liftoff time
            self.liftoff_time = t
        # transition flight --> stance
        elif transition == 2:
            # switch the state
            self.state = 2
            # update the touchdown time
            self.touchdown_time = t
            # switch to force feedback
            self.control_type = 1

        # transition stance --> flight
        elif transition == 3:
            # switch to position feedback
            self.control_type = 0

    # init state for the leg - just stands there
    def init(self,t):
        # return the joint positions corresponding to the init x and y
        # coords
        joints_des = self.cartesian_to_joint(self.curr_pos,self.init_pos[0],self.init_pos[1])
        return joints_des
    
    # flight state for the leg - position control along a bezier curve
    def flight(self,t):
        # update the position along the bezier trajectory
        flight_percent = (t-self.liftoff_time)/self.t_flight
        flight_idx = round(self.bez_res * flight_percent)
        if flight_idx > (np.size(self.flight_traj_x) - 1):
            flight_idx = np.size(self.flight_traj_x)-1
        x = self.flight_traj_x[flight_idx]
        y = self.flight_traj_y[flight_idx]

        # convert bezier (x,y) to joints (th1,th2)
        joints_des = self.cartesian_to_joint(self.curr_pos,x,y)
    
        #print(f"X: {x} Y: {y}       joints_Des: {joints_des}")

        return joints_des

# ----------------------------------------------------- Helper Functions ------------------------------------------------------------
    # Inverse Kinematics:
    # Inverse Kinematics - Jacobian
    def jacobian(self,th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        j_11 = -self.l2*np.sin(th1) - self.l2*np.sin(th1 + th2)
        j_12 = -self.l1*np.sin(th1 +th2)
        j_21 = self.l2*np.cos(th1) + self.l2*np.cos(th1+th2)
        j_22 = self.l1*np.cos(th1 + th2)
        J = np.array([[j_11,j_12],[j_21,j_22]])
        return J

    # Inverse Kinematics - F(th1,th2)
    def f(self,th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        '''if abs(th1) > np.pi:
            th1 = th1 % np.pi
        elif abs(th2) > np.pi:
            th2 = th2 % np.pi'''
        f_11 = self.l1*np.cos(th1) + self.l2*np.cos(th1 + th2) - self.y_inv
        f_21 = self.l1*np.sin(th1) + self.l2*np.sin(th1 + th2) - self.x_inv

        f = np.array([f_11,f_21])
        return f

    # Inverse Kinematics - Solution
    def cartesian_to_joint(self,th_guess,x_inv,y_inv):
        # unpack the cartesian points and account for the origin transform
        self.x_inv = x_inv
        self.y_inv = -y_inv + self.l1 + self.l2

        # check if the desired output is outside of the joint space
        if np.sqrt(self.x_inv**2 + (self.y_inv - self.l1 - self.l2)**2) > (self.l1 + self.l2):
            th1 = np.pi/4
            th2 = -np.pi/2
            print("Inverse kinematics are asking the leg to go somewhere it can't reach!")
            return np.array([th1,th2])

        # solve the inverse kinematics problem
        solution = fsolve(func=self.f, x0=th_guess, fprime=self.jacobian, full_output=False, xtol=1e-5)
        solution[0] = solution[0] % np.pi - np.pi/2
        solution[1] = solution[1] % np.pi
        return solution


    # Forward kinematics - joint space to cartesian space
    def joint_to_cartesian(self,th1,th2):

        # calculate x and y positions
        y = self.l2*np.cos(th1+th2) + self.l1*np.cos(th1)
        x = self.l2*np.sin(th1+th2) + self.l1*np.sin(th1)
        
        # shift the origin down to the leg's zero point
        y = self.l1 + self.l2 - y

        return np.array([x,y])


    # grf to joint torques - grf space to joint space
    def force_map(self,th1,th2,F):
        # calculate the Jacobian
        j = np.array([[-self.L1*np.sin(th1) - self.L2*np.sin(th1+th1) , -self.L2*np.sin(th1+th2)],[self.L1*np.cos(th1) + self.L2*np.cos(th1+th2) , self.L2*np.cos(th1+th2)]])
        
        # Transform from grf space to joint space
        torque = -np.transpose(j)*F

        return torque


    # bezier curve generator
    def bernstein_poly(self,i, n, t):
        return comb(n, i) * ( t**(n-i) ) * (1 - t)**i
    
    def bezier(self, xPoints,yPoints):

        nTimes = self.bez_res
        nPoints = len(xPoints)

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([ self.bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return np.array([xvals,yvals])


    # Gain modifier
    def gfb(self,sst):
        if 0 <= sst < .2:
            return 5*sst
        elif .2 <= sst < .8:
            return 1
        elif .8 <= 1:
            return 5 - 5*sst
        else:
            return 0