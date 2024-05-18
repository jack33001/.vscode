import numpy as np
from scipy.optimize import fsolve

class gait_scheduler:
# A high level controller for coordinating the motion of the four legs. Commands are given to this object by some sort of motion planning object
# (not implemented in this thesis), and this object gives commands to instances of the "leg_controller" object.
    pass

class leg_controller:
# A mid-level controller for control of each leg. Commands are given to this object by instances of the "gait_scheduler" object, and this object
# gives comands to the "pd" object.
    def __init__(self):
        kp_pos = 10
        kd_pos = .5
        kp_force = 10
        kd_force = .5
        self.pd = pd_controller(kp_pos,kd_pos,kp_force,kd_force)
        self.curr_pos = np.array([np.pi/4 , np.pi/4*-2])
        self.curr_vel = 0
        # all/most controllers
        self.v_des = 1                              # desired velocity
        self.contact = 0                            # boolean, is the foot touching the ground?
        self.hip_pos = np.pi/4                      # current hip position
        self.knee_pos = np.pi/4                     # current knee position
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
        # init state
        self.init_pos = np.array([-2,7])              # foot's init state position
        # flight state
        self.liftoff_time = 0                       # time the foot last left the ground
        self.t_flight = 1                           # time the foot has been in the flight state
        self.bezier_pts = np.array([[self.init_pos[0],0,-self.init_pos[0]],[self.init_pos[1],12,self.init_pos[1]]])
        #[self.flight_traj_x,self.flight_traj_y] = self.bezier(self.bezier_pts[1,:],self.bezier_pts[2,:],np.linspace(0,.2,100))
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
            #if t >= 1:
            #    self.transitions(0,t)
            return self.init(t)
        # flight state
        elif self.state == 1:
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
            tspan = np.linspace(0,1,100)
            [self.flight_traj_x,self.flight_traj_y] = self.bezier(self.bezier_pts[1,:],self.bezier_pts[2,:],tspan)
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
        joints_des = self.cartesian_to_joint(self.init_x,self.init_y,np.pi/4,np.pi/4)
        effort = self.pd.run_pos(joints_des,self.curr_pos,self.curr_vel)
        return effort

# ----------------------------------------------------- Helper Functions ------------------------------------------------------------
    # Inverse Kinematics:
    # Inverse Kinematics - Jacobian
    def jacobian(self,th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        j_11 = -self.l1*np.sin(th1) - self.l2*np.sin(th1 + th2)
        j_12 = -self.l2*np.sin(th1 +th2)
        j_21 = self.l1*np.cos(th1) + self.l2*np.cos(th1+th2)
        j_22 = self.l2*np.cos(th1 + th2)
        J = np.array([[j_11,j_12],[j_21,j_22]])
        return J

    # Inverse Kinematics - F(th1,th2)
    def f(self,th_vector):
        th1 = th_vector[0]
        th2 = th_vector[1]
        f_11 = self.l1*np.cos(th1) + self.l2*np.cos(th1 + th2) - self.x_inv
        f_21 = self.l1*np.sin(th1) + self.l2*np.sin(th1 + th2) - self.y_inv
        f = np.array([f_11,f_21])
        return f

    # Inverse Kinematics - Solution
    def cartesian_to_joint(self,th_guess,x_inv,y_inv):
        # unpack the cartesian points and account for the origin transform
        self.x_inv = x_inv
        self.y_inv =  - y_inv + self.l1 + self.l2
        # solve the inverse kinematics problem
        solution = fsolve(func=self.f, x0=th_guess, fprime=self.jacobian, full_output=False, xtol=1e-10)
        # adjust the reference frame (theta is defined off the vertical, not off the horizontal)
        solution[0] = solution[0] - np.pi/2
        return solution


    # Forward kinematics - joint space to cartesian space
    def joint_to_cartesian(self,th1,th2):

        # calculate x and y positions
        y = self.l2*np.cos(th1+th2) + self.l1*np.cos(th1)
        x = self.l2*np.sin(th1+th2) + self.l1*np.sin(th1)
        
        # shift the origin down to the leg's zero point
        y = -(y - self.l1 - self.l2)

        return np.array([x,y])


    # grf to joint torques - grf space to joint space
    def force_map(self,th1,th2,F):
        # calculate the Jacobian
        j = np.array([[-self.L1*np.sin(th1) - self.L2*np.sin(th1+th1) , -self.L2*np.sin(th1+th2)],[self.L1*np.cos(th1) + self.L2*np.cos(th1+th2) , self.L2*np.cos(th1+th2)]])
        
        # Transform from grf space to joint space
        torque = -np.transpose(j)*F

        return torque


    # bezier curve generator
    def bezier(self,a,b,t):
        n = np.size(a)
        offset_a = a(1)
        offset_b = b(1)
        
        a = a - offset_a
        b = b - offset_b

        x = np.zeros(np.size(t))
        y = np.zeros(np.size(t))
        
        for i in range(n):
            ni = np.math.factorial(n)/(np.math.factorial(n-i)*np.math.factorial(i))

            x = x + ni * np.multiply(np.power((1-t),(n-i)),np.power(t,np.multiply(i,a(i))))
        
            y = y + ni * np.multiply(np.power((1-t),(n-i)),np.power(t,np.multiply(i,b(i))))
        

        x = x + offset_a
        y = y + offset_b

        return np.array([x,y])


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
    



class pd_controller:
    # A simple low-level class for PD control of each joint. Commands to this object are given only by instances of the "leg_controller" object.
    def __init__(self,kp_pos,kd_pos,kp_force,kd_force):
        self.kp_pos = kp_pos*np.array([1,1])
        self.kd_pos = kd_pos*np.array([1,1])
        self.kp_force = kp_force*np.array([1,1])
        self.kd_force = kd_force*np.array([1,1])

    def run_pos(self,pos_des,pos_act,vel_act):
        error = pos_des - pos_act
        return error * self.kp_pos - vel_act * self.kd_pos