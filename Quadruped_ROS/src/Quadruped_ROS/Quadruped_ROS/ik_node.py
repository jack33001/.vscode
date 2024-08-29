#! usr/bin/env python3

# import the necessary packages
import rclpy
import numpy as np
from scipy.optimize import fsolve
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# Performs inverse kinematics on all updates to robot state topic, and also can do inverse kinematics on service requests
#
# PUBLISHES TO (topic,msg): (robot_state,cartesion_pos)
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class ik_node:
    def __init__(self):
        super().__init__("ik_node")
        self.get_logger().info("Inverse Kinematics Node has been started")
        self.cartesian_subscriber = self.create_subscription(Point, "cartesian_position", 10)
        self.joint_publisher = self.create_publisher(JointState, "joint_position", self.joint_callback, 10)
        

    def cartesian_callback(self, msg):
        positions = 


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

def main(args=None):
    rclpy.init(args=args)
    node = ik_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()