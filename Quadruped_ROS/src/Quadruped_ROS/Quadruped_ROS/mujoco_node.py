#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node is the simulation environment - it handles the physics of the robot and the rendering of the robot
#
# PUBLISHES TO (topic,msg): 
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class mujoco_node:
    def __init__(self):
        super().__init__("mujoco_node")
        self.mujoco_publisher = self.create_publisher(joint_state,"robot_state",10)

    def 

def main(args=None):
    rclpy.init(args=args)
    node = mujoco_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()