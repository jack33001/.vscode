#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node represents the motors - it handles initialization messages, any sort of message decoding and encoder filtering
#
# PUBLISHES TO (topic,msg): (robot_state,joint_state)
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class motors_node:
    def __init__(self):
        super().__init__("motors_node")
        self.motors_publisher = self.create_publisher(joint_state,"robot_state",10)

def main(args=None):
    rclpy.init(args=args)
    node = motors_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()