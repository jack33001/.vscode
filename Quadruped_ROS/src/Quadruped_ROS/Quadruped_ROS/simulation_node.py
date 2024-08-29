#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node houses the MuJoCo simulation environment. It's interchangeable with the 'Jetson' node. 
#
# PUBLISHES TO (topic,msg):
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class simulation_node:
    def __init__(self):
        super().__init__("simulation_node")

def main(args=None):
    rclpy.init(args=args)
    node = simulation_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()