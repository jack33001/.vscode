#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node logs everything of any interest. 
#
# PUBLISHES TO (topic,msg): 
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class logger_node:
    def __init__(self):
        super().__init__("logger_node")

def main(args=None):
    rclpy.init(args=args)
    node = logger_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()