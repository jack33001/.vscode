#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node represents the jetson - it handles anything to do with interfacing with jetson hardware (CAN, SPI, video input, etc)
#
# PUBLISHES TO (topic,msg): 
# 
# SUBSCRIBES TO (topic): 
# 
# INVOLVED WITH SERVICES (srv): 
#   
class jetson_node:
    def __init__(self):
        super().__init__("jetson_node")

def main(args=None):
    rclpy.init(args=args)
    node = jetson_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()