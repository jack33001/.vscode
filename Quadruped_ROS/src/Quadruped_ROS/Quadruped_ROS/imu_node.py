#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node represents the imu - it handles initialization messages and any sort of memessage decoding
#
# PUBLISHES TO (topic,msg): (imu_reading,imu_reading)
# 
# SUBSCRIBES TO (topic):
# 
# INVOLVED WITH SERVICES (srv):
#   
class imu_node:
    def __init__(self):
        super().__init__("imu_node")
        self.imu_publisher = self.create_publisher(imu_reading,"imu_reading_noisy",10)

def main(args=None):
    rclpy.init(args=args)
    node = imu_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()