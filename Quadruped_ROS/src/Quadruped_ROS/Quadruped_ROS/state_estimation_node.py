#! usr/bin/env python3

# import the necessary packages
import rclpy

# this node handles state estimation. This primarily means inferring foot contact state, but in the future may mean more. 
#
# PUBLISHES TO (topic,msg): (robot_state,contacts)
# 
# SUBSCRIBES TO (topic): (robot_state)
# 
# INVOLVED WITH SERVICES (srv):
#   
class state_estimation_node:
    def __init__(self):
        super().__init__("state_estimation_node")
        self.state_estimation_subscriber = self.create_subscription(joint_state,self.detect_contact,"robot_state",10)
        self.state_estimation_publisher = self.create_publisher(contacts,"robot_state",10)

    # determine the contact state of all the legs
    def detect_contact(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = state_estimation_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()