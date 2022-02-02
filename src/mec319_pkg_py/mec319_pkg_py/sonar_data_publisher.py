#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from brping import Ping1D

from mec319_interfaces.msg import Distance

class SonarPublisherNode(Node): 
    def __init__(self):
        super().__init__("sonar_data_publisher")
        self.pinger_ = Ping1D()
        self.pinger_.connect_serial("/dev/ttyUSB0", 115200)
        self.get_logger().info("Sonar Publisher has been started.")
        self.distance_publisher_ = self.create_publisher(Distance, "distance_data", 10)
        self.timer_ = self.create_timer(0.0001, self.publish_sonar_data)

    def publish_sonar_data(self):
        self.data_ = self.pinger_.get_distance()
        msg = Distance()
        msg.distance = self.data_["distance"]
        self.get_logger().info("Distance is " + str(msg.distance) + " cm.")
        self.distance_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()