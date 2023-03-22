#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGpsPosition
import random

class GPSSensorNode(Node):
    def __init__(self):
        super().__init__("rid_queue_test")
        self.gps_publisher_ = self.create_publisher(
            VehicleGpsPosition, "fmu/vehicle_gps_position/out", 10)
        self.gps_timer_ = self.create_timer(
            0.1, self.publish_gps)
        self.i = 0
    def publish_gps(self):
        msg = VehicleGpsPosition()
        msg.lat = self.i
        self.gps_publisher_.publish(msg)
        self.i = self.i + 1

def main(args=None):
    rclpy.init(args=args)
    node = GPSSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
