#!/usr/bin/env python3
"""Republish each Gazebo cloud one frame (100 ms) late (manual-flight bench shim).

A real Livox frame is complete only about 100 ms after its earliest ray; the simulated one arrives at once. The
cloud content and stamp are unchanged. Subscribes to points_raw, publishes points.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Relay(Node):
    def __init__(self):
        super().__init__('relay_one_frame')
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=4)
        self.pub = self.create_publisher(PointCloud2, 'points', q)
        self.held = None
        self.create_subscription(PointCloud2, 'points_raw', self.on_cloud, q)

    def on_cloud(self, msg):
        if self.held is not None:
            self.pub.publish(self.held)
        self.held = msg


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == '__main__':
    main()
