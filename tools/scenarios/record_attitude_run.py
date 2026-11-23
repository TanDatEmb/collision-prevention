#!/usr/bin/env python3
"""Record one run for analyze_attitude_time.py: Gazebo ground truth, FMU odometry, point clouds (bench tool).

usage: record_attitude_run.py <seconds> <out.npz> [--model x500_lidar_multi_ring_0] [--cloud-topic points]

Records at the same time, all in the simulation clock:
  gt     (N, 8)  Gazebo model pose from /world/default/dynamic_pose/info: stamp_ns, x, y, z, qw, qx, qy, qz
                 (world ENU, body FLU, quaternion body -> world)
  odo    (M, 9)  every /fmu/out/vehicle_odometry sample: timestamp_sample * 1000 [ns], /clock at arrival [ns],
                 x, y, z, qw, qx, qy, qz (NED, body FRD, quaternion body -> world). The sample time is read as
                 the node reads it (ADR 0003: one clock in the simulation).
  meta   (K, 2)  header stamp [ns] and /clock at arrival [ns] of each recorded cloud
  clouds (K, R, 3) float32 xyz in the sensor frame, one row per ray (non-finite for no return); every second
                 cloud is kept to bound the file size.
Needs ROS 2 with the px4_msgs overlay sourced and the gz python bindings (gz.transport13, gz.msgs10).
"""
import argparse
import time

import numpy as np
import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', type=float)
    ap.add_argument('out')
    ap.add_argument('--model', default='x500_lidar_multi_ring_0')
    ap.add_argument('--cloud-topic', default='points')
    args = ap.parse_args()

    gt, odo, clouds, meta = [], [], [], []

    def on_pose(msg):
        for pose in msg.pose:
            if pose.name == args.model:
                o = pose.orientation
                gt.append((msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec, pose.position.x,
                           pose.position.y, pose.position.z, o.w, o.x, o.y, o.z))
                break

    gz = GzNode()
    gz.subscribe(Pose_V, '/world/default/dynamic_pose/info', on_pose)

    class Recorder(Node):
        def __init__(self):
            super().__init__('record_attitude_run')
            self.clock_ns = 0
            self.clouds_seen = 0
            self.create_subscription(Clock, '/clock', self.on_clock, 10)
            self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odo,
                                     qos_profile_sensor_data)
            reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST, depth=4)
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, reliable)

        def on_clock(self, msg):
            self.clock_ns = msg.clock.sec * 10**9 + msg.clock.nanosec

        def on_odo(self, msg):
            odo.append((msg.timestamp_sample * 1000, self.clock_ns, *msg.position, *msg.q))

        def on_cloud(self, msg):
            self.clouds_seen += 1
            if self.clouds_seen % 2:
                return
            a = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=False)
            clouds.append(np.stack([a['x'], a['y'], a['z']], axis=-1).reshape(-1, 3).astype(np.float32))
            meta.append((msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, self.clock_ns))

    rclpy.init()
    recorder = Recorder()
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(recorder, timeout_sec=0.05)
    np.savez_compressed(args.out, gt=np.array(gt), odo=np.array(odo), meta=np.array(meta),
                        clouds=np.array(clouds, dtype=np.float32))
    print('gt %d, odo %d, clouds %d' % (len(gt), len(odo), len(clouds)))


if __name__ == '__main__':
    main()
