#!/usr/bin/env python3
"""Fly a hold and sinusoidal velocity oscillations in Offboard mode over the DDS agent (bench tool).

usage: fly_oscillation.py [--amplitude 2.0] [--altitude 2.5] [--yaw-deg 90]

Takes off to --altitude (m, above the start), holds 15 s, then oscillates east-west (pitch) at 0.3 and 0.7 Hz
for 12 s each, holds 6 s, oscillates north-south (roll) at 0.3 and 0.7 Hz for 12 s each, holds 6 s and lands.
The two frequencies give small and large attitude rates; the holds give the static attitude error. The
heading is held at --yaw-deg (NED, from north; 90 is east, the Gazebo +x). The vehicle stays within about
amplitude / (2 pi f) of the start, 1.2 m at the defaults. It arms with the force-arm parameter: the
simulation has no GCS link, so set NAV_DLL_ACT 0 and COM_RC_IN_MODE 4 first and reset them afterwards.
The run ends with one line listing the phase start times in seconds. Bench values only.
"""
import argparse
import math
import time

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

NAN = float('nan')
SEQUENCE = [('hold0', 15), ('ew_slow', 12), ('ew_fast', 12), ('hold1', 6), ('ns_slow', 12),
            ('ns_fast', 12), ('hold2', 6)]
FREQUENCY_HZ = {'ew_slow': 0.3, 'ew_fast': 0.7, 'ns_slow': 0.3, 'ns_fast': 0.7}
CMD_ARM, CMD_MODE, CMD_LAND = 400, 176, 21
FORCE_ARM = 21196


class Flight(Node):
    def __init__(self, amplitude, altitude_m, yaw_rad):
        super().__init__('fly_oscillation')
        self.amplitude, self.down_m, self.yaw_rad = amplitude, -altitude_m, yaw_rad
        self.offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 lambda m: setattr(self, 'odo', m), qos_profile_sensor_data)
        self.odo, self.phase, self.t0, self.t_phase = None, 'wait', time.time(), 0.0
        self.arm_sent, self.index, self.log = 0, 0, []
        self.create_timer(0.05, self.tick)

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def send(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, float(p1), float(p2)
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external, m.timestamp = True, self.now_us()
        self.command.publish(m)

    def tick(self):
        if self.odo is None:
            return
        t, p = time.time() - self.t0, self.odo.position
        o = OffboardControlMode()
        sp = TrajectorySetpoint()
        sp.position, sp.velocity, sp.acceleration = [NAN] * 3, [NAN] * 3, [NAN] * 3
        sp.yaw, sp.yawspeed = self.yaw_rad, NAN

        def hold():
            sp.position = [self.n0, self.e0, self.down_m]
            o.position = True

        if self.phase == 'wait':
            self.n0, self.e0 = p[0], p[1]
            hold()
            if t > 3:
                self.phase, self.t_phase = 'climb', t
        elif self.phase == 'climb':
            hold()
            if self.arm_sent < 5:
                if self.arm_sent == 0:
                    self.send(CMD_MODE, 1, 6)
                self.send(CMD_ARM, 1, FORCE_ARM)
                self.arm_sent += 1
            if abs(p[2] - self.down_m) < 0.2 and t - self.t_phase > 4:
                self.phase, self.t_phase = SEQUENCE[0][0], t
                self.log.append((round(t, 1), self.phase))
        elif self.phase == 'land':
            self.send(CMD_LAND)
            self.phase = 'done'
        elif self.phase != 'done':
            name, duration = SEQUENCE[self.index]
            if name.startswith('hold'):
                hold()
            else:
                v = self.amplitude * math.sin(2 * math.pi * FREQUENCY_HZ[name] * (t - self.t_phase))
                o.velocity, o.position = True, False
                sp.position = [NAN, NAN, self.down_m]
                sp.velocity = [0.0, v, NAN] if name.startswith('ew') else [v, 0.0, NAN]
            if t - self.t_phase > duration:
                self.index += 1
                self.phase = 'land' if self.index >= len(SEQUENCE) else SEQUENCE[self.index][0]
                self.t_phase = t
                self.log.append((round(t, 1), self.phase))
        o.timestamp = sp.timestamp = self.now_us()
        self.offboard.publish(o)
        self.setpoint.publish(sp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--amplitude', type=float, default=2.0)
    ap.add_argument('--altitude', type=float, default=2.5)
    ap.add_argument('--yaw-deg', type=float, default=90.0)
    args = ap.parse_args()
    rclpy.init()
    flight = Flight(args.amplitude, args.altitude, math.radians(args.yaw_deg))
    while rclpy.ok() and flight.phase != 'done' and time.time() - flight.t0 < 200:
        rclpy.spin_once(flight, timeout_sec=0.1)
    print('phases', flight.log, 'final', flight.phase)


if __name__ == '__main__':
    main()
