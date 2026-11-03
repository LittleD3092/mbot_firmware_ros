#!/usr/bin/env python3
"""Drive open-loop squares to compare /odom and /gyrodom (Task 1.3).

Mark the robot's start position and heading on the floor, reset odometry, then:
    python3 gyrodom_test.py            # one 1 m square
    python3 gyrodom_test.py --laps 4
The motion is timed, so the robot will not trace a perfect square. That is fine:
compare each estimate's final pose with where the robot actually stopped.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

SIDE_M = 1.0
LINEAR_SPEED = 0.25  # m/s
TURN_SPEED = math.pi / 4  # rad/s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--laps", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = Node("gyrodom_test")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def move(linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        end = time.time() + duration
        while time.time() < end:
            pub.publish(msg)
            time.sleep(0.05)

    try:
        move(0.0, 0.0, 3.0)  # Let the /cmd_vel connection settle.
        for _ in range(4 * args.laps):
            move(LINEAR_SPEED, 0.0, SIDE_M / LINEAR_SPEED)
            move(0.0, 0.0, 0.5)
            move(0.0, TURN_SPEED, (math.pi / 2) / TURN_SPEED)
            move(0.0, 0.0, 0.5)
    finally:
        move(0.0, 0.0, 0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
