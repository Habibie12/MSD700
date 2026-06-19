#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveDistance(Node):

    def __init__(self):
        super().__init__('move_distance')

        self.target_distance = 3.0      # meter
        self.speed = 0.2                # m/s

        self.start_x = None
        self.start_y = None
        self.finished = False

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info(
            'Waiting for odometry...'
        )

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

            self.get_logger().info(
                f'Start position = ({self.start_x:.3f}, {self.start_y:.3f})'
            )

    def control_loop(self):

        if self.finished:
            return

        if self.start_x is None:
            return

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y

        distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f'Distance = {distance:.3f} m'
        )

        cmd = Twist()

        if distance < self.target_distance:

            cmd.linear.x = self.speed
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

        else:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.finished = True

            self.get_logger().info(
                f'Target reached ({distance:.3f} m)'
            )

            self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MoveDistance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()