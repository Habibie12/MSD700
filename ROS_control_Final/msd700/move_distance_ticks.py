#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class MoveDistanceTicks(Node):

    def __init__(self):

        super().__init__('move_distance_ticks')

        # ===== PARAMETERS =====

        self.target_distance_m = 1.0
        self.speed = 0.2

        self.ticks_per_meter = 2690.0

        self.target_ticks = (
            self.target_distance_m *
            self.ticks_per_meter
        )

        # ===== STATE =====

        self.start_ticks = None
        self.current_ticks = None

        self.motion_started = False
        self.finished = False

        # ===== ROS =====

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.encoder_sub = self.create_subscription(
            Float64MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            f'Target distance = {self.target_distance_m:.2f} m'
        )

        self.get_logger().info(
            f'Target ticks = {self.target_ticks:.0f}'
        )

        self.get_logger().info(
            'Waiting for encoder...'
        )

    # def encoder_callback(self, msg):

    #     if len(msg.data) < 3:
    #         return

    #     avg_ticks_abs = float(msg.data[2])

    #     self.current_ticks = avg_ticks_abs

    #     # Capture start position once
    #     if self.start_ticks is None:

    #         self.start_ticks = avg_ticks_abs

    #         self.get_logger().info(
    #             f'START TICKS = {self.start_ticks:.1f}'
    #         )
    def encoder_callback(self, msg):

        avg_ticks_abs = float(msg.data[2])

        self.current_ticks = avg_ticks_abs

        self.get_logger().info(
            f"ENCODER UPDATE = {avg_ticks_abs}"
        )

        # if self.start_ticks is None:
        #     self.start_ticks = avg_ticks_abs
        #     self.get_logger().info(
        #         f"START TICKS = {self.start_ticks}"
        #     )
        if self.start_ticks is None:

            self.start_ticks = avg_ticks_abs

            self.get_logger().info(
                f"START TICKS CAPTURED = {self.start_ticks}"
            )

            return

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        # send multiple stop frames
        for _ in range(10):
            self.cmd_pub.publish(cmd)

    def control_loop(self):

        if self.finished:
            return

        # if self.start_ticks is None:
        #     return

        # if self.current_ticks is None:
        #     return

        if self.current_ticks is None:

            cmd = Twist()
            cmd.linear.x = self.speed
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                'Searching first encoder update...'
            )

            return

        travelled_ticks = abs(
            self.current_ticks -
            self.start_ticks
        )

        travelled_distance = (
            travelled_ticks /
            self.ticks_per_meter
        )

        self.get_logger().info(
            f'START={self.start_ticks:.1f} '
            f'CURRENT={self.current_ticks:.1f} '
            f'TICKS={travelled_ticks:.1f} '
            f'DIST={travelled_distance:.3f} m'
        )
        

        # ===== TARGET REACHED =====

        if travelled_ticks >= self.target_ticks:

            self.get_logger().info(
                'TARGET REACHED'
            )

            self.stop_robot()

            self.finished = True

            rclpy.shutdown()

            return

        # ===== MOVE =====

        cmd = Twist()

        cmd.linear.x = self.speed
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        if not self.motion_started:

            self.motion_started = True

            self.get_logger().info(
                f'Moving at {self.speed:.2f} m/s'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MoveDistanceTicks()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()