#!/usr/bin/env python3
"""
Drives the pedestrian model back and forth along the Y axis in Gazebo,
alternating between a fixed move duration and a short pause.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MOVE_SPEED = 0.8
MOVE_TIME = 10.0
PAUSE_TIME = 0.5


class PedestrianMover(Node):
    def __init__(self):
        super().__init__('pedestrian_mover')
        self.pub = self.create_publisher(Twist, '/pedestrian/cmd_vel', 10)
        self.direction = -1.0
        self.state = 'MOVING'
        self.state_start = time.time()
        self.create_timer(0.1, self.move_loop)
        self.get_logger().info('Pedestrian mover started')

    def move_loop(self):
        elapsed = time.time() - self.state_start
        msg = Twist()
        if self.state == 'MOVING':
            msg.linear.y = self.direction * MOVE_SPEED
            if elapsed >= MOVE_TIME:
                self.state = 'PAUSE'
                self.state_start = time.time()
        elif self.state == 'PAUSE':
            msg.linear.y = 0.0
            if elapsed >= PAUSE_TIME:
                self.direction *= -1.0
                self.state = 'MOVING'
                self.state_start = time.time()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PedestrianMover())


if __name__ == '__main__':
    main()
