#!/usr/bin/env python3
"""Small terminal teleop supporting WASD and arrow keys."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist


HELP = """
Dieu khien TurtleBot 4 (giu phim hoac bam nhieu lan):

       W / mui ten len     : tien
 A / mui ten trai          : quay trai
 D / mui ten phai          : quay phai
       S / mui ten xuong   : lui

 Space hoặc X : dung ngay
 + / -        : tang / giam toc do
 Q            : thoat
"""


def read_key(timeout=0.1):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return None

    key = sys.stdin.read(1)
    if key == '\x1b':
        # Arrow keys arrive as ESC, [, A/B/C/D.
        if select.select([sys.stdin], [], [], 0.02)[0]:
            key += sys.stdin.read(1)
        if select.select([sys.stdin], [], [], 0.02)[0]:
            key += sys.stdin.read(1)
    return key


def publish_velocity(publisher, linear, angular):
    message = Twist()
    message.linear.x = float(linear)
    message.angular.z = float(angular)
    publisher.publish(message)


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Teleop phai duoc chay trong terminal tuong tac.')

    rclpy.init()
    node = rclpy.create_node('assign4_wasd_teleop')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)
    linear_speed = 0.25
    angular_speed = 0.8

    print(HELP)
    print(f'Toc do: {linear_speed:.2f} m/s, {angular_speed:.2f} rad/s')

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key is None:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue

            lower_key = key.lower()
            if lower_key in ('q', '\x03'):
                break
            if lower_key in ('w', '\x1b[a'):
                publish_velocity(publisher, linear_speed, 0.0)
            elif lower_key in ('s', '\x1b[b'):
                publish_velocity(publisher, -linear_speed, 0.0)
            elif lower_key in ('a', '\x1b[d'):
                publish_velocity(publisher, 0.0, angular_speed)
            elif lower_key in ('d', '\x1b[c'):
                publish_velocity(publisher, 0.0, -angular_speed)
            elif lower_key in (' ', 'x'):
                publish_velocity(publisher, 0.0, 0.0)
            elif key in ('+', '='):
                linear_speed = min(linear_speed + 0.10, 1.0)
                angular_speed = min(angular_speed + 0.20, 3.0)
                print(
                    f'\r\nToc do: {linear_speed:.2f} m/s, '
                    f'{angular_speed:.2f} rad/s\r'
                )
            elif key in ('-', '_'):
                linear_speed = max(linear_speed - 0.10, 0.05)
                angular_speed = max(angular_speed - 0.20, 0.2)
                print(
                    f'\r\nToc do: {linear_speed:.2f} m/s, '
                    f'{angular_speed:.2f} rad/s\r'
                )
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        publish_velocity(publisher, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nDa dung robot.')


if __name__ == '__main__':
    main()
