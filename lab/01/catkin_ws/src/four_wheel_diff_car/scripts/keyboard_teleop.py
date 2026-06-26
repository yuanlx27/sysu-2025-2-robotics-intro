#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
Keyboard control for four_wheel_diff_car
---------------------------------------
w / s : forward / backward
a / d : turn left / turn right
x     : stop
q / e : increase / decrease linear speed
z / c : increase / decrease angular speed
Ctrl-C: quit
"""


def read_key(timeout):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return ""


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    rospy.init_node("keyboard_teleop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    linear_speed = rospy.get_param("~linear_speed", 0.25)
    angular_speed = rospy.get_param("~angular_speed", 0.9)
    max_linear_speed = rospy.get_param("~max_linear_speed", 1.0)
    max_angular_speed = rospy.get_param("~max_angular_speed", 2.5)

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    print("linear: %.2f m/s, angular: %.2f rad/s" % (linear_speed, angular_speed))

    try:
        tty.setraw(sys.stdin.fileno())
        rate = rospy.Rate(20)
        current = Twist()

        while not rospy.is_shutdown():
            key = read_key(0.05)

            if key == "\x03":
                break
            if key == "w":
                current.linear.x = linear_speed
                current.angular.z = 0.0
            elif key == "s":
                current.linear.x = -linear_speed
                current.angular.z = 0.0
            elif key == "a":
                current.linear.x = 0.0
                current.angular.z = angular_speed
            elif key == "d":
                current.linear.x = 0.0
                current.angular.z = -angular_speed
            elif key == "x" or key == " ":
                current = Twist()
            elif key == "q":
                linear_speed = clamp(linear_speed + 0.05, 0.05, max_linear_speed)
                print("\rlinear speed: %.2f m/s" % linear_speed)
            elif key == "e":
                linear_speed = clamp(linear_speed - 0.05, 0.05, max_linear_speed)
                print("\rlinear speed: %.2f m/s" % linear_speed)
            elif key == "z":
                angular_speed = clamp(angular_speed + 0.1, 0.1, max_angular_speed)
                print("\rangular speed: %.2f rad/s" % angular_speed)
            elif key == "c":
                angular_speed = clamp(angular_speed - 0.1, 0.1, max_angular_speed)
                print("\rangular speed: %.2f rad/s" % angular_speed)

            pub.publish(current)
            rate.sleep()
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
