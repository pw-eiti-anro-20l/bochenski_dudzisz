#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
    return key

def turtle_control():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('turtle_control', anonymous=True)
    control_keys = rospy.get_param("/control/")

    print("\nReading from keyboard")  
    print("---------------------------")
    print("Use WSAD keys to move the turtle.\n")

    while not rospy.is_shutdown():
        twist = Twist()
        key = getKey()

        if key == control_keys["forward"]:
            twist.linear.x = 2
        elif key == control_keys["backward"]:
            twist.linear.x = -2
        elif key == control_keys["left"]:
            twist.angular.z = 2
        elif key == control_keys["right"]:
            twist.angular.z = -2

        pub.publish(twist)

        if (key == '\x03'):
            break

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        turtle_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
