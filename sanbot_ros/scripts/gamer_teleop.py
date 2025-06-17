#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Controls:
  w = forward
  s = backward
  a = left
  d = right
  q = turn left
  e = turn right
  space = stop

Speed adjustments:
  + = increase linear and angular by 10%
  - = decrease linear and angular by 10%
  [ = increase linear only
  ] = decrease linear only
  { = increase angular only
  } = decrease angular only

CTRL+C = exit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

linear_speed = 0.5
angular_speed = 1.0
max_linear = 2.0
max_angular = 3.0
min_speed = 0.05

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('gamer_teleop')
    pub = rospy.Publisher('/ros/cmd_vel', Twist, queue_size=10)

    print(msg)
    last_key = ''

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == '\x03':  # Ctrl+C
                break

            twist = Twist()

            # Speed adjustment (always executes, no filter by last_key)
            if key == '+':
                linear_speed = min(max_linear, linear_speed * 1.1)
                angular_speed = min(max_angular, angular_speed * 1.1)
                print(f">> Linear speed: {linear_speed:.2f}, angular: {angular_speed:.2f}")
                continue
            elif key == '-':
                linear_speed = max(min_speed, linear_speed * 0.9)
                angular_speed = max(min_speed, angular_speed * 0.9)
                print(f">> Linear speed: {linear_speed:.2f}, angular: {angular_speed:.2f}")
                continue
            elif key == '[':
                linear_speed = min(max_linear, linear_speed * 1.1)
                print(f">> Linear speed: {linear_speed:.2f}")
                continue
            elif key == ']':
                linear_speed = max(min_speed, linear_speed * 0.9)
                print(f">> Linear speed: {linear_speed:.2f}")
                continue
            elif key == '{':
                angular_speed = min(max_angular, angular_speed * 1.1)
                print(f">> Angular speed: {angular_speed:.2f}")
                continue
            elif key == '}':
                angular_speed = max(min_speed, angular_speed * 0.9)
                print(f">> Angular speed: {angular_speed:.2f}")
                continue

            # Movement — now filters by repeated key
            if key == last_key or key == '':
                continue

            if key == 'w':
                twist.linear.x = linear_speed
            elif key == 's':
                twist.linear.x = -linear_speed
            elif key == 'a':
                twist.linear.y = linear_speed
            elif key == 'd':
                twist.linear.y = -linear_speed
            elif key == 'q':
                twist.angular.z = angular_speed
            elif key == 'e':
                twist.angular.z = -angular_speed
            elif key == ' ':
                twist = Twist()
            else:
                continue  # unrecognized key

            pub.publish(twist)
            last_key = key  # only updates if it was movement

    except Exception as e:
        print(e)

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
