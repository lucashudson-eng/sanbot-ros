#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Controles:
  w = frente
  s = trás
  a = esquerda
  d = direita
  q = girar esquerda
  e = girar direita
  espaço = parar

Ajustes de velocidade:
  + = aumentar linear e angular em 10%
  - = diminuir linear e angular em 10%
  [ = aumentar só linear
  ] = diminuir só linear
  { = aumentar só angular
  } = diminuir só angular

CTRL+C = sair
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
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    print(msg)
    last_key = ''

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == '\x03':  # Ctrl+C
                break

            twist = Twist()

            # Ajuste de velocidade (sempre executa, sem filtro por last_key)
            if key == '+':
                linear_speed = min(max_linear, linear_speed * 1.1)
                angular_speed = min(max_angular, angular_speed * 1.1)
                print(f">> Velocidade linear: {linear_speed:.2f}, angular: {angular_speed:.2f}")
                continue
            elif key == '-':
                linear_speed = max(min_speed, linear_speed * 0.9)
                angular_speed = max(min_speed, angular_speed * 0.9)
                print(f">> Velocidade linear: {linear_speed:.2f}, angular: {angular_speed:.2f}")
                continue
            elif key == '[':
                linear_speed = min(max_linear, linear_speed * 1.1)
                print(f">> Velocidade linear: {linear_speed:.2f}")
                continue
            elif key == ']':
                linear_speed = max(min_speed, linear_speed * 0.9)
                print(f">> Velocidade linear: {linear_speed:.2f}")
                continue
            elif key == '{':
                angular_speed = min(max_angular, angular_speed * 1.1)
                print(f">> Velocidade angular: {angular_speed:.2f}")
                continue
            elif key == '}':
                angular_speed = max(min_speed, angular_speed * 0.9)
                print(f">> Velocidade angular: {angular_speed:.2f}")
                continue

            # Movimento — agora sim filtra por tecla repetida
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
                continue  # tecla não reconhecida

            pub.publish(twist)
            last_key = key  # só atualiza se foi movimento

    except Exception as e:
        print(e)

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
