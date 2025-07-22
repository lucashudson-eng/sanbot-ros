#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

# Inicialização do nó
rospy.init_node("omni_inverse_kinematics")

# Parâmetros da base (podem ser ajustados em tempo de execução)
r = rospy.get_param("~wheel_radius", 0.06)   # [m]
L = rospy.get_param("~wheel_distance", 0.16) # [m]

# Publishers para as rodas
pub_back  = rospy.Publisher("/wheel_back_velocity_controller/command", Float64, queue_size=10)
pub_right = rospy.Publisher("/wheel_right_velocity_controller/command", Float64, queue_size=10)
pub_left  = rospy.Publisher("/wheel_left_velocity_controller/command", Float64, queue_size=10)

# Callback do cmd_vel
def cmd_vel_callback(msg):
    vx = msg.linear.x
    vy = msg.linear.y
    w  = msg.angular.z

    # Velocidade linear de cada roda (m/s) usando modelo genérico
    thetas = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)  # orientação dos eixos das rodas
    v_list = [(-math.sin(t) * vx + math.cos(t) * vy + L * w) for t in thetas]
    v1, v2, v3 = v_list

    # Velocidade angular de cada roda (rad/s)
    w_back, w_left, w_right = [v / r for v in (v1, v2, v3)]

    # Publicação
    pub_back.publish(Float64(-w_back))
    pub_left.publish(Float64(-w_left))
    pub_right.publish(Float64(-w_right))

    rospy.loginfo(f"cmd_vel → back: {w_back:.2f}, left: {w_left:.2f}, right: {w_right:.2f}")

# Subscriber
rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

# Loop
rospy.spin()