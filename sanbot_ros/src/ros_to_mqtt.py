#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

MQTT_BROKER_IP = "localhost"  # IP do robô com o broker MQTT
MQTT_PORT = 1883
TOPIC_CMD_VEL = "sanbot/cmd_vel"

def callback_cmd_vel(msg):
    payload = {
        "linear_x": msg.linear.x,
        "linear_y": msg.linear.y,
        "linear_z": msg.linear.z,
        "angular_x": msg.angular.x,
        "angular_y": msg.angular.y,
        "angular_z": msg.angular.z
    }
    mqtt_client.publish(TOPIC_CMD_VEL, json.dumps(payload))

def on_connect(client, userdata, flags, rc):
    rospy.loginfo(f"Conectado ao broker MQTT com código: {rc}")

if __name__ == "__main__":
    rospy.init_node("ros_to_mqtt")

    # Inicializa cliente MQTT
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    mqtt_client.loop_start()

    # Assina tópico ROS
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)
    rospy.loginfo("Aguardando mensagens /cmd_vel para publicar via MQTT...")
    rospy.spin()
