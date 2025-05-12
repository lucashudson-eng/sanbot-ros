#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Configurações do MQTT
MQTT_BROKER_IP = "localhost"
MQTT_PORT = 1883

# Tópicos MQTT que o app Android escuta
TOPIC_MOVE = "ros/move"
TOPIC_HEAD = "ros/head"
TOPIC_LIGHT = "ros/light"
TOPIC_LED = "ros/led"
TOPIC_MP3 = "ros/mp3"

def callback_cmd_vel(msg):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z

    direction = "stop"

    if z != 0:
        direction = "turn_left" if z > 0 else "turn_right"
    elif x > 0 and y == 0:
        direction = "forward"
    elif x < 0 and y == 0:
        direction = "backward"
    elif y > 0 and x == 0:
        direction = "left"
    elif y < 0 and x == 0:
        direction = "right"
    elif x > 0 and y > 0:
        direction = "left_forward"
    elif x < 0 and y > 0:
        direction = "left_back"
    elif x > 0 and y < 0:
        direction = "right_forward"
    elif x < 0 and y < 0:
        direction = "right_back"

    speed = min(10, int(max(abs(x), abs(y), abs(z)) * 10))

    payload = {
        "direction": direction,
        "speed": speed,
    }

    mqtt_client.publish(TOPIC_MOVE, json.dumps(payload))
    rospy.loginfo(f"[MQTT] ros/move (from /cmd_vel): {payload}")

def callback_move(msg):
    try:
        data = json.loads(msg.data)
        mqtt_client.publish(TOPIC_MOVE, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/move (custom): {data}")
    except Exception as e:
        rospy.logwarn(f"Erro no JSON de /move: {e}")

def callback_head(msg):
    try:
        data = json.loads(msg.data)
        mqtt_client.publish(TOPIC_HEAD, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/head: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro no JSON de ros/head: {e}")

def callback_light(msg):
    try:
        data = json.loads(msg.data)
        mqtt_client.publish(TOPIC_LIGHT, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/light: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro no JSON de ros/light: {e}")

def callback_led(msg):
    try:
        data = json.loads(msg.data)
        mqtt_client.publish(TOPIC_LED, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/led: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro no JSON de ros/led: {e}")

def callback_mp3(msg):
    mqtt_client.publish(TOPIC_MP3, "")
    rospy.loginfo("[MQTT] ros/mp3: Trigger")

def on_connect(client, userdata, flags, rc):
    rospy.loginfo(f"Conectado ao broker MQTT (rc={rc})")

if __name__ == "__main__":
    rospy.init_node("ros_to_mqtt")

    # Cliente MQTT
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    mqtt_client.loop_start()

    # Assinaturas ROS
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("/move", String, callback_move)
    rospy.Subscriber("/head", String, callback_head)
    rospy.Subscriber("/light", String, callback_light)
    rospy.Subscriber("/led", String, callback_led)
    rospy.Subscriber("/mp3", String, callback_mp3)

    rospy.loginfo("🔁 Enviando comandos do ROS para o app Android via MQTT...")
    rospy.spin()
