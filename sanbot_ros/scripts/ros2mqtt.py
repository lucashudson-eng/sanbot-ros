#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

# Configurações do MQTT
MQTT_BROKER_IP = "localhost"
MQTT_PORT = 1883

# Tópicos MQTT que o app Android escuta
TOPIC_MOVE = "ros/move"
TOPIC_HEAD = "ros/head"
TOPIC_LIGHT = "ros/light"
TOPIC_LED = "ros/led"
TOPIC_SPEAK = "ros/speak"

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
    data_str = msg.data.strip()
    try:
        parts = data_str.split()
        direction = parts[0]
        speed = int(parts[1]) if len(parts) > 1 else 5
        distance = int(parts[2]) if len(parts) > 2 else None
        duration = int(parts[3]) if len(parts) > 3 else None

        data = {"direction": direction, "speed": speed}
        if distance is not None:
            data["distance"] = distance
        if duration is not None:
            data["duration"] = duration

        mqtt_client.publish(TOPIC_MOVE, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/move: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro ao interpretar comando ros/move: {e}")

def callback_head(msg):
    data_str = msg.data.strip()
    try:
        parts = data_str.split()
        direction = parts[0]
        angle = int(parts[1]) if len(parts) > 1 else 10
        speed = int(parts[2]) if len(parts) > 2 else 50
        motor = int(parts[3]) if len(parts) > 3 else 1

        data = {
            "direction": direction,
            "angle": angle,
            "speed": speed,
            "motor": motor
        }

        mqtt_client.publish(TOPIC_HEAD, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/head: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro ao interpretar comando ros/head: {e}")

def callback_led(msg):
    data_str = msg.data.strip()
    try:
        parts = data_str.split()
        part = parts[0]
        mode = parts[1]
        duration = int(parts[2]) if len(parts) > 2 else 1
        random = int(parts[3]) if len(parts) > 3 else 1

        data = {
            "part": part,
            "mode": mode,
            "duration": duration,
            "random": random
        }

        mqtt_client.publish(TOPIC_LED, json.dumps(data))
        rospy.loginfo(f"[MQTT] ros/led: {data}")
    except Exception as e:
        rospy.logwarn(f"Erro ao interpretar comando ros/led: {e}")

def callback_speak(msg):
    data_str = msg.data.strip()
    data = { "msg": data_str }

    mqtt_client.publish(TOPIC_SPEAK, json.dumps(data))
    rospy.loginfo(f"[MQTT] ros/speak: {data}")

def callback_light(msg):
    # Converter UInt8 para o formato esperado pelo MQTT
    payload = {"white": msg.data}
    mqtt_client.publish(TOPIC_LIGHT, json.dumps(payload))
    rospy.loginfo(f"[MQTT] ros/light: {payload}")

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
    rospy.Subscriber("/ros/cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("/ros/move", String, callback_move)
    rospy.Subscriber("/ros/head", String, callback_head)
    rospy.Subscriber("/ros/light", UInt8, callback_light)
    rospy.Subscriber("/ros/led", String, callback_led)
    rospy.Subscriber("/ros/speak", String, callback_speak)

    rospy.loginfo("🔁 Enviando comandos do ROS para o app Android via MQTT...")
    rospy.spin()
