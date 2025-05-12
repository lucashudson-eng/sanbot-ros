#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String

MQTT_BROKER_IP = "localhost"
MQTT_PORT = 1883

topics = {
    "sanbot/touch": "touch",
    "sanbot/pir": "pir",
    "sanbot/ir": "ir",
    "sanbot/voice_angle": "voice_angle",
    "sanbot/obstacle": "obstacle",
    "sanbot/battery": "battery",
    "sanbot/info": "info",
}

ros_publishers = {}

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Conectado ao broker MQTT")
    for topic in topics:
        client.subscribe(topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        ros_topic = topics.get(msg.topic)
        if ros_topic:
            ros_publishers[ros_topic].publish(payload)
    except Exception as e:
        rospy.logerr(f"Erro ao processar mensagem: {e}")

if __name__ == "__main__":
    rospy.init_node("mqtt_to_ros")

    # Inicializa publishers ROS
    for _, ros_topic in topics.items():
        ros_publishers[ros_topic] = rospy.Publisher(ros_topic, String, queue_size=10)

    # Conecta MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.spin()
