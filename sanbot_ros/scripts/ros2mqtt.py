#!/usr/bin/env python3
import rospy
import json
import math
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
from sanbot_ros.msg import Move, Led
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String, UInt8

# MQTT Configuration (overridable via ROS parameters "mqtt_broker_ip" e "mqtt_port")
DEFAULT_MQTT_BROKER_IP = "localhost"
DEFAULT_MQTT_PORT = 1883

# MQTT topics that the Android app listens to
TOPIC_MOVE = "ros/move"
TOPIC_LIGHT = "ros/light"
TOPIC_LED = "ros/led"
TOPIC_SPEAK = "ros/speak"
TOPIC_JOINTS = "ros/joints"

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
    data = {
        "direction": msg.direction
    }
    if msg.speed != 0:
        data["speed"] = msg.speed
    if msg.distance != 0:
        data["distance"] = msg.distance
    if msg.duration != 0:
        data["duration"] = msg.duration

    mqtt_client.publish(TOPIC_MOVE, json.dumps(data))
    rospy.loginfo(f"[MQTT] ros/move: {data}")

def callback_joints(msg):
    # Extract joint names and positions from JointTrajectory
    if len(msg.points) > 0 and len(msg.joint_names) > 0:
        point = msg.points[0]
        
        # Process each joint separately
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in ["head_pan", "head_tilt", "wing_left", "wing_right"]:
                # Create data for this specific joint
                data = {
                    "joint": joint_name,
                }
                # Add angle if position is available
                if len(point.positions) > i:
                    angle_deg = int(math.degrees(point.positions[i]))
                    data["angle"] = angle_deg
                # Add speed if velocity is available
                if len(point.velocities) > i:
                    speed_percent = int(point.velocities[i] * 100)
                    data["speed"] = speed_percent
                # Publish to MQTT for this joint
                mqtt_client.publish(TOPIC_JOINTS, json.dumps(data))
                rospy.loginfo(f"[MQTT] ros/joints ({joint_name}): {data}")
    else:
        rospy.logwarn("JointTrajectory message is empty or missing joint names")

def callback_light(msg):
    data = {"white": msg.data}
    mqtt_client.publish(TOPIC_LIGHT, json.dumps(data))
    rospy.loginfo(f"[MQTT] ros/light: {data}")

def callback_led(msg):
    data = {
        "part": msg.part,
        "mode": msg.mode,
        "duration": msg.duration
    }

    mqtt_client.publish(TOPIC_LED, json.dumps(data))
    rospy.loginfo(f"[MQTT] ros/led: {data}")

def callback_speak(msg):
    data = {"msg": msg.data}
    mqtt_client.publish(TOPIC_SPEAK, json.dumps(data))
    rospy.loginfo(f"[MQTT] ros/speak: {data}")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("✅ Connected to MQTT broker")
    else:
        rospy.logwarn(f"❌ Failed to connect to MQTT broker. Code: {rc}")

if __name__ == "__main__":
    rospy.init_node("ros_to_mqtt")

    # Read ROS parameters (global or private). Falls back to defaults if not provided.
    MQTT_BROKER_IP = rospy.get_param("/mqtt_broker_ip", DEFAULT_MQTT_BROKER_IP)
    MQTT_PORT = int(rospy.get_param("/mqtt_port", DEFAULT_MQTT_PORT))

    # MQTT Client
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    mqtt_client.loop_start()

    # ROS Subscriptions
    rospy.Subscriber("/ros/cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("/ros/move", Move, callback_move)
    rospy.Subscriber("/ros/joints", JointTrajectory, callback_joints)
    rospy.Subscriber("/ros/light", UInt8, callback_light)
    rospy.Subscriber("/ros/led", Led, callback_led)
    rospy.Subscriber("/ros/speak", String, callback_speak)

    rospy.loginfo("🔁 Sending ROS commands to Android app via MQTT...")
    rospy.spin()
