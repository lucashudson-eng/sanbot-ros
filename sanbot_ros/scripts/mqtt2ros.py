#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Imu, BatteryState, Range
from sanbot_ros.msg import Info
import math
import tf.transformations

DEFAULT_MQTT_BROKER_IP = "localhost"
DEFAULT_MQTT_PORT = 1883

topics = ["/touch", "/pir", "/ir", "/voice_angle", "/obstacle", "/battery", "/info", "/speech", "/imu"]

ros_publishers = {}

# Namespace-aware topic handling (will be filled in __main__)
namespaced_topics = []           # List of MQTT topics with namespace prefix (used for subscribe)
topic_to_base = {}              # Mapping: namespaced → base topic ("/robo/ir" → "/ir")
namespace_prefix = ""          # e.g. "/robo" or "" when no namespace

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("✅ Connected to MQTT broker")
        for topic in namespaced_topics:
            client.subscribe(topic)
    else:
        rospy.logwarn(f"❌ Failed to connect to MQTT broker. Code: {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")

        # Map the received MQTT topic back to its base form (without namespace) so that
        # the logic below remains unchanged regardless of whether a ROS namespace is used.
        base_topic = topic_to_base.get(msg.topic, msg.topic)

        if base_topic == "/voice_angle":
            # Convert voice angle to Int32
            data = json.loads(payload)
            angle_msg = Int32()
            angle_msg.data = int(data["angle"])
            ros_publishers[msg.topic].publish(angle_msg)
            return

        elif base_topic == "/obstacle":
            # Convert obstacle detection to Bool
            data = json.loads(payload)
            obstacle_msg = Bool()
            obstacle_msg.data = bool(data["status"])
            ros_publishers[msg.topic].publish(obstacle_msg)
            return

        elif base_topic == "/info":
            # Convert system information to Info
            data = json.loads(payload)
            info_msg = Info()
            info_msg.robot_id = data["robot_id"]
            info_msg.ip = data["ip"]
            info_msg.main_service_version = data["main_service_version"]
            info_msg.android_version = data["android_version"]
            info_msg.device_model = data["device_model"]
            ros_publishers[msg.topic].publish(info_msg)
            return

        elif base_topic == "/ir":
            # Convert IR data to Range
            data = json.loads(payload)
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = f"ir_{data['sensor']}_link"
            
            # Set sensor type as IR
            range_msg.radiation_type = Range.INFRARED
            
            # Convert distance from cm to meters
            range_msg.range = float(data["distance_cm"]) / 100.0
            
            # Set sensor limits (in meters)
            range_msg.min_range = 0.0  # 0 cm
            range_msg.max_range = 0.64  # 64 cm
            
            ros_publishers[msg.topic].publish(range_msg)
            return

        elif base_topic == "/battery":
            # Convert battery data to BatteryState
            data = json.loads(payload)
            battery_msg = BatteryState()
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.header.frame_id = "battery"
            battery_msg.present = True
            
            # Convert percentage to value between 0 and 1
            battery_level = float(data["battery_level"])
            battery_msg.percentage = battery_level / 100.0
            
            # Set charging status
            if battery_level >= 100 and (data["battery_status"] == "charging_by_wire" or data["battery_status"] == "charging_by_pile"):
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif data["battery_status"] == "charging_by_wire" or data["battery_status"] == "charging_by_pile":
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:  # not_charging
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            
            # Set battery type as Li-ion
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            
            ros_publishers[msg.topic].publish(battery_msg)
            rospy.loginfo_throttle(1, f"🔋 Battery status: {data['battery_level']}%, {data['battery_status']}")
            return

        elif base_topic == "/imu":
            # Handle gyroscope data (orientation angles)
            data = json.loads(payload)
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            
            # Convert Euler angles (in degrees) to quaternion
            quaternion = euler_to_quaternion(
                float(data["x"]),  # roll
                float(data["y"]),  # pitch
                float(data["z"])   # yaw
            )
            
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            ros_publishers[msg.topic].publish(imu_msg)
            rospy.loginfo_throttle(1, f"📊 Orientation angles published: roll={data['x']}, pitch={data['y']}, yaw={data['z']}")
            return

        # For other topics, publish as String
        string_msg = String()
        string_msg.data = payload
        ros_publishers[msg.topic].publish(string_msg)

    except Exception as e:
        rospy.logerr(f"❌ Error processing MQTT message from topic '{msg.topic}': {e}")

if __name__ == "__main__":
    rospy.init_node("mqtt_to_ros")
    MQTT_BROKER_IP = rospy.get_param("/mqtt_broker_ip", DEFAULT_MQTT_BROKER_IP)
    MQTT_PORT = int(rospy.get_param("/mqtt_port", DEFAULT_MQTT_PORT))

    # Build namespace prefix (if any). rospy.get_namespace() returns '/' when no namespace.
    ros_ns = rospy.get_namespace().rstrip('/')  # e.g. '/robo' or ''
    namespace_prefix = '' if ros_ns in ['', '/'] else ros_ns

    # Generate namespaced MQTT topics and mapping
    namespaced_topics = [(namespace_prefix + t) if namespace_prefix else t for t in topics]
    topic_to_base = {ns_t: base_t for ns_t, base_t in zip(namespaced_topics, topics)}

    # Create ROS publishers for the namespaced topics
    for base_topic, ns_topic in zip(topics, namespaced_topics):
        if base_topic == "/imu":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, Imu, queue_size=10)
        elif base_topic == "/battery":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, BatteryState, queue_size=10)
        elif base_topic == "/ir":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, Range, queue_size=10)
        elif base_topic == "/voice_angle":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, Int32, queue_size=10)
        elif base_topic == "/obstacle":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, Bool, queue_size=10)
        elif base_topic == "/info":
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, Info, queue_size=10)
        else:
            ros_publishers[ns_topic] = rospy.Publisher(ns_topic, String, queue_size=10)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.loginfo("🔁 Sending ROS commands to Android app via MQTT...")
    rospy.spin()
