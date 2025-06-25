#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Int32, Bool, UInt8
from sensor_msgs.msg import Image, Imu, BatteryState, Range
from sanbot_ros.msg import Info, Move, Head, Led
from cv_bridge import CvBridge
import cv2
import threading
import math
import tf.transformations

MQTT_BROKER_IP = "localhost"
MQTT_PORT = 1883

topics = {
    "sanbot/touch": "sanbot/touch",
    "sanbot/pir": "sanbot/pir",
    "sanbot/ir": "sanbot/ir",
    "sanbot/voice_angle": "sanbot/voice_angle",
    "sanbot/obstacle": "sanbot/obstacle",
    "sanbot/battery": "sanbot/battery",
    "sanbot/info": "sanbot/info",
    "sanbot/speech": "sanbot/speech",
    "sanbot/gyro": "sanbot/gyro",
}

ros_publishers = {}
bridge = CvBridge()

lock = threading.Lock()

# Flag indicating if camera threads have been started
video_started = False

# Helper to start camera capture threads once the robot IP is known
def start_camera_threads(robot_ip):
    """Spawn capture threads for both camera streams if not already running."""
    global video_started
    with lock:
        if video_started:
            return

        camera1_url = f"http://{robot_ip}:8080/camera1"
        camera2_url = f"http://{robot_ip}:8080/camera2"

        # Ensure publishers exist (created later in main as well, but this is idempotent)
        if "/sanbot/camera1" not in ros_publishers:
            ros_publishers["/sanbot/camera1"] = rospy.Publisher("/sanbot/camera1", Image, queue_size=1)
        if "/sanbot/camera2" not in ros_publishers:
            ros_publishers["/sanbot/camera2"] = rospy.Publisher("/sanbot/camera2", Image, queue_size=1)

        threading.Thread(target=video_capture_loop, args=(camera1_url, ros_publishers["/sanbot/camera1"], "/sanbot/camera1"), daemon=True).start()
        threading.Thread(target=video_capture_loop, args=(camera2_url, ros_publishers["/sanbot/camera2"], "/sanbot/camera2"), daemon=True).start()

        video_started = True

# -----------------------------------------------------------------------------
# 📷 Video streaming configuration
# -----------------------------------------------------------------------------
# Two separate MJPEG streams são suportados (http://<ip>:8080/camera1 e
# http://<ip>:8080/camera2). O IP é obtido automaticamente a partir da
# mensagem recebida em "sanbot/info". Assim que o IP é conhecido, iniciamos
# duas threads que capturam e publicam em /sanbot/camera1 e /sanbot/camera2.
# -----------------------------------------------------------------------------

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def video_capture_loop(stream_url, pub, topic_name):
    """Continuously capture frames from *stream_url* and publish them on *pub*.

    A short log message (throttled) is emitted to indicate successful
    publishing; the *topic_name* is used here purely for readability.
    """
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        rospy.logerr(f"❌ Não foi possível abrir o stream em: {stream_url}")
        return

    rospy.loginfo(f"📡 Capturando vídeo de {stream_url} → {topic_name}")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                pub.publish(ros_img)
                rospy.loginfo_throttle(5, f"📤 Frame publicado em {topic_name}")
            except Exception as e:
                rospy.logerr(f"Erro ao converter/publicar frame do topic {topic_name}: {e}")
        else:
            rospy.logwarn_throttle(10, f"⚠️ Falha ao capturar frame do vídeo ({topic_name})")
        rate.sleep()

    cap.release()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("✅ Connected to MQTT broker")
        for topic in topics.keys():
            client.subscribe(topic)
    else:
        rospy.logwarn(f"❌ Failed to connect to MQTT broker. Code: {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        ros_topic = topics.get(msg.topic)
        
        if msg.topic == "sanbot/voice_angle":
            # Convert voice angle to Int32
            data = json.loads(payload)
            angle_msg = Int32()
            angle_msg.data = int(data["angle"])
            ros_publishers[ros_topic].publish(angle_msg)
            return

        elif msg.topic == "sanbot/obstacle":
            # Convert obstacle detection to Bool
            data = json.loads(payload)
            obstacle_msg = Bool()
            obstacle_msg.data = bool(data["status"])
            ros_publishers[ros_topic].publish(obstacle_msg)
            return

        elif msg.topic == "sanbot/info":
            # Convert system information to Info
            data = json.loads(payload)
            info_msg = Info()
            info_msg.robot_id = data["robot_id"]
            info_msg.ip = data["ip"]
            info_msg.main_service_version = data["main_service_version"]
            info_msg.android_version = data["android_version"]
            info_msg.device_model = data["device_model"]
            ros_publishers[ros_topic].publish(info_msg)

            # Once we know the robot's IP, start video capture threads
            start_camera_threads(data["ip"])
            return

        elif msg.topic == "sanbot/ir":
            # Convert IR data to Range
            data = json.loads(payload)
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = f"ir_sensor_{data['sensor']}"
            
            # Set sensor type as IR
            range_msg.radiation_type = Range.INFRARED
            
            # Convert distance from cm to meters
            range_msg.range = float(data["distance_cm"]) / 100.0
            
            # Set sensor limits (in meters)
            range_msg.min_range = 0.0  # 0 cm
            range_msg.max_range = 0.64  # 64 cm
            
            # Set approximate field of view (FOV) in radians
            # Using a typical value for IR sensors
            range_msg.field_of_view = math.radians(5.0)  # ~5 degrees
            
            ros_publishers[ros_topic].publish(range_msg)
            return

        elif msg.topic == "sanbot/battery":
            # Convert battery data to BatteryState
            data = json.loads(payload)
            battery_msg = BatteryState()
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.header.frame_id = "battery"
            
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
            
            # Set battery capacity in Ah
            battery_msg.capacity = 20.0  # Current capacity in Ah
            battery_msg.design_capacity = 20.0  # Design capacity in Ah
            
            # Indicate battery is present
            battery_msg.present = True
            
            # Fields we don't have real-time info for
            battery_msg.voltage = float('nan')
            battery_msg.current = float('nan')
            battery_msg.charge = float('nan')
            battery_msg.temperature = float('nan')
            
            ros_publishers[ros_topic].publish(battery_msg)
            rospy.loginfo_throttle(1, f"🔋 Battery status: {data['battery_level']}%, {data['battery_status']}")
            return

        elif msg.topic == "sanbot/gyro":
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
            
            # Set covariance (using default values)
            imu_msg.orientation_covariance = [0.0] * 9
            imu_msg.orientation_covariance[0] = 0.01  # X axis
            imu_msg.orientation_covariance[4] = 0.01  # Y axis
            imu_msg.orientation_covariance[8] = 0.01  # Z axis
            
            # Zero out unused fields
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.angular_velocity_covariance = [-1.0] * 9  # -1 indicates unused
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            imu_msg.linear_acceleration_covariance = [-1.0] * 9  # -1 indicates unused
            
            ros_publishers[ros_topic].publish(imu_msg)
            rospy.loginfo_throttle(1, f"📊 Orientation angles published: roll={data['x']}, pitch={data['y']}, yaw={data['z']}")
            return

        # For other topics, publish as String
        string_msg = String()
        string_msg.data = payload
        ros_publishers[ros_topic].publish(string_msg)

    except Exception as e:
        rospy.logerr(f"❌ Error processing MQTT message from topic '{msg.topic}': {e}")

if __name__ == "__main__":
    rospy.init_node("mqtt_to_ros")

    # ------------------------------------------------------------------
    # 🗞️ Camera publishers (threads will start after IP is received via
    #     sanbot/info).  We prepare the publishers here so other nodes can
    #     already subscribe.
    # ------------------------------------------------------------------
    ros_publishers["/sanbot/camera1"] = rospy.Publisher("/sanbot/camera1", Image, queue_size=1)
    ros_publishers["/sanbot/camera2"] = rospy.Publisher("/sanbot/camera2", Image, queue_size=1)

    for mqtt_topic, ros_topic in topics.items():
        if mqtt_topic == "sanbot/gyro":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Imu, queue_size=10)
        elif mqtt_topic == "sanbot/battery":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, BatteryState, queue_size=10)
        elif mqtt_topic == "sanbot/ir":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Range, queue_size=10)
        elif mqtt_topic == "sanbot/voice_angle":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Int32, queue_size=10)
        elif mqtt_topic == "sanbot/obstacle":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Bool, queue_size=10)
        elif mqtt_topic == "sanbot/info":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Info, queue_size=10)
        else:
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, String, queue_size=10)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.loginfo("🔁 Sending ROS commands to Android app via MQTT...")
    rospy.spin()
