#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
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

camera_url = None
video_started = False
lock = threading.Lock()

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)

def video_capture_loop(stream_url, pub):
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        rospy.logerr(f"❌ Não foi possível abrir o stream em: {stream_url}")
        return
    rospy.loginfo(f"📡 Capturando vídeo de {stream_url}")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                ros_img = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                pub.publish(ros_img)
                rospy.loginfo_throttle(5, "📤 Frame publicado em /sanbot/camera")
            except Exception as e:
                rospy.logerr(f"Erro ao converter/publicar frame: {e}")
        else:
            rospy.logwarn_throttle(10, "⚠️ Falha ao capturar frame do vídeo")
        rate.sleep()

    cap.release()

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("✅ Conectado ao broker MQTT")
    for topic in topics:
        client.subscribe(topic)

def on_message(client, userdata, msg):
    global camera_url, video_started

    try:
        payload = msg.payload.decode("utf-8")
        ros_topic = topics.get(msg.topic)
        
        if msg.topic == "sanbot/gyro":
            # Handle gyroscope data (orientation angles)
            data = json.loads(payload)
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            
            # Convert Euler angles (in degrees) to quaternion
            # x = roll (rotation around X)
            # y = pitch (rotation around Y)
            # z = yaw (rotation around Z)
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
        
        elif msg.topic == "sanbot/speech":
            # Handle speech recognition
            data = json.loads(payload)
            speech_msg = String()
            speech_msg.data = data["text"]
            ros_publishers[ros_topic].publish(speech_msg)
            rospy.loginfo(f"🗣️ Speech recognized: {data['text']}")
        
        elif ros_topic:
            # Handle other topics as before
            ros_publishers[ros_topic].publish(payload)

        if msg.topic == "sanbot/info" and not video_started:
            data = json.loads(payload)
            ip = data.get("ip")
            if ip:
                stream_url = f"http://{ip}:8080"
                ros_publishers["/sanbot/stream_url"].publish(stream_url)
                rospy.loginfo(f"[INFO] URL do stream recebida: {stream_url}")

                with lock:
                    if not video_started:
                        video_started = True
                        t = threading.Thread(
                            target=video_capture_loop,
                            args=(stream_url, ros_publishers["/sanbot/camera"]),
                            daemon=True
                        )
                        t.start()

    except Exception as e:
        rospy.logerr(f"Erro ao processar mensagem: {e}")

if __name__ == "__main__":
    rospy.init_node("mqtt_to_ros")

    # Publishers
    ros_publishers["/sanbot/stream_url"] = rospy.Publisher("/sanbot/stream_url", String, queue_size=1)
    ros_publishers["/sanbot/camera"] = rospy.Publisher("/sanbot/camera", Image, queue_size=1)

    for mqtt_topic, ros_topic in topics.items():
        if mqtt_topic == "sanbot/gyro":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Imu, queue_size=10)
        else:
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, String, queue_size=10)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.spin()
