#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

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
}

ros_publishers = {}
bridge = CvBridge()

camera_url = None
video_started = False
lock = threading.Lock()

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
        if ros_topic:
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

    for _, ros_topic in topics.items():
        ros_publishers[ros_topic] = rospy.Publisher(ros_topic, String, queue_size=10)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.spin()
