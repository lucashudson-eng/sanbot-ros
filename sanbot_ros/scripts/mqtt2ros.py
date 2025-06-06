#!/usr/bin/env python3
import rospy
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Int32, Bool, UInt8
from sensor_msgs.msg import Image, Imu, BatteryState, Range
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
        
        if msg.topic == "sanbot/voice_angle":
            # Converter ângulo da voz para Int32
            data = json.loads(payload)
            angle_msg = Int32()
            angle_msg.data = int(data["angle"])
            ros_publishers[ros_topic].publish(angle_msg)
            return

        elif msg.topic == "sanbot/obstacle":
            # Converter detecção de obstáculo para Bool
            data = json.loads(payload)
            obstacle_msg = Bool()
            obstacle_msg.data = bool(data["status"])
            ros_publishers[ros_topic].publish(obstacle_msg)
            return

        elif msg.topic == "ros/light":
            # Converter nível de luz para UInt8 (0-3)
            data = json.loads(payload)
            light_msg = UInt8()
            if isinstance(data, dict) and "white" in data:
                light_msg.data = int(data["white"])
            else:
                light_msg.data = int(data)
            ros_publishers[ros_topic].publish(light_msg)
            return

        elif msg.topic == "sanbot/ir":
            # Converter dados do IR para Range
            data = json.loads(payload)
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = f"ir_sensor_{data['sensor']}"
            
            # Definir o tipo do sensor como IR
            range_msg.radiation_type = Range.INFRARED
            
            # Converter distância de cm para metros
            range_msg.range = float(data["distance_cm"]) / 100.0
            
            # Definir os limites do sensor (em metros)
            range_msg.min_range = 0.0  # 0 cm
            range_msg.max_range = 0.64  # 64 cm
            
            # Definir o campo de visão (FOV) aproximado em radianos
            # Usando um valor típico para sensores IR
            range_msg.field_of_view = math.radians(5.0)  # ~5 graus
            
            ros_publishers[ros_topic].publish(range_msg)
            return
            
        elif msg.topic == "sanbot/battery":
            # Converter dados da bateria para BatteryState
            data = json.loads(payload)
            battery_msg = BatteryState()
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.header.frame_id = "battery"
            
            # Converter porcentagem para valor entre 0 e 1
            battery_level = float(data["battery_level"])
            battery_msg.percentage = battery_level / 100.0
            
            # Definir o status de carregamento
            if battery_level >= 100 and (data["battery_status"] == "charging_by_wire" or data["battery_status"] == "charging_by_pile"):
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif data["battery_status"] == "charging_by_wire" or data["battery_status"] == "charging_by_pile":
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:  # not_charging
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            
            # Definir o tipo de bateria como Li-ion
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            
            # Definir capacidade da bateria em Ah
            battery_msg.capacity = 20.0  # Capacidade atual em Ah
            battery_msg.design_capacity = 20.0  # Capacidade de projeto em Ah
            
            # Indicar que a bateria está presente
            battery_msg.present = True
            
            # Campos que não temos informação em tempo real
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
            try:
                data = json.loads(payload)
                if isinstance(data, dict):
                    # Concatena os valores do dicionário em uma string separada por espaço
                    string_msg = String()
                    string_msg.data = ' '.join(str(v) for v in data.values())
                    ros_publishers[ros_topic].publish(string_msg)
                    rospy.loginfo(f"[ROS] {ros_topic} <- '{string_msg.data}'")
                else:
                    # Se não for dicionário (por exemplo, string direta ou número), publica como está
                    msg = String()
                    msg.data = str(data)
                    ros_publishers[ros_topic].publish(msg)
                    rospy.loginfo(f"[ROS] {ros_topic} <- '{msg.data}'")
            except Exception as e:
                rospy.logwarn(f"Erro ao converter JSON para string simples em {ros_topic}: {e}")

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
        elif mqtt_topic == "sanbot/battery":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, BatteryState, queue_size=10)
        elif mqtt_topic == "sanbot/ir":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Range, queue_size=10)
        elif mqtt_topic == "sanbot/voice_angle":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Int32, queue_size=10)
        elif mqtt_topic == "sanbot/obstacle":
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, Bool, queue_size=10)
        else:
            ros_publishers[ros_topic] = rospy.Publisher(ros_topic, String, queue_size=10)

    # MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    rospy.loginfo("🔁 Enviando comandos do ROS para o app Android via MQTT...")
    rospy.spin()
