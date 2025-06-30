#!/usr/bin/env python3
"""
rtmp2ros.py
============
Transforma um stream RTMP em mensagens ROS do tipo sensor_msgs/Image.

Uso:
-----
rosrun sanbot_ros rtmp2ros.py _rtmp_url:=rtmp://localhost/live/stream _topic:=/rtmp_frame _width:=1280 _height:=720 _show_image:=false

Parâmetros ROS:
  ~rtmp_url   (string)  URL do stream RTMP a ser consumido
  ~topic      (string)  Nome do tópico onde publicar as imagens (default: /rtmp_frame)
  ~width      (int)     Largura do frame do stream (default: 1280)
  ~height     (int)     Altura do frame do stream (default: 720)
  ~show_image (bool)    Exibe ou não a janela OpenCV com o vídeo (default: false)
"""

import subprocess
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def create_ffmpeg_pipe(rtmp_url: str, width: int, height: int) -> subprocess.Popen:
    """Cria um pipe subprocess com ffmpeg que converte o stream RTMP em frames brutos (BGR24)."""
    ffmpeg_cmd = [
        "ffmpeg",
        "-i",
        rtmp_url,
        "-f",
        "image2pipe",
        "-pix_fmt",
        "bgr24",
        "-vcodec",
        "rawvideo",
        "-loglevel",
        "error",  # minimiza logs no terminal
        "-",
    ]

    return subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def publish_rtmp_stream(
    rtmp_url: str,
    topic_name: str,
    width: int,
    height: int,
    show_image: bool = False,
    rate_hz: Optional[float] = None,
) -> None:
    """Publica frames do stream RTMP em um tópico ROS."""

    bridge = CvBridge()
    pub = rospy.Publisher(topic_name, Image, queue_size=1)

    pipe = create_ffmpeg_pipe(rtmp_url, width, height)
    frame_size = width * height * 3  # 3 bytes por pixel (BGR)

    rospy.loginfo(
        f"🚀 Iniciando streaming RTMP '{rtmp_url}' (tamanho: {width}x{height}) para o tópico '{topic_name}'"
    )

    rate = rospy.Rate(rate_hz) if rate_hz else None

    try:
        while not rospy.is_shutdown():
            raw_frame = pipe.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                rospy.logwarn("⚠️  Stream encerrou ou houve erro ao ler frame. Encerrando...")
                break

            # Converter para array numpy e depois para mensagem ROS Image
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((height, width, 3))
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            pub.publish(img_msg)

            if show_image:
                cv2.imshow("RTMP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("Janela fechada pelo usuário")
                    break

            if rate is not None:
                rate.sleep()

    finally:
        # Limpeza
        pipe.terminate()
        pipe.wait()
        if show_image:
            cv2.destroyAllWindows()
        rospy.loginfo("✅ Node rtmp2ros finalizado.")


if __name__ == "__main__":
    rospy.init_node("rtmp2ros", anonymous=True)

    # Parâmetros
    rtmp_url_param: str = rospy.get_param("~rtmp_url", "rtmp://localhost/live/stream")
    topic_param: str = rospy.get_param("~topic", "/camera/image_raw")
    width_param: int = int(rospy.get_param("~width", 1280))
    height_param: int = int(rospy.get_param("~height", 720))
    show_image_param: bool = bool(rospy.get_param("~show_image", False))
    rate_param: Optional[float] = rospy.get_param("~rate", None)
    if rate_param is not None:
        try:
            rate_param = float(rate_param)
        except ValueError:
            rospy.logwarn("Parâmetro ~rate inválido; ignorando.")
            rate_param = None

    publish_rtmp_stream(
        rtmp_url=rtmp_url_param,
        topic_name=topic_param,
        width=width_param,
        height=height_param,
        show_image=show_image_param,
        rate_hz=rate_param,
    )
