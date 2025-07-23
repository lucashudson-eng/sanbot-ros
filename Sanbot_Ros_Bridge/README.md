# Sanbot ROS Bridge (Android)

An **Android** application that acts as a bridge between a **Sanbot** robot and a computer running **ROS**. It uses:

* **MQTT** (port **1883**) for telemetry and control messages
* **RTMP** (port **1935**) for streaming the robot's camera

The app forwards sensor data from the robot to MQTT, receives control commands from ROS via MQTT, and streams the video feed to the RTMP server.

---
## :bookmark_tabs: Table of Contents
1. [Prerequisites](#prerequisites)
2. [App Installation](#app-installation)
3. [Running](#running)
4. [Usage](#usage)
5. [License](#license)

---
## Prerequisites

On the **PC** running Ubuntu (ROS + Docker host):

| Software | Minimum version | Notes |
|----------|-----------------|-------|
| Docker   | 20.10           | Required for the MQTT/RTMP container |
| ROS Noetic | —             | Validated on Ubuntu 20.04 |
| Python 3 | 3.8             | Dependencies for the `sanbot_ros` package |

On the **Sanbot robot**:

* Free USB port (to enable ADB-over-Wi-Fi once or to transfer APK through File Manager)

Optional tools (for installation / debugging):

```bash
sudo apt install android-tools-adb
```

---
## App Installation

### 1. Flash Drive and File Manager (quick method)

1. Copy the pre-built APK `librarydemod-debug.apk` (or the latest release) to a FAT32-formatted USB flash drive.  
2. Plug the flash drive into the USB port on the robot’s head.  
3. On the robot, open **File Manager**, navigate to the USB drive (often labelled **udisk**) and tap the APK.  
4. When prompted, allow installation from unknown sources and confirm **Install**.  
5. Wait until the installation completes and remove the flash drive.  
6. You can now skip directly to the [Running](#running) section.

### 2. Connect robot to PC via ADB

You have two options to establish an **ADB** connection, depending on whether you prefer a permanent wireless setup or a quick USB connection to enable ADB-over-Wi-Fi.

**A) USB-only (quick install)**

1. Plug a USB cable between the PC and the robot’s head USB port.
2. Verify the connection:
   ```bash
   adb devices            # the robot should appear as «device»
   ```
3. Skip to step *3. Install the pre-built APK* below.
4. Unplug the cable when you are done.

**B) Enable ADB-over-Wi-Fi (recommended for development)**

1. Connect the robot to the same Wi-Fi network as the PC.
2. Connect a USB cable just once to switch ADB to TCP mode:
   ```bash
   adb devices                   # make sure it is detected over USB first
   adb shell ip addr show wlan0  # to discover <ROBOT_IP> 
   adb tcpip 5555                # restarts ADB daemon on the robot at TCP port 5555
   ```
3. Connect wirelessly and unplug the cable:
   ```bash
   adb connect <ROBOT_IP>:5555
   # Example:
   adb connect 192.168.0.42:5555
   ```
4. From now on you can install / debug the app wirelessly until the robot reboots (repeat if necessary).
5. Skip to step *4. Build from source* below.

### 3. Install the pre-built APK (recommended)

```bash
adb install -r ~/catkin_ws/src/sanbot-ros/Sanbot_Ros_Bridge/librarydemod-debug.apk
```

Remove a previous version if needed:

```bash
adb uninstall com.grin.sanbotrosbridge
```

### 4. Build from source (optional)

1. Open the `Sanbot_Ros_Bridge` project in **Android Studio**
2. Perform **Sync Project with Gradle**  
3. Press **Run ▶** or generate APK and run:

```bash
adb install -r ~/catkin_ws/src/sanbot-ros/Sanbot_Ros_Bridge/librarydemod/build/outputs/apk/debug/librarydemod-debug.apk
```

---
## Running

1. Make sure the **sanbot_mqtt_rtmp** container is running:

```bash
docker ps | grep sanbot_mqtt_rtmp
```

If the command prints nothing, the MQTT/RTMP container is not running yet. Build (first time only) and start it with:

(If docker is not installed check process of installation in */docker_mqtt_rtmp*.)

```bash
cd ~/catkin_ws/src/sanbot-ros/docker_mqtt_rtmp
docker build -t sanbot_mqtt_rtmp .
# Run in detached mode and restart automatically at boot
docker run -d --name sanbot_mqtt_rtmp --restart unless-stopped -p 1883:1883 -p 1935:1935 sanbot_mqtt_rtmp
```
(To stop later: `docker stop sanbot_mqtt_rtmp && docker rm sanbot_mqtt_rtmp`)

2. On the PC, start the ROS-MQTT-RTMP bridge:

```bash
roslaunch sanbot_ros bridge.launch
```

3. On the robot, open the **Sanbot ROS Bridge** app.

   * Discover the PC IP (broker host) on the PC itself:
     ```bash
     hostname -I | awk '{print $1}'   # e.g. 192.168.0.10
     ```
   * In the app top field, enter this IP in **Server IP**.
   * Tap **Connect** to establish the MQTT link.
   * Enable the RTMP camera stream by toggling the **Camera** switch.

4. In a ROS terminal you should see topics appearing, e.g.:

```bash
rostopic list
```

---
## Usage

Below is a quick reference of the MQTT topics and RTMP transmission to understood by the **Sanbot ROS Bridge** app and the JSON payloads expected / produced. All topics are rooted at the broker running on the PC (`mqtt://<PC_IP>:1883`).

### 1. ROS → Robot (App subscribes)
| MQTT topic | Purpose | JSON schema | Example |
|------------|---------|-------------|---------|
| `ros/move` | Discrete movements and basic navigation | `{ "direction": string, "speed": int?, "distance": int?, "duration": int? }` | `{ "direction": "forward", "speed": 7, "duration": 2 }` |
| `ros/light` | White forehead LED (brightness) | `{ "white": 0-3 }` | `{ "white": 2 }` |
| `ros/led` | Color LEDs (head / hands) | `{ "part": string, "mode": string, "duration": int }` | `{ "part": "all_head", "mode": "blue", "duration": 5 }` |
| `ros/joints` | Head & wing servos | `{ "joint": string, "angle": int°, "speed": int%? }` | `{ "joint": "head_pan", "angle": 45 }` |
| `ros/speak` | Text-to-Speech | `{ "msg": string }` | `{ "msg": "Hello, I am Sanbot" }` |

> Notes
> * Optional fields may be omitted.
> * Angles are integers in **degrees**; speeds are percentages `0-100`.

### 2. Robot → ROS (App publishes)
| MQTT topic | Content | Example payload |
|------------|---------|-----------------|
| `sanbot/touch` | Touch sensor events | `{ "id": 3, "name": "chest_right" }` |
| `sanbot/pir` | Presence sensor | `{ "part": "front", "status": 1 }` |
| `sanbot/ir` | IR distance (cm) | `{ "sensor": 2, "distance_cm": 34 }` |
| `sanbot/voice_angle` | Sound source angle | `{ "angle": 127 }` |
| `sanbot/obstacle` | Obstacle detection | `{ "status": true }` |
| `sanbot/battery` | Battery level & status | `{ "battery_level": 87, "battery_status": "charging_by_wire" }` |
| `sanbot/info` | System info | `{ "robot_id": "9ad1e4…", "ip": "192.168.0.42", ... }` |
| `sanbot/gyro` | Orientation (roll, pitch, yaw in °) | `{ "x": 2.3, "y": 0.8, "z": -45 }` |
| `sanbot/speech` | Speech recognition result | `{ "msg": "Hello Sanbot" }` |

The `mqtt2ros.py` and `ros2mqtt.py` scripts inside the `sanbot_ros` package automatically convert these JSON messages to/from standard ROS messages, so you can use native ROS topics in your own nodes.

### 3. RTMP Video Stream
The app pushes the camera feed to the broker host via:
```text
rtmp://<PC_IP>:1935/live/stream
```
The `rtmp2ros.py` script inside the `sanbot_ros` package automatically convert the RTMP transmision to /camera/raw ROS topic.

(Default resolution is 1280×720 @ 30 fps.)

These 3 scripts already run automatically within roslaunch.

---
## License

This project is licensed under the **MIT License**. See the [`LICENSE`](../LICENSE) file for details.