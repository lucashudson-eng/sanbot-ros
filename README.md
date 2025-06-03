# sanbot-ros

A ROS driver for interacting with Sanbot NANO robots via MQTT and ROS topics.

## :book: Table of Contents
- [Android Apps](#iphone-android-apps)
  - [Connection Setup](#electric_plug-connection-setup)
  - [App Installation](#package-app-installation)
- [ROS Package](#robot-ros-package-sanbot_ros)
  - [Installation](#installation)
  - [Running](#running)
  - [Available Topics](#white_check_mark-available-topics)
  - [Detailed Topic Descriptions](#-detailed-topic-descriptions)

## :iphone: Android Apps

Two Android applications are provided to interface with the Sanbot robot:

1. **Sanbot_OpenSDK20191118**: Official example app from [Sanbot's Developer Center](http://blue.sanbotcloud.com:98/dev/docs/robot.html) showcasing all robot functionalities.
2. **Sanbot_OpenSDK_MQTT**: Custom version adapted for MQTT communication, designed to work with this ROS package.

### :electric_plug: Connection Setup

#### Prerequisites
Install ADB (Android Debug Bridge):
```bash
sudo apt install android-tools-adb
```

#### Connection Methods
There are two ways to connect to the Sanbot:

1. **Wi-Fi Connection (Recommended)**
   - Connect robot to the same Wi-Fi network as your PC
   - Temporarily connect USB cable to enable ADB over Wi-Fi
   - Get robot's IP and enable ADB:
     ```bash
     adb shell ip addr show wlan0
     adb tcpip 5555
     adb connect <ROBOT_IP>:5555
     ```

2. **USB Connection (Not Recommended)**
   - :warning: USB port location on robot's head makes cable vulnerable to damage
   - Only use if Wi-Fi connection is not possible

### :package: App Installation

#### Installing the App
```bash
# Install APK
adb install ~/catkin_ws/src/sanbot-ros/Sanbot_OpenSDK_MQTT/librarydemod/build/outputs/apk/debug/librarydemod-debug.apk

# If needed, uninstall previous version
adb uninstall com.grin.sanbotmqtt
```

#### Running the App
Two ways to launch the app:
1. Find it in the App Store under "Come into life" section (some versions)
2. Launch manually via ADB:
   ```bash
   adb shell am start -n com.grin.sanbotmqtt/.MainActivity
   ```

#### Useful ADB Commands
```bash
# Force close the app
adb shell am force-stop com.grin.sanbotmqtt

# View app logs
adb logcat
```

## :robot: ROS Package sanbot_ros

### Installation

1. Clone the repository:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/lucashudson-eng/sanbot-ros.git
```

2. Install Python dependencies:
```bash
pip install -r ~/catkin_ws/src/sanbot-ros/sanbot_ros/requirements.txt
```

3. Build and source the workspace:
```bash
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Running

1. Start MQTT broker:
```bash
mosquitto -p 1883 -d
```

2. Configure network:
   - Connect robot to same network as computer
   - Get computer's IP: `ifconfig`
   - Set computer's IP in robot's app

3. Launch ROS-MQTT bridge:
```bash
roslaunch sanbot_ros bridge.launch
```

4. Control options:
   - Custom gamepad control:
     ```bash
     rosrun sanbot_ros gamer_teleop.py
     ```
   - Standard ROS keyboard control:
     ```bash
     sudo apt install ros-noetic-teleop-twist-keyboard
     rosrun teleop_twist_keyboard teleop_twist_keyboard.py
     ```

### :white_check_mark: Available Topics

| **Topic**                          | **Direction** | **Description**                                              | **Message Format Example** |
|-----------------------------------|---------------|--------------------------------------------------------------|-----------------------------|
| `sanbot/touch`                   | subscribe       | Touch sensor                                                | `{"part": 3, "description": "chest_right"}` |
| `sanbot/pir`                     | subscribe       | PIR presence sensor (front or back)                         | `{"part": "front", "status": 1}` |
| `sanbot/ir`                      | subscribe       | Infrared sensor (distance in cm)                            | `{"sensor": 1, "distance_cm": 45}` |
| `sanbot/voice_angle`             | subscribe       | Angle of detected voice                                     | `{"angle": 123}` |
| `sanbot/obstacle`                | subscribe       | Obstacle detection sensor                                   | `{"status": 1}` |
| `sanbot/battery`                 | subscribe       | Battery level and charging status                           | `{"battery_level": 82, "battery_status": "charging_by_wire"}` |
| `sanbot/info`                    | subscribe       | System information                                          | `{"robot_id": "...", "ip": "..."}` |
| `sanbot/camera`                  | subscribe       | HD camera video stream                                      | `sensor_msgs/Image` |
| `sanbot/gyro`                    | subscribe       | Robot orientation angles                                    | `sensor_msgs/Imu` |
| `sanbot/speech`                  | subscribe       | Speech recognition results (English)                        | `{"text": "recognized speech text"}` |
| `ros/light`                      | publish         | White forehead LED control                                  | `{"white": 2}` |
| `ros/move`                       | publish         | Robot movement control                                      | `{"direction": "forward", "speed": 6}` |
| `ros/cmd_vel`                    | publish         | Standard ROS velocity control                               | `geometry_msgs/Twist` |
| `ros/head`                       | publish         | Head movement control                                       | `{"direction": "up", "angle": 10}` |
| `ros/led`                        | publish         | Color LED control                                           | `{"part": "all_head", "mode": "blue"}` |
| `ros/speak`                      | publish         | Text-to-speech (English)                                    | `{"msg": "Hello World"}` |

### 📝 Detailed Topic Descriptions

#### Sensor Topics (Subscribe)

##### `sanbot/touch`
Touch sensor events from various parts of the robot.
- **Format**: `{"part": <part_id>, "description": "<location>"}`
- **Part IDs**:
  - 1-2: Chin (right, left)
  - 3-4: Chest (right, left)
  - 5-6: Back head (left, right)
  - 7-8: Back (left, right)
  - 9-10: Hand (left, right)
  - 11: Head middle
  - 12-13: Head front (right, left)

##### `sanbot/pir`
PIR (Passive Infrared) presence detection sensors.
- **Format**: `{"part": "<location>", "status": <0|1>}`
- **Locations**: "front" or "back"
- **Status**: 1 = presence detected, 0 = no presence

##### `sanbot/ir`
Infrared distance sensor readings.
- **Format**: `{"sensor": <sensor_id>, "distance_cm": <distance>}`
- **Distance**: Range in centimeters (0-100)

##### `sanbot/voice_angle`
Sound source localization angle.
- **Format**: `{"angle": <degrees>}`
- **Angle**: 0-360 degrees, indicating sound source direction

##### `sanbot/obstacle`
Obstacle detection sensor status.
- **Format**: `{"status": <0|1>}`
- **Status**: 1 = obstacle detected, 0 = no obstacle

##### `sanbot/battery`
Battery status information.
- **Format**: `{"battery_level": <percentage>, "battery_status": "<status>"}`
- **Level**: 0-100
- **Status**: "not_charging", "charging_by_pile", "charging_by_wire"

##### `sanbot/info`
System information and robot status.
- **Format**: 
```json
{
  "robot_id": "<id>",
  "ip": "<ip_address>",
  "main_service_version": "<version>",
  "android_version": "<version>",
  "device_model": "<model>"
}
```

##### `sanbot/camera`
HD camera video stream.
- **Type**: `sensor_msgs/Image`
- **Alternative**: MJPEG stream at `http://<robot_ip>:8080`
- **Resolution**: 1280x720 pixels

##### `sanbot/gyro`
Robot orientation in 3D space.
- **Type**: `sensor_msgs/Imu`
- **Data**: Orientation in quaternion (converted from roll, pitch, yaw angles)
- **Frame**: "base_link"

##### `sanbot/speech`
Speech recognition results.
- **Format**: `{"text": "<recognized_text>"}`
- **Language**: English
- **Note**: Recognition works best in quiet environments

#### Control Topics (Publish)

##### `ros/light`
Control the white forehead LED.
- **Format**: `{"white": <level>}`
- **Level**: 0 = off, 1-3 = brightness levels

##### `ros/move`
Direct robot movement control.
- **Format**: `{"direction": "<direction>", "speed": <1-10>, "distance": <cm>, "duration": <seconds>}`
- **Directions**:
  - Basic: "forward", "backward", "left", "right", "stop"
  - Combined: "left_forward", "right_forward", "left_back", "right_back"
  - Rotation: "turn_left", "turn_right"
- **Parameters**:
  - speed: Movement speed (1-10) (optional)
  - distance: Movement distance in cm (optional)
  - duration: Movement duration in seconds (optional)

##### `ros/cmd_vel`
Standard ROS velocity control.
- **Type**: `geometry_msgs/Twist`
- **Fields**:
  - linear.x: Forward/backward velocity (-1.0 to 1.0)
  - linear.y: Left/right velocity (-1.0 to 1.0)
  - angular.z: Rotation velocity (-1.0 to 1.0)
- **Note**: Preferred method for smooth continuous movement

##### `ros/head`
Control robot's head movement.
- **Format**: `{"direction": "<direction>", "angle": <degrees>, "motor": <motor_id>, "speed": <1-100>}`
- **Directions**: "up", "down", "left", "right"
- **Motors**:
  - 1: Neck movement
  - 2: Vertical movement
  - 3: Horizontal movement
- **Parameters**:
  - angle: Movement angle in degrees (0-90)
  - speed: Movement speed (1-100)

##### `ros/led`
Control colored LED lights.
- **Format**: `{"part": "<location>", "mode": "<color>", "duration": <seconds>, "random": <0|1>}`
- **Locations**:
  - "all_head", "all_hand": All head/hand LEDs
  - "head_left", "head_right": Individual head LEDs
  - "arm_left", "arm_right": Individual hand LEDs
- **Colors**: 
  - Normal mode: "white", "red", "green", "blue", "yellow", "purple", "pink"
  - Flicker mode: "flicker_white", "flicker_red", "flicker_green", "flicker_pink", "flicker_purple", "flicker_blue", "flicker_yellow", "flicker_random"
- **Parameters**:
  - duration: Light effect duration in seconds (0 = infinite)
  - random: Enable random color mode (0 or 1)

##### `ros/speak`
Text-to-speech command.
- **Format**: `{"msg": "<text>"}`
- **Language**: English
- **Volume**: Fixed at system volume level

## :handshake: Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## :page_facing_up: License

This project is licensed under the MIT License - see the LICENSE file for details.

## :envelope: Contact

For support or questions, please open an issue or contact the maintainer at lucashudson.eng@gmail.com
