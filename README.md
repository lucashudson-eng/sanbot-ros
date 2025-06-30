# sanbot-ros

A ROS driver for interacting with Sanbot NANO robots via MQTT and ROS topics.

## :book: Table of Contents
- [Custom Message Types](#-custom-message-types)
- [Android Apps](#iphone-android-apps)
  - [Connection Setup](#electric_plug-connection-setup)
  - [App Installation](#package-app-installation)
- [ROS Package](#robot-ros-package-sanbot_ros)
  - [Installation](#installation)
  - [Running](#running)
  - [Available Topics](#white_check_mark-available-topics)
  - [Detailed Topic Descriptions](#-detailed-topic-descriptions)
- [Docker Setup](#docker-setup)

## 📦 Custom Message Types

This package includes custom ROS message types for structured communication:

- **`sanbot_ros/Info`**: System information (robot_id, ip, versions, device_model)
- **`sanbot_ros/Move`**: Movement control (direction, speed, distance, duration)
- **`sanbot_ros/Head`**: Head movement (direction, angle, speed, motor)
- **`sanbot_ros/Led`**: LED control (part, mode, duration, random)

### Optional Fields
Fields marked as "optional" use `0` to indicate "not specified". When publishing via MQTT, fields with value `0` are automatically excluded from the JSON payload for cleaner communication.

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
adb install -r ~/catkin_ws/src/sanbot-ros/Sanbot_OpenSDK_MQTT/librarydemod/build/outputs/apk/debug/librarydemod-debug.apk

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

1. Start MQTT and RTMP servers (recommended):

Follow the instructions in the [Docker Setup](#docker-setup) section to build and run the container that bundles both services.

```bash
cd docker_mqtt_rtmp
docker build -t sanbot_mqtt_rtmp .
docker run -d --name sanbot_mqtt_rtmp -p 1883:1883 -p 1935:1935 sanbot_mqtt_rtmp
```

2. Configure network:
   - Connect robot to same network as computer
   - Get computer's IP: `ifconfig`
   - Set computer's IP in robot's app

3. Launch ROS-MQTT-RTMP bridge:
```bash
roslaunch sanbot_ros bridge.launch
```

4. For testing if all connections were succesfull:
    - Custom gamepad control:
      ```bash
      rosrun sanbot_ros gamer_teleop.py
      ```
    - Standard ROS keyboard control:
      ```bash
      sudo apt install ros-noetic-teleop-twist-keyboard
      rosrun teleop_twist_keyboard teleop_twist_keyboard.py
      ```
    - Rviz to see imagem:
      ```bash
      rviz
      ```

### :white_check_mark: Available Topics

| **Topic**           | **Direction** | **Description**                            | **Message Type**     | **Example/Details**                       |
|---------------------|---------------|--------------------------------------------|----------------------|-------------------------------------------|
| `sanbot/touch`      | subscribe     | Touch sensor                                | `std_msgs/String`     | `'3 chest_right'`                          |
| `sanbot/pir`        | subscribe     | PIR presence sensor                         | `std_msgs/String`     | `'front 1'`                                |
| `sanbot/ir`         | subscribe     | Infrared distance sensor                    | `sensor_msgs/Range`   | Range in meters, radiation_type=INFRARED |
| `sanbot/voice_angle`| subscribe     | Angle of detected voice                     | `std_msgs/Int32`      | 0–360 degrees                            |
| `sanbot/obstacle`   | subscribe     | Obstacle detection sensor                   | `std_msgs/Bool`       | true / false                             |
| `sanbot/battery`    | subscribe     | Battery status                              | `sensor_msgs/BatteryState` | Percentage, status, etc.        |
| `sanbot/info`       | subscribe     | System information                          | `sanbot_ros/Info`     | Structured robot information             |
| `sanbot/gyro`       | subscribe     | Robot orientation                           | `sensor_msgs/Imu`     | Quaternion                               |
| `sanbot/speech`     | subscribe     | Speech recognition result                   | `std_msgs/String`     | `'Hello Sanbot'`                           |
| `/camera/image_raw` | subscribe     | RTMP camera stream                  | `sensor_msgs/Image`   | 1280x720 BGR8 30 fps                            |
| `ros/light`         | publish       | White forehead LED control                  | `std_msgs/UInt8`      | `data: 2`                                |
| `ros/move`          | publish       | Movement control (structured format)        | `sanbot_ros/Move`     | Direction, speed, distance, duration     |
| `ros/cmd_vel`       | publish       | Standard ROS velocity control               | `geometry_msgs/Twist` | Linear and angular velocities            |
| `ros/head`          | publish       | Head movement (structured format)           | `sanbot_ros/Head`     | Direction, angle, speed, motor           |
| `ros/led`           | publish       | Color LED control (structured format)       | `sanbot_ros/Led`      | Part, mode, duration, random             |
| `ros/speak`         | publish       | Text-to-speech                              | `std_msgs/String`     | `'Hello, I am Sanbot'`                   |

### 📝 Detailed Topic Descriptions

#### Sensor Topics (Subscribe)

##### `sanbot/touch`
Touch sensor events are published as space-separated strings.
- **Type**: std_msgs/String
- **Format**: `'part description'`
- **Part Description**:
  - 1-2: Chin (right, left)
  - 3-4: Chest (right, left)
  - 5-6: Back head (left, right)
  - 7-8: Back (left, right)
  - 9-10: Hand (left, right)
  - 11: Head middle
  - 12-13: Head front (right, left)
- **Example**:
  ```bash
  data: '3 chest_right'
  ```

##### `sanbot/pir`
PIR presence detection is published as a simple string.
- **Type**: std_msgs/String
- **Format**: `'location status'`
- **Locations**: "front" or "back"
- **Status**: 1 = presence detected, 0 = no presence
- **Example**:
  ```bash
  data: 'front 1'
  ```

##### `sanbot/ir` (sensor_msgs/Range)
Infrared distance sensor readings.
- **Frame ID**: `ir_sensor_X` (where X is the sensor number)
- **Range**: 0.0 to 0.64 meters
- **Radiation Type**: INFRARED

##### `sanbot/voice_angle` (std_msgs/Int32)
Sound source localization angle.
- **Value**: 0-360 degrees, indicating sound source direction

##### `sanbot/obstacle` (std_msgs/Bool)
Obstacle detection sensor status.
- **Value**: true = obstacle detected, false = no obstacle

##### `sanbot/battery` (sensor_msgs/BatteryState)
Battery status information.
- **Percentage**: 0.0 to 1.0 (0% to 100%)
- **Status**: FULL(4), CHARGING(1), or NOT_CHARGING(3)
- **Technology**: LION (Lithium-ion)
- **Capacity**: 20.0 Ah

##### `sanbot/info`
Provides system information about the robot in a structured format.
- **Type**: sanbot_ros/Info
- **Fields**:
  - robot_id: Unique robot identifier
  - ip: Robot's IP address
  - main_service_version: Main service version
  - android_version: Android OS version
  - device_model: Device model information
- **Example**:
  ```bash
  robot_id: "9ad1e4c2f058d23761c9b035de74a1fc"
  ip: "192.168.0.100"
  main_service_version: "1.2.0"
  android_version: "6.0.1"
  device_model: "0.1.118"
  ```

##### `sanbot/gyro`
Robot orientation in 3D space.
- **Type**: `sensor_msgs/Imu`
- **Data**: Orientation in quaternion (converted from roll, pitch, yaw angles)
- **Frame**: "base_link"

##### `sanbot/speech`
Speech recognition result is published as a simple string.
- **Type**: std_msgs/String
- **Format**: `'msg'`
- **Language**: English
- **Example**:
  ```bash
  data: 'Hello, how are you?'
  ```

##### `/camera/image_raw`
Camera stream decoded from an RTMP source.
- **Type**: `sensor_msgs/Image`
- **Source**: RTMP stream `rtmp://<robot_ip>:1935/live/stream`
- **Resolution**: 1280x720 pixels
- **FPS**: 30 frames per second

#### Control Topics (Publish)

##### `ros/light`
Control the white forehead LED.
- **Type**: `std_msgs/UInt8`
- **Values**: 
  - 0: Off
  - 1: Low brightness
  - 2: Medium brightness
  - 3: High brightness
- **Example**: 
  ```bash
  # Turn on LED at medium brightness
  rostopic pub /ros/light std_msgs/UInt8 "data: 2"
  
  # Turn off LED
  rostopic pub /ros/light std_msgs/UInt8 "data: 0"
  ```

##### `ros/move`
Controls robot movement using structured message format.
- **Type**: sanbot_ros/Move
- **Fields**:
  - direction (string): Movement direction
    - basic: "forward", "backward", "left", "right", "stop"
    - combined: "left_forward", "right_forward", "left_back", "right_back"
    - rotation: "turn_left", "turn_right"
  - speed (int8): Movement speed 1-10 (optional, 0 = not specified)
  - distance (int32): Movement distance in cm (optional, 0 = not specified)
  - duration (int32): Movement duration in seconds (optional, 0 = not specified)
- **Example**: 
  ```bash
  rostopic pub /ros/move sanbot_ros/Move "direction: 'forward'
  speed: 7
  distance: 0
  duration: 2"
  ```

##### `ros/cmd_vel`
Standard ROS velocity control.
- **Type**: `geometry_msgs/Twist`
- **Fields**:
  - linear.x: Forward/backward velocity (-1.0 to 1.0)
  - linear.y: Left/right velocity (-1.0 to 1.0)
  - angular.z: Rotation velocity (-1.0 to 1.0)
- **Note**: Preferred method for smooth continuous movement
- **Example**: 
  ```bash
  rostopic pub /ros/cmd_vel geometry_msgs/Twist "linear:
    x: 0.5
    y: 0.8
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
  ```

##### `ros/head`
Controls head movement using structured message format.
- **Type**: sanbot_ros/Head
- **Fields**:
  - direction (string): "up", "down", "left", "right"
  - angle (int16): Movement angle in degrees
  - speed (int8): Speed percentage 1-100 (optional, 0 = not specified)
  - motor (int8): Motor selection (optional, 0 = not specified)
    - 1: neck
    - 2: vertical
    - 3: horizontal
- **Example**: 
  ```bash
  rostopic pub /ros/head sanbot_ros/Head "direction: 'up'
  angle: 30
  speed: 80
  motor: 0"
  ```

##### `ros/led`
Controls color LEDs using structured message format.
- **Type**: sanbot_ros/Led
- **Fields**:
  - part (string): LED part selection
    - "all_head", "all_hand": All head/hand LEDs
    - "head_left", "head_right": Individual head LEDs
    - "arm_left", "arm_right": Individual hand LEDs
  - mode (string): LED mode
    - Normal: "white", "red", "green", "blue", "yellow", "purple", "pink"
    - Flicker: "flicker_white", "flicker_red", "flicker_green", "flicker_pink", "flicker_purple", "flicker_blue", "flicker_yellow", "flicker_random"
  - duration (int8): Duration in seconds
  - random (int8): Random mode (0 or 1)
- **Example**: 
  ```bash
  rostopic pub /ros/led sanbot_ros/Led "part: 'all_head'
  mode: 'blue'
  duration: 5
  random: 1"
  ```

##### `ros/speak`
Triggers text-to-speech using a plain string.
- **Type**: std_msgs/String
- **Format**: `'text'`
- **Language**: English
- **Volume**: Fixed at system volume level
- **Example**: 
  ```bash
  rostopic pub /ros/speak std_msgs/String "data: 'Hello, I am Sanbot!'"
  ```

## 🐳 Docker Setup

This repository provides a lightweight Docker image that starts both a **Mosquitto MQTT broker** (port **1883**) and an **NGINX RTMP server** (port **1935**) in a single container. This removes the need to install and configure these services manually on the host machine, which can be tricky.

### Install Docker

```bash
# Install prerequisites
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the stable repository (replace "$(lsb_release -cs)" if needed)
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine, CLI and Buildx/Compose plugins
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Start and enable the service
sudo systemctl enable --now docker

# (optional) run docker without sudo
sudo usermod -aG docker $USER
newgrp docker
```

This installs the latest stable Docker **Engine** directly from Docker's repository. For other distributions see the guide at <https://docs.docker.com/engine/install/>.

### Build the image

```bash
cd docker_mqtt_rtmp
docker build -t sanbot_mqtt_rtmp .
```

### Run the container

```bash
docker run -d --name sanbot_mqtt_rtmp -p 1883:1883 -p 1935:1935 sanbot_mqtt_rtmp
```

The container will stay in the foreground (managed by Docker) and expose both services:

* **MQTT broker**: `mqtt://<host-ip>:1883`
* **RTMP server**: `rtmp://<host-ip>:1935/live/stream`

### Container management

Stop the container:

```bash
docker stop sanbot_mqtt_rtmp
```

Remove the container:

```bash
docker rm sanbot_mqtt_rtmp
```

Check logs:

```bash
docker logs -f sanbot_mqtt_rtmp
```

With the container running, you can proceed to launch the ROS–MQTT bridge as described in the [Running](#running) section.

## :handshake: Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## :page_facing_up: License

This project is licensed under the MIT License - see the LICENSE file for details.

## :envelope: Contact

For support or questions, please open an issue or contact the maintainer at lucashudson.eng@gmail.com
