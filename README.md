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

| **Topic**           | **Direction** | **Description**                            | **Message Type**     | **Example/Details**                       |
|---------------------|---------------|--------------------------------------------|----------------------|-------------------------------------------|
| `sanbot/touch`      | subscribe     | Touch sensor                                | `std_msgs/String`     | `'3 chest_right'`                          |
| `sanbot/pir`        | subscribe     | PIR presence sensor                         | `std_msgs/String`     | `'front 1'`                                |
| `sanbot/ir`         | subscribe     | Infrared distance sensor                    | `sensor_msgs/Range`   | Range in meters, radiation_type=INFRARED |
| `sanbot/voice_angle`| subscribe     | Angle of detected voice                     | `std_msgs/Int32`      | 0–360 degrees                            |
| `sanbot/obstacle`   | subscribe     | Obstacle detection sensor                   | `std_msgs/Bool`       | true / false                             |
| `sanbot/battery`    | subscribe     | Battery status                              | `sensor_msgs/BatteryState` | Percentage, status, etc.        |
| `sanbot/info`       | subscribe     | System information                          | `std_msgs/String`     | `'9ad1e4c2f058d23761c9b035de74a1fc 192.168.0.100 1.2.0 6.0.1 0.1.118'`                  |
| `sanbot/camera`     | subscribe     | HD camera video stream                      | `sensor_msgs/Image`   | 1280x720 BGR8                            |
| `sanbot/gyro`       | subscribe     | Robot orientation                           | `sensor_msgs/Imu`     | Quaternion                               |
| `sanbot/speech`     | subscribe     | Speech recognition result                   | `std_msgs/String`     | `'Hello Sanbot'`                           |
| `ros/light`         | publish       | White forehead LED control                  | `std_msgs/UInt8`      | `data: 2`                                |
| `ros/move`          | publish       | Movement control (string format)            | `std_msgs/String`     | `'forward 6 100 2'`                      |
| `ros/cmd_vel`       | publish       | Standard ROS velocity control               | `geometry_msgs/Twist` | Linear and angular velocities            |
| `ros/head`          | publish       | Head movement (string format)               | `std_msgs/String`     | `'up 30 80 2'`                           |
| `ros/led`           | publish       | Color LED control (string format)           | `std_msgs/String`     | `'all_head blue 10 1'`                   |
| `ros/speak`         | publish       | Text-to-speech (string only)                | `std_msgs/String`     | `'Hello, I am Sanbot'`                   |

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
Provides system information about the robot and automatically triggers the video stream when a valid IP is received.
- **Type**: std_msgs/String
- **Format**: `'robot_id ip main_service_version android_version device_model'`
- **Example**:
  ```bash
  data: '9ad1e4c2f058d23761c9b035de74a1fc 192.168.0.100 1.2.0 6.0.1 0.1.118'
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
Speech recognition result is published as a simple string.
- **Type**: std_msgs/String
- **Format**: `'msg'`
- **Language**: English
- **Example**:
  ```bash
  data: 'Hello, how are you?'
  ```

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
Controls robot movement using space-separated string input.
- **Type**: std_msgs/String
- **Format**: `'direction [speed] [distance] [duration]'`
- **c**:
  - direction:
    - basic: "forward", "backward", "left", "right", "stop" (linear.x or linexar.y)
    - combined: "left_forward", "right_forward", "left_back", "right_back" (linear.x with linear.y)
    - rotation: "turn_left", "turn_right" (angular.z)
  - speed: Movement speed (1-10) (optional, defautlt to 5)
  - distance: Movement distance in cm/degree (optional, defautl to forever)
  - duration: Movement duration in seconds (optional, defautlt to forever, priority in relation to distance)
- **Example**: 
  ```bash
  rostopic pub /ros/move std_msgs/String "data: 'forward 7 0 2'"
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
Controls head movement using space-separated string input.
- **Type**: std_msgs/String
- **Format**: `'direction [angle] [speed] [motor]'`
- **Parameters**: 
  - direction: "up", "down", "left", "right"
  - angle: in degrees (optional, default to 10)
  - speed: speed percentage (optional, default to 50)
  - motor: 1 (neck), 2 (vertical), 3 (horizontal) (optional, default to 1)
- **Example**: 
  ```bash
  rostopic pub /ros/head std_msgs/String "data: 'up 30 80'"
  ```

##### `ros/led`
Controls color LEDs using space-separated string input.
- **Type**: std_msgs/String
- **Format**: `'part mode [duration] [random]'`
- **Parameters**: 
  - part:
    - "all_head", "all_hand": All head/hand LEDs
    - "head_left", "head_right": Individual head LEDs
    - "arm_left", "arm_right": Individual hand LEDs
  - mode:
    - Normal mode: "white", "red", "green", "blue", "yellow", "purple", "pink"
    - Flicker mode: "flicker_white", "flicker_red", "flicker_green", "flicker_pink", "flicker_purple", "flicker_blue", "flicker_yellow", "flicker_random"
  - duration: seconds (optional, default to 1)
  - random: 0 or 1 (optional, default to 1)
- **Example**: 
  ```bash
  rostopic pub /ros/led std_msgs/String "data: 'all_head blue 5'"
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

## :handshake: Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## :page_facing_up: License

This project is licensed under the MIT License - see the LICENSE file for details.

## :envelope: Contact

For support or questions, please open an issue or contact the maintainer at lucashudson.eng@gmail.com
