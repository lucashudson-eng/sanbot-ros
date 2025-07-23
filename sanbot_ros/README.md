 z# sanbot_ros

**sanbot_ros** is the core ROS package that lets you interface with a **Sanbot** robot through standard ROS topics by transparently bridging them to MQTT control/telemetry and RTMP video streaming.  
It bundles custom message definitions, a ready-to-use launch file, helper conversion scripts and a requirements file so you can get up and running in minutes.

---
## :bookmark_tabs: Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Running](#running)
4. [Available Topics](#available-topics)
5. [Detailed Topic Descriptions](#detailed-topic-descriptions)
6. [License](#license)

---
## Prerequisites

| Target | Software | Minimum version | Purpose |
|--------|----------|-----------------|---------|
| **PC (Ubuntu)** | ROS Noetic | — | Core ROS1 framework |
|                | Docker | 20.10 | Runs the MQTT + RTMP container |
|                | Python 3 | 3.8 | For helper scripts |

---
## Installation

1. Clone this repository into a Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/lucashudson-eng/sanbot-ros.git
   ```
2. Install Python dependencies for the package:
   ```bash
   pip install -r ~/catkin_ws/src/sanbot-ros/sanbot_ros/requirements.txt
   ```
3. Build the workspace and source the setup script:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---
## Running

1. **Start the MQTT + RTMP broker** (once per boot):
   ```bash
   cd ~/catkin_ws/src/sanbot-ros/docker_mqtt_rtmp
   docker build -t sanbot_mqtt_rtmp .            # first time only
   docker run -d --name sanbot_mqtt_rtmp --restart unless-stopped \
     -p 1883:1883 -p 1935:1935 sanbot_mqtt_rtmp
   ```
2. **Connect the robot & PC to the same network** and note the PC IP (`hostname -I | awk '{print $1}'`).
3. **Launch the ROS bridge** on the PC:
   ```bash
   roslaunch sanbot_ros bridge.launch
   ```
4. **Install & open the Android bridge app** (`Sanbot_Ros_Bridge`) on the robot.

   Enter the PC IP in *Server IP* and tap *Connect*.  

   Enable the *Camera* toggle if you want live video.
5. Confirm that topics are visible on the PC:
   ```bash
   rostopic list
   ```

---
## :white_check_mark: Available Topics

| Topic | Direction | Description | Message type |
|-------|-----------|-------------|--------------|
| `sanbot/touch` | robot → ROS | Touch sensor events | `std_msgs/String` |
| `sanbot/pir` | robot → ROS | PIR presence sensor | `std_msgs/String` |
| `sanbot/ir` | robot → ROS | Infra-red distance | `sensor_msgs/Range` |
| `sanbot/voice_angle` | robot → ROS | Sound source angle | `std_msgs/Int32` |
| `sanbot/obstacle` | robot → ROS | Obstacle detection | `std_msgs/Bool` |
| `sanbot/battery` | robot → ROS | Battery level & status | `sensor_msgs/BatteryState` |
| `sanbot/info` | robot → ROS | System information | `sanbot_ros/Info` |
| `sanbot/gyro` | robot → ROS | Orientation (IMU) | `sensor_msgs/Imu` |
| `sanbot/speech` | robot → ROS | Speech recognition | `std_msgs/String` |
| `/camera/image_raw` | robot → ROS | RTMP camera stream | `sensor_msgs/Image` |
| `ros/light` | ROS → robot | White forehead LED | `std_msgs/UInt8` |
| `ros/move` | ROS → robot | Discrete movement commands | `sanbot_ros/Move` |
| `ros/cmd_vel` | ROS → robot | Continuous velocity | `geometry_msgs/Twist` |
| `ros/joints` | ROS → robot | Head & wing servos | `trajectory_msgs/JointTrajectory` |
| `ros/led` | ROS → robot | Color LEDs | `sanbot_ros/Led` |
| `ros/speak` | ROS → robot | Text-to-speech | `std_msgs/String` |

[Click here](#detailed-topic-descriptions) for in-depth payload definitions.

---
## Detailed Topic Descriptions

<details>
<summary><strong>Sensor Topics (robot → ROS)</strong></summary>

### `sanbot/touch`
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

### `sanbot/pir`
PIR presence detection is published as a simple string.
- **Type**: std_msgs/String
- **Format**: `'location status'`
- **Locations**: "front" or "back"
- **Status**: 1 = presence detected, 0 = no presence
- **Example**:
  ```bash
  data: 'front 1'
  ```

### `sanbot/ir` (sensor_msgs/Range)
Infrared distance sensor readings.
- **Frame ID**: `ir_sensor_X` (where X is the sensor number)
- **Range**: 0.0 to 0.64 meters
- **Radiation Type**: INFRARED

### `sanbot/voice_angle` (std_msgs/Int32)
Sound source localization angle.
- **Value**: 0-360 degrees, indicating sound source direction

### `sanbot/obstacle` (std_msgs/Bool)
Obstacle detection sensor status.
- **Value**: true = obstacle detected, false = no obstacle

### `sanbot/battery` (sensor_msgs/BatteryState)
Battery status information.
- **Percentage**: 0.0 to 1.0 (0% to 100%)
- **Status**: FULL(4), CHARGING(1), or NOT_CHARGING(3)
- **Technology**: LION (Lithium-ion)
- **Capacity**: 20.0 Ah

### `sanbot/info`
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

### `sanbot/gyro`
Robot orientation in 3D space.
- **Type**: `sensor_msgs/Imu`
- **Data**: Orientation in quaternion (converted from roll, pitch, yaw angles)
- **Frame**: "base_link"

### `sanbot/speech`
Speech recognition result is published as a simple string.
- **Type**: std_msgs/String
- **Format**: `'msg'`
- **Language**: English
- **Example**:
  ```bash
  data: 'Hello, how are you?'
  ```

### `/camera/image_raw`
Camera stream decoded from an RTMP source.
- **Type**: `sensor_msgs/Image`
- **Source**: RTMP stream `rtmp://<robot_ip>:1935/live/stream`
- **Resolution**: 1280x720 pixels
- **FPS**: 30 frames per second

</details>

<details>
<summary><strong>Control Topics (ROS → robot)</strong></summary>

### `ros/light`
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

### `ros/move`
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

### `ros/cmd_vel`
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

### `ros/joints`
Set a angle to any robot joints using trajectory message format with joint name, angle and velocity.

- **Type**: trajectory_msgs/JointTrajectory
- **Fields**:
  - joint_names: List of joint names to control
  - points: List of trajectory points containing:
    - positions: Joint angles in radians
    - velocities: Joint velocities (0.0 to 1.0, converted to 0-100%)
- **Available Joints**:
  - `head_pan`: Horizontal head rotation (left/right) - Range: -90° to +90° (-1.57 to +1.57 rad)
  - `head_tilt`: Vertical head rotation (up/down) - Range: 0° to +37° (0 to +0.65 rad)
  - `wing_left`: Left wing movement (not implemented, not found in SDK) - Range: -90° to +180° (-1.57 to +3.14 rad)
  - `wing_right`: Right wing movement (not implemented, not found  in SDK) - Range: -90° to +180° (-1.57 to +3.14 rad)
- **Example**: 
  ```bash
  # Rotate head left to 45 degrees (~0.79 radians)
  rostopic pub /ros/joints trajectory_msgs/JointTrajectory "joint_names: ['head_pan']
  points:
  - positions: [-0.79]
    velocities: [0.5]"
  
  # Tilt head up to 29 degrees (~0.52 radians)
  rostopic pub /ros/joints trajectory_msgs/JointTrajectory "joint_names: ['head_tilt']
  points:
  - positions: [0.52]
    velocities: [0.8]"
  ```

### `ros/led`
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

### `ros/speak`
Triggers text-to-speech using a plain string with TTS engine selection.
- **Type**: std_msgs/String
- **Format**: `'text'`
- **TTS Engine Options**: 
  - SDK TTS: Uses Sanbot's built-in TTS engine
  - Android TTS: Uses Android's system TTS with multiple language support
- **Languages**: Multiple languages available through Android TTS (English, Portuguese, Spanish, etc.)
- **Volume**: Fixed at system volume level
- **Example**: 
  ```bash
  rostopic pub /ros/speak std_msgs/String "data: 'Hello, I am Sanbot!'"
  ```

</details>

---
## Contributing

Pull-requests are welcome! For major changes please open an issue first to discuss what you would like to change.

---
## License

This package is distributed under the **MIT License**. See the [`LICENSE`](../LICENSE) file for full text. 