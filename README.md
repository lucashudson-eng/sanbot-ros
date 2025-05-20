# sanbot-ros

A ROS driver for interacting with Sanbot robots via MQTT and ROS topics.

## :iphone: Android APPs

To use either of the provided apps, make sure ADB (Android Debug Bridge) is installed:

```
sudo apt install android-tools-adb
```

APK installation and debugging will be managed through ADB.

There are two ways to connect to the Sanbot: via *USB* or *Wi-Fi*. 

We recommend using the *Wi-Fi* connection because the USB port is located on the back of the robot’s head—connecting via USB can risk damaging the cable or connector when the head moves.

### :electric_plug: Connecting via Wi-Fi

First, ensure the robot is connected to the same Wi-Fi network as your PC.  
Then, temporarily connect a USB cable to open the ADB port. :warning: *Be careful — the head might move!*

Once the USB is connected, run the following commands to retrieve the robot's IP and enable ADB over Wi-Fi:

```
adb shell ip addr show wlan0
adb tcpip 5555
```

Then, connect to the robot using the retrieved IP address:

```
adb connect <ROBOT_IP>:5555
```

### :wrench: Useful ADB Commands

```
adb install ~/catkin_ws/src/sanbot-ros/Sanbot_OpenSDK_MQTT/librarydemod/build/outputs/apk/debug/librarydemod-debug.apk # Install APK
adb uninstall com.grin.sanbotmqtt                        # Uninstall app (required to install a different version)
adb shell am force-stop com.grin.sanbotmqtt              # Force close the app
adb shell am start -n com.grin.sanbotmqtt/.MainActivity  # Launch MainActivity
adb logcat                                               # Debug APP
```

Installing the app via ADB does not automatically launch it. To run the app, make sure to execute the last ADB command to start it manually.

### :package: Sanbot_OpenSDK20191118

This is the official example app of using the SDK provided by the [Sanbot's Developoer Center](http://blue.sanbotcloud.com:98/dev/docs/robot.html).

It showcases all available functionalities of the robot.

### :package: Sanbot_OpenSDK_MQTT

A custom version based on the official app, adapted to receive commands and send information via MQTT.

The graphical interface is limited to MQTT configuration and connection management.

It can run independently with any MQTT broker, but it is primarily intended to work with the ROS package included in this repository.

## :robot: ROS Package sanbot_ros

### Instalation

Clone the repository:

```
cd ~/catkin_ws/src/

git clone https://github.com/lucashudson-eng/sanbot-ros.git
```

Install Python dependencies:

```
pip install -r ~/catkin_ws/src/sanbot-ros/sanbot_ros/requirements.txt
```

Build the workspace and source it:

```
cd ~/catkin_ws/

catking build

source ~/catkin_ws/devel/setup.bash
```

### Running

Start an MQTT broker (Mosquitto):

```
mosquitto -p 1883 -d
```

Connect the robot to the same network as your computer.

Get your computer's IP with:

```
ifconfig
```

Set that IP in the app's IP field on the robot.

Launch the bridge between ROS and MQTT:

```
roslaunch sanbot_ros bridge.launch
```

Optionally, control the robot using:

```
rosrun sanbot_ros gamer_teleop.py
```

Or use the official ROS package:

```
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
| `sanbot/obstacle`                | subscribe       | Obstacle detection sensor (only 1 if obstacle)                                  | `{"status": 1}` |
| `sanbot/battery`                 | subscribe       | Battery level and charging status                           | `{"battery_level": 82, "battery_status": "charging_by_wire"}` |
| `sanbot/info`                    | subscribe       | System information                                          | `{"robot_id": "...", "ip": "...", "main_service_version": "...", "android_version": "...", "device_model": "..."}` |
| `sanbot/camera`                  | subscribe       | Real-time image stream from robot's HD camera (ROS or Mjpeg via through IP:8080) | `sensor_msgs/Image` |
| `ros/light`                      | publish         | Control the white forehead LED                              | `{"white": 2}` |
| `ros/move`                       | publish         | Move the robot                                              | `{"direction": "forward", "speed": 6, "distance": 30, "duration": 3}` |
| `ros/cmd_vel`                    | publish         | Standard ROS velocity control topic (only ROS)              | `geometry_msgs/Twist` |
| `ros/head`                       | publish         | Head movement (direction, angle, motor, speed)              | `{"direction": "up", "angle": 10, "motor": 1, "speed": 50}` |
| `ros/led`                        | publish         | Color LED control                                           | `{"part": "all_head", "mode": "blue", "duration": 2, "random": 1}` |
| `ros/speak`                      | publish         | Speak a custom message                                      | `{"msg": "Hello World"}` |

### :no_entry_sign: Currently Not Working

I'm not sure whether the issue stems from the fact that the only robot I have for testing has become partially non-functional due to extended inactivity, or if the SDK version is simply not fully compatible with this specific robot model — or perhaps even with this outdated software version. 

In any case, certain features did not work as expected — not even when using the official example app provided by Sanbot’s company. Because I wasn’t able to properly test these features, I decided not to implement them for now.

- Speech recognizer

- Arms movement control

- Ultrasonic sensors

- Gyroscope

:bulb: Feel free to contribute with bug fixes, improvements, or new features!
