# sanbot-ros

A ROS driver for interacting with Sanbots's robots.

## :iphone: APPs

To use either of the provided apps, you must have ADB (Android Debug Bridge) installed:

```
sudo apt install android-tools-adb
```

APK installation and debugging will be managed through ADB.

There are two ways to connect to the Sanbot: via *USB* or *Wi-Fi*. 

We recommend using the *Wi-Fi* connection because the USB port is located on the back of the robot’s head—connecting via USB can risk damaging the cable or connector when the head moves.

### :electric_plug: Connecting via Wi-Fi

First, make sure the robot is connected to the same Wi-Fi network as your PC.
Then, you’ll need to *plug in the USB cable temporarily* to open the ADB port. :warning: *Watch out for head movement!*

Once the USB is connected, run the following commands to retrieve the robot's IP and enable ADB over Wi-Fi:

```
adb shell ip addr show wlan0
adb tcpip 5555
```

Then, connect to the robot using the retrieved IP address:

```
adb connect 192.168.xxx.xxx:5555
```

### :wrench: Useful ADB Commands

```
adb install ~/catkin_ws/src/sanbot-ros/Sanbot_OpenSDK_MQTT/librarydemod/build/outputs/apk/debug/librarydemod-debug.apk                          # Install APK
adb uninstall com.grin.sanbotopensdkmqtt                        # Uninstall app (required to install a different version)
adb shell am force-stop com.grin.sanbotopensdkmqtt              # Force close the app
adb shell am start -n com.grin.sanbotopensdkmqtt/.MainActivity  # Launch MainActivity
```

Installing the app via ADB does not automatically launch it. To run the app, make sure to execute the last ADB command to start it manually.

### :package: Sanbot_OpenSDK20191118

This is the official example app of using the SDK provided by the [Sanbot's Developoer Center](http://blue.sanbotcloud.com:98/dev/docs/robot.html).

It showcases all available functionalities of the robot.

### :package: Sanbot_OpenSDK_MQTT

A custom version based on the official app, adapted to receive commands and senf informations via MQTT.

The graphical interface is limited to MQTT configuration and connection management.

It can run independently with any MQTT broker, but it is primarily intended to work with the ROS package included in this repository.

## :robot: ROS Package sanbot_ros

### Instalation

### Running

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
| `sanbot/camera`                  | subscribe       | Real-time image stream from robot's HD camera (ROS or Mjpeg via through IP:8080) | - |
| `ros/light`                      | publish         | Control the white forehead LED                              | `{"white": 2}` |
| `ros/move`                       | publish         | Move the robot                                              | `{"direction": "forward", "speed": 6, "distance": 30, "duration": 3}` |
| `ros/head`                       | publish         | Head movement (direction, angle, motor, speed)              | `{"direction": "up", "angle": 10, "motor": 1, "speed": 50}` |
| `ros/led`                        | publish         | Color LED control                                           | `{"part": "all_head", "mode": "blue", "duration": 2, "random": 1}` |

### :no_entry_sign: Curently Not Working

I'm not sure whether the issue is due to the only robot I have for testing being partially non-functional from not having been unused for too long, or if the SDK version simply isn't fully compatible with this particular robot model/version. Either way, some features just didn't work — not even with the official example app provided by Sanbot's company. Since I wasn't able to test them properly, I chose not to implement those features.

- Speach and Speech Regognizer

- Arms movement control

- Ultrasonic sensors

:bulb: Feel free to contribute with bug fixes, improvements, or new features!