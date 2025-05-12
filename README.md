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

A custom version based on the official app, adapted to receive commands via MQTT.

The graphical interface is limited to MQTT configuration and connection management.

It can run independently with any MQTT broker, but it is primarily intended to work with the ROS package included in this repository.

## :robot: ROS Package sanbot_ros

### :white_check_mark: Available Topics

#### Infraed sensors (sanbot/ir)

#### PIR Presence sensors (sanbot/pir)

#### Touch sensors (sanbot/touch)

#### Angle of detected voice (sanbot/voice_angle)

#### Battery and charging status (sanbot/battery)

#### System Infos (sanbot/info)

#### Obstacle Sensor (sanbot/obstacle)

#### Control of forehead white light (ros/light)

#### Wheel movement control (ros/move)

#### Head movement control (ros/head)

#### Control of head and arm LEDs (ros/led)

### HD Cmaera

### :no_entry_sign: Curently Not Working

#### Speach and Speech Regognizer

#### Arms movement control

#### Ultrasonic sensors

### :heavy_plus_sign: Additional Features

#### Command to play a fixed mp3 audio (ros/mp3)

Plays a .mp3 file upon receiving any message from the topic ros/mp3.

TODO: Implement functionality to receive text, convert it to speech, and play the generated audio file.

:bulb: Feel free to contribute with bug fixes, improvements, or new features!