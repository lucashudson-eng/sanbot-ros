# sanbot-ros

A ROS driver for interacting with Sanbots's robots.

## APPs

To run either of the two apps, you must have ADB (Android Debug Bridge) installed:

```
sudo apt install android-tools-adb
```

The APK installation and debugging will be done through ADB.

There are two ways to connect to the Sanbot: via *USB* or *Wi-Fi*. We recommend using the *Wi-Fi* connection because the USB port is located on the back of the robot’s head—connecting via USB can risk damaging the cable or connector when the head moves.

### Connecting via Wi-Fi

First, make sure the robot is connected to the same Wi-Fi network as your PC.
Then, you’ll need to *plug in the USB cable temporarily* to open the ADB port. :warning: *Watch out for head movement!*

Once the USB is connected, run the following commands to retrieve the robot's IP and enable ADB over Wi-Fi:

```
adb shell ip addr show wlan0
adb tcpip 5555
```

Then, connect to the robot using the retrieved IP address:

```
adb connect 192.168.xxx.xxx:555
```

### Useful ADB Commands

```
adb install -r -d path/to/your_app.apk                          # Install APK
adb uninstall com.grin.sanbotopensdkmqtt                        # Uninstall app (required to install a different version)
adb shell am force-stop com.grin.sanbotopensdkmqtt              # Force close the app
adb shell am start -n com.grin.sanbotopensdkmqtt/.MainActivity  # Launch MainActivity
```

Installing the app via ADB does not automatically launch it. To run the app, make sure to execute the last ADB command to start it manually.

### Sanbot_OpenSDK20191118

This app is the official example provided by the [Sanbot's Developoer Center](http://blue.sanbotcloud.com:98/dev/docs/robot.html). It demonstrates all of the robot’s available functionalities.

### Sanbot_OpenSDK_MQTT

This app is based on the official example provided by Sanbot’s company, but it has been adapted to receive commands via *MQTT*. The only graphical interface available is for configuring and managing the MQTT connection.

It can be used independently with any MQTT broker, but it is primarily intended to be used alongside the ROS package included in this repository, which is its main purpose.

## Package sanbot_ros

