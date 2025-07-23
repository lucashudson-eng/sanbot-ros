# sanbot-ros

Minimal driver, simulation and auxiliary tools for interacting with **Sanbot's robot** from ROS.

## Repository Layout

| Path | Brief description |
|------|-------------------|
| `sanbot_ros/` | Core ROS1 package: custom messages, MQTT/RTMP bridge launch files and helper scripts. |
| `sanbot_nano_urdf/` | Complete URDF model, meshes and Gazebo launch files for simulation. |
| `sanbot_nano_solidworks/` | Original SolidWorks CAD files of the robot for mechanical reference. |
| `docker_mqtt_rtmp/` | Lightweight Docker image that starts a Mosquitto MQTT broker (1883) and an NGINX RTMP server (1935) for local robot control. |
| `Sanbot_OpenSDK20191118/` | Unmodified official Android demo app shipped by Qihan Tech (reference only). |
| `Sanbot_Ros_Bridge/` | Custom Android application that exposes Sanbot SDK functions over MQTT and RTMP to integrate with ROS. |

Please refer to the **README.md** inside each directory for in-depth setup and usage instructions.

## Quick Start

1. Clone into a Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/lucashudson-eng/sanbot-ros.git
   ```
2. Build and source:
   ```bash
   cd ~/catkin_ws && catkin_make && source devel/setup.bash
   ```
3. Follow the per-folder READMEs to run simulation (`sanbot_nano_urdf/`) or connect to real hardware (`sanbot_ros/`) through the bridge app (`Sanbot_Ros_Bridge/`).

## Contributing

Pull-requests are welcome! For major changes open an issue first to discuss the proposal.

## License

This project is licensed under the MIT License – see the [`LICENSE`](LICENSE) file for details.
