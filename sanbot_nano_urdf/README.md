# Sanbot Nano – URDF Package

This package provides the kinematic, dynamic and visual model of **Sanbot Nano** for Gazebo/ROS simulations.

---

## 1. How to launch the simulation

```bash
# Inside your already built ROS workspace
roslaunch sanbot_nano_urdf gazebo.launch   # Spawns the robot in an empty world
```

---

## 2. Published / Subscribed topics

| Sub-system | Topic | Message type | Note |
|------------|-------|--------------|------|
| **Chin camera** | `/camera_chin/image_raw` | `sensor_msgs/Image` | RGB 1280×720 @ 30 Hz |
| **Forehead camera** | `/camera_forehead/image_raw` | `sensor_msgs/Image` | RGB 1280×720 @ 30 Hz |
| **Depth camera** | `/camera_depth/depth/image_raw` | `sensor_msgs/Image` | Depth 1280×720 @ 30 Hz |
| **IMU** | `/imu` | `sensor_msgs/Imu` | Rigid body `base_link` |
| **Infra-red** | `/ir_1 … /ir_17` | `sensor_msgs/Range` | 17 sensors distributed on the body (see URDF) |
| **Velocity** | `/cmd_vel` (input) | `geometry_msgs/Twist` | Planar movement (plugin `planar_move`) |

Tip: use `rostopic echo /ir_3` or `rqt_image_view` to quickly visualize sensors.

---

## 3. Planar movement (plugin *planar_move*)

Locomotion in XY and yaw is commanded by publishing to **`/cmd_vel`**. Example:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

> Originally individual wheel control with inverse kinematics from `omni_inverse_kinematics.py` was intended, but due to friction the robot motion was unsatisfactory.

---

## 4. Actuated joints

| Joint | Description | Limits (rad) | Controller |
|-------|-------------|--------------|------------|
| `head_pan` | Horizontal head rotation | −1.57 ↔ +1.57 | `head_controller` |
| `head_tilt` | Vertical head inclination | 0.00 ↔ 0.656 | `head_controller` |
| `wing_left` | Left wing | −1.57 ↔ +3.14 | `wing_left_controller` |
| `wing_right` | Right wing | −1.57 ↔ +3.14 | `wing_right_controller` |

All joints use **`JointTrajectoryController`** (see `config/joint_trajectory_controller.yaml`). Example commands:

• Combined pan & tilt:

```bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "
  joint_names: ['head_pan', 'head_tilt']
  points:
  - positions: [0.5, 0.3]
    time_from_start: {secs: 2}
"
```

• Left wing flap:

```bash
rostopic pub /wing_left_controller/command trajectory_msgs/JointTrajectory "
  joint_names: ['wing_left']
  points:
  - positions: [1.0]
    time_from_start: {secs: 1}
"
```

---

## 5. Directory structure

```
sanbot_nano_urdf/
├── config/           # Controller parameters
├── launch/           # *.launch files for Gazebo
├── meshes/           # STL geometries
├── textures/         # Colored DAE files
├── scripts/          # Python examples & utilities
└── urdf/             # Main URDF model
```
