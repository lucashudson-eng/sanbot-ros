# Sanbot Nano URDF – Quick Test Guide

This guide explains how to publish commands to the pre-configured ROS topics in *sanbot_nano_urdf* for a quick check of the available features: two head joints, two wings, planar base movement, three cameras and an IMU.

---

## 1. Prerequisites

1. Build and source your workspace:
   ```bash
   cd ~/catkin_ws && catkin build
   source devel/setup.bash
   ```
2. Launch the robot (RViz display **or** Gazebo simulation) in a new terminal:
   ```bash
   roslaunch sanbot_nano_urdf sanbot_nano.launch
   ```
3. In every additional terminal, remember to *source* the workspace again before publishing commands.

---

## 2. Quick Degree → Radian Conversion
Most examples use radians. For a rough conversion keep in mind:

| Degrees | Radians |
|-------:|---------:|
| 30°    | 0.524 | 
| 45°    | 0.785 |
| 90°    | 1.571 |

Need another value? Use:
```bash
python -c 'import math; print(math.radians(DEGREES))'
```

---

## 3. Head Control (`head_pan` & `head_tilt`)

The controller is `JointTrajectory`. Even if you want to move only one joint, you must publish **both** in the same message. Example: tilt the head **+30°** (0.524 rad) while keeping the neck still:

```bash
rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}   # will be replaced by 'now'
  frame_id: ''
joint_names: ['head_pan', 'head_tilt']
points:
- positions: [0.0, 0.523599]   # [pan, tilt]
  velocities: [0.0, 0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

More quick tests:

| Action | `positions` | Comment |
|--------|-------------|---------|
| Pan +45° | `[0.785, 0.0]` | tilt stays at zero |
| Pan +45° & Tilt +20° | `[0.785, 0.349]` | combined motion |
| Back to neutral | `[0.0, 0.0]` | |

> ⚠️ Respect the limits defined in `scripts/joint_limits_test.py`:
> * `head_pan`: ±90°
> * `head_tilt`: 0° to +37.6°

---

## 4. Wing Control

### 4.1 Left Wing (`wing_left`)
```bash
rostopic pub -1 /wing_left_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}
joint_names: ['wing_left']
points:
- positions: [1.0]        # ~57° up
  velocities: [0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

### 4.2 Right Wing (`wing_right`)
```bash
rostopic pub -1 /wing_right_controller/command trajectory_msgs/JointTrajectory \
"header:
  stamp: {secs: 0, nsecs: 0}
joint_names: ['wing_right']
points:
- positions: [1.0]
  velocities: [0.0]
  time_from_start: {secs: 2, nsecs: 0}"
```

*Use `positions: [0.0]` to return to the resting position.*

---

## 5. Planar Base Movement

2-D omnidirectional movement is already enabled in Gazebo through the `libgazebo_ros_planar_move.so` plugin.  
Simply publish `geometry_msgs/Twist` messages to **/cmd_vel**:

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.3   # m/s
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"  # rad/s
```

> 💡 The script `scripts/omni_inverse_kinematics.py` contains the inverse kinematics implementation for the three wheels and can be used on real hardware later on. In the current simulation it is **not required** (nor executed) because the plugin automatically converts `cmd_vel` into wheel velocities.

---

## 6. Camera Visualization

The robot features **three** cameras already configured in the simulation:

| Frame | Type | Image topic |
|-------|------|-------------|
| `camera_chin_link` | RGB | `/camera_chin/image_raw` |
| `camera_forehead_link` | RGB | `/camera_forehead/image_raw` |
| `camera_depth_link` | Depth | `/camera_depth/depth/image_raw` |

1. List the available topics:
   ```bash
   rostopic list | grep _image
   ```
2. Visualise any camera in real time with **rqt_image_view**:
   ```bash
   rqt_image_view /camera_chin/image_raw
   ```
   or from the command line:
   ```bash
   rosrun image_view image_view image:=/camera_chin/image_raw
   ```

---

## 7. IMU

A virtual IMU is mounted on `base_link` and publishes to **/imu** at 10 Hz. For instance:

```bash
rostopic echo /imu
```
inspect orientation, linear acceleration and angular velocity data.

---

Have fun testing the Sanbot Nano! 🚀 
Boa diversão testando o Sanbot Nano! 🚀 