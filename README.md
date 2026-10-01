# genisom_l1w_ros2

ROS 2 Humble packages for the Genisom L1W (zsibot) wheel-legged quadruped, meant to be used as a
git submodule inside another workspace.

| Package | Description |
|---|---|
| `l1w_driver` | `l1w_driver_node`, bridge to `genisom_L1_sdk` (UDP), plus `srv/SetWifi` |
| `l1w_description` | xacro URDF (body, Livox Mid-360, RealSense D435i, front camera), meshes, RViz |
| `l1w_bringup` | Full bringup: driver + description + Livox + RealSense |

`livox_ros_driver2` and `realsense2_camera` are not vendored here. They must live in the parent
workspace so other robot repos can share the same drivers without duplicate packages.

## Requirements

Install the zsibot SDK (headers to `/usr/local/include/zsibot_sdk`, `libzsibot.a` to `/usr/local/lib`):

```bash
git clone https://github.com/zsibot/genisom_l1_sdk.git && cd genisom_l1_sdk
mkdir build && cd build && cmake .. && make && sudo make install
```

For another prefix use `colcon build --cmake-args -DZSIBOT_SDK_ROOT=<prefix>` or `export ZSIBOT_SDK_ROOT=<prefix>`.

## Installation

```bash
cd <your_ws>/src
git submodule add git@github.com:DarmawanRobotics/genisom_l1w_ros2.git
cp <path>/meshes/*.STL genisom_l1w_ros2/l1w_description/meshes/
cd .. && rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to l1w_bringup
```

## Usage

```bash
# full robot
ros2 launch l1w_bringup bringup.launch.py
ros2 launch l1w_bringup bringup.launch.py use_camera:=false lidar_xfer_format:=0

# driver + robot_state_publisher only
ros2 launch l1w_driver l1w_driver.launch.py namespace:=l1w robot_ip:=192.168.234.1

# URDF only, no robot
ros2 launch l1w_description display.launch.py

# sensor frames vs STL meshes -> PNG + distance table
ros2 run l1w_description check_frames.py --out /tmp/l1w_frames.png
```

## Interface contract

Every `<robot>_bringup` exposes the same names so the application stack stays robot-agnostic.

| | Topics | Frame |
|---|---|---|
| base | `<ns>/cmd_vel`, `<ns>/odom`, `<ns>/imu`, `<ns>/joint_states`, `<ns>/battery` | `base_link` |
| lidar | `/livox/lidar`, `/livox/imu` | `livox_frame` |
| camera | `/camera/camera/...` | `camera_link` (TF from URDF) |

## l1w_driver_node

Topics and services are relative to the namespace (default `l1w`).

| Name | Type | Notes |
|---|---|---|
| `cmd_vel` (sub) | `geometry_msgs/Twist` | Scaled to joystick input, zeroed after `cmd_timeout` |
| `imu`, `imu/rpy` | `sensor_msgs/Imu`, `geometry_msgs/Vector3Stamped` | 50 Hz |
| `odom`, `odom/world_velocity` | `nav_msgs/Odometry`, `geometry_msgs/Vector3Stamped` | SDK odometry, no TF |
| `joint_states` | `sensor_msgs/JointState` | 16 joints, names match the URDF |
| `motor_temperature` | `std_msgs/Float32MultiArray` | 16 motors |
| `battery` | `sensor_msgs/BatteryState` | 1 Hz |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Modes, versions, faults, Wi-Fi |
| `<command>` (srv) | `std_srvs/Trigger` | `stand_up`, `sit_down`, `move_mode`, `balance_stand_mode`, `lock_mode`, `emergency_stop`, `slow_speed`, `normal_speed`, `fast_speed`, `crawl_forward`, `climbing_high_platform`, `hand_stand`, `unload_squat`, `enter_lab_mode`, `exit_lab_mode`, `enter_entertainment_mode`, `exit_entertainment_mode`, `*_control_right` |
| `set_wifi` (srv) | `l1w_driver/srv/SetWifi` | |

The SDK accepts only one active control connection, so never run two nodes that create a
`ZsibotExecutor`.

## License

Apache-2.0
