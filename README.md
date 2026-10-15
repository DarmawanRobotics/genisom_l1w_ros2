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
| `cmd_vel` (sub) | `geometry_msgs/Twist` | Scaled to joystick input, zeroed after `cmd_timeout`; yaw and lateral channels and directions in `stick_*_axis` / `stick_invert_*` |
| `imu`, `imu/rpy` | `sensor_msgs/Imu`, `geometry_msgs/Vector3Stamped` | 50 Hz |
| `odom`, `odom/world_velocity` | `nav_msgs/Odometry`, `geometry_msgs/Vector3Stamped` | SDK odometry, no TF |
| `joint_states` | `sensor_msgs/JointState` | 16 joints, names match the URDF |
| `motor_temperature` | `std_msgs/Float32MultiArray` | 16 motors |
| `battery` | `sensor_msgs/BatteryState` | 1 Hz |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Modes, versions, faults, Wi-Fi |
| `<command>` (srv) | `std_srvs/Trigger` | `stand_up`, `sit_down`, `move_mode`, `balance_stand_mode`, `lock_mode`, `emergency_stop`, `slow_speed`, `normal_speed`, `fast_speed`, `crawl_forward`, `climbing_high_platform`, `hand_stand`, `unload_squat`, `enter_lab_mode`, `exit_lab_mode`, `enter_entertainment_mode`, `exit_entertainment_mode`, `*_control_right` |
| `set_wifi` (srv) | `l1w_driver/srv/SetWifi` | |

The driver runs as `ROLE_SDK`. The robot streams state to any client but ignores commands and
`cmd_vel` until it reports `function_mode=sdk` (see `diagnostics`). With `auto_control_right`
the driver sends `CMD_SDK_CONTROL_RIGHT` on connect and retries until confirmed, then stops so
the handheld remote can still take over. To take control back, call `sdk_control_right`.
Commands other than `emergency_stop` and `*_control_right` are rejected while SDK control is not held.

The SDK accepts only one active control connection, so never run two nodes that create a
`ZsibotExecutor`.

## Pashupati

Clone into `src/xtras/robots/` of [Pashupati](https://github.com/DarmawanRobotics/Pashupati),
then set in `/etc/pashupati/pashupati.env`:

```bash
ROBOT_BRINGUP=l1w_bringup
ROBOT_NS=l1w
ROBOT_IP=192.168.234.1
ROBOT_SETUP=/home/robot/dev/Pashupati/src/xtras/robots/genisom_l1w_ros2/script/setup.sh
# the vendor autostart runs its own Livox driver and motion stack; quoted, it has spaces
ROBOT_PRESTART="/usr/local/bin/robot-launch stop driver-livox forward"
```

and the robot part of `/etc/pashupati/site.yaml`:

```yaml
control_mux_node:
  ros__parameters:
    field.front: 0.40
    field.rear: 0.40
    field.half_width: 0.28
    field.max_decel: 0.6          # measure on the site's floor
    field.reaction_time: 0.25
    max_vx: 1.0
    max_vy: 0.5
    max_wz: 1.5

path_follower_node:
  ros__parameters:
    holonomic: true

lidar_sector_node:
  ros__parameters:
    self_filter_x_min: -0.40
    self_filter_x_max: 0.40
    self_filter_y_min: -0.28
    self_filter_y_max: 0.28

teleop_udp_node:
  ros__parameters:
    token: "<secret>"             # the same in the handheld app
    # the handheld's action menu, in this order
    robot_commands: [stand_up, sit_down, balance_stand_mode, move_mode, lock_mode, slow_speed,
                     normal_speed, fast_speed, sdk_control_right, emergency_stop]
```

- `script/setup.sh install robot` (Pashupati) runs this repo's `script/setup.sh` to install the SDK.
- The vendor stack talks `rmw_zenoh` on its own domain; Pashupati stays on `rmw_fastrtps_cpp`,
  domain 0. In a shell, `source ~/dev/Pashupati/script/env.sh` before any `ros2` command.
- The vendor remote can take control back at any time; `sdk_control_right` (handheld action menu)
  hands it to the SDK again.
- The RealSense runs through librealsense's libusb backend: the kernel `uvcvideo` module must not
  bind it as well (Pashupati's `script/setup.sh install realsense` blacklists it on the Jetson).

## License

Apache-2.0
