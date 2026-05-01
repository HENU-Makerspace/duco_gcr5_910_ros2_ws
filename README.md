# DUCO GCR5-910 ROS 2 Jazzy Workspace

Slim ROS 2 workspace for the DUCO GCR5-910 arm. It includes the real robot driver, MoveIt configuration, Gazebo simulation, and a synchronized digital twin launch.

## Packages

- `duco_support`: GCR5-910 URDF and meshes
- `duco_msg`: custom DUCO messages and services
- `duco_ros_driver`: real robot driver and trajectory relay nodes
- `duco_gcr5_910_moveit_config`: MoveIt, Gazebo, and launch configuration

## Build

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## Network

The robot controller is expected at:

```text
Robot IP: 192.168.4.23
TCP port: 7003
PC wired IP: 192.168.4.100/24
```

Quick checks:

```bash
ip -br addr show enp7s0
ip route get 192.168.4.23
ping -I enp7s0 -c 3 -W 1 192.168.4.23
nc -vz 192.168.4.23 7003
```

## Real Robot With RViz

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config demo.launch.py use_rviz:=true
```

Slow override:

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py \
  use_rviz:=true \
  robot_speed:=0.1 \
  robot_acc:=0.15
```

## Gazebo Simulation Only

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config gazebo.launch.py use_rviz:=true
```

## Real Robot + Gazebo Digital Twin

This launch sends one MoveIt trajectory to both Gazebo and the real DUCO driver.

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config digital_twin_real.launch.py \
  use_rviz:=true \
  robot_speed:=0.2 \
  robot_acc:=0.3 \
  trajectory_speed_scale:=2.0 \
  sync_with_trajectory_timing:=true \
  sim_time_scale:=1.0
```

Tuning:

- Real robot too slow: increase `trajectory_speed_scale`, then `robot_speed` if needed.
- Real robot too fast: decrease `trajectory_speed_scale` or `robot_speed`.
- Gazebo too fast: increase `sim_time_scale`.
- Gazebo too slow: decrease `sim_time_scale`.

Simulation-only test using the same digital twin path:

```bash
ros2 launch duco_gcr5_910_moveit_config digital_twin_real.launch.py \
  use_real_robot:=false \
  use_real_control_services:=false \
  use_rviz:=true
```

## Joint Waypoint Task

Edit waypoints here:

```text
src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml
```

Send the waypoint task after `digital_twin_real.launch.py` is running:

```bash
ros2 run duco_gcr5_910_moveit_config send_joint_waypoints.py \
  src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml
```

Check states:

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /real_joint_states
ros2 action list
```

Expected actions:

```text
/arm_1_controller/follow_joint_trajectory
/sim_arm_1_controller/follow_joint_trajectory
```

## Notes

- `build/`, `install/`, and `log/` are generated locally and are ignored by git.
- This repository contains a vendor static library: `src/duco_ros_driver/lib/libDucoCobotAPI.a`. Keep the GitHub repository private unless the vendor license allows redistribution.
