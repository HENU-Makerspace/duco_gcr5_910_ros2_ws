# GCR5-910 ROS 2 数字孪生说明

本文用于讲解本项目的 GCR5-910 机械臂数字孪生部分，也用于现场运行和排查。

## 1. 数字孪生是什么

数字孪生不是单纯在 Gazebo 中显示一个机械臂模型，而是让仿真机械臂与真实机械臂使用同一套机器人模型、关节名称和运动轨迹，并让真实机械臂的执行状态反馈回来。

本项目的控制链路如下：

```text
MoveIt 规划
    |
    v
arm_1_controller/follow_joint_trajectory
    |
    v
DualTrajectoryAction
    |------------------------------|
    v                              v
Gazebo 仿真控制器                 joint_path_command
sim_arm_1_controller              |
    |                              v
    |                         DucoDriver
    |                              |
    v                              v
仿真关节状态                 GCR5-910 真机
                                   |
                                   v
                           DucoRobotStatus
                                   |
                 real_feedback_states + /duco_cobot/robot_state
                                   |
                                   v
                         DualTrajectoryAction
```

MoveIt 只连接 `arm_1_controller/follow_joint_trajectory`。这个 action server 由 `DualTrajectoryAction` 提供，因此 MoveIt 不需要知道 Gazebo 和真机各自的底层接口。

## 2. 当前已经完成的功能

- 使用 `gcr5_910_gazebo.urdf.xacro` 在 Gazebo 中加载 GCR5-910 模型。
- 使用 `gz_ros2_control` 和 `joint_trajectory_controller` 控制仿真关节。
- MoveIt 规划出的同一条轨迹发送给 Gazebo 和真机。
- 仿真轨迹时间可以通过 `sim_time_scale` 调整。
- 真机速度由 `robot_speed`、`robot_acc`、`trajectory_speed_scale` 和 `sync_with_trajectory_timing` 共同决定。
- 真机状态节点发布实际关节位置、速度、急停、碰撞和错误码。
- 开启 `wait_for_real:=true` 后，MoveIt 只有在 Gazebo 和真机都完成后才收到成功结果。
- 真机发生急停、碰撞、错误码或执行超时，轨迹 action 会失败。
- 取消 action 时会向真机发布空轨迹，驱动会清空轨迹队列并调用停止接口。
- Gazebo 控制器改为延时启动，避免控制器管理器尚未初始化时加载失败。

## 3. 两种同步概念

### 3.1 轨迹同步

仿真和真机收到同一组 `JointTrajectory` 路点，因此理论目标一致。仿真轨迹发布到：

```text
/sim_arm_1_controller/follow_joint_trajectory
```

真机轨迹发布到：

```text
/joint_path_command
```

### 3.2 完成状态同步

之前的程序只等待 Gazebo 完成，所以 Gazebo 先到位时 MoveIt 就会报告成功，真实机械臂可能还在运动。

现在的程序在 `wait_for_real:=true` 时还检查：

1. 真机在轨迹下发后产生了新的反馈；
2. 每个关节实际位置与最后一个轨迹点的误差不超过 `real_goal_tolerance`；
3. 每个关节速度不超过 `real_velocity_tolerance`；
4. 真机没有急停、碰撞或错误码；
5. 在 `real_execution_timeout` 内完成。

注意：当前是“同一条轨迹驱动两套系统，并等待真机反馈完成”，不是把真机关节反馈强制写回 Gazebo 的实时状态。Gazebo 仍然根据轨迹独立仿真，这样可以保留碰撞检测和仿真物理过程。

## 4. 仿真环境运行

终端 1：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config gazebo.launch.py \
  use_rviz:=true
```

检查：

```bash
ros2 action list
ros2 topic echo /joint_states --once
```

应能看到：

```text
/sim_arm_1_controller/follow_joint_trajectory
/arm_1_controller/follow_joint_trajectory
```

## 5. 只启动数字孪生链路，不连接真机

这个模式适合课堂演示、轨迹调试和修改路点。终端 1：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config digital_twin_real.launch.py \
  use_real_robot:=false \
  use_real_control_services:=false \
  use_rviz:=true \
  wait_for_real:=false
```

终端 2：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run duco_gcr5_910_moveit_config send_joint_waypoints.py \
  src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml
```

示例路点文件是：

```text
src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml
```

路点中的 `positions` 单位是弧度，`duration` 是从轨迹起点开始累计的秒数，必须严格递增。

## 6. 真机和 Gazebo 同时运行

现场确认机械臂工作区域安全、急停可用、机械臂已经上电使能后再执行。

当前网络参数：

```text
ROS 电脑：192.168.6.99
GCR5-910：192.168.6.4
PLC：192.168.6.1
摄像头：192.168.6.5
```

终端 1：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch duco_gcr5_910_moveit_config digital_twin_real.launch.py \
  use_rviz:=true \
  use_real_robot:=true \
  use_real_control_services:=true \
  server_host_1:=192.168.6.4 \
  robot_speed:=0.1 \
  robot_acc:=0.15 \
  trajectory_speed_scale:=1.0 \
  sync_with_trajectory_timing:=true \
  sim_time_scale:=1.0 \
  wait_for_real:=true \
  real_execution_timeout:=120.0
```

终端 2：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run duco_gcr5_910_moveit_config send_joint_waypoints.py \
  src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml
```

终端 3：

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic echo /real_feedback_states
ros2 topic echo /duco_cobot/robot_state
ros2 topic echo /real_joint_states
```

真实机械臂和仿真都到达最后一个路点后，发送程序才会打印：

```text
Trajectory finished with error_code=0
```

## 7. 速度参数如何理解

- `robot_speed`：真机允许使用的最大关节轨迹速度，单位约为 rad/s。
- `robot_acc`：真机轨迹加速度限制，单位约为 rad/s^2。
- `sync_with_trajectory_timing:=true`：根据轨迹相邻路点和时间间隔估计真机速度。
- `trajectory_speed_scale`：对估计速度进行倍率调整，之后仍会被 `robot_speed` 限制。
- `sim_time_scale`：只调整发送给 Gazebo 的轨迹时间。大于 1 会使仿真变慢，小于 1 会使仿真变快。

第一轮真机联调建议：

```text
robot_speed=0.05
robot_acc=0.08
trajectory_speed_scale=1.0
sim_time_scale=1.0
wait_for_real=true
```

如果仿真明显快于真机，不要直接把仿真加速；先降低真机轨迹路点要求或提高真机允许速度，但必须以机械臂安全和控制柜限制为前提。若只为演示视觉同步，可以把 `sim_time_scale` 调大，让仿真等待真机。

## 8. 运行前检查清单

1. 确认 ROS 电脑有线网卡是 `192.168.6.99`。
2. 确认 `ping 192.168.6.4` 正常。
3. 确认机械臂已上电、使能且没有急停。
4. 确认 `DucoDriver` 和 `DucoRobotStatus` 使用同一个机械臂 IP。
5. 确认 `/real_feedback_states` 有持续消息。
6. 确认 `/duco_cobot/robot_state` 中 `emc_stop_signal=false`、`collision=false`、`robot_error=0`。
7. 确认第一个测试路点接近真实机械臂当前姿态。
8. 先低速执行单个小范围路点，再执行多中间点任务。
9. 任何异常优先按现场急停，不要依赖 ROS 取消命令替代安全急停。

## 9. 常用排查命令

查看 action：

```bash
ros2 action list
```

查看控制器：

```bash
ros2 control list_controllers
```

查看双轨迹节点参数：

```bash
ros2 param list /DualTrajectoryAction
```

查看真机反馈频率：

```bash
ros2 topic hz /real_feedback_states
ros2 topic hz /real_joint_states
```

查看真实轨迹是否发出：

```bash
ros2 topic echo /joint_path_command --once
```

如果 action 被拒绝并且日志出现：

```text
Gazebo action server [sim_arm_1_controller/follow_joint_trajectory] is not ready
```

先确认 Gazebo 控制器是否 active，再重新发送轨迹。当前 launch 已加入 5 秒控制器启动延时，正常情况下不应再因启动竞态失败。

如果真机模式下最终超时：

```text
real robot execution timed out
```

检查真机是否仍在运动、反馈 topic 是否有消息、轨迹关节名称是否完全匹配，以及 `real_execution_timeout` 是否过小。

## 10. 讲解时的总结

可以把整个系统概括成三层：

1. **模型层**：URDF/Xacro、SRDF、关节限制和初始姿态描述 GCR5-910。
2. **规划与控制层**：MoveIt 生成统一的关节轨迹，`DualTrajectoryAction` 将轨迹分发给 Gazebo 和真机。
3. **反馈与安全层**：`DucoRobotStatus` 采集真机关节和故障状态，双轨迹节点根据反馈决定成功、超时或失败。

因此，本项目当前的数字孪生闭环是：

```text
统一规划 -> 仿真/真机执行 -> 真机状态反馈 -> 完成或故障判定
```
