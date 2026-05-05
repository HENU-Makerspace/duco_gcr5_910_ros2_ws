# Siemens S7-1214C PLC 与 ROS2 当前联调方案

本文档给 PLC 同学使用。当前阶段只完成 **ROS2 与 PLC 通讯、机械臂上电/下电、上使能/下使能、反馈状态读取**。

PLC 同学已经在原有 `DB5` 中建立了变量，所以本阶段直接使用 `DB5`，不需要新建 `DB1`。

目前吸盘还没有连接气泵，所以 `ros请求吸气` 只作为预留位，当前不要接气泵或电磁阀。

## 1. 当前 DB5 地址

按 PLC 同学截图，当前地址如下：

```text
绝对地址      数据类型  变量含义
DB5.DBD0     Real      in10x
DB5.DBD4     Real      in10y
DB5.DBX8.0   Bool      ros请求上电
DB5.DBX8.1   Bool      ros请求下电
DB5.DBX8.2   Bool      ros请求上使能
DB5.DBX8.3   Bool      ros请求下使能
DB5.DBX8.4   Bool      ros请求吸气，当前预留，暂时不用
DB5.DBX8.5   Bool      PLC上电反馈已就绪
DB5.DBX8.6   Bool      PLC上使能反馈已就绪
DB5.DBX8.7   Bool      PLC下电反馈已就绪
DB5.DBX9.0   Bool      PLC下使能反馈已就绪
```

注意：这两张截图里没有看到新的急停地址。ROS2 节点现在默认不读取急停位，`/plc/e_stop` 会保持 `False`。如果 PLC 同学后续给出急停地址，比如 `DB5.DBX9.1`，ROS2 启动时再加参数：

```bash
-p estop_byte:=9 -p estop_bit:=1
```

## 2. 当前目标

当前要跑通的是：

```text
ROS2 -> PLC: 保持上电请求，同时取消下电请求
ROS2 -> PLC: 保持下电请求，同时取消上电请求
ROS2 -> PLC: 保持上使能请求，同时取消下使能请求
ROS2 -> PLC: 保持下使能请求，同时取消上使能请求
PLC -> ROS2: 反馈上电是否就绪
PLC -> ROS2: 反馈上使能是否就绪
PLC -> ROS2: 反馈下电是否完成
PLC -> ROS2: 反馈下使能是否完成
```

这里“上电/下电/上使能/下使能”是四个独立请求位。上电和下电互斥，上使能和下使能互斥。

## 3. TIA Portal 必须设置

### 3.1 PLC 网络

```text
PLC IP:      192.168.6.1
ROS2 电脑:   192.168.6.99
子网掩码:    255.255.255.0
```

### 3.2 开启 PUT/GET 通讯

在 PLC 设备属性中勾选：

```text
允许来自远程对象的 PUT/GET 通信访问
```

### 3.3 DB5 关闭优化块访问

右键 `DB5`，打开属性，取消勾选：

```text
优化的块访问
```

ROS2 是按绝对地址访问的，例如：

```text
DB5.DBX8.0
DB5.DBX8.1
DB5.DBX9.0
```

如果 `DB5` 开启优化访问，绝对地址可能不可用。

## 4. PLC 程序建议

### 4.1 ROS 请求位作为互斥保持信号

ROS2 不再发送短脉冲，而是保持请求位状态。

上电/下电互斥：

```text
请求上电时:
DB5.DBX8.0 = True
DB5.DBX8.1 = False

请求下电时:
DB5.DBX8.0 = False
DB5.DBX8.1 = True
```

上使能/下使能互斥：

```text
请求上使能时:
DB5.DBX8.2 = True
DB5.DBX8.3 = False

请求下使能时:
DB5.DBX8.2 = False
DB5.DBX8.3 = True
```

PLC 程序可以直接按保持信号写逻辑，不需要必须做上升沿。

### 4.2 反馈位由 PLC 写入

PLC 根据现场实际状态写反馈：

```text
DB5.DBX8.5 = 上电完成/上电就绪
DB5.DBX8.6 = 上使能完成/使能就绪
DB5.DBX8.7 = 下电完成/下电就绪
DB5.DBX9.0 = 下使能完成/下使能就绪
```

ROS2 只读取这些反馈，不应该写这些反馈位。

### 4.3 当前吸盘位不要接输出

当前不要把这个位接到气泵：

```text
DB5.DBX8.4
```

也就是当前不要写：

```text
Q0.x = DB5.DBX8.4
```

等吸盘和气泵接好后再启用。

### 4.4 急停

急停应在 PLC 硬件/安全逻辑里切断输出。

如果希望 ROS2 也能读取急停状态，请 PLC 同学再提供一个明确 DB 地址，例如：

```text
DB5.DBX9.1   PLC反馈急停触发
```

然后 ROS2 这边启动时加：

```bash
-p estop_byte:=9 -p estop_bit:=1
```

## 5. ROS2 使用命令

启动 PLC 通讯节点：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1
```

默认映射：

```text
DB number: 5
Command/status byte: 8
Disable feedback byte: 9
```

### 5.1 推荐命令

请求上电：

```bash
ros2 service call /plc/request_power_on std_srvs/srv/Trigger "{}"
```

请求下电：

```bash
ros2 service call /plc/request_power_off std_srvs/srv/Trigger "{}"
```

请求上使能：

```bash
ros2 service call /plc/request_enable std_srvs/srv/Trigger "{}"
```

请求下使能：

```bash
ros2 service call /plc/request_disable std_srvs/srv/Trigger "{}"
```

### 5.2 兼容旧命令

旧命令仍可用：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: false}"
```

对应关系：

```text
set_power_on true    -> DB5.DBX8.0=True,  DB5.DBX8.1=False
set_power_on false   -> DB5.DBX8.0=False, DB5.DBX8.1=True
set_enable true      -> DB5.DBX8.2=True,  DB5.DBX8.3=False
set_enable false     -> DB5.DBX8.2=False, DB5.DBX8.3=True
```

### 5.3 清空命令位

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

会清空：

```text
DB5.DBX8.0 = False
DB5.DBX8.1 = False
DB5.DBX8.2 = False
DB5.DBX8.3 = False
DB5.DBX8.4 = False
```

### 5.4 查看反馈状态

```bash
ros2 topic echo /plc/power_on_ready
ros2 topic echo /plc/enable_ready
ros2 topic echo /plc/power_off_ready
ros2 topic echo /plc/disable_ready
ros2 topic echo /plc/robot_ready
```

对应关系：

```text
/plc/power_on_ready   = DB5.DBX8.5
/plc/enable_ready     = DB5.DBX8.6
/plc/power_off_ready  = DB5.DBX8.7
/plc/disable_ready    = DB5.DBX9.0
/plc/robot_ready      = power_on_ready AND enable_ready AND NOT e_stop
```

如果没有配置急停地址，`e_stop` 默认为 False。

## 6. 联调步骤

### 6.1 测试 ROS 写 PLC

ROS2 发送：

```bash
ros2 service call /plc/request_power_on std_srvs/srv/Trigger "{}"
```

PLC 同学在线监控，应看到：

```text
DB5.DBX8.0 = True
DB5.DBX8.1 = False
```

依次测试：

```bash
ros2 service call /plc/request_power_off std_srvs/srv/Trigger "{}"
ros2 service call /plc/request_enable std_srvs/srv/Trigger "{}"
ros2 service call /plc/request_disable std_srvs/srv/Trigger "{}"
```

PLC 同学应分别看到：

```text
request_power_off:
DB5.DBX8.0 = False
DB5.DBX8.1 = True

request_enable:
DB5.DBX8.2 = True
DB5.DBX8.3 = False

request_disable:
DB5.DBX8.2 = False
DB5.DBX8.3 = True
```

### 6.2 测试 PLC 写 ROS

PLC 同学手动或程序置位：

```text
DB5.DBX8.5 = True
DB5.DBX8.6 = True
DB5.DBX8.7 = True
DB5.DBX9.0 = True
```

ROS2 应看到对应 topic 变成 True。

### 6.3 真实动作顺序

建议顺序：

```text
1. request_power_on
2. 等 /plc/power_on_ready=True
3. request_enable
4. 等 /plc/enable_ready=True
5. 确认 /plc/robot_ready=True
6. 再启动机械臂 MoveIt 控制
```

停止顺序：

```text
1. request_disable
2. 等 /plc/disable_ready=True
3. request_power_off
4. 等 /plc/power_off_ready=True
```

## 7. 通过标准

```text
1. ROS2 能连接 PLC。
2. request_power_on 后，DB5.DBX8.0=True，DB5.DBX8.1=False。
3. request_power_off 后，DB5.DBX8.0=False，DB5.DBX8.1=True。
4. request_enable 后，DB5.DBX8.2=True，DB5.DBX8.3=False。
5. request_disable 后，DB5.DBX8.2=False，DB5.DBX8.3=True。
6. PLC 写 DB5.DBX8.5 后，ROS2 /plc/power_on_ready 变化。
7. PLC 写 DB5.DBX8.6 后，ROS2 /plc/enable_ready 变化。
8. PLC 写 DB5.DBX8.7 后，ROS2 /plc/power_off_ready 变化。
9. PLC 写 DB5.DBX9.0 后，ROS2 /plc/disable_ready 变化。
```

## 8. 安全注意事项

1. ROS2 只发请求，PLC 必须负责安全互锁。
2. 急停必须能硬件切断机械臂输出，不能只靠 ROS2 软件。
3. 当前阶段不要把吸盘、气泵、电磁阀接到 `DB5.DBX8.4`。
4. 第一次动作前，现场人员必须确认机械臂周围安全。
5. 第一次真实运动时，机械臂速度建议限制在 10% 左右。
