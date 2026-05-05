# Siemens S7-1214C PLC 与 ROS2 当前联调方案

本文档给 PLC 同学使用。当前阶段只完成 **ROS2 与 PLC 通讯、机械臂上电/使能、机械臂就绪反馈、急停反馈**。

目前虽然现场有吸盘和气泵，但吸盘还没有连接到气泵，所以本阶段 **不控制吸盘、不控制气泵、不做真空检测**。吸盘相关地址先作为预留位保留，后续气路接好后再启用。

## 1. 当前目标

当前要跑通的是：

```text
ROS2 电脑 -> PLC: 请求机械臂上电/使能
ROS2 电脑 -> PLC: 请求机械臂下电/取消使能
PLC -> ROS2 电脑: 反馈机械臂是否就绪
PLC -> ROS2 电脑: 反馈急停是否触发
```

暂时不做：

```text
吸盘吸气
气泵启动
破真空/放气
真空压力检测
吸住工件检测
```

## 2. 网络参数

```text
ROS2 电脑:     192.168.6.99
PLC:           192.168.6.1
机械臂控制柜:  192.168.6.4
相机:          192.168.6.5
PLC 编程电脑:  192.168.6.66
子网掩码:      255.255.255.0
```

ROS2 使用 Siemens S7 通讯直接访问 PLC 的 `DB1`，PLC 侧 TCP 端口是 `102`。

PLC 同学可以先确认 ROS2 电脑能连到 PLC：

```text
ROS2 电脑能 ping 通 192.168.6.1
ROS2 电脑能连接 PLC 的 TCP 102 端口
```

## 3. TIA Portal 必须设置

### 3.1 设置 PLC IP

在 TIA Portal 中设置 PLC：

```text
IP 地址:   192.168.6.1
子网掩码: 255.255.255.0
```

### 3.2 开启 PUT/GET 通讯

打开 PLC 设备属性，在保护/连接机制相关页面中勾选：

```text
允许来自远程对象的 PUT/GET 通信访问
```

如果不勾选，ROS2 可能可以 ping 通 PLC，但无法读写 `DB1`。

### 3.3 建立 DB1 数据块

新建一个全局数据块：

```text
DB 名称: DB1
类型: 全局 DB
```

右键 `DB1`，打开属性，取消勾选：

```text
优化的块访问
```

必须关闭优化块访问。ROS2 程序是按绝对地址访问的，例如：

```text
DB1.DBX0.0
DB1.DBX1.0
```

如果 DB1 开启优化访问，绝对地址可能不可用或地址不固定。

## 4. DB1 数据类型与地址

当前阶段只需要 `Bool` 类型。上电、下电、急停、就绪都是开关量，不是 `Int`，也不是浮点型。

PLC 同学在 DB1 中建立下面这些变量。变量名可以改，但 **绝对地址必须一致**。

### 4.1 ROS2 写入 PLC 的命令区

这些位由 ROS2 写入，PLC 读取后决定是否控制输出。

```text
绝对地址      数据类型  建议变量名                   当前阶段用途
DB1.DBX0.0   Bool      ros_power_enable_cmd       使用：ROS 请求机械臂上电/使能
DB1.DBX0.1   Bool      reserved_suction_on        预留：吸盘吸气，当前不用
DB1.DBX0.2   Bool      reserved_suction_release   预留：破真空/放气，当前不用
DB1.DBX0.3   Bool      reserved_reset             预留：报警复位，当前可不用
```

当前 PLC 程序只需要使用：

```text
DB1.DBX0.0
```

不要把 `DB1.DBX0.1`、`DB1.DBX0.2` 接到气泵或电磁阀输出，因为当前吸盘气路还没接好。

### 4.2 PLC 写给 ROS2 的状态区

这些位由 PLC 写入，ROS2 读取。

```text
绝对地址      数据类型  建议变量名                    当前阶段用途
DB1.DBX1.0   Bool      plc_robot_ready             使用：机械臂已上电且就绪
DB1.DBX1.1   Bool      plc_estop_active            使用：急停触发
DB1.DBX1.2   Bool      reserved_suction_feedback   预留：吸盘输出反馈，当前固定 False
DB1.DBX1.3   Bool      reserved_object_detected    预留：吸住工件检测，当前固定 False
DB1.DBX1.4   Bool      plc_alarm                   可选：PLC 报警，当前可固定 False
```

当前 PLC 程序必须写：

```text
DB1.DBX1.0
DB1.DBX1.1
```

建议当前阶段把下面这些预留状态固定为 `False`：

```text
DB1.DBX1.2 = False
DB1.DBX1.3 = False
DB1.DBX1.4 = False
```

## 5. 当前阶段推荐 PLC 物理 I/O

实际 I/O 地址按现场接线为准。下面是推荐最小方案。

### 5.1 输入 I 点

```text
I0.0   robot_ready_input   机械臂就绪反馈
I0.1   estop_input         急停反馈，True 表示急停触发
```

如果暂时没有机械臂就绪反馈线 `I0.0`，可以先不接，但 PLC 程序中要用临时逻辑生成 `DB1.DBX1.0`，见第 6.3 节。

### 5.2 输出 Q 点

```text
Q0.0   robot_enable_output   机械臂上电/使能继电器或使能输出
```

当前阶段不要接：

```text
Q0.1   吸盘吸气
Q0.2   破真空/放气
```

这些后续再做。

## 6. PLC 梯形图逻辑

下面按网络写，PLC 同学照这个逻辑搭即可。变量名可以按项目习惯改，但 DB 地址不要变。

### 6.1 网络 1：急停状态

逻辑：

```text
DB1.DBX1.1 = I0.1
```

含义：

```text
I0.1 = True  -> 急停触发，ROS2 看到 /plc/e_stop=True
I0.1 = False -> 急停未触发
```

如果现场急停输入是常闭信号，也就是断开才表示急停，则需要按实际电气逻辑取反，不能照抄。

### 6.2 网络 2：机械臂使能输出

最小逻辑：

```text
Q0.0 = DB1.DBX0.0 AND NOT DB1.DBX1.1
```

等价理解：

```text
ROS2 请求上电/使能
并且急停没有触发
PLC 才打开机械臂使能输出
```

不要写成：

```text
Q0.0 = DB1.DBX0.0
```

因为这样急停触发时 ROS2 如果还保持命令为 True，Q0.0 可能仍然输出，风险很高。

### 6.3 网络 3：机械臂就绪反馈

如果已经接了机械臂真实就绪反馈 `I0.0`，使用正式逻辑：

```text
DB1.DBX1.0 = I0.0 AND Q0.0 AND NOT DB1.DBX1.1
```

含义：

```text
机械臂真实反馈就绪
并且 PLC 使能输出已经打开
并且急停未触发
才告诉 ROS2 机械臂 ready
```

如果暂时没有真实就绪反馈线，可以先用临时联调逻辑：

```text
DB1.DBX1.0 = Q0.0 AND NOT DB1.DBX1.1
```

临时逻辑只表示 PLC 输出已经打开，不代表机械臂控制柜真的 ready。正式运行前最好接入真实反馈。

### 6.4 网络 4：预留吸盘状态固定为 False

当前吸盘和气泵还没有连接，所以预留状态不要误报。

```text
DB1.DBX1.2 = False
DB1.DBX1.3 = False
DB1.DBX1.4 = False
```

### 6.5 网络 5：吸盘命令暂时不处理

当前不要把下面这些位接到输出：

```text
DB1.DBX0.1
DB1.DBX0.2
```

也就是当前不要写：

```text
Q0.1 = DB1.DBX0.1
Q0.2 = DB1.DBX0.2
```

等吸盘和气泵接好后，再新增吸气、放气、真空检测逻辑。

## 7. ROS2 侧当前使用命令

PLC 同学主要知道这些命令对应 DB 哪个位即可。

启动 ROS2 PLC 通讯节点：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1 -p db_number:=1
```

ROS2 请求上电/使能：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

对应 PLC：

```text
DB1.DBX0.0 = True
```

ROS2 请求下电/取消使能：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
```

对应 PLC：

```text
DB1.DBX0.0 = False
```

ROS2 清空所有命令位：

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

对应 PLC：

```text
DB1.DBX0.0 = False
DB1.DBX0.1 = False
DB1.DBX0.2 = False
DB1.DBX0.3 = False
```

ROS2 查看 PLC 状态：

```bash
ros2 topic echo /plc/robot_ready
ros2 topic echo /plc/e_stop
```

当前阶段不要使用吸盘命令：

```bash
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_suction_release std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_gripper_close std_srvs/srv/SetBool "{data: true}"
```

## 8. 联调步骤

### 8.1 第一步：检查网络

ROS2 电脑检查：

```bash
ping -c 3 192.168.6.1
nc -vz -w 2 192.168.6.1 102
```

期望：

```text
ping 192.168.6.1 正常
102 端口连接成功
```

### 8.2 第二步：检查 ROS2 是否能连接 PLC

ROS2 电脑运行：

```bash
ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1 -p db_number:=1
```

期望日志：

```text
成功连接到 PLC: 192.168.6.1
PLC 状态更新: 机械臂就绪=False, 急停=False
```

如果连接失败，PLC 同学检查：

```text
PLC IP 是否为 192.168.6.1
PUT/GET 是否开启
DB1 是否存在
DB1 是否关闭优化块访问
PLC 是否已经下载最新程序
```

### 8.3 第三步：测试 ROS2 写 PLC

ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

PLC 同学在线监控 DB1，确认：

```text
DB1.DBX0.0 = True
```

然后 ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
```

PLC 同学确认：

```text
DB1.DBX0.0 = False
```

### 8.4 第四步：测试 PLC 控制 Q0.0

确认现场安全后，ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

PLC 同学确认：

```text
Q0.0 = True
```

ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
```

PLC 同学确认：

```text
Q0.0 = False
```

### 8.5 第五步：测试急停

在确认不会造成危险的情况下，触发急停。

PLC 同学确认：

```text
I0.1 = True
DB1.DBX1.1 = True
Q0.0 = False
```

ROS2 电脑确认：

```bash
ros2 topic echo /plc/e_stop
```

应看到：

```text
data: true
```

释放急停后，PLC 同学确认：

```text
I0.1 = False
DB1.DBX1.1 = False
```

### 8.6 第六步：确认机械臂 ready

如果使用正式 ready 输入：

```text
I0.0 = True
Q0.0 = True
DB1.DBX1.1 = False
```

则 PLC 应输出：

```text
DB1.DBX1.0 = True
```

ROS2 电脑确认：

```bash
ros2 topic echo /plc/robot_ready
```

应看到：

```text
data: true
```

只有在：

```text
/plc/robot_ready = True
/plc/e_stop = False
```

之后，才允许启动机械臂 MoveIt 控制。

## 9. 当前阶段通过标准

本阶段联调通过，需要满足：

```text
1. ROS2 能连接 PLC。
2. ROS2 发送 set_power_on true 后，DB1.DBX0.0=True。
3. ROS2 发送 set_power_on false 后，DB1.DBX0.0=False。
4. DB1.DBX0.0=True 且无急停时，Q0.0=True。
5. 急停触发时，Q0.0=False，DB1.DBX1.1=True。
6. 机械臂就绪条件满足时，DB1.DBX1.0=True。
7. ROS2 能通过 /plc/robot_ready 和 /plc/e_stop 看到 PLC 状态。
```

吸盘和气泵等后续接线完成后，再启用：

```text
DB1.DBX0.1  吸盘吸气
DB1.DBX0.2  破真空/放气
DB1.DBX1.2  吸盘输出反馈
DB1.DBX1.3  吸住工件检测
```

## 10. 安全注意事项

1. ROS2 只负责写 DB 请求，PLC 必须负责安全互锁。
2. 急停必须能硬件切断机械臂使能输出。
3. 当前阶段不要把吸盘、气泵、电磁阀接到 ROS2 命令位。
4. 第一次控制 Q0.0 前，现场人员必须确认机械臂周围安全。
5. 第一次真实运动时，机械臂速度建议限制在 10% 左右。
