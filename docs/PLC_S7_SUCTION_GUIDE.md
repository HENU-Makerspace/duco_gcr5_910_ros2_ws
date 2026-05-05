# Siemens S7-1214C PLC 与 ROS2 当前联调方案

本文档给 PLC 同学使用。当前阶段只完成 **ROS2 与 PLC 通讯、机械臂上电、机械臂使能、就绪反馈、急停反馈**。

PLC 同学已经在原有 DB 块中建立了变量，本阶段直接使用原来的 `DB5`，不需要再新建 `DB1`。

目前虽然现场有吸盘和气泵，但吸盘还没有连接到气泵，所以本阶段 **不控制吸盘、不控制气泵、不做真空检测**。`DB5.DBX8.2` 先作为预留位保留，后续气路接好后再启用。

## 1. 当前目标

当前要跑通的是：

```text
ROS2 电脑 -> PLC: 请求机械臂上电
ROS2 电脑 -> PLC: 请求机械臂使能
PLC -> ROS2 电脑: 反馈机械臂上电就绪
PLC -> ROS2 电脑: 反馈机械臂使能就绪
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

ROS2 使用 Siemens S7 通讯直接访问 PLC 的 `DB5`，PLC 侧 TCP 端口是 `102`。

## 3. TIA Portal 必须设置

### 3.1 设置 PLC IP

```text
IP 地址:   192.168.6.1
子网掩码: 255.255.255.0
```

### 3.2 开启 PUT/GET 通讯

打开 PLC 设备属性，在保护/连接机制相关页面中勾选：

```text
允许来自远程对象的 PUT/GET 通信访问
```

如果不勾选，ROS2 可能可以 ping 通 PLC，但无法读写 DB。

### 3.3 检查 DB5 的访问方式

PLC 同学使用的是原来的 `DB5`，可以，不需要新建 DB。

但必须确认 `DB5` 支持绝对地址访问。也就是右键 `DB5`，打开属性，取消勾选：

```text
优化的块访问
```

ROS2 程序按绝对地址访问，例如：

```text
DB5.DBX8.0
DB5.DBX8.1
DB5.DBX8.5
```

如果 DB5 开启优化访问，绝对地址可能不可用或地址不固定。

## 4. DB5 数据类型与地址

当前阶段只需要 `Bool` 类型。上电、使能、急停、就绪都是开关量，不是 `Int`，也不是浮点型。

PLC 同学当前截图中的地址是：

```text
绝对地址      数据类型  变量含义
DB5.DBX8.0   Bool      ros请求上电
DB5.DBX8.1   Bool      ros请求使能
DB5.DBX8.2   Bool      ros请求吸气，当前预留，不接气泵
DB5.DBX8.3   Bool      PLC上电反馈已就绪
DB5.DBX8.4   Bool      PLC使能反馈已就绪
DB5.DBX8.5   Bool      PLC反馈急停触发
```

### 4.1 ROS2 写入 PLC 的命令位

```text
DB5.DBX8.0   ROS2 写入   请求机械臂上电
DB5.DBX8.1   ROS2 写入   请求机械臂使能
DB5.DBX8.2   ROS2 写入   预留吸盘吸气，当前不要接气泵
```

### 4.2 PLC 写给 ROS2 的状态位

```text
DB5.DBX8.3   PLC 写入    上电反馈已就绪
DB5.DBX8.4   PLC 写入    使能反馈已就绪
DB5.DBX8.5   PLC 写入    急停触发
```

### 4.3 关于同一个 Byte 的注意事项

现在命令位和反馈位都在 `DB5.DBB8` 这个同一个字节里：

```text
DB5.DBX8.0 ~ DB5.DBX8.2  ROS2 写
DB5.DBX8.3 ~ DB5.DBX8.5  PLC 写
```

这个可以先联调。ROS2 节点写命令时会先读取 `DB5.DBB8` 当前值，再只修改 8.0、8.1、8.2，尽量保留 8.3、8.4、8.5。

但从工业项目习惯来说，更推荐后续改成两个字节：

```text
DB5.DBB8   ROS2 -> PLC 命令
DB5.DBB9   PLC -> ROS2 状态
```

当前先按 PLC 同学已经建好的 `DB5.DBX8.x` 方案联调即可。

## 5. 当前阶段推荐 PLC 物理 I/O

实际 I/O 地址按现场接线为准。下面是推荐最小方案。

```text
I0.0   robot_power_ready_input    机械臂上电就绪反馈
I0.1   robot_enable_ready_input   机械臂使能就绪反馈
I0.2   estop_input                急停反馈，True 表示急停触发

Q0.0   robot_power_output         机械臂上电输出或上电继电器
Q0.1   robot_enable_output        机械臂使能输出或使能继电器
```

当前不要接：

```text
Q0.2   吸盘吸气
Q0.3   破真空/放气
```

这些后续气路接好后再做。

## 6. PLC 梯形图逻辑

下面是当前最小逻辑。变量名可以按项目习惯改，但 DB 地址要保持一致。

### 6.1 网络 1：急停反馈

假设急停输入 `I0.2=True` 表示急停触发：

```text
DB5.DBX8.5 = I0.2
```

如果现场急停输入是常闭信号，也就是断开才表示急停，PLC 同学需要按实际电气逻辑取反。

### 6.2 网络 2：机械臂上电输出

```text
Q0.0 = DB5.DBX8.0 AND NOT DB5.DBX8.5
```

含义：

```text
ROS2 请求上电
并且急停没有触发
PLC 才允许上电输出
```

不要写成：

```text
Q0.0 = DB5.DBX8.0
```

急停必须能切断输出。

### 6.3 网络 3：机械臂使能输出

建议使能必须建立在已请求上电、且没有急停的基础上：

```text
Q0.1 = DB5.DBX8.1 AND DB5.DBX8.0 AND NOT DB5.DBX8.5
```

含义：

```text
ROS2 请求使能
ROS2 请求上电仍然为 True
急停没有触发
PLC 才允许使能输出
```

如果现场还要求“上电反馈已就绪后才允许使能”，可以用更严格逻辑：

```text
Q0.1 = DB5.DBX8.1 AND DB5.DBX8.3 AND NOT DB5.DBX8.5
```

这更推荐。

### 6.4 网络 4：上电反馈已就绪

如果已经接了真实上电就绪反馈 `I0.0`，用正式逻辑：

```text
DB5.DBX8.3 = I0.0 AND Q0.0 AND NOT DB5.DBX8.5
```

如果暂时没有真实上电反馈线，可以先用临时联调逻辑：

```text
DB5.DBX8.3 = Q0.0 AND NOT DB5.DBX8.5
```

临时逻辑只表示 PLC 输出打开，不代表机械臂控制柜真的上电就绪。

### 6.5 网络 5：使能反馈已就绪

如果已经接了真实使能就绪反馈 `I0.1`，用正式逻辑：

```text
DB5.DBX8.4 = I0.1 AND Q0.1 AND NOT DB5.DBX8.5
```

如果暂时没有真实使能反馈线，可以先用临时联调逻辑：

```text
DB5.DBX8.4 = Q0.1 AND NOT DB5.DBX8.5
```

临时逻辑只表示 PLC 使能输出打开，不代表机械臂真的使能就绪。

### 6.6 网络 6：吸盘命令暂时不处理

当前不要把下面这个位接到气泵或电磁阀：

```text
DB5.DBX8.2
```

也就是当前不要写：

```text
Q0.2 = DB5.DBX8.2
```

等吸盘和气泵接好后，再新增吸气、放气、真空检测逻辑。

## 7. ROS2 侧当前使用命令

启动 ROS2 PLC 通讯节点：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1
```

节点默认使用：

```text
DB number: 5
Byte:      8
```

如果想显式写出来，也可以运行：

```bash
ros2 run plc_s7_bridge plc_s7_bridge --ros-args \
  -p plc_ip:=192.168.6.1 \
  -p db_number:=5 \
  -p io_byte:=8
```

### 7.1 请求上电

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

对应 PLC：

```text
DB5.DBX8.0 = True
```

取消上电请求：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
```

对应 PLC：

```text
DB5.DBX8.0 = False
```

### 7.2 请求使能

```bash
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: true}"
```

对应 PLC：

```text
DB5.DBX8.1 = True
```

取消使能请求：

```bash
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: false}"
```

对应 PLC：

```text
DB5.DBX8.1 = False
```

### 7.3 清空命令

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

对应 PLC：

```text
DB5.DBX8.0 = False
DB5.DBX8.1 = False
DB5.DBX8.2 = False
```

### 7.4 查看状态

```bash
ros2 topic echo /plc/power_ready
ros2 topic echo /plc/enable_ready
ros2 topic echo /plc/robot_ready
ros2 topic echo /plc/e_stop
```

含义：

```text
/plc/power_ready   对应 DB5.DBX8.3
/plc/enable_ready  对应 DB5.DBX8.4
/plc/e_stop        对应 DB5.DBX8.5
/plc/robot_ready   等于 power_ready AND enable_ready AND NOT e_stop
```

当前阶段不要使用吸盘命令：

```bash
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_gripper_close std_srvs/srv/SetBool "{data: true}"
```

## 8. 联调步骤

### 8.1 检查网络

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

### 8.2 检查 ROS2 是否能连接 PLC

ROS2 电脑运行：

```bash
ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1
```

期望日志：

```text
成功连接到 PLC: 192.168.6.1, rack=0, slot=1, DB5.DBX8.0~7
PLC 状态更新: 上电反馈已就绪=False, 使能反馈已就绪=False, 机械臂就绪=False, 急停=False
```

如果连接失败，PLC 同学检查：

```text
PLC IP 是否为 192.168.6.1
PUT/GET 是否开启
DB5 是否存在
DB5 是否关闭优化块访问
PLC 是否已经下载最新程序
```

### 8.3 测试 ROS2 写 DB5

ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

PLC 同学在线监控，确认：

```text
DB5.DBX8.0 = True
```

ROS2 执行：

```bash
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: true}"
```

PLC 同学确认：

```text
DB5.DBX8.1 = True
```

清空命令：

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

PLC 同学确认：

```text
DB5.DBX8.0 = False
DB5.DBX8.1 = False
DB5.DBX8.2 = False
```

### 8.4 测试 PLC 输出

确认现场安全后，ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

PLC 同学确认：

```text
Q0.0 = True
DB5.DBX8.3 = True
```

然后 ROS2 执行：

```bash
ros2 service call /plc/set_enable std_srvs/srv/SetBool "{data: true}"
```

PLC 同学确认：

```text
Q0.1 = True
DB5.DBX8.4 = True
```

### 8.5 测试急停

触发急停后，PLC 同学确认：

```text
DB5.DBX8.5 = True
Q0.0 = False
Q0.1 = False
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
DB5.DBX8.5 = False
```

### 8.6 确认 ready

当：

```text
DB5.DBX8.3 = True
DB5.DBX8.4 = True
DB5.DBX8.5 = False
```

ROS2 应看到：

```text
/plc/power_ready = True
/plc/enable_ready = True
/plc/e_stop = False
/plc/robot_ready = True
```

只有在 `/plc/robot_ready=True` 且 `/plc/e_stop=False` 后，才允许启动机械臂 MoveIt 控制。

## 9. 当前阶段通过标准

本阶段联调通过，需要满足：

```text
1. ROS2 能连接 PLC。
2. ROS2 发送 set_power_on true 后，DB5.DBX8.0=True。
3. ROS2 发送 set_enable true 后，DB5.DBX8.1=True。
4. 清空命令后，DB5.DBX8.0/8.1/8.2=False。
5. 急停触发时，DB5.DBX8.5=True，Q0.0=False，Q0.1=False。
6. 上电就绪时，DB5.DBX8.3=True。
7. 使能就绪时，DB5.DBX8.4=True。
8. ROS2 能通过 /plc/power_ready、/plc/enable_ready、/plc/e_stop、/plc/robot_ready 看到状态。
```

## 10. 安全注意事项

1. ROS2 只负责写 DB 请求，PLC 必须负责安全互锁。
2. 急停必须能硬件切断机械臂上电和使能输出。
3. 当前阶段不要把吸盘、气泵、电磁阀接到 `DB5.DBX8.2`。
4. 第一次控制输出前，现场人员必须确认机械臂周围安全。
5. 第一次真实运动时，机械臂速度建议限制在 10% 左右。
