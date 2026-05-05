# Siemens S7-1214C PLC 与 ROS2 吸盘联动实施说明

本文档用于现场 PLC 同学在 TIA Portal 中完成 S7-1214C 与 ROS2 的 DB 通讯、机械臂使能、吸盘气路控制和状态反馈。

## 1. 网络参数

```text
ROS2 电脑:     192.168.6.99
PLC:           192.168.6.1
机械臂控制柜:  192.168.6.4
相机:          192.168.6.5
PLC 编程电脑:  192.168.6.66
子网掩码:      255.255.255.0
```

ROS2 节点使用 Siemens S7 通讯直接访问 PLC 的 DB1。PLC 侧需要开放 PUT/GET 通讯，TCP 端口为 102。

## 2. TIA Portal 必须设置

1. 打开 PLC 设备属性。
2. 设置 PLC IP 为 `192.168.6.1`。
3. 在保护/连接机制中勾选：

```text
允许来自远程对象的 PUT/GET 通信访问
```

4. 新建全局数据块 `DB1`。
5. 右键 `DB1`，打开属性，取消勾选：

```text
优化的块访问
```

必须关闭优化块访问，否则 ROS2 不能用绝对地址 `DB1.DBX0.0` 这种方式读写。

## 3. DB1 数据类型与绝对地址

本项目主要使用 `Bool`。上电、吸气、急停、就绪、吸住检测都属于开关量，不需要 INT，也不需要 Float。

只有坐标、压力数值、速度比例这类连续数值才使用 `Real` 或 `Int`。

### 3.1 ROS2 写入 PLC 的命令区

这些变量由 ROS2 写入，PLC 读取后控制实际输出。

```text
绝对地址      数据类型  建议变量名                   含义
DB1.DBX0.0   Bool      ros_power_enable_cmd       ROS 请求机械臂上电/使能
DB1.DBX0.1   Bool      ros_suction_on_cmd         ROS 请求吸盘吸气
DB1.DBX0.2   Bool      ros_suction_release_cmd    ROS 请求破真空/放气
DB1.DBX0.3   Bool      ros_reset_cmd              ROS 请求报警复位
```

### 3.2 PLC 写给 ROS2 的状态区

这些变量由 PLC 写入，ROS2 读取后决定是否允许机械臂运动。

```text
绝对地址      数据类型  建议变量名                   含义
DB1.DBX1.0   Bool      plc_robot_ready            机械臂已上电且就绪
DB1.DBX1.1   Bool      plc_estop_active           急停触发
DB1.DBX1.2   Bool      plc_suction_on_feedback    吸盘吸气输出已打开
DB1.DBX1.3   Bool      plc_object_detected        真空开关检测到工件已吸住
DB1.DBX1.4   Bool      plc_alarm                  PLC 或气路报警
```

## 4. 推荐 PLC 物理 I/O 点

实际地址按现场接线为准，下面是推荐分配。

### 4.1 输入 I 点

```text
I0.0   robot_ready_input        机械臂就绪反馈
I0.1   estop_input              急停反馈，1 表示急停触发
I0.2   vacuum_switch_input      真空压力开关，1 表示吸住工件
I0.3   air_pressure_low_input   气压低报警，1 表示气压异常
I0.4   pump_fault_input         气泵/真空发生器故障，1 表示故障
```

### 4.2 输出 Q 点

```text
Q0.0   robot_enable_output      机械臂使能/上电继电器
Q0.1   suction_valve_output     吸盘吸气电磁阀或气泵启动
Q0.2   release_valve_output     破真空/放气电磁阀
Q0.3   alarm_output             报警灯或蜂鸣器
```

如果现场只有一个气泵，没有破真空阀，则 `Q0.2` 可以不用。

## 5. PLC 程序逻辑

下面按梯形图网络说明。变量名可以按现场习惯改，但 DB 地址必须保持一致。

### 网络 1：安全条件汇总

建议定义内部变量：

```text
safe_condition
```

逻辑：

```text
safe_condition =
    NOT estop_input
    AND NOT air_pressure_low_input
    AND NOT pump_fault_input
```

含义：没有急停、气压正常、气泵无故障时，才允许执行输出。

### 网络 2：机械臂上电/使能输出

```text
robot_enable_output =
    ros_power_enable_cmd
    AND safe_condition
```

对应实际地址：

```text
Q0.0 =
    DB1.DBX0.0
    AND NOT I0.1
    AND NOT I0.3
    AND NOT I0.4
```

注意：ROS2 只能提出请求，PLC 必须负责最终安全互锁。

### 网络 3：机械臂就绪反馈

```text
plc_robot_ready =
    robot_ready_input
    AND robot_enable_output
    AND safe_condition
```

对应实际地址：

```text
DB1.DBX1.0 =
    I0.0
    AND Q0.0
    AND NOT I0.1
    AND NOT I0.3
    AND NOT I0.4
```

如果现场没有机械臂就绪输入 `I0.0`，临时联调时可以先用：

```text
DB1.DBX1.0 = Q0.0 AND NOT I0.1
```

但正式运行建议必须接入真实就绪反馈。

### 网络 4：急停反馈

```text
plc_estop_active = estop_input
```

对应实际地址：

```text
DB1.DBX1.1 = I0.1
```

急停触发时还要强制关闭输出：

```text
I0.1 = True 时:
    Q0.0 = False
    Q0.1 = False
    Q0.2 = False
```

### 网络 5：吸盘吸气控制

吸气条件：

```text
suction_valve_output =
    ros_suction_on_cmd
    AND plc_robot_ready
    AND safe_condition
    AND NOT ros_suction_release_cmd
```

对应实际地址：

```text
Q0.1 =
    DB1.DBX0.1
    AND DB1.DBX1.0
    AND NOT I0.1
    AND NOT I0.3
    AND NOT I0.4
    AND NOT DB1.DBX0.2
```

含义：ROS 请求吸气，并且机械臂就绪，没有急停、没有气压/气泵报警、没有破真空请求，才打开吸盘。

### 网络 6：破真空/放气控制

如果现场有破真空电磁阀，建议使用 TON 定时器，避免放气阀长时间打开。

逻辑：

```text
当 ros_suction_release_cmd 上升沿触发:
    关闭 Q0.1 吸气
    打开 Q0.2 破真空
    保持 0.5 秒
    关闭 Q0.2
```

推荐定时器：

```text
TON_Release
PT = T#500ms
IN = DB1.DBX0.2
```

输出：

```text
Q0.2 = DB1.DBX0.2 AND NOT TON_Release.Q AND safe_condition
```

如果现场没有破真空阀，则释放逻辑简化为：

```text
DB1.DBX0.1 = False -> Q0.1 = False
```

### 网络 7：吸盘输出反馈

```text
plc_suction_on_feedback = suction_valve_output
```

对应实际地址：

```text
DB1.DBX1.2 = Q0.1
```

### 网络 8：吸住工件检测

如果是真空压力开关：

```text
DB1.DBX1.3 = I0.2
```

如果是真空模拟量传感器，则需要模拟量比较，例如：

```text
真空压力 <= -60 kPa -> DB1.DBX1.3 = True
否则 -> DB1.DBX1.3 = False
```

模拟量方案需要额外建立 `Real` 类型变量保存压力值；普通真空开关不需要。

### 网络 9：报警状态

```text
plc_alarm =
    air_pressure_low_input
    OR pump_fault_input
```

对应实际地址：

```text
DB1.DBX1.4 = I0.3 OR I0.4
```

报警输出：

```text
Q0.3 = DB1.DBX1.4 OR DB1.DBX1.1
```

### 网络 10：复位命令

ROS2 发出的复位位：

```text
DB1.DBX0.3 = ros_reset_cmd
```

建议 PLC 程序中把它作为报警复位请求。是否能真正复位，必须满足现场安全条件。

示例：

```text
如果 DB1.DBX0.3 = True
并且 I0.1 = False
并且故障源已消失:
    清除 PLC 内部报警锁存
```

不要用 ROS2 复位命令绕过急停或硬件故障。

## 6. ROS2 命令

启动 PLC 通讯节点：

```bash
cd /home/me/duco_ros2_ws
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run plc_s7_bridge plc_s7_bridge --ros-args -p plc_ip:=192.168.6.1 -p db_number:=1
```

查看状态：

```bash
ros2 topic echo /plc/robot_ready
ros2 topic echo /plc/e_stop
ros2 topic echo /plc/suction_on_feedback
ros2 topic echo /plc/object_detected
ros2 topic echo /plc/plc_alarm
```

机械臂上电/使能请求：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
```

机械臂下电/取消使能：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: false}"
```

吸盘吸气：

```bash
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: true}"
```

吸盘停止吸气：

```bash
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: false}"
```

破真空/放气：

```bash
ros2 service call /plc/set_suction_release std_srvs/srv/SetBool "{data: true}"
```

破真空结束后关闭释放请求：

```bash
ros2 service call /plc/set_suction_release std_srvs/srv/SetBool "{data: false}"
```

报警复位请求：

```bash
ros2 service call /plc/set_reset std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_reset std_srvs/srv/SetBool "{data: false}"
```

清空所有 ROS2 命令位：

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

旧命令仍然兼容：

```bash
ros2 service call /plc/set_gripper_close std_srvs/srv/SetBool "{data: true}"
```

这个旧命令现在等价于：

```bash
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: true}"
```

## 7. 联调步骤

### 第一步：只测网络

ROS2 电脑执行：

```bash
ping -c 3 192.168.6.1
nc -vz -w 2 192.168.6.1 102
```

必须能 ping 通，且 102 端口能连接。

### 第二步：只测 DB 通讯，不接实际输出

ROS2 启动节点。

PLC 同学在 TIA 在线监控 DB1，手动修改：

```text
DB1.DBX1.0 = True
DB1.DBX1.1 = False
DB1.DBX1.3 = True
```

ROS2 侧应能看到 topic 变化。

### 第三步：只测 ROS2 写 DB，不接实际输出

ROS2 执行：

```bash
ros2 service call /plc/set_power_on std_srvs/srv/SetBool "{data: true}"
ros2 service call /plc/set_suction_on std_srvs/srv/SetBool "{data: true}"
```

PLC 在线监控应看到：

```text
DB1.DBX0.0 = True
DB1.DBX0.1 = True
```

然后 ROS2 执行：

```bash
ros2 service call /plc/clear_commands std_srvs/srv/Trigger "{}"
```

PLC 应看到：

```text
DB1.DBX0.0 = False
DB1.DBX0.1 = False
DB1.DBX0.2 = False
DB1.DBX0.3 = False
```

### 第四步：接 PLC 输出，但先不让机械臂运动

确认：

```text
Q0.0 能控制机械臂使能继电器
Q0.1 能控制吸气
Q0.2 能控制破真空
急停能硬件切断 Q0.0/Q0.1/Q0.2
```

### 第五步：低速机械臂联调

只有当以下条件满足才启动运动：

```text
/plc/robot_ready = True
/plc/e_stop = False
/plc/plc_alarm = False
```

真实机械臂启动建议低速：

```bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py \
  use_rviz:=true \
  robot_speed:=0.1 \
  robot_acc:=0.15
```

## 8. 现场安全要求

1. ROS2 只能发请求，PLC 必须做安全互锁。
2. 急停必须硬件切断输出，不能只依赖 ROS2 软件。
3. 首次测试吸盘时，不要让机械臂运动。
4. 首次运动时速度限制在 10% 左右。
5. 急停测试必须单独做一次：运动过程中按下急停，确认 PLC 切断输出且 ROS2 收到 `/plc/e_stop=True`。
