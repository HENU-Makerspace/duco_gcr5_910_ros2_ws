# 双臂轨迹可靠性 — 数字孪生验证

本目录是 [`docs/DUAL_TRAJECTORY_RELIABILITY_REPORT.md`](../../docs/DUAL_TRAJECTORY_RELIABILITY_REPORT.md)
中数字孪生验证部分的脚本化复现。它在 Gazebo 数字孪生上驱动**真实的**
`DualTrajectoryAction` 节点，并用一个**合成的**“真实机器人”发布该节点所消费的
两个厂商话题（关节反馈 + 机器人状态）。

## 安全性

这些脚本**不可能**让真机运动：

- 使用隔离的 ROS 域（`ROS_DOMAIN_ID=77`）并启用 `ROS_LOCALHOST_ONLY=1`；
- 不启动任何真机驱动（`DucoDriver` / `DucoRobotStatus` / `DucoRobotControl`）；
- Phase B/C 中该节点的所有真实话题与服务都被改名为测试专用名
  （`/twin_joint_path_command`、`/twin_real_feedback`、`/twin_robot_state`、
  `/twin_clear_fault`），节点发出的“真实命令”进入**无人订阅**的话题。

请不要修改 `env.sh` 中的域名与话题名，除非你确认当前没有真机会话在运行。

## 前置条件

- 工作区已构建：`colcon build --packages-select duco_msg duco_ros_driver duco_gcr5_910_moveit_config`
  （孪生还需要 `duco_support`）；
- 已安装 `gz-sim`、`ros_gz`、MoveIt、`ros2_control`；
- 使用**系统 Python**：ROS 2 Jazzy 的 `rclpy` 不兼容 conda 的 `python3`。
  `env.sh` 通过 `TWIN_PYTHON`（默认 `/usr/bin/python3`）指定。

## 用法

```bash
# 一次跑完 A/B/C 三个阶段（结束时自动清理）
scripts/dual_twin_verification/verify_all.sh

# 或分步执行（自行管理后台进程）
scripts/dual_twin_verification/phase_a_moveit_gazebo.sh &   # 启动孪生
$TWIN_PYTHON -u scripts/dual_twin_verification/phase_a_check.py
scripts/dual_twin_verification/phase_b_node.sh &            # 被测节点实例
$TWIN_PYTHON -u scripts/dual_twin_verification/phase_b_checks.py
$TWIN_PYTHON -u scripts/dual_twin_verification/phase_c_check.py
scripts/dual_twin_verification/stop_all.sh
```

> `verify_all.sh` 用 `setsid` 启动两个后台进程组，并通过
> `$TWIN_LOG_DIR/*.pgid` 记录进程组号，退出时按进程组整体回收；
> `stop_all.sh` 只按“记录的进程组 / 孪生 launch 文件名 / `ros2 run ... DualTrajectoryAction`”
> 清理，**不会**按 `move_group`、`robot_state_publisher`、`ruby` 或裸的
> `DualTrajectoryAction` 等通用名字杀进程，以免误伤同时在跑的真机栈。

日志默认写到 `$TWIN_LOG_DIR`（`/tmp/duco_dual_trajectory_twin_logs`）。
退出码：0 表示所有检查通过。

## 各阶段内容

| 阶段 | 脚本 | 验证内容 |
|------|------|----------|
| A | `phase_a_check.py` | MoveIt + Gazebo 链路：一条 3 点轨迹经 `DualTrajectoryAction` 转发到 Gazebo，动作以 `SUCCEEDED` 结束，且 `/joint_states` 确认关节实际运动 |
| B | `phase_b_checks.py` | 健康目标成功；取消已下发真实命令的目标 → `CANCELED` **且闭锁**；闭锁期间拒绝新目标；操作者恢复后可用；注入急停 → `ABORTED` 且闭锁 |
| C | `phase_c_check.py` | 消除“Gazebo 目标未决”这一干扰后，确认“状态不健康”是**独立的**恢复拒绝理由 |

## 文件

| 文件 | 说明 |
|------|------|
| `env.sh` | 共享环境（隔离域、工作区路径、Python、标记文件、日志目录） |
| `twin_common.py` | 共享的合成真实 I/O 与目标/结果辅助函数 |
| `phase_a_moveit_gazebo.sh` | 启动孪生（headless，`use_real_robot:=false`） |
| `phase_a_check.py` | Phase A 检查 |
| `phase_b_node.sh` | 启动被测节点实例（`publish_real:=true`，话题改名为 `/twin_*`） |
| `phase_b_checks.py` | Phase B 场景 |
| `phase_c_check.py` | Phase C 场景 |
| `stop_all.sh` | 按记录的进程组收尾（不使用通用进程名匹配，避免误伤真机栈） |
| `verify_all.sh` | 一键运行 A/B/C 并汇总 |

## 已知限制

- 合成“真实机器人”恒定上报“完美跟随”（立即到达目标、零速度），因此**不覆盖**
  真机的断连、缓存重发与抖动时序；那些场景由 `test_real_feedback_gate.cpp`
  的单元测试覆盖。
- 该验证不会、也不能证明“经认证的安全停止”：厂商没有停止确认接口。
- `phase_b_node.sh` 显式设置 `fault_recovery_enabled:=true`（该参数默认为
  `false`），因为本脚本要验证人工恢复流程本身；真机部署应由操作者按需开启。
- `phase_b_node.sh` 使用 `wait_for_real:=true`（闭环模式）。若使用
  `wait_for_real:=false`，每次实体轨迹结束后都会保留“实体完成未确认”的闭锁，
  需要操作者恢复后才能下发下一条实体轨迹——这是刻意的保守行为。

## 参数与行为变化（相对上一轮）

- `real_feedback_timeout` → `real_feedback_max_age`；`real_status_timeout` →
  `real_status_max_age`（语义不变，仅改名以与需求文档一致）。
- 新增 `real_require_velocity`（默认 `true`）、`real_motion_state_topic`
  （诊断话题）、`fault_recovery_enabled`（默认 `false`）、`latch_on_real_fault`
  （默认 `true`）、`real_unhealthy_robot_states`（默认空）。
- 节点默认值 `publish_real` 由 `true` 改为 `false`：实体通道必须显式开启。
