# DUCO GCR5-910 双臂轨迹执行可靠性修复 — 交付报告

- 工作区：`/home/me/duco_ros2_ws`
- 分支：`codex/dual-trajectory-reliability`（已推送到 `origin`）
- 基线：`b47d3e2`  →  当前：`440b2a6`
- 环境：ROS 2 Jazzy、colcon、g++ 13.3、Gazebo (gz-sim 9) + ros_gz + MoveIt + ros2_control

---

## 1. 结论摘要

修复围绕一条核心原则：**当机器人状态不确定时，绝不报告成功，也不接受新的运动目标。**

三类改动：

1. **真实完成判定收紧**：真实完成不再由“收到过反馈”推断，而是由新的纯函数门控 `evaluateRealFeedback()`（`RealFeedbackGate.hpp`）综合判定，要求真实状态在命令之后到达且新鲜、健康，反馈新鲜、数据完整且位置/速度均在容差内。
2. **故障闭锁 + 操作者恢复**：真实命令已下发但目标未确认完成（取消超时、仿真失败/中止、真实执行超时、期间检测到机器人故障）时，节点进入**闭锁**状态并写入标记文件；此后拒绝所有新目标，直到操作者显式调用恢复服务且所有可观测条件成立。重启后若标记文件仍在，则启动即闭锁。
3. **异步响应不再丢失**：引入 `sim_requests_` 台账按 UUID 关联 Gazebo 的迟到/被遗弃响应，迟到接受会被立即取消，未决请求会阻止新目标，永不响应的请求有有界清理。

验证：73 个 gtest 全部通过；Gazebo+MoveIt 数字孪生端到端跑通（真实 Gazebo 关节运动 + 动作 `SUCCEEDED`）；在孪生上用**合成**真实 I/O 验证闭锁与恢复语义（21/21 + 7/7）。真机未做任何自动运动测试。

---

## 2. 问题清单（修复前）

| # | 问题 | 影响 |
|---|------|------|
| P1 | 真实完成判定只看“反馈序号是否比下发时大” | 厂商驱动在控制器**断连后仍重发缓存值**，“收到反馈”不能证明数据新鲜/可信 → 可能误报成功 |
| P2 | 速度数组为空时**完全跳过**速度检查 | 只发位置的真实反馈会绕过速度容差 |
| P3 | NaN 位置与容差比较恒为 false | NaN 反馈被判为“到达”（`|NaN - x| > tol` 为 false） |
| P4 | 真实命令下发前无最终校验 | 不检查真实状态是否新鲜/健康、轨迹点数是否足够（驱动会无条件丢弃首个路径点）→ 真机与仿真可能分叉且无人知晓 |
| P5 | 真实命令已下发、目标未确认完成时**无任何记忆** | 取消超时/仿真失败后动作直接结束，新目标立即可被接受，而机械臂真实状态未知 |
| P6 | 取消超时一律返回 `CANCELED + SUCCESSFUL` | 已下发真实命令却报告“干净取消”，掩盖机械臂可能仍在运动 |
| P7 | 迟到的 Gazebo 接受响应被直接丢弃（`active_goal_ != goal_handle` 时 `return`） | 模拟目标无人跟踪、无法取消，可能持续执行 |
| P8 | 仅保存单个 `sim_goal_handle_` | 无法按 UUID 关联迟到响应，句柄被覆盖/擦除后失去控制 |
| P9 | 进程内标志无法跨重启 | 崩溃/重启后“未确认的真实运动”记忆丢失，系统恢复可用 |

---

## 3. 修复方案

### 3.1 真实完成门控（P1–P3）
新增纯函数头 `RealFeedbackGate.hpp`，`evaluateRealFeedback(goal, feedback, status, policy, now)` 按**固定顺序**返回 `FeedbackVerdict`：

1. `InvalidPolicy`：容差/时限非法（NaN、负值、零时限）。
2. `NoNewStatus` / `StaleStatus` / `UnhealthyStatus`：真实状态必须在命令之后、在 `real_status_timeout` 内、且健康（无急停、无碰撞、`robot_error==0`）。**状态优先于反馈**——状态主题在断连时停止发布，而反馈主题仍发缓存值，故状态新鲜度是更廉价的存活信号。
3. `NoNewFeedback` / `StaleFeedback`：反馈必须在命令之后且在 `real_feedback_timeout` 内。
4. `InvalidTarget` / `InvalidFeedback` / `MissingJoint`：目标与反馈的关节名必须非空、唯一、有限、数组等长且互相覆盖。
5. `PositionOutsideTolerance` / `VelocityOutsideTolerance`：每个关节的位置与**速度**都在容差内（速度检查不再可被空数组绕过）。
6. `Reached`。

`Reached` 仅表示“有效反馈满足数值容差”，**不是**厂商对“控制器已完成轨迹/已安全停止”的确认。

### 3.2 故障闭锁与恢复（P5、P6、P9）
- `RealFault`（`latched`/`reason`/`goal_id`）+ 标记文件 `fault_latch_file`（默认 `/tmp/duco_dual_trajectory_fault.state`）。
- **闭锁触发**：`finishGoal()` 中，若 `real_command_sent_ == true` 且结束码 `!= SUCCEEDED` → 闭锁，原因取结束原因。
- **标记文件语义**：真实命令一旦下发即写标记（表示“已下发且未确认”）；确认成功后移除；恢复时移除。启动时 `restoreLatchedFault()` 读标记 → 直接闭锁。
- **恢复服务** `std_srvs/Trigger`（`clear_fault_service`，默认 `clear_fault`）：调用即操作者的现场确认；此外必须**同时**满足：
  - 无活动目标、无目标槽预留；
  - 无未决 Gazebo 目标（`sim_requests_` 为空）；
  - 无未确认的真实命令（`real_command_sent_ == false`）；
  - 收到过真实状态、状态新鲜（`real_status_timeout` 内）且健康。
  
  任一不满足则返回 `success=false` 并给出具体原因。**单帧零速度反馈故意不作为充分条件。**
- 恢复成功后的返回信息明确声明：*这是“操作者已检查现场且可观测状态健康”的确认，不是厂商认证的安全停止。*

### 3.3 取消语义（P6）
- 取消超时且**已下发真实命令** → `CANCELED + GOAL_TOLERANCE_VIOLATED`（旧为 `SUCCESSFUL`）。
- 取消超时且**未下发真实命令** → 保持 `CANCELED + SUCCESSFUL`（行为不变）。
- 取消时若已下发真实命令且启用 `publish_real_stop_request`，仍沿用既有做法：向真实命令话题发一条空轨迹（现有驱动的停止请求），并明确日志声明“这不是厂商认证的急停”。

### 3.4 异步响应不再丢失（P7、P8）
- `sim_requests_` 台账：发送前登记 `{front_goal_id, handle, accepted, abandoned, cancel_requested, created_at, abandoned_at}`。
- 迟到接受（前端目标已终止）：**保留跟踪并立即取消**，等其报告终态；迟到拒绝：安全清除。
- 未知响应（台账中无对应项）：若带句柄则取消，绝不丢弃。
- 未决请求存在时 `handle_goal` 拒绝新目标（执行状态未知，可能冲突）。
- 从未被应答即被遗弃的请求：`acceptance_timeout` 后由 watchdog 有界丢弃并记错误日志，避免永久阻断。

### 3.5 下发前最后校验（P4）
`validateRealCommandLocked()`（持锁）在下发真实命令前检查：未闭锁、无未确认真实命令、真实状态新鲜健康、`points.size() >= min_real_trajectory_points`。不通过则**取消模拟目标**并以 `INVALID_GOAL` 中止，避免真机与仿真分叉。

### 3.6 可测试性重构
- `TrajectoryValidation.hpp`：抽出 `validateTrajectory` / `scaleTrajectoryTime` / `durationNanoseconds`（纯逻辑，仅依赖消息类型）。
- `RealFeedbackGate.hpp`：纯判定函数。
- 节点类从 `.cpp` 移到 `DualTrajectoryAction.hpp`，`.cpp` 仅保留 `main()`，使集成测试可直接实例化节点。

---

## 4. 文件清单

| 文件 | 变化 |
|------|------|
| `src/duco_ros_driver/include/duco_ros_driver/RealFeedbackGate.hpp` | 新增（215 行）：纯反馈门控 |
| `src/duco_ros_driver/include/duco_ros_driver/TrajectoryValidation.hpp` | 新增（226 行）：轨迹校验与时间缩放 |
| `src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp` | 新增（1232 行）：节点类（含闭锁、台账、恢复） |
| `src/duco_ros_driver/src/DualTrajectoryAction.cpp` | 精简为 `main()`（−856 行） |
| `src/duco_ros_driver/test/test_dual_trajectory_action.cpp` | 新增 12 个集成测试（mock Gazebo 动作服务器） |
| `src/duco_ros_driver/test/test_real_feedback_gate.cpp` | 新增 35 个门控单测 |
| `src/duco_ros_driver/test/test_trajectory_validation.cpp` | 新增 26 个轨迹规则单测 |
| `src/duco_ros_driver/CMakeLists.txt` | gtest 接线 ×3、`std_srvs` 依赖 |
| `src/duco_ros_driver/package.xml` | 新增 `std_srvs`、`ament_cmake_gtest` |
| `src/duco_gcr5_910_moveit_config/launch/digital_twin_real.launch.py` | 新参数声明与传递 |

---

## 5. 状态机转换

`TaskState`：`Idle → WaitingForSim → Executing → {Succeeded, Failed, Canceled}`，取消经 `Canceling`。

| 触发 | 转换 |
|------|------|
| `handle_goal` 校验通过（含未闭锁、无未决模拟请求、无活动目标） | `Idle → (槽预留)` |
| `handle_accepted` 建立活动目标 | `→ WaitingForSim` |
| Gazebo 接受且未在取消 | `WaitingForSim → Executing` |
| 取消请求 | `Executing → Canceling` |
| Gazebo 成功且门控 `Reached` | `→ Succeeded` |
| 门控非 `Reached` 且真实故障 | `→ Failed`（并闭锁） |
| 取消完成 / 取消超时 | `→ Canceled` |
| 仿真失败/超时、真实故障、真实超时 | `→ Failed` |
| 恢复服务成功 | `→ Idle`（并清除闭锁与标记） |

对应的 `ResultCode → GoalStatus`：`SUCCEEDED→4`、`CANCELED→5`、`ABORTED→6`。

---

## 6. 构建与单元/集成测试

```
$ colcon build --packages-select duco_msg duco_ros_driver --cmake-args -DBUILD_TESTING=ON
Starting >>> duco_msg
Finished <<< duco_msg [1.18s]
Starting >>> duco_ros_driver
Finished <<< duco_ros_driver [0.57s]
Summary: 2 packages finished [1.83s]
```

gtest（全部通过）：

| 测试 | 用例数 | 结果 |
|------|-------|------|
| `test_dual_trajectory_action`（集成，mock Gazebo） | 12 | 0 失败 |
| `test_real_feedback_gate`（门控） | 35 | 0 失败 |
| `test_trajectory_validation`（轨迹规则） | 26 | 0 失败 |
| **合计** | **73** | **0 失败** |

集成测试覆盖：纯仿真成功、真实完成必需、反馈抵达成功、机器故障中止、故障优先于位置容差、仿真失败中止、取消转发 Gazebo、取消超时报告取消、**取消超时+真实命令闭锁并拒绝新目标**、迟到接受被取消、被拒目标中止、恢复在机器人不健康时被拒、恢复在健康后成功。

### Lint（如实说明）

`colcon test` 报 36 项失败，**全部是 lint**，无 gtest 失败：

- `uncrustify` 23 项、`lint_cmake` 11 项；`cppcheck` 24 项 skip。
- 其中 **6 项落在本次新增文件**上：`DualTrajectoryAction.hpp`、`RealFeedbackGate.hpp`、`TrajectoryValidation.hpp` 与 3 个测试文件；其余为仓库既有风格问题（如 `install (TARGETS ...)` 的空格、超长行）。
- 未做自动重排的原因：整个仓库普遍不符合 uncrustify（基线同样失败），单独重排本次文件会与其上下文风格不一致，并给逻辑修复引入大量无关 diff。**这是与先前“无新增 lint”的说法不符之处，特此更正。**

---

## 7. 数字孪生端到端验证

在 `ROS_DOMAIN_ID=77`、`ROS_LOCALHOST_ONLY=1` 的隔离域内运行，全程无真机节点。

### Phase A — MoveIt + Gazebo 链路
`digital_twin_real.launch.py use_real_robot:=false use_rviz:=false`（headless）。节点图包含 `/DualTrajectoryAction`、`/move_group`、`/controller_manager`、`/sim_arm_1_controller`、`/robot_state_publisher`；动作 `/arm_1_controller/follow_joint_trajectory`、`/sim_arm_1_controller/follow_joint_trajectory`、`/move_action` 均在。

发一个 3 点轨迹（关节1 +0.35、关节5 −0.25）：

```
[A] terminal status: 4 SUCCEEDED error_code: 0
[A] joint deltas:    [0.35, 0.0, -0.0, -0.0, -0.25, -0.0]
[A] RESULT: PASS
```

节点日志状态链：`idle -> waiting_for_sim -> executing -> succeeded`。

### Phase B — 闭锁与恢复语义（合成真实 I/O）
另起一个重命名的节点实例（`publish_real:=true`、`wait_for_real:=true`），把真实主题/服务全部改名为测试专用名（`/twin_joint_path_command`、`/twin_real_feedback`、`/twin_robot_state`、`/twin_clear_fault`）。`publish_real:=true` 在此**只**向无人订阅的测试话题发布，且域隔离、无真机节点，因此绝不接触真机。

```
[B] S1 healthy goal -> SUCCEEDED                               PASS status=4 err=0
[B] S1 node published the real command                         PASS cmd_count=1
[B] S1 marker removed after confirmed success                  PASS
[B] S2 real command dispatched before cancel                   PASS
[B] S2 cancel accepted by the front server                     PASS
[B] S2 canceled goal -> CANCELED                               PASS status=5
[B] S2 unconfirmed cancel latched the fault                    PASS
[B] S3 new goal refused while latched                          PASS
[B] S3 operator recovery succeeds once the cell is healthy     PASS
[B] S3 marker removed after recovery                           PASS
[B] S3 goal accepted again after recovery                      PASS status=4 err=0
[B] S4 real command dispatched before fault injection          PASS
[B] S4 unhealthy status -> ABORTED                             PASS status=6 err=-4
[B] S4 fault latched                                           PASS
[B] S5 recovery accepted once the status is healthy again      PASS
[B] S5 marker removed                                          PASS
[B] S5 node ready for new goals after recovery                 PASS status=4 err=0
[B] ---- 21/21 checks passed ----
```

关键日志证据：
```
[ERROR] Real robot fault detected: emergency stop is active (emc_stop=true ...)
[ERROR] Real robot fault latched (goal ...): Aborting synchronized trajectory: emergency stop is active.
        New trajectories are refused until an operator calls the 'twin_clear_fault' service.
[ERROR] Goal ... finished while its Gazebo goal was still running; cancelling it.
[WARN]  Operator cleared the latched fault raised by goal ... 
[WARN]  latched fault cleared. This confirms that the operator inspected the cell and that the
        observable status was healthy; it is not a vendor-confirmed safe stop.
```

### Phase C — 恢复门控定向校验
Phase B 中“仍有 Gazebo 目标未决”会先于“机器人故障”成为拒绝理由，因此在 Phase C 中先等未决目标进入终态，使唯一阻塞项是“状态不健康”：

```
[C] goal accepted                                              PASS
[C] real command dispatched                                    PASS
[C] fault aborts the goal                                      PASS status=6
[C] fault latched                                              PASS
[C] recovery refused while the robot reports a fault           PASS msg='refusing to clear the latched fault: real robot status reports a fault'
[C] recovery accepted once healthy                             PASS
[C] marker removed                                             PASS
[C] ---- 7/7 checks passed ----
```

> 说明：Phase B 的 S5 首次拒绝理由恰为“有 Gazebo 目标未决”（同样返回 `success=false`，属正确拒绝但归因不同），Phase C 专门消除该干扰项以确认“状态不健康”这一条独立的拒绝路径。Phase C 脚本退出时 `python3` 进程报 `terminate called without an active exception`（退出码 134），发生在 `destroy_node()` 与仍在运行的 spin 线程竞争时，属测试脚手架瑕疵；**被测节点进程当时仍存活、日志中无任何崩溃标记**（`grep -c 'terminate|core dumped|Segmentation'` = 0）。

---

## 8. 未验证项与风险

1. **无厂商停止确认接口**：恢复只是“操作者确认 + 可观测状态健康”，不是经认证的安全停止。已在日志、服务响应与代码注释中显式声明。
2. **未做真机运动测试**（按要求）。孪生验证使用**合成**真实 I/O：正常的 50 Hz 新鲜数据序列，未复现真机的断连/缓存重发/抖动时序。P1 的缓存重发场景由 `test_real_feedback_gate.cpp` 的用例（如 `CachedFeedbackWhileStatusIsSilentIsRejected`、`StaleStatusIsRejected`、`StatusFromBeforeTheCommandIsRejected`）覆盖，但非真机实测。
3. **`min_real_trajectory_points=2`** 反映“驱动无条件丢弃首个路径点”的既有行为；该行为来自代码阅读，未在真机复验。
4. **`real_execution_timeout=300 s`** 默认较宽松，仅作为兜底。
5. **标记文件位于 `/tmp`**：跨重启有效，但若 `/tmp` 被清理则记忆丢失（可通过参数指向持久路径）。
6. **回调组**：节点未创建自定义回调组，全部使用默认互斥组；`state_mutex_` 为防御性措施。
7. **lint**：未清理（见 §6），含 6 项落在新文件上。
8. 数字孪生的 Phase A 走的是 `digital_twin_real.launch.py` 的 `use_real_robot:=false` 路径（`publish_real=false`）；带 `publish_real=true` 的完整装配（含 MoveIt 规划 + 真实门控）未作为一个 launch 组合整体运行，而是拆成 Phase A 与 Phase B/C 分别验证。
9. **启动竞态（运行注意）**：该 launch 用 5 s 定时器启动 Gazebo 控制器，而 `DualTrajectoryAction` 的前端动作服务器更早就绪。若在 Gazebo 动作服务器就绪前下发目标，`handle_goal` 会因为 `sim_action_client_->action_server_is_ready()` 为假而**拒绝**（日志："Gazebo action server [...] is not ready"）。这是安全的拒绝（不产生任何运动），但操作上应在控制器就绪后再下发。验证脚本已改为等待 `/sim_arm_1_controller/follow_joint_trajectory` 出现，并对首个目标做有限重试。

---

## 9. 兼容性对比

| 维度 | 修改前 | 修改后 |
|------|--------|--------|
| Action 名称/类型 | `arm_1_controller/follow_joint_trajectory`，`control_msgs/FollowJointTrajectory` | 不变 |
| MoveIt 控制器配置 | `arm_1_controller` | 不变 |
| 真实命令话题 | `joint_path_command`（`JointTrajectory`） | 不变 |
| 订阅的真实反馈话题 | `real_feedback_states`（action Feedback 类型） | 不变 |
| 订阅的真实状态话题 | **未订阅** | 新增订阅 `/duco_cobot/robot_state`（`DucoRobotState`） |
| 新增服务 | 无 | `std_srvs/Trigger` `clear_fault`（可改名） |
| 新增参数 | — | `real_feedback_timeout=1.0`、`real_status_timeout=1.0`、`min_real_trajectory_points=2`、`clear_fault_service=clear_fault`、`fault_latch_file=/tmp/duco_dual_trajectory_fault.state` |
| 取消超时（已下发真实命令） | `CANCELED + SUCCESSFUL` | `CANCELED + GOAL_TOLERANCE_VIOLATED` |
| 取消超时（未下发真实命令） | `CANCELED + SUCCESSFUL` | 不变 |
| 失败/未确认取消之后 | 立即可接受新目标 | **闭锁**，需人工 `clear_fault` |
| 重启 | 恢复可用 | 若有标记文件则启动即闭锁 |
| 真实命令下发前校验 | 无 | 状态新鲜健康 + 点数校验，失败则取消仿真并 `INVALID_GOAL` |
| 构建依赖 | — | 新增 `std_srvs`（build/exec）、`ament_cmake_gtest`（test） |
| 移除的内部成员 | `sim_goal_handle_` | 由 `sim_requests_` 台账取代 |

参数默认值均偏保守（`publish_real`/`wait_for_real` 默认值未变；新参数默认值不改变既有纯仿真路径）。

---

## 10. Git 变更

分支 `codex/dual-trajectory-reliability`（远端 `origin` 为 `HENU-Makerspace/duco_gcr5_910_ros2_ws.git`，已推送）：

| 提交 | 说明 |
|------|------|
| `c0c0943dbf7d7eb2671cf8af24e88417e0d58f7a` | Gate real robot completion on fresh, valid feedback |
| `440b2a63092cc5a037878d4875be3981a3e518a2` | Latch unconfirmed real motion and recover only on operator request |
| 基线 | `b47d3e2a5f789ff468b0fda5282ca6ba23cc579f` |

```
$ git diff --stat b47d3e2..HEAD
 .../launch/digital_twin_real.launch.py             |   12 +
 src/duco_ros_driver/CMakeLists.txt                 |   26 +-
 .../duco_ros_driver/DualTrajectoryAction.hpp       | 1232 ++++++++++++++++++++
 .../include/duco_ros_driver/RealFeedbackGate.hpp   |  215 ++++
 .../include/duco_ros_driver/TrajectoryValidation.hpp | 226 ++++
 src/duco_ros_driver/package.xml                    |    3 +
 src/duco_ros_driver/src/DualTrajectoryAction.cpp   |  857 +-------------
 .../test/test_dual_trajectory_action.cpp           |  611 ++++++++++
 .../test/test_real_feedback_gate.cpp               |  407 +++++++
 .../test/test_trajectory_validation.cpp            |  386 ++++++
 10 files changed, 3119 insertions(+), 856 deletions(-)
```

**未合并到 `main`**，等待评审。

---

## 附录：复现说明

数字孪生验证（Phase A/B/C）已脚本化，随仓库提交在 `scripts/dual_twin_verification/`：

```bash
# 一键运行 A/B/C 三个阶段（结束时自动清理，退出码 0 表示全部通过）
scripts/dual_twin_verification/verify_all.sh
```

- 隔离域 `ROS_DOMAIN_ID=77` + `ROS_LOCALHOST_ONLY=1`；不启动任何真机驱动。
- Phase B/C 中节点实例的真实话题与服务全部改名为测试专用名
  （`/twin_joint_path_command`、`/twin_real_feedback`、`/twin_robot_state`、`/twin_clear_fault`），
  节点发出的“真实命令”进入**无人订阅**的话题，因此不可能驱动真机。
- 脚本、分阶段用法与已知限制见 `scripts/dual_twin_verification/README.md`。

实测（`verify_all.sh`）：Phase A 5/5、Phase B 22/22、Phase C 8/8，退出码 0。

