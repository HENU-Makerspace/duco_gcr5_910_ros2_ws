# DUCO GCR5-910 双轨迹节点 — 第二轮安全加固测试报告

- 分支：`codex/dual-trajectory-safety-fixes`
- 被测提交：`b6dfbca`（*Separate real motion permission from the action goal*）
- 基线提交：`96bbd6e`
- 测试日期：2026-10-17
- 主机：Ubuntu 24.04 / ROS 2 **Jazzy** / GCC 13 / x86_64
- 测试对象：`duco_ros_driver`（`DualTrajectoryAction`）、`duco_gcr5_910_moveit_config`（launch）

> 本报告中的每一项结果都来自本机**实际执行**的命令。文中没有推断出的
> 「应当通过」，也没有任何真机运动测试——真机运动测试**未执行**（未获授权），
> 这一点在第六节单独说明。

## 一、测试环境与命令

```bash
source /opt/ros/jazzy/setup.bash
cd ~/duco_ros2_ws

# 1) 编译
colcon build --packages-select duco_msg duco_ros_driver duco_gcr5_910_moveit_config

# 2) 单元 + 集成测试（gtest）
colcon test --packages-select duco_ros_driver --event-handlers console_direct+

# 3) 结果汇总
colcon test-result --verbose

# 4) 数字孪生端到端验证（隔离域 77 + ROS_LOCALHOST_ONLY=1）
bash scripts/dual_twin_verification/verify_all.sh
```

编译结果：

```
Starting >>> duco_ros_driver
Finished <<< duco_ros_driver [17.0s]
Summary: 1 package finished
```

无编译告警（对本次改动的文件单独 `touch` 后全量重编，`grep -E "warning|error"`
输出为空）。

## 二、单元测试（纯逻辑，可独立编译）

三个纯逻辑测试**不依赖 ROS 图**，可脱离 `colcon` 用一条 `g++` 直接编译运行，
因此结论不依赖任何 ROS 运行时状态。（其中两个额外需要 rosidl 头文件路径。）

```bash
g++ -std=c++17 -Wall -Wextra -Iinclude \
    -I/opt/ros/jazzy/include/{trajectory_msgs,builtin_interfaces,rosidl_runtime_c,rosidl_typesupport_interface,std_msgs,geometry_msgs,sensor_msgs,rcutils} \
    test/test_real_motion_interlock.cpp -lgtest -lgtest_main -pthread -o /tmp/t_interlock
/tmp/t_interlock
```

| 测试二进制 | 用例数 | 结果 | 覆盖内容 |
|------------|--------|------|----------|
| `test_real_motion_interlock` | **14** | **全部通过** | 状态机（上锁/结清/粘滞/严重度/代次校验/空闲故障/仅操作者恢复） |
| `test_real_feedback_gate` | **39** | **全部通过** | 数值有效性、维度、空名/重名、缺失/部分/非有限速度、按名匹配、时效、健康、策略非法 |
| `test_trajectory_validation` | **26** | **全部通过** | （上一轮已有）轨迹本身合法性 |
| **小计** | **79** | **全部通过** | |

其中新增/加强的关键用例（与任务书条目对应）：

- §3.1 数值有效性：`NaNAndInfPositionsAreRejected`、`NaNAndInfVelocitiesAreRejected`、
  `PositionAndNameSizeMismatchIsRejected`、`EmptyJointNamesAreRejected`、
  `DuplicateFeedbackJointNamesAreRejected`、`MissingGoalJointIsRejected`、
  `EmptyTargetIsRejected`、`TargetDimensionMismatchIsRejected`、
  `NonFiniteTargetIsRejected`、`NonFiniteToleranceIsInvalidPolicy`、
  `NegativeToleranceIsInvalidPolicy`、`ZeroMaxAgeIsInvalidPolicy`。
- §3.1 速度可选（新增 `require_velocity`）：
  `MissingVelocitiesAreRejectedWhenVelocityIsRequired`、
  `MissingVelocitiesAreNotTreatedAsZeroVelocity`（**缺数组 ≠ 零速度**）、
  `PartialVelocityArrayIsRejectedEvenWhenVelocityIsOptional`、
  `NonFiniteVelocityIsRejectedEvenWhenVelocityIsOptional`。
- §3.1 顺序无关：`FeedbackJointOrderMayDifferFromTheTargetOrder`、
  `ExtraUnrelatedJointsAreTolerated`。
- §3.2 时效：`StaleFeedbackIsRejected`、`StaleStatusIsRejected`、
  `TimestampInTheFutureIsRejected`、**`CachedFeedbackWhileStatusIsSilentIsRejected`**
  （复现「断连后状态话题静默而反馈话题仍重发缓存」）、
  `FreshFeedbackButAgedOutStatusIsRejected`。
- §4 联锁：`GoalEndingWithoutConfirmationLatches`、
  `LatchedInterlockRefusesNewCommands`、`AStaleGenerationCannotClearTheInterlock`、
  `UncertaintyEscalatesFromAwaitingConfirmation`、
  `FaultIsTheMostSevereAndIsSticky`、`FaultCanBeRaisedWhileIdle`、
  `OnlyOperatorRecoveryClearsALatchedState`。

CMake 中的 gtest 目标（`colcon test` 输出 "gtest 4 tests"，即四个测试进程）：

```
./build/duco_ros_driver/test_real_motion_interlock      -> 14 passed
./build/duco_ros_driver/test_real_feedback_gate         -> 39 passed
./build/duco_ros_driver/test_trajectory_validation      -> 26 passed
```

## 三、集成测试（`test_dual_trajectory_action`）

27 个用例，覆盖 Action 层、取消语义、发布顺序、闭锁与恢复。全部通过：

```
[==========] Running 27 tests from 1 test suite.
[==========] 27 tests from 1 test suite ran. (31842 ms total)
[  PASSED  ] 27 tests.
```

| # | 用例 | 对应条目 | 验证点 |
|---|------|----------|--------|
| 1 | `SimulationOnlyGoalSucceeds` | §7 | 纯仿真模式仍可正常完成 |
| 2 | `RejectedGoalIsAborted` | §7 | Gazebo 拒收 → `ABORTED` |
| 3 | `FailedSimulationIsAborted` | §7 | 仿真失败 → `ABORTED` |
| 4 | `CancelWhileExecutingIsForwardedToGazebo` | §5 | 取消正确转发 |
| 5 | `CancelTimeoutStillReportsCanceled` | §5 | 无实体命令时超时取消仍报 `CANCELED`（未过度保守） |
| 6 | `RealCompletionIsRequiredWhenWaitForRealIsSet` | §7 | `wait_for_real=true` 时仅仿真成功**不**算完成 |
| 7 | `RealCompletionSucceedsWhenFeedbackReachesTheGoal` | §3/§7 | 真实反馈到位 → `SUCCEEDED` |
| 8 | `FaultReportedWhileExecutingAbortsTheGoalAndLatches` | §4.2 | 执行中故障 → 中止并闭锁，且闭锁**跨目标存活** |
| 9 | `FaultReportedWhileIdleRejectsRealGoals` | §4.2 | **空闲时**上报故障即闭锁，后续目标被拒 |
| 10 | `ConfiguredControllerStateIsTreatedAsAFault` | §3.2 | 操作者声明 `[5]` 后，状态 5 视为故障并闭锁 |
| 11 | `UndeclaredControllerStateIsNotInterpreted` | §3.2 | 未声明时**不猜测**该未公开枚举 |
| 12 | `SimulationOnlyModeIgnoresRealRobotFaults` | §4.3 | 纯仿真通道与实体故障隔离 |
| 13 | `CancelTimeoutWithRealCommandLatchesAndBlocksNewGoals` | §5.2 | 已下发实体命令的取消超时 → 闭锁 + 拒绝新目标 |
| 14 | `RecoveryIsRefusedWhileTheRobotIsUnhealthy` | §4.4 | 不健康时拒绝恢复 |
| 15 | `RecoverySucceedsOnceTheRobotIsHealthyAgain` | §4.4 | 健康后允许恢复 |
| 16 | `LateAcceptanceAfterAcceptanceTimeoutIsCancelled` | §5/§6 | 迟到的接受被取消而非静默丢弃 |
| 17 | **`UnconfirmedRealMotionBlocksTheNextRealTrajectory`** | **§7 核心** | `wait_for_real=false` + 实体：动作报仿真成功，但**实体运动未被确认 → 下一条实体轨迹被拒**，恢复后才放行 |
| 18 | `ConfirmedCompletionReleasesTheChannelForTheNextGoal` | §7 | 已确认完成的实体运动正确释放通道（未过度锁定） |
| 19 | `RealExecutionTimeoutLatchesAndBlocksNewGoals` | §4.1/§4.3 | 实体执行超时 → 中止 + 闭锁 + 拒绝新目标 |
| 20 | `InterlockIsRestoredFromTheMarkerFile` | §4.4 | 重启后由 marker 恢复上锁；**纯仿真不受历史 marker 影响** |
| 21 | `RejectedCancelIsNotReportedAsACleanCancellation` | §5.1 | Gazebo 拒绝取消 → 超时路径给 `GOAL_TOLERANCE_VIOLATED`，不报干净取消 |
| 22 | `CancelLosingTheRaceReportsTheRealOutcome` | §5.1 | 取消输给完成 → 上报实际完成，不伪装成取消 |
| 23 | `RepeatedCancelRequestsTerminateTheGoalOnce` | §5.2 | 重复取消只终结一次，无重复终结/崩溃 |
| 24 | `SecondGoalWhileOneIsActiveIsRejected` | §7 | 冲突目标被拒 |
| 25 | **`AStopIsNeverOvertakenByTheMotionItBelongsTo`** | **§6 核心** | 监听真实命令话题，断言**不存在「停止之后又出现同一目标的运动」** |
| 26 | `RecoveryIsDisabledUnlessExplicitlyEnabled` | §4.4 | 默认 `fault_recovery_enabled=false`，服务拒绝且给出原因 |
| 27 | `DiagnosticStateSeparatesSimulationAndRealOutcome` | §7 | 诊断话题同时体现 `sim=succeeded` 与 `real=unconfirmed`、`latched=1`、`state=awaiting_confirmation` |

用例 17、25 是本轮的核心回归护栏：前者防止「仿真结束＝实体安全」，
后者以实际发布次序证明停止不会被运动反超。

## 四、数字孪生端到端验证（Gazebo + MoveIt + 真实节点）

```bash
bash scripts/dual_twin_verification/verify_all.sh
# exit=0
```

结果：

```
[A] ---- 5/5 checks passed ----      # Gazebo 轨迹真实执行
[B] ---- 22/22 checks passed ----    # 闭锁 / 取消 / 故障 / 恢复场景
[C] ---- 8/8 checks passed ----      # 故障注入 + 恢复拒绝
Phase A: PASS
Phase B: PASS
Phase C: PASS
```

阶段 A（真实 Gazebo 执行）：动作服务器在线、`/joint_states` 在线、
初始位姿读取、目标到达终态（`status=4 error_code=0`）、
关节位移与命令一致（`[0.35, 0.0, -0.0, -0.0, -0.25, -0.0]`）。

阶段 B（隔离节点实例，`publish_real:=true`、`wait_for_real:=true`、
`fault_recovery_enabled:=true`，全部真实话题/服务改名为 `twin_*`）：

| 场景 | 关键断言 | 结果 |
|------|----------|------|
| S1 | 健康目标 `SUCCEEDED`；已发布实体命令；确认成功后 marker 被删除 | PASS |
| S2 | 取消前实体命令已下发；取消被接受；结果 `CANCELED`；未确认取消触发闭锁 | PASS |
| S3 | 闭锁期间拒绝新目标；健康后恢复成功；marker 删除；恢复后可再次下发 | PASS |
| S4 | 故障注入 → `ABORTED`；闭锁生效 | PASS |
| S5 | 存在未结清 Gazebo 目标时**拒绝**恢复；可观测条件满足后允许 | PASS |

阶段 C：从「未上锁」状态开始 → 故障中止目标 → 闭锁 → **机器人仍报故障时
拒绝恢复** → 健康后允许恢复 → marker 删除。8/8 通过。

隔离说明：整个验证运行在 `ROS_DOMAIN_ID=77` + `ROS_LOCALHOST_ONLY=1`，
不启动任何真机驱动（`DucoDriver`/`DucoRobotStatus`/`DucoRobotControl`），
节点的真实话题/服务全部改名到无人订阅的 `twin_*` 名字上，
因此**不可能**让真机运动。

清理确认（验证结束后）：

```
marker 文件: 不存在
残留进程:    仅 ros2 daemon（正常）
```

## 五、Lint / 静态检查（如实记录，本分支未整改）

`colcon test-result --verbose`：

```
Summary: 178 tests, 0 errors, 37 failures, 26 skipped
```

逐项拆解（来自 xunit XML）：

| 检查器 | 数量 | 说明 |
|--------|------|------|
| `uncrustify` | **24 / 26 个文件不符合风格** | 仓库整体历史风格问题，非本轮引入 |
| `lint_cmake` | **11 条信息**（`CMakeLists.txt` 缩进/行宽） | 同上 |
| `cppcheck` | 26 skipped | 本机未安装，未执行 |
| `copyright` / `cpplint` | 已按仓库既有设置跳过 | 同上 |

- **功能性测试失败数：0。**
- 本分支新增/修改的文件中，`src/DualTrajectoryAction.cpp` 与
  `test/test_real_motion_interlock.cpp` 的 uncrustify **无差异**；
  其余 5 个（`DualTrajectoryAction.hpp`、`RealFeedbackGate.hpp`、
  `RealMotionInterlock.hpp`、`test_dual_trajectory_action.cpp`、
  `test_real_feedback_gate.cpp`）沿用仓库既有代码风格，与仓库其余 19 个文件
  一样不满足 `uncrustify`。
- 刻意**未**做 lint 整改：任务书未要求，且改动仓库风格基线会掩盖真实差异。
  结果按原样记录，不做任何粉饰。

## 六、未执行/未复现的测试及原因

| 项目 | 原因 |
|------|------|
| **真机运动测试** | **未获授权**。任务书明确禁止在未授权时命令真机运动；未进行任何实机连接。 |
| 真机急停/碰撞的物理联动 | 同上；仅通过合成状态话题验证逻辑闭锁。 |
| 厂商停止确认 | **厂商无此接口**（见开发报告 §2.3），因此「停止已确认」这一状态**无法验证**，代码中标注为无法确认。 |
| 真机断连/缓存重发时序 | 合成真实 I/O 恒定「完美跟随」，无法复现；仅由反馈门控单元测试覆盖逻辑分支。 |
| 多节点恢复滥用（授权绕过） | 服务为 `std_srvs/Trigger`，ROS 2 自身无鉴权机制 → 属**已知局限**，以默认关闭补偿。 |
| 长时间压力/竞态专项 | 未做；已在报告中列为未验证项。 |

## 七、结论

- 单元 + 集成共 **106 个用例全部通过（0 失败）**；
- 数字孪生端到端 **A 5/5、B 22/22、C 8/8，exit=0**；
- 全部失败项均为 lint 风格问题，且**均为仓库历史基线**，与本轮改动无关；
- **未做真机运动测试**；本报告的结论仅限于软件逻辑与仿真，不能替代实机验收与
  硬件安全回路。

复现命令汇总见第一节。
