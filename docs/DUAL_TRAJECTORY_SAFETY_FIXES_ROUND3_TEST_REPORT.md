# DUCO GCR5-910 双轨迹节点 — 第三轮可靠性修复测试报告

- 分支：`codex/dual-trajectory-safety-fixes`
- 被测提交：`f5b2c42`
- 环境：ROS 2 Jazzy / Ubuntu 24.04 / x86_64；`ROS_DOMAIN_ID=77`、`ROS_LOCALHOST_ONLY=1`（数字孪生）
- 结论：**C++ 单元与集成测试 130/130 通过；数字孪生端到端 Phase A/B/C 全部通过；
  `colcon test` 剩余失败全部是仓库既有的 lint（uncrustify / lint_cmake），0 个功能失败。
  未执行任何实机运动测试（未获授权）。**

> 本报告中所有命令与结果均为实际执行所得；未执行的项目在第七节单独列出。

## 一、测试对象与新增测试

| 测试文件 | 用例数 | 说明 |
|----------|--------|------|
| `test/test_interlock_persistence.cpp` | 15（**新增**） | 纯 C++/POSIX 持久化模块：默认路径、原子替换、临时文件残留、写失败、删除失败、空记录、不可读记录 |
| `test/test_dual_trajectory_action.cpp` | 36（原 27 → +9） | mock Gazebo 集成测试，新增取消语义 4 例、持久化 5 例 |
| `test/test_real_feedback_gate.cpp` | 39（未改） | 反馈门控 |
| `test/test_real_motion_interlock.cpp` | 14（未改） | 联锁状态机 |
| `test/test_trajectory_validation.cpp` | 26（未改） | 轨迹校验 |
| **合计** | **130** | |

新增/修改的集成测试（与三个问题对应）：

| 用例 | 覆盖 |
|------|------|
| `CancelTimeoutIsReportedAsAbortedNotCanceled` | 问题一：取消超时 → `ABORTED` + `GOAL_TOLERANCE_VIOLATED` |
| `CancelTimeoutWithRealCommandLatchesAndBlocksNewGoals` | 问题一 + 问题二：超时 → `ABORTED`、闭锁、新目标被拒、持久记录存在 |
| `RejectedCancelIsNotReportedAsACleanCancellation` | 问题一：取消被拒绝 + 超时 → `ABORTED` |
| `RepeatedCancelRequestsTerminateTheGoalOnce` | 问题一：重复取消 → 只终结一次，且不谎报取消成功 |
| `CancelWhileExecutingIsForwardedToGazebo`（既有） | 问题一：Gazebo 真的回复 CANCELED → 仍为 `CANCELED` |
| `CancelLosingTheRaceReportsTheRealOutcome`（既有） | 问题一：取消输给正常完成 → 仍为 `SUCCEEDED` |
| `AUsableStateFileIsReportedAsReady` | 问题二：诊断暴露 `persistence=ok` |
| `RequiredPersistenceRefusesRealMotionWhenTheStateFileIsUnusable` | 问题二：不可写 + 必须持久化 → 目标被拒、无实体命令、诊断 `persistence=unavailable` |
| `OptionalPersistenceStillAllowsRealMotion` | 问题二：不可写 + 显式声明可放弃持久化 → 仍可下发（打印 WARN） |
| `ARuntimeWriteFailureBlocksRealMotion` | 问题二：运行期目录变只读 → 写失败被检测、实体目标被拒、目录恢复后通道自愈 |
| `ARestartWithARecordLatchesUntilAnOperatorRecovers` | 问题二：重启恢复 → 闭锁 → 人工恢复 → 可用 |
| `AnEmptyRecordStillLatchesOnRestart` | 问题二：空记录也必须闭锁（保守语义） |
| `TheDurableRecordExistsWhenTheResultIsDelivered` | 问题三：结果送达时持久记录必须已存在（同一事务） |
| `UnconfirmedFinishAndANewGoalAreNeverInconsistent` | 问题三：任务终结 ↔ 新目标到达（3 轮循环，结束即刻发新目标） |
| `ASuccessfulRecoveryNeverErasesAConcurrentFault` | 问题三：人工恢复 ↔ 故障消息到达（8 轮竞争，断言故障不会被抹掉） |

新增的持久化单元测试（15 例）：`DefaultPathIsAPerUserStateDirectory`、
`DefaultPathFollowsXdgStateHome`、`PrepareFailsWithoutAConfiguredPath`、
`PrepareCreatesTheDirectoryChain`、`PrepareFailsOnAnUnusableDirectory`、
`WriteThenReadRoundTrips`、`WriteAtomicallyReplacesPreviousContent`、
`WriteLeavesNoTemporaryFileBehind`、`AStaleTemporaryFileIsIgnored`、
`ReadOfAMissingMarkerIsNotAnError`、`AnEmptyMarkerStillExistsSoARestartLatches`、
`RemoveIsIdempotentAndDropsTemporaryFiles`、`WriteFailsWhenThePathCannotBeUsed`、
`RemoveFailsWhenTheDirectoryBecomesUnwritable`、
`AnUnreadableMarkerIsReportedInsteadOfAssumedClean`。

## 二、构建

```bash
cd /home/me/duco_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build                       # 全部 6 个包
colcon build --packages-select duco_ros_driver
```

结果：全部成功，**无编译错误、无编译警告**（`-Wall -Wextra` 干净）。

```
Summary: 6 packages finished
Finished <<< duco_ros_driver [17.9s]
```

## 三、C++ 测试（实际执行）

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./build/duco_ros_driver/test_interlock_persistence
./build/duco_ros_driver/test_real_motion_interlock
./build/duco_ros_driver/test_real_feedback_gate
./build/duco_ros_driver/test_trajectory_validation
./build/duco_ros_driver/test_dual_trajectory_action
```

结果：

| 二进制 | 用例数 | 通过 | 失败 | 耗时 |
|--------|--------|------|------|------|
| `test_interlock_persistence` | 15 | 15 | 0 | 31 ms |
| `test_real_motion_interlock` | 14 | 14 | 0 | <1 ms |
| `test_real_feedback_gate` | 39 | 39 | 0 | <1 ms |
| `test_trajectory_validation` | 26 | 26 | 0 | <1 ms |
| `test_dual_trajectory_action` | 36 | 36 | 0 | 46.9 s |
| **合计** | **130** | **130** | **0** | |

关键日志片段（取消超时语义，实际输出）：

```
[INFO]  Task ... state: executing -> canceling
[WARN]  Cancel requested for goal ...: accepting a cancel request does not mean the robot has stopped
[INFO]  Task ... state: canceling -> canceled            <- Gazebo 真的回复 CANCELED 的用例
[WARN]  Goal ... completed with status canceled: Gazebo canceled the trajectory
```

```
[       OK ] DualActionTest.CancelTimeoutIsReportedAsAbortedNotCanceled
[       OK ] DualActionTest.CancelTimeoutWithRealCommandLatchesAndBlocksNewGoals
[       OK ] DualActionTest.RejectedCancelIsNotReportedAsACleanCancellation
[       OK ] DualActionTest.RepeatedCancelRequestsTerminateTheGoalOnce
```

持久化闸门（实际输出）：

```
[ERROR] Interlock state file [/proc/self/duco_test_marker/marker.state] cannot be used reliably: cannot create the directory chain ...
[ERROR] Rejecting goal ...: real_persistence_required is true but the interlock state file [...] cannot be written (...)
[       OK ] DualActionTest.RequiredPersistenceRefusesRealMotionWhenTheStateFileIsUnusable
```

运行期写失败（实际输出）：

```
[ERROR] Unable to persist the real motion interlock to [.../marker.state]: cannot open [.../marker.state.tmp] for writing: Permission denied
[ERROR] Real robot fault (goal -): robot error code 42. ...
[       OK ] DualActionTest.ARuntimeWriteFailureBlocksRealMotion
```

### 3.1 测试基础设施的两处修正（重要，否则测试会得出错误结论）

1. **mock Gazebo 必须在「接受取消但不回结果」时持有 goal handle**。
   原先 mock 线程直接返回、释放最后一个 `ServerGoalHandle` 引用，rclcpp_action
   会用 `CANCELING` 状态回答挂起的 `GetResult` 请求，看起来就像控制器回了
   `CANCELED`。结果是「取消超时」用例实际上从未走到超时分支（本轮已用
   `fprintf` 调试确认：mock 侧 `answer_cancel_with_result=false` 且未发送结果，
   节点侧却收到 `CANCELED`）。现在 mock 把 handle 保存在成员容器里，
   才真正模拟「接受了取消、但永不报告终止状态」。
2. **恢复服务需要先等到第一帧真实状态**。节点在没有任何 `robot_status` 之前
   会正确拒绝恢复（`no real robot status has been received`），测试改为
   轮询等待（`waitForRecovery`），而不是假设状态已到达。

## 四、并发/时序测试

| 用例 | 断言的不变量 | 结果 |
|------|--------------|------|
| `TheDurableRecordExistsWhenTheResultIsDelivered` | 客户端拿到结果时，持久记录**已经**存在（槽位释放与持久化属于同一事务） | 通过 |
| `UnconfirmedFinishAndANewGoalAreNeverInconsistent` | 未确认结束之后、人工恢复之前，任何新实体目标都必须被拒；恢复后必须被接受（3 轮，结束即刻发新目标，无 sleep） | 通过 |
| `ASuccessfulRecoveryNeverErasesAConcurrentFault` | 故障在恢复请求**之前**已上线时，无论两个回调谁先拿到锁，故障最终必须处于闭锁状态（8 轮） | 通过 |
| `AStopIsNeverOvertakenByTheMotionItBelongsTo`（既有） | 同一目标的停止请求不会被它自己的运动命令超越 | 通过 |
| `SecondGoalWhileOneIsActiveIsRejected`（既有） | 活动目标期间第二个目标被拒 | 通过 |

**诚实说明（限制）**：`ASuccessfulRecoveryNeverErasesAConcurrentFault` 断言的是
「必须永远成立」的不变量，但**不能**确定性地复现旧实现中两个临界区之间的
微秒级窗口。本轮做过实测：把 `clearFaultCallback()` 临时改回「先检查、后无条件
解除」的旧写法并重新构建，该压力测试**仍然通过**。因此：

- 原子性由「单临界区 + 持久化先于释放」的代码结构保证；
- 压力测试的作用是把不变量固化成回归防线，而不是证明竞态不存在；
- 报告不声称「测试证明了旧竞态可复现」。

真正**可确定性区分**新旧实现的用例是
`ARuntimeWriteFailureBlocksRealMotion`（旧实现只打印一句 ERROR 就继续发布实体
轨迹，不会把 `persistence` 置为不可用，也不会拒绝目标）。

## 五、数字孪生端到端（实际执行）

```bash
cd /home/me/duco_ros2_ws
bash scripts/dual_twin_verification/verify_all.sh
```

结果：**退出码 0**

```
== summary ==
Phase A: PASS          (5/5 检查)
Phase B: PASS          (22/22 检查)
Phase C: PASS          (8/8 检查)
```

关键检查（实际输出）：

```
[B] node starts unlatched (or is clearable)                    PASS msg='no latched real robot fault'
[B] S1 healthy goal -> SUCCEEDED                               PASS status=4 error_code=0
[B] S1 marker removed after confirmed success                  PASS
[B] S2 canceled goal -> CANCELED                               PASS status=5
[B] S2 unconfirmed cancel latched the fault                    PASS
[B] S3 new goal refused while latched                          PASS
[B] S3 operator recovery succeeds once the cell is healthy     PASS
[B] S3 marker removed after recovery                           PASS
[B] S4 unhealthy status -> ABORTED                             PASS status=6
[B] S4 fault latched                                           PASS
[B] S5 recovery refused while latched and unresolved           PASS msg='refusing to clear the latched fault: a Gazebo goal is still outstanding'
[C] fault aborts the goal                                      PASS status=6
[C] recovery refused while the robot reports a fault           PASS
```

说明：

- Phase A 用真实 launch 文件（`use_real_robot`/`wait_for_real` 默认 false），
  验证 MoveIt + Gazebo 正常联动（5/5）；
- Phase B 用 `publish_real:=true` 的隔离节点实例，`real_*` 话题全部改名、
  无真实机器人节点、独立 ROS domain，因此**不接触机械臂**；
- S2 的 `status=5`（CANCELED）是**预期**的：真实 `joint_trajectory_controller`
  会正常应答取消，走的是「Gazebo 回复 CANCELED」分支，本轮未改动该映射。
  「取消被拒绝」「取消超时」两种情形**无法**用真实 ros2_control 控制器制造，
  由第二节的 mock 集成测试覆盖。

## 六、`colcon test`（实际执行）

```bash
colcon test --packages-select duco_ros_driver
colcon test-result --verbose
```

结果：

```
Summary: 207 tests, 0 errors, 38 failures, 28 skipped
```

| 测试类型 | 用例 | 失败 | 性质 |
|----------|------|------|------|
| gtest（5 个二进制） | 130 | **0** | 功能测试全部通过 |
| `uncrustify` | 28 个文件 | 25 | **仓库既有**风格偏差 |
| `lint_cmake` | 11 条 | 11 | **仓库既有** `CMakeLists.txt` 空白/行长问题 |

- **0 个功能失败**；38 个失败全部来自 lint。
- `uncrustify` 偏差原因：本仓库的 ament 配置要求 `namespace {`、`struct X {`
  的大括号换行、行宽 100 等，而仓库内**所有**源文件（含未改动的
  `src/DucoDriver.cpp`、`include/duco_ros_driver/utils.h` 等）都不符合，
  共 25/28。本轮新增的 `InterlockPersistence.hpp` **通过** uncrustify；
  新增的 `test_interlock_persistence.cpp` 与既有 3 个测试文件同样使用
  `namespace {` 风格，因此同样被标记（保持仓库内一致性）。
- 与上一轮相同，本轮**没有**为通过 lint 而改动厂商 SDK 或无关文件。

## 七、未执行 / 无法执行的测试

| 项目 | 原因 |
|------|------|
| 实机运动测试 | **未获授权**，且按要求不自动开展实体机械臂运动；节点从未连接真实控制器 |
| 实机急停 / 断连 / 缓存重发时序 | 无实机、无厂商停止确认接口，合成话题无法复现真实断连时序 |
| 真实控制器「取消超时」「取消被拒绝」 | 真实 `joint_trajectory_controller` 总会应答取消；由 mock 集成测试覆盖 |
| 旧竞态窗口的确定性复现 | 微秒级窗口，见第四节说明（实测旧实现下压力测试仍通过） |
| 多进程/多机并发 | 未做；仅在单进程多线程 executor 下验证 |
| 持久化文件所在磁盘写满 / 只读根文件系统 | 未做；只验证了「目录不可创建」「目录变只读」两种失败 |

## 八、复现命令汇总

```bash
cd /home/me/duco_ros2_ws
source /opt/ros/jazzy/setup.bash

# 1. 构建
colcon build

# 2. C++ 测试
source install/setup.bash
./build/duco_ros_driver/test_interlock_persistence
./build/duco_ros_driver/test_real_motion_interlock
./build/duco_ros_driver/test_real_feedback_gate
./build/duco_ros_driver/test_trajectory_validation
./build/duco_ros_driver/test_dual_trajectory_action     # ~47 s

# 3. colcon test（含 lint，预期 38 个既有 lint 失败）
colcon test --packages-select duco_ros_driver
colcon test-result --verbose

# 4. 数字孪生端到端（Phase A/B/C，退出码 0）
bash scripts/dual_twin_verification/verify_all.sh

# 5. 单独跑某个用例
./build/duco_ros_driver/test_dual_trajectory_action \
  --gtest_filter='*CancelTimeoutIsReportedAsAbortedNotCanceled'
```
