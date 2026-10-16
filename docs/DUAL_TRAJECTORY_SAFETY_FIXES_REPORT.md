# DUCO GCR5-910 双轨迹节点 — 第二轮安全加固开发报告

- 仓库：`git@github.com:HENU-Makerspace/duco_gcr5_910_ros2_ws.git`
- 修复分支：`codex/dual-trajectory-safety-fixes`
- 起始提交（本轮基线，即上一轮分支 HEAD）：`96bbd6e` *Add a reproducible digital-twin verification harness*
- 本轮代码提交：`b6dfbca` *Separate real motion permission from the action goal*
- 本轮报告提交：见文末「Git 记录」
- 与上一轮基线差异：11 个文件，+1557 / −156

> 任务书给出的仓库为 `lsh953/duco_gcr5_910`。本机真实 remote 是
> `HENU-Makerspace/duco_gcr5_910_ros2_ws`，所有工作以本地真实仓库为准，未对
> 任务书中的仓库做任何操作（无该仓库的推送权限，也无法确认其与本工程同源）。

## 一、阶段一：代码审查结论

### 1.1 审查前的版本确认

| 项目 | 结果 |
|------|------|
| 远端分支 | `origin/main`、`origin/codex/dual-trajectory-reliability` |
| 起始 HEAD | `96bbd6ef5e49123edbd97225974fa8ac845f66b8` |
| 未提交修改 | 无（`git status --porcelain` 为空） |
| stash | 无 |
| 实际源码行数 | `DualTrajectoryAction.hpp` 1232、`RealFeedbackGate.hpp` 215、`TrajectoryValidation.hpp` 226、`DucoDriver.cpp` 347、`DucoRobotStatus.cpp` 313、`DualTrajectoryAction.cpp` 14、`CMakeLists.txt` 85、`digital_twin_real.launch.py` 294 |

起始工作区是干净的，没有需要保护的未提交成果；本轮从 `96bbd6e` 新建
`codex/dual-trajectory-safety-fixes`，未改动 `codex/dual-trajectory-reliability`，
未合并、未触碰 `main`。

### 1.2 定位到的真实缺陷（按任务书条目对齐）

先记录，再修复。逐条给出「现象 → 根因 → 位置」。

**P0-1（§3.1）数值有效性**：已由上一轮的 `RealFeedbackGate` 覆盖大部分（NaN/Inf
位置与速度、维度、空名、重名、缺关节、缺失速度数组）。仍有两处不足：

- 速度数组缺失时的处理是「一律判失败」，没有显式表达「是否需要速度参与到位判断」；
- 失败原因只有笼统的 `InvalidFeedback`，无法区分「维度错」与「数值非有限」，
  不利于诊断与针对性测试。

**P0-2（§3.2）反馈时效**：已有 `steady_clock` 时间戳、序号、`*_timeout` 参数，
但参数名与需求文档不一致；`DucoRobotState.robot_state` 字段完全未被使用。

**P0-3（§4）实体运动锁定 —— 本轮最严重缺陷**：

- `finishGoal()` 在结束时会执行 `real_command_sent_ = false`，而故障闭锁只在
  「已下发真实命令 **且** 结果 ≠ SUCCEEDED」时才触发；
- 因此 `wait_for_real = false` 且 `publish_real = true` 时，**目标以 SUCCEEDED
  结束却根本不触发任何闭锁**，下一条实体轨迹可以立刻覆盖尚未确认的运动。
  这正是任务书 §7「不能直接允许下一条实体轨迹覆盖当前未确认的运动」所禁止的；
- `handle_goal()` 只检查 `latched_fault_` 与任务槽，没有独立的「实体运动是否允许」
  判断（§4.1/§4.3）；
- 故障只在 `finishGoal()` 中闭锁：机器人**空闲时**上报急停/碰撞/错误码不会闭锁，
  而是在 `handle_accepted` 中被 `real_faulted_ = false` 清掉（§4.2「机器人报告故障」
  必须触发锁定）；
- 恢复服务 `std_srvs/Trigger` **无任何授权**，同域任意节点可调用（§4.4）。

**P0-4（§5）取消结果处理**：

- 只要进入取消分支就固定上报 `CANCELED + SUCCESSFUL`（原 `result_callback`），
  Gazebo 实际返回 `SUCCEEDED`/`ABORTED`、或取消被拒绝时，同样会被报成「干净取消」；
- 取消超时路径只在「已下发真实命令」时给 `GOAL_TOLERANCE_VIOLATED`，**取消被拒绝**
  （`goals_canceling` 为空）时仍报 `SUCCESSFUL`。

**P0-5（§6）命令发布顺序**：

- 真实轨迹在 `goal_response_callback` 中**锁外**发布（原第 675 行），条件在锁内
  计算；停止请求在 `handle_cancel` 中同样锁外发布。两者之间没有顺序约束，
  取消与发布竞争时可能出现「先停后动」或「旧任务下发新运动」；
- 没有任务代次（generation）概念，迟到回调只能靠 `active_goal_` 指针比较，
  无法证明「本次命令属于当前任务」。

**P1（§7/§8）语义与配置**：`use_real_robot` 默认为 `true`、`server_host_1`
硬编码 `192.168.6.4`、`wait_for_real + !publish_real` 会让目标挂到仿真超时才失败；
参数缺少显式类型；无仿真/实体结果的独立诊断输出。

**§9 辅助组件**：工作区与磁盘上**不存在** `duco_reliability_guard.zip`、
`trajectory_reliability_guard.hpp`、`test_trajectory_reliability_guard.cpp`。
因此无法复用，也未引入对应抽象层；其中「轨迹与反馈校验」的功能已由既有的
`TrajectoryValidation.hpp` + `RealFeedbackGate.hpp` 覆盖。

## 二、阶段二：反馈验证（§3）

### 2.1 数值有效性

`RealFeedbackGate` 的判决被细分为可独立测试的原因：

```
reached / invalid_policy / no_new_status / stale_status / unhealthy_status
no_new_feedback / stale_feedback / invalid_target
invalid_feedback_dimensions / empty_joint_name / duplicate_joint_name
non_finite_position / non_finite_velocity / missing_joint
position_outside_tolerance / velocity_outside_tolerance
```

校验顺序与要点：

1. 容差、时效参数合法（非 NaN/Inf、非负、时效 > 0）；
2. **状态先于反馈**：先要求「命令之后收到过状态」、时效内、健康，
   再检查反馈——因为 `DucoRobotStatus` 在断连后停止发布状态，
   而 `real_feedback_states` 仍在重发缓存值，状态才是可靠的心跳；
3. 反馈必须晚于命令（序号 + 时间戳），且时间晚于当前检查时刻判为异常；
4. 先做**维度自洽**（数组与关节名一一对应），再逐元素检查，
   避免用一个数组的索引去访问另一个数组；
5. 逐关节：空名 → `empty_joint_name`；重名 → `duplicate_joint_name`；
   非有限位置/速度 → `non_finite_position` / `non_finite_velocity`；
6. 关节按**名字**匹配而非下标，因此反馈顺序与目标顺序不同不会误判；
7. `require_velocity`（新增，默认 `true`）为真时速度数组必须完整；
   为假时缺数组表示「没有速度证据」而**跳过**速度判据，
   **绝不**把缺失当作「速度为零」；只要数组存在就必须完整且有限。

节点侧新增参数 `real_require_velocity`（默认 `true`，即始终要求速度），
`evaluateRealAtGoalLocked()` 把该参数注入策略。

### 2.2 反馈时效

- 记录项：真实命令发布时间 `real_command_time_`、反馈到达时间、
  状态到达时间、命令时的反馈/状态序号、当前目标身份（代次 + UUID）。
- `std::chrono::steady_clock` 计时；`反馈时间 > now` 判为异常。
- 「状态有效时不能报告正常完成」→ `unhealthy_status` 直接使到位判断失败。
- 参数改名：`real_feedback_timeout` → **`real_feedback_max_age`**、
  `real_status_timeout` → **`real_status_max_age`**（语义不变）。

### 2.3 厂商接口实际能力（§3.2 尾段，实测 `DucoRobotStatus.cpp` / `DucoDriver.cpp`）

| 需要的状态 | 厂商是否提供 | 结论 |
|------------|--------------|------|
| 真实连接状态 | 否。`controller_connected_flag_` 是 `DucoRobotStatus` 的**内部**成员，不发布 | **无法确认**。唯一可观测线索是：断连后状态话题静默，而反馈话题继续重发缓存 → 因此代码以「状态序号/时效」作为活性判据 |
| 设备通信异常状态 | 否 | **无法确认** |
| 实际运动状态 | 部分。`DucoRobotState.robot_state`（厂商 `robotState`）会发布；SDK 的 `robotmoving()` **未发布** | 枚举值**无文档**，不做猜测 |
| 机器人故障码 | 是。`robot_error`，另有 `emc_stop_signal`、`collision`、`collision_axis`、`slave_ready` | 可用，已用于闭锁 |
| 停止确认状态 | 否 | **无法确认**。未凭空增加停止接口 |

`robot_state` 的处理方式：新增参数 `real_unhealthy_robot_states`（默认**空**），
由操作者在现场确认后声明哪些控制器状态视为故障（`DucoRobotStatus` 自身把
「运动中掉到状态 5」当作停事件，故 `[5]` 是候选起点）。默认空即「不做推测」；
该值同时写入诊断输出，便于观察未公开枚举的真实取值。

## 三、阶段三：实体运动锁定（§4）

新增纯逻辑模块 `RealMotionInterlock.hpp`（不依赖 ROS，可独立单测）：

| 状态 | 含义 | 是否阻止新实体轨迹 |
|------|------|--------------------|
| `allowed` | 无未结清的实体运动 | 否 |
| `executing` | 真实轨迹已发布且目标在运行 | 否（运行中的目标仍需发停止） |
| `awaiting_confirmation` | 目标已结束，但实体完成/停止未被确认 | **是** |
| `uncertain` | 实体运动状态无法建立（超时、反馈丢失、取消被拒…） | **是** |
| `forbidden` | 机器人上报故障（急停/碰撞/错误码） | **是** |

关键性质：

- 后三者为**粘滞态**，严重度单调上升（`forbidden` > `uncertain` >
  `awaiting_confirmation`），迟到的温和事件**不能降级**更严重的状态；
- 只有 `onOperatorRecovery()` 能把粘滞态清回 `allowed`；
- 真实命令的发布必须携带**代次**，且只有「当前代次」能被确认完成所结清；
- 闭锁**独立于** `active_goal_`：`finishGoal()` 释放 Action 槽位时不会解除闭锁。

节点接入点：

| 位置 | 行为 |
|------|------|
| `handle_goal()` | `publish_real && 闭锁` → **REJECT**，日志给出状态、原因、恢复服务名 |
| `publishRealMotionLocked()` | 代次/句柄/未终结/未发停止/未闭锁，全部在同一临界区内判定并发布 |
| `finishGoal()` | 已下发真实命令：**确认完成→结清**；否则→ `awaiting_confirmation`（`wait_for_real=false` 的仿真成功同样如此） |
| `watchdogCallback()` | 各类超时先 `latchUncertainLocked()`（严重度更高的 `uncertain`），再结束逻辑任务 |
| `realStatusCallback()` | 上报故障（且 `latch_on_real_fault=true，默认`）→ `forbidden` 闭锁，**空闲时同样闭锁** |
| `restoreLatchedFault()` | 仅当 `publish_real` 时读取 marker，恢复为 `uncertain`；纯仿真不受历史 marker 影响 |

### 3.1 恢复流程与授权（§4.4）

恢复服务保持 `std_srvs/Trigger`（不新增厂商不存在的接口），但：

- 新增参数 **`fault_recovery_enabled`，默认 `false`**：默认状态下服务存在但
  一律拒绝，并返回明确原因。开启它本身即「操作者的显式授权动作」，
  真机部署由操作者实例按需开启。
- 允许清锁的**前置可观测条件**（任一不满足即拒绝，`success=false` 且给出原因）：
  无活动目标、无未结清 Gazebo 目标、无未结清真实命令、已收到过状态、
  状态在 `real_status_max_age` 内、状态健康。
- 调用服务＝操作者对「已停机、已检查急停/碰撞/故障、通信可靠」的**声明**；
  代码不做任何自动解锁：无定时解锁、无超时解锁、不因重启默认解锁、
  不因单条正常反馈解锁、不因仿真成功解锁。
- 服务返回文案明确写清「这不是厂商确认的安全停止」。

## 四、阶段四：取消与发布修复（§5、§6）

### 4.1 取消结果映射（`result_callback` / `watchdogCallback`）

| Gazebo 实际结果 | 处理 |
|-----------------|------|
| 取消中返回 `SUCCEEDED`（取消输给了完成） | 恢复 `executing`，走正常完成判定；**不伪装成已取消**；动作结果按实际（可为 SUCCEEDED） |
| 取消中返回 `CANCELED` | 动作 `CANCELED`；已下发真实命令 → `error_code = GOAL_TOLERANCE_VIOLATED`，否则透传 Gazebo 错误码；闭锁由 `finishGoal` 处理 |
| 取消中返回 `ABORTED` | 动作 `ABORTED` + Gazebo 错误码，**不报取消成功** |
| 取消被拒绝 | 记录 `cancel_rejected`；超时结束时按 `GOAL_TOLERANCE_VIOLATED`，**即使没有真实命令**也不报干净取消 |
| 无响应/取消超时 | `CANCELED`；`(已下发真实命令 ‖ 取消被拒)` → `GOAL_TOLERANCE_VIOLATED`，并以 `uncertain` 闭锁 |
| 实体停止状态未知 | 一律闭锁，绝不由「取消被接受」推出「机器人已停」 |

同时新增：`!sim_done_` 时仿真超时才生效，避免「仿真已成功」被仿真超时误杀。

### 4.2 不重复终结（§5.2）

`finishGoal()` 在 `state_mutex_` 内检查 `active_goal_ == goal && !terminal_state_`
并置位 `terminal_state_`，`succeed()/abort()/canceled()` 一律在锁外调用。
同一目标至多终结一次；迟到结果回调还额外校验**代次**，
旧目标的结果不可能影响已经启动的新任务。

### 4.3 统一真实命令出口（§6.1/§6.2）

新增两个唯一出口，**必须持锁调用**：

- `publishRealMotionLocked(goal_id, generation, trajectory)`
- `publishRealStopLocked(goal_id, generation, joint_names)`

满足的约束：

1. 所有实体轨迹、所有停止请求都只经这两个入口发布；
2. 每条命令都绑定当前任务身份（UUID + 代次）；
3. 发布前在同一临界区内复核「任务是否仍有效」；
4. 已取消（`real_stopped_generation_ == generation`）或已闭锁的任务不能再发运动；
5. 停止请求只发一次，且不被新任务覆盖（按代次记录）；
6. 停止与运动之间有确定时序：谁先拿到 `state_mutex_` 谁先生效，
   且停止会把该代次的运动发布**永久拒绝**，因此不存在「先停后动」；
7. 旧任务的异步回调因代次不符被拒绝，无法下发新的实体运动。

并发与锁：全部状态在同一 `state_mutex_` 下；`rclcpp_action` 调用（
`async_cancel_goal`、`succeed/abort/canceled`）一律在锁外，因此不存在
「持锁回调上层」造成的锁序反转；发布（非阻塞）在锁内完成，
不会长时间持锁。未依赖「默认回调组串行」来代替状态检查。

## 五、阶段五：完成语义、配置与集成（§7、§8）

### 5.1 两种模式

- **模式 A（`publish_real=false`）**：仿真结果即动作结果，不下发任何真实命令，
  实体闭锁状态与真实故障**都不参与**仿真路径（纯仿真与实体通道隔离）。
- **模式 B（`publish_real=true`）**：
  - `wait_for_real=true`：只有「仿真成功 **且** 反馈门控确认到位」才报成功；
  - `wait_for_real=false`：按接口语义报仿真结果，但**不认定**实体完成，
    实体闭锁置为 `awaiting_confirmation`，需要操作者恢复后才允许下一条实体轨迹。

### 5.2 独立诊断输出（§7 尾段）

新增 latched 话题 `real_motion_state_topic`（默认
`dual_trajectory/real_motion_state`，`std_msgs/String`，仅在状态变化时发布）：

```
state=awaiting_confirmation;latched=1;goal=<uuid|->;sim=succeeded;real=unconfirmed;
robot_state=<厂商值>;reason=<原因>
```

`sim=` 与 `real=` 分离，上层可据此区分「仿真成功」与「实体运动结果」。
该话题仅用于诊断，不参与任何控制决策。

### 5.3 启动参数与默认值（§8）

| 参数 | 变化 |
|------|------|
| `use_real_robot` | `true` → **`false`**（实体通道必须显式开启） |
| `wait_for_real` | `true` → **`false`**（只在实体模式下有意义） |
| `server_host_1` | `192.168.6.4` → **空**；不再把历史 IP 当作当前机器人地址 |
| `real_feedback_timeout` / `real_status_timeout` | 改名为 `real_feedback_max_age` / `real_status_max_age` |
| 新增 | `real_require_velocity`、`real_motion_state_topic`、`fault_recovery_enabled`、`latch_on_real_fault`、`real_unhealthy_robot_states` |
| 参数类型 | 数值/布尔参数统一用 `ParameterValue(..., value_type=...)` 显式声明 |

新增 `OpaqueFunction` 启动校验，明确报错而非静默忽略：

- `use_real_robot:=true` 但 `server_host_1` 为空 → 启动失败并说明原因；
- `wait_for_real:=true` 但 `use_real_robot:=false` → 启动失败（没有任何东西可等待）。

节点侧对应校验：`wait_for_real && !publish_real` 直接抛
`std::invalid_argument`；`DualTrajectoryAction.cpp` 的 `main()` 捕获后打印
`DualTrajectoryAction: invalid configuration: ...` 并返回 1（不再以未捕获异常终止）。

## 六、修改文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `src/duco_ros_driver/include/duco_ros_driver/RealMotionInterlock.hpp` | 新增 | 实体运动联锁状态机（纯逻辑，199 行） |
| `src/duco_ros_driver/include/duco_ros_driver/RealFeedbackGate.hpp` | 修改 | 细分判决 + `require_velocity` |
| `src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp` | 修改 | 联锁接入、统一发布出口、代次、取消映射、诊断、参数 |
| `src/duco_ros_driver/src/DualTrajectoryAction.cpp` | 修改 | 配置错误清晰报错 |
| `src/duco_ros_driver/test/test_real_motion_interlock.cpp` | 新增 | 联锁单元测试（14 例） |
| `src/duco_ros_driver/test/test_real_feedback_gate.cpp` | 修改 | 判决更新 + 顺序无关/速度可选等新用例 |
| `src/duco_ros_driver/test/test_dual_trajectory_action.cpp` | 修改 | 集成测试扩展至 27 例 |
| `src/duco_ros_driver/CMakeLists.txt` | 修改 | 注册 `test_real_motion_interlock` |
| `src/duco_gcr5_910_moveit_config/launch/digital_twin_real.launch.py` | 修改 | 默认值、参数类型、启动校验 |
| `scripts/dual_twin_verification/phase_b_node.sh` | 修改 | 同步新参数名与恢复开关 |
| `scripts/dual_twin_verification/README.md` | 修改 | 记录参数/行为变化 |

## 七、是否修改现有接口

- **未新增消息类型、未新增服务类型、未改动厂商 SDK、未删除任何版权声明、
  未改动 PLC 相关功能、未引入数据库/Web 服务/新框架。**
- 沿用 `control_msgs/action/FollowJointTrajectory` 前台接口与既有
  `joint_path_command`（空轨迹＝停止）约定。
- 保留 `std_srvs/Trigger` 恢复服务，但增加 `fault_recovery_enabled` 授权开关。
- **兼容性变化（需上层/脚本同步）**：

| 变化 | 影响 | 兼容性说明 |
|------|------|------------|
| `publish_real` 节点默认 `false` | 不显式传参的启动变成纯仿真 | 方向安全；真机 launch 已显式传参 |
| `use_real_robot` 默认 `false` | 默认启动为纯仿真 | 任务书 §8 要求 |
| `wait_for_real` 默认 `false` | 闭环需显式开启 | 任务书 §8 要求 |
| `server_host_1` 无默认 | 真机启动必须给地址，否则**报错** | 任务书 §8 要求 |
| `real_feedback_timeout`/`real_status_timeout` 改名 | 旧名会因「未声明参数」启动报错（非静默忽略） | 已同步 launch 与孪生脚本；文档已更新 |
| 恢复服务默认关闭 | 需 `fault_recovery_enabled:=true` | 任务书 §4.4 要求 |
| 空闲时上报故障即闭锁 | 故障后需操作者确认才能再动 | 任务书 §4.2 要求 |
| `wait_for_real=false` + 实体 | 每次实体轨迹后需操作者恢复 | 任务书 §7 明确要求 |
| 真机急停不再中止纯仿真目标 | 仿真与实体通道隔离 | 任务书 §4.3「明确隔离」 |
| 取消被拒/停止未确认不再报「干净取消」 | 上层据 `error_code` 可区分 | 任务书 §5.1 要求 |
| 取消输给完成时上报 SUCCEEDED | 不再伪装成取消 | 任务书 §5.1 要求 |

## 八、尚未解决 / 未验证项（如实标注）

1. **未做真机运动测试**（从未获得授权，也未连接实体设备）。所有结论均来自
   单元测试 + 数字孪生 + 静态审查。
2. **没有厂商停止确认接口**，因此闭锁解除是**操作者声明**，不是经认证的安全
   停止；该软件保护**不能替代硬件急停与安全控制器**。
3. **没有厂商连接状态接口**：断连只能靠「状态话题静默 + 缓存重发」间接推断，
   属于**无法确认**等级的证据。
4. `robot_state` 枚举**无文档**，默认不解释；`real_unhealthy_robot_states`
   需现场确认后填写，否则该字段仅作记录。
5. 合成真实 I/O 恒定「完美跟随」，**未复现**真机断连、缓存重发、抖动时序；
   这些场景由反馈门控单元测试覆盖，真机时序仍属**未验证**。
6. marker 文件位于 `/tmp`：清空 `/tmp` 会丢失「上次未确认运动」的记忆
   （进程内状态仍在，但重启后不再恢复）。
7. 节点仍使用默认互斥回调组（`state_mutex_` 为防御性冗余）；压力下的
   真实调度时序未做专项测试。
8. **本分支刻意未整改 lint**（仓库整体不符合 `uncrustify`/`lint_cmake`，
   属历史风格问题）。详见测试报告。
9. 25 个环境（无 ROS/Jazzy 时的独立编译）未验证：本机具备 ROS 2 Jazzy，
   因此未走「仅独立编译」的降级路径。

## 九、Git 记录

| 提交 | 说明 |
|------|------|
| `96bbd6e` | 本轮基线（上一轮分支 HEAD） |
| `b6dfbca` | 本轮全部代码/测试/脚本/配置修改（11 文件） |
| `32786a4` | 本开发报告与测试报告（`docs/`） |

分支：`codex/dual-trajectory-safety-fixes` → 已推送至 `origin`。

审查提示：`DualTrajectoryAction.hpp` 是从 .cpp 拆分并大幅演进的单文件节点类，
Git 不会把它识别为重命名，**真正的语义变化请用**

```bash
git diff 96bbd6e b6dfbca -- src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp
```

逐段阅读；其中新增的 `RealMotionInterlock.hpp` 与
`publishRealMotionLocked` / `publishRealStopLocked` / `finishGoal`
是本次修复的核心。
