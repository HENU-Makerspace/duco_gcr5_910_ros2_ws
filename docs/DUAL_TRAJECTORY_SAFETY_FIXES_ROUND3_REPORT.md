# DUCO GCR5-910 双轨迹节点 — 第三轮可靠性修复开发报告

- 仓库：`git@github.com:HENU-Makerspace/duco_gcr5_910_ros2_ws.git`（`origin`）
- 目标仓库：`git@github.com:lsh953/duco_gcr5_910.git`（remote 名 `lsh953`，与本仓库同源）
- 修复分支：`codex/dual-trajectory-safety-fixes`
- 本轮基线（上一轮分支 HEAD）：`7e7c34b` *Record the lsh953 upload in the development report*
- 本轮代码提交：`f5b2c42` *Make the cancel, interlock durability and task finalization honest*
- 本轮报告提交：见文末「Git 记录」
- 与基线差异：8 个文件，+1250 / −94
- **未合并 `main`、未删除任何分支、未改写历史、未做 force push**

> 本轮只处理任务书列出的三个问题（取消超时语义、联锁标记文件可靠性、
> 任务终结与联锁更新的原子性），保留上一轮全部功能：ROS 2 Action 接口、
> Gazebo 联动、反馈校验、任务代次、人工恢复原则均未改动。

## 一、问题一：取消超时被报告为「取消成功」

### 1.1 现象与根因

`watchdogCallback()` 在取消超时分支里写死了：

```cpp
result_code = rclcpp_action::ResultCode::CANCELED;
error_code  = (real_command_sent_ || cancel_rejected)
              ? GOAL_TOLERANCE_VIOLATED : SUCCESSFUL;
```

ROS 2 Jazzy Action 状态机中 `CANCELED` 的含义是「该目标被取消了」（状态迁移
`CANCELING → CANCELED`），而不是「我们发出了取消请求」。超时意味着**从未收到
Gazebo 的终止结果**，此时：

- 控制器是否停下来没有任何证据；
- `CANCELING → CANCELED` 是一次断言，节点无法证明；
- 而 `CANCELING → ABORTED` 是状态机允许的迁移（`rcl_action`
  状态表：`CANCELING` 接受 `CANCELED`/`ABORTED`/`SUCCEEDED` 三个事件），
  语义是「执行未成功、结果未知」，与本节点真实掌握的信息一致。

### 1.2 修改

`watchdogCallback()` 取消超时分支（`DualTrajectoryAction.hpp`）：

- `result_code` 固定为 `ABORTED`；
- `error_code` 固定为 `GOAL_TOLERANCE_VIOLATED`（不再出现「ABORTED + SUCCESSFUL」
  这种自相矛盾的组合）；
- 失败原因区分「取消被 Gazebo 拒绝」与「取消后没有任何结果」；
- **保留**原有的实体运动不确定处理：`uncertain = real_command_sent_`，
  仍然调用 `latchUncertainLocked()` 升级联锁，`finishGoal()` 仍按
  `real_command_sent_ + real_completion_confirmed_` 判定闭锁。

四种取消情形的最终映射（全部有回归测试，见测试报告）：

| 情形 | 结果 | error_code | 说明 |
|------|------|-----------|------|
| Gazebo 回复 CANCELED（取消成功） | `CANCELED` | 有实体命令时 `GOAL_TOLERANCE_VIOLATED`，否则沿用 Gazebo 的 error_code | 与上一轮一致，未改动 |
| Gazebo 拒绝取消 + 超时 | `ABORTED` | `GOAL_TOLERANCE_VIOLATED` | 本轮修正（原为 `CANCELED`） |
| Gazebo 接受取消但不回结果 + 超时 | `ABORTED` | `GOAL_TOLERANCE_VIOLATED` | 本轮修正（原为 `CANCELED`，且无实体命令时 error_code 为 `SUCCESSFUL`） |
| 取消输给正常完成 | `SUCCEEDED` | Gazebo 的 error_code | 与上一轮一致，未改动 |

## 二、问题二：联锁标记文件可靠性

### 2.1 现象与根因

| 缺陷 | 位置（修改前） |
|------|----------------|
| 默认路径在共享临时目录 `/tmp`，会被 tmp 清理服务与重启清空 | `fault_latch_file` 默认值 `/tmp/duco_dual_trajectory_fault.state` |
| 写入是 `std::ofstream` 直接截断覆盖，非原子；写到一半崩溃会留下半条记录 | `writeFaultMarkerLocked()` |
| 写失败只 `RCLCPP_ERROR` 一句就继续，实体轨迹照发 | 同上（`marker.good()` 检查后 `return`） |
| 删除失败被完全忽略（`std::remove` 返回值未检查） | `removeFaultMarker()` |
| 读不出来就当作没有故障（只 `getline`，无错误区分） | `restoreLatchedFault()` |
| 没有「必须持久化」这一配置概念，无法表达「写不进去就别发实体轨迹」 | 无 |

### 2.2 新增纯逻辑模块 `InterlockPersistence.hpp`

与 ROS 无关（只用 C++17 + POSIX），可独立编译测试，共 350 行：

- **默认目录**：`$XDG_STATE_HOME/duco/dual_trajectory_interlock.state`，
  回退 `$HOME/.local/state/duco/dual_trajectory_interlock.state`；
  两者都不存在时返回空串，**不猜测**，由节点判定「无法持久化」。
- **原子更新**：写入同目录临时文件 → `fsync` → `rename(2)`。
  `rename` 在同一文件系统内是原子的，因此不会暴露半条记录；
  失败时清理临时文件并返回原因。
- **可靠目录**：`mkdir -p`（0700，逐级创建），并用探针文件验证目录可写
  （原子替换需要目录写权限，只验证旧文件权限是不够的）。
- **失败可见**：`prepare()/write()/remove()/read()` 全部返回 `bool` 并回填
  `std::string * error`，不再有静默失败。
- **重启恢复的保守语义**：`exists()` 独立于 `read()`；**记录存在即视为未确认**，
  即使内容为空或无法解析（崩溃发生在创建与写入之间时也不会被读成「干净」）。
- **幂等删除**：删除不存在的记录返回成功；同时清理崩溃残留的 `.tmp`。

### 2.3 节点侧改动

- 新参数 `real_persistence_required`（bool，默认 **true**）；
  `fault_latch_file` 默认值由 `/tmp/...` 改为 `""`（空 = 使用上述可靠默认路径）。
- 启动时 `prepare()`：不可用则打印 ERROR 并置 `persistence_ready_ = false`，
  同时把结果暴露在诊断话题里（新增 `persistence=ok|unavailable` 字段）。
- **写失败必须禁止新的实体轨迹发布**，共三道闸门：
  1. `handle_goal()`：`publish_real && real_persistence_required && !persistence_ready_`
     → `REJECT`，并附带原因；此处会重试一次 `prepare()`，因此操作员修好目录后
     通道可自愈，不必重启节点；
  2. `publishRealMotionLocked()`：**先写持久记录，再发布实体轨迹**。
     写失败且要求持久化 → 直接拒绝发布。失败方向因此只有
     「有记录没运动」（重启只会更保守），不会是「有运动没记录」；
  3. 运行期写失败（例如目录中途变为只读）会把 `persistence_ready_` 置为 false，
     之后的实体目标同样被拒绝。
- `persistInterlockLocked()`：把内存联锁状态与记录写入放在**同一个临界区**内。
- 恢复服务：**先**删除持久记录，删不掉就拒绝解除（`real_persistence_required`
  为 true 时），保证重启看到的状态不会比本次运行「更干净」。
- `restoreLatchedFault()`：改用模块读取；读不出来但文件存在 → 仍然闭锁，
  原因记为 `unreadable marker: ...`。

> **没有**把软件联锁描述成硬件安全机制；**没有**改变人工恢复原则
> （仍然只有 `clear_fault` 服务、仍然需要 `fault_recovery_enabled:=true`）。
> 记录文件只是「辅助持久化」，它不能证明机械臂的任何状态。

## 三、问题三：任务终结与联锁更新的原子性

### 3.1 现象与根因

修改前 `finishGoal()` 是两段临界区：

```
{ lock: terminal_state_=true; 释放槽位 active_goal_.reset(); slot_reserved_=false; real_command_sent_=false; }
   ← 这里没有锁：requestSimulationCancel() 等
{ lock: 联锁状态转换 }
if (remove_marker) removeFaultMarker();      ← 甚至不在锁里
```

`clearFaultCallback()` 同样先「检查」后「释放」，中间放开锁：

```
{ lock: 检查阻塞条件，记录 cleared_from/… }   ← 放开锁
{ lock: 无条件 onOperatorRecovery(); … }
removeFaultMarker();
```

因此存在真实的时间窗口：

1. 槽位已释放但联锁尚未闭锁 —— 并发到达的新目标可能被接受，
   于是一条实体轨迹可能覆盖尚未确认的上一条；
2. 联锁已闭锁、记录刚写好 —— 随后 `removeFaultMarker()` 可能把**新**记录删掉；
3. 检查通过（当时状态健康、无未决命令）→ 放开锁 → 期间故障消息到达并闭锁 →
   重新加锁后**无条件清除**，操作员收到 success，而故障被抹掉。

### 3.2 修改

- `finishGoal()`：任务槽位释放、联锁状态转换、持久记录写入/删除
  **合并为同一个临界区**，作为一个事务提交：
  - 确认完成 → `onConfirmedCompletion` + 删除记录；
  - 未确认完成 → `onGoalFinishedUnconfirmed` + **写入**记录（原因更新为
    「未确认」而不是下发时的「执行中」，重启诊断更准确）；
  - 既未下发实体命令、也无闭锁 → 删除旧记录；
  - 之后才释放 `active_goal_`/`slot_reserved_` 等；
  - 陈旧 Gazebo 目标的取消请求仍然在锁**外**执行（网络调用不能持锁）。
- `clearFaultCallback()`：检查、持久记录删除、联锁解除**合并为同一个临界区**。
  删除记录**先**执行，删不掉（且要求持久化）就整体拒绝解除。
  并发到达的故障消息要么先被检查看到（拒绝），要么在释放之后重新闭锁
  （故障获胜），不再存在「被静默抹掉」的第三种结果。
- 顺带修正：原先「无闭锁则直接返回 success」的早退发生在阻塞条件检查之前，
  当联锁处于 `Executing` 时可能返回 success；现在阻塞条件优先判断。

### 3.3 关于并发测试的诚实说明

- 已补充的并发/时序测试见测试报告第四节，它们断言的是**必须永远成立的不变量**：
  结果已送达 ⇒ 持久记录已存在；未确认结束 ⇒ 下一个目标被拒；恢复成功 ⇒
  先于该次恢复上报的故障不会被静默丢弃。
- **限制**：旧实现的两个临界区之间的窗口只有微秒级，测试无法**确定性地**
  复现该窗口（本轮已实测：把旧实现临时改回后，压力测试仍然通过）。
  因此原子性本身是通过「单临界区」的代码结构保证的，压力测试只是把
  不变量固化为回归防线，不能声称「测试证明了竞态不存在」。

## 四、兼容性

| 项目 | 结论 |
|------|------|
| `FollowJointTrajectory` Action 接口 | 未改动 |
| MoveIt / Gazebo 联动 | 未改动（仅取消超时的结果码语义变化） |
| ROS 2 Jazzy | 保持；`ABORTED` 由 `CANCELING` 迁移合法（已由集成测试实测） |
| 反馈校验、任务代次、统一发布顺序 | 未改动 |
| 厂商 SDK / 版权声明 / 无关 PLC 代码 | 未改动 |
| 参数兼容性 | 新增 `real_persistence_required`（默认 true）；`fault_latch_file` 默认值由 `/tmp/...` 改为空（=可靠默认路径）。旧默认值仍可显式传入，行为不变 |
| 结果码兼容性 | **有意变更**：取消超时由 `CANCELED` 变为 `ABORTED`（+`GOAL_TOLERANCE_VIOLATED`）。上层若依赖旧语义需同步调整 |

## 五、剩余限制（未解决 / 无法确认）

1. **没有厂商停止确认接口**：`DucoRobotStatus` 不发布 `controller_connected_flag_`，
   `robotmoving()` 未发布，`robot_state` 枚举无文档。因此「已停止」始终**无法确认**，
   超时路径的 `ABORTED` 只是不再谎报取消成功，不代表机械臂已停。
2. **持久化不等于安全**：记录文件只影响重启后的保守程度，不能证明机械臂状态；
   文件丢失只会让节点更保守，不会更激进。
3. **恢复服务无法认证**：ROS 2 `std_srvs/Trigger` 无鉴权，只能靠
   `fault_recovery_enabled` 显式开启来限制。
4. **未做任何实机运动测试**（未获授权，且不应自动执行）：全部结论来自
   mock Gazebo 集成测试与数字孪生，`publish_real:=true` 只用于让节点认为
   「已下发真实命令」，真实话题在孪生中被改名且无消费者。
5. **并发窗口不可确定性复现**（见 3.3）。
6. **`real_persistence_required=false` 会降低重启安全性**：这是显式的操作员取舍，
   节点会打印 WARN；默认值为 true。
7. **文件系统故障的自愈**：`handle_goal()` 中的重试 `prepare()` 是「每次被拒目标
   探测一次」的启发式自愈，不是主动监控；目录长期不可用时每个目标都会被拒并打印 ERROR。
8. **旧默认路径的残留**：若此前有人用默认值运行过，`/tmp/duco_dual_trajectory_fault.state`
   可能仍存在；新默认路径不会读取它。需要时请手动删除（它只影响旧版本进程）。

## 六、Git 记录

- 远端：`origin` = `HENU-Makerspace/duco_gcr5_910_ros2_ws`；
  `lsh953` = `lsh953/duco_gcr5_910`（同一提交历史）
- 目标分支：`codex/dual-trajectory-safety-fixes`（**未**合并 `main`）
- 提交：`f5b2c42`（代码/测试/launch/孪生脚本） + 报告提交（见 `git log`）
- 基线：`7e7c34b`
- 推送方式：普通推送（fast-forward），无 force、无历史改写

审查用的 compare 链接：

- `lsh953`：`https://github.com/lsh953/duco_gcr5_910/compare/main...codex/dual-trajectory-safety-fixes`
- `origin`：`https://github.com/HENU-Makerspace/duco_gcr5_910_ros2_ws/compare/main...codex/dual-trajectory-safety-fixes`
