# Mission Planner 显式命令集升级对接文档（31010~31017）

## 1. 目标与背景

当前 MP 群控页使用 `MAV_CMD_USER_1~USER_5`（数值 31010~31014）做“参数复用”语义，程序侧已演进为“显式命令集”语义（`31010~31017` 一命令一动作）。  
为避免同号异义、状态歧义与 ACK 判读混乱，建议将 MP 升级为显式命令集对接。

本文档定义 MP 端改造规范，确保与当前群控程序一致。

---

## 2. 升级结论（先看）

- **命令数值不变系**：仍基于 MAVLink `COMMAND_LONG.command`，使用 `31010~31017`。
- **语义改为显式**：每个命令只做一件事，不再用 `USER_1~5 + param` 复用流程状态。
- **目标保持不变**：群控命令始终发给长机 `target_system=leader_sysid`，`target_component=191`（或 0）。
- **ACK 来源统一**：应答以伴机 `ONBOARD_COMPUTER` 组件为准，避免和飞控 ACK 混读。

---

## 3. 显式命令集定义（MP -> 群控程序）

统一使用 `COMMAND_LONG`（`msgid=76`）：

| 动作 | command | 参数 | 说明 |
|---|---:|---|---|
| 开始编队 | 31010 | 忽略 | 长机进入编队执行（Guided） |
| 停止编队 | 31011 | 忽略 | 停止编队并转 Loiter（按程序实现） |
| 群控 RTL | 31012 | 忽略 | 群控返航流程 |
| 群控 LAND | 31013 | 忽略 | 群控降落流程 |
| 解锁起飞 | 31014 | `param1=起飞高度(m)` | 先导引模式后解锁并起飞 |
| 设置执行模式 | 31015 | `param1: 1=FORMATION, 2=INDEPENDENT` | 模式切换 |
| 启动执行 | 31016 | 忽略 | 按当前模式启动（编队 or 独立） |
| 停止执行 | 31017 | 忽略 | 按当前模式停止 |

字段约定：

- `target_system = leader_sysid`
- `target_component = MAV_COMP_ID_ONBOARD_COMPUTER (191)`（推荐固定）
- `confirmation = 0`
- 未使用参数全部填 `0`

---

## 4. MP 侧改造点（代码层）

以下描述针对 `FlightData.SwarmControl.cs` 一类群控页实现（命名以实际工程为准）。

### 4.1 命令发送层改造

将原先“USER_1~5 + 参数复用”的发送函数，改为“显式动作 -> 显式命令号”：

- 新增命令常量：
  - `CMD_START_FORMATION = 31010`
  - `CMD_STOP_FORMATION = 31011`
  - `CMD_RTL = 31012`
  - `CMD_LAND = 31013`
  - `CMD_ARM_TAKEOFF = 31014`
  - `CMD_SET_MODE = 31015`
  - `CMD_START_EXEC = 31016`
  - `CMD_STOP_EXEC = 31017`

- 新增统一发送接口（示意）：

```csharp
void SendSwarmExplicitCommand(ushort cmd, float p1=0, float p2=0, float p3=0,
                              float p4=0, float p5=0, float p6=0, float p7=0)
{
    var tid = leaderSysId;
    var tcomp = MAVLink.MAV_COMPONENT.MAV_COMP_ID_ONBOARD_COMPUTER; // 191
    MainV2.comPort.doCommand(tid, tcomp, (MAVLink.MAV_CMD)cmd, p1,p2,p3,p4,p5,p6,p7);
}
```

### 4.2 UI 事件映射改造

推荐将按钮和命令做一一对应：

- `应用模式` -> `31015(param1=1/2)`
- `开始执行` -> `31016`
- `停止执行` -> `31017`
- `开始编队`（可选独立按钮）-> `31010`
- `停止编队`（可选独立按钮）-> `31011`
- `终止动作=RTL/LAND` -> 分别发 `31012/31013`
- `解锁起飞` -> `31014(param1=高度)`

说明：  
“暂停/恢复”若仍需要，建议后续单独扩展新命令号，不要再挤占 `31010~31017` 语义。

### 4.3 参数复用逻辑下线

下列旧逻辑建议删除或仅保留兼容开关：

- `USER_1(mode,strategy,apply)` 这类复用语义
- `USER_5(abortAction)` 这类“同一命令+参数分支”
- 由 UI 文本直接决定参数分支的隐式状态机

---

## 5. 与程序侧协议对齐要点

1. 群控命令只投递到长机伴机（191 组件），不面向飞控组件语义。  
2. 命令结果以 `COMMAND_ACK` 为准，优先按“命令 id + 目标 sysid + 来源 compid(191)”做关联。  
3. 任务仍走 `MISSION_*` 标准流，目标是长机飞控 sysid。  
4. 僚机“代理控制”（单机操作）继续走标准飞控命令，不并入群控显式命令集。

---

## 6. 兼容与灰度策略（推荐）

为避免一次性切换风险，建议双阶段：

### 阶段 A：双协议兼容（短期）

- MP 增加开关：`SwarmProtocol = Legacy | Explicit`
- 默认 `Legacy`（USER 复用），联调时切 `Explicit`
- Telemetry/ACK 面板显示当前协议模式

### 阶段 B：默认显式（中期）

- 默认改为 `Explicit`
- `Legacy` 仅做回滚通道
- 两个版本稳定后移除 `Legacy` 逻辑

---

## 7. 回归测试清单（MP 端）

最小通过集：

1. `31015(param1=1)` 后 `31016`，编队能启动。  
2. `31015(param1=2)` 后 `31016`，独立任务链能启动。  
3. `31017` 在两种模式下都能停止。  
4. `31012/31013` 可触发收尾动作。  
5. `31014(param1=10)` 可执行起飞流程。  
6. 同一命令只出现一条可判读 ACK（避免双路径误判）。  
7. 上传 `MISSION_*` 后 `MISSION_ACK type=0`，并可进入执行流程。  

异常集：

- `target_system` 非长机时，群控命令应被拒绝或无效。  
- `target_component` 非法时，应返回拒绝/不支持。  
- 无任务时发 `31016`（独立模式）应给出可解释反馈。

---

## 8. 日志与可观测性建议

建议 MP 在日志中输出以下字段，便于联调：

- `tx`: 时间戳、cmd、target_system、target_component、params  
- `ack`: 时间戳、cmd、result、source_sysid、source_compid  
- `session`: 当前协议模式（Legacy/Explicit）

建议在 UI 状态栏显示：

- 当前长机 sysid
- 当前协议模式
- 最近一条群控 ACK（cmd/result）

---

## 9. 迁移完成判据

满足以下条件可认为升级完成：

- MP 群控页主流程全部使用 `31010~31017` 显式语义。  
- 不再依赖 `USER_1~5` 参数复用状态机。  
- 现场日志中不存在“同一群控命令成功执行但飞控返回不支持”的误导性判读。  
- 操作手册与代码一致，联调人员可按按钮直达动作。

---

## 10. 附：建议的一页式按钮映射（可直接贴到 MP 文档）

- 模式切换：`应用模式` -> `31015(param1=1/2)`  
- 启动：`开始执行` -> `31016`  
- 停止：`停止执行` -> `31017`  
- 收尾返航：`RTL` -> `31012`  
- 收尾降落：`LAND` -> `31013`  
- 起飞：`解锁起飞` -> `31014(param1=高度)`  
- 编队专用：`开始编队/停止编队` -> `31010/31011`

