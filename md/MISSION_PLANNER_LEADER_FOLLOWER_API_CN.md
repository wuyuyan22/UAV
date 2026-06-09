# Mission Planner 经长机下发指令控制僚机 — 接口方案

本文档定义 **Mission Planner（MP）** 在本群控工程中如何通过 **长机伴飞节点** 间接控制僚机，供地面站配置、脚本与联调验收使用。实现依据：`SwarmController`、`MavlinkVehicle`、`P900FramedLink`、`include/common/types.h`（`SwarmMavCmd`）。

更完整的 MP 解析、拓扑与带宽说明见 `MISSION_PLANNER_PARSING_CN.md`。

---

## 1. 设计原则

| 原则 | 说明 |
|------|------|
| **下行目标为长机** | 所有「群控业务」相关的 `COMMAND_LONG`（`USER_1`～`STOP_EXEC` 等）必须 **`target_system = 长机 MAVLink sysid`**。对 **僚机** 的 **任务 / 参数 / 航线上传** 等**非代理项**伴飞**不会**透明转发到僚机飞控。 |
| **僚机简单控制** | 解锁、起飞、降落、RTL、切模式等见 **§4.1**：MP 选中僚机时经 **`CTRL_CMD`** 由僚机伴飞下发本机飞控；**代理**与「飞控串口直连」在带宽与能力上仍不等价。 |
| **组件 ID** | 群控 `COMMAND_LONG`：`target_component` 为 **`ALL`（0）** 或 **`ONBOARD_COMPUTER`（191）**。僚机代理另接受 **`AUTOPILOT1`（1）**（与 MP 常见目标一致），见 `gcs_ctrl_target_component_ok`。 |
| **僚机实际控制** | 长机根据模式将任务拆解或计算编队目标，经 **机间链路**（`LinkPacket`：`FOLLOWER_CMD`、`TASK_*` 等）发往各僚机；MP 侧表现为「选长机发令、多机协同」，而非「选僚机直连飞控」。 |
| **任务上传对象** | `MISSION_*` 中 **`target_system` 须为长机飞控 sysid**（与 `cfg.local_fc.sysid` 一致），用于镜像到伴飞航点并驱动编队/独立拆分逻辑。 |

---

## 2. 连接与数据路径（摘要）

- **实验室 / UDP**：MP ↔ 长机 **独立 GCS 串口**；机间 **UDP**（`InterUavLink`）。
- **外场 / P900**：MP ↔ **地面 P900 从站** ↔ 空口 ↔ **长机 P900 主站（如 UART9）**；地面侧可用 `tools/p900_gcs_bridge.py` 将 MAVLink 封装为 `src=0xFE`、`dst=长机 sysid` 的应用帧。

无论物理层如何，**MP 侧接口（MAVLink 语义）相同**。

---

## 3. Mission Planner 接收与处理长机、僚机数据方案

本节说明 **MP 作为 GCS 如何接收一路串口/虚拟串口字节流**，并 **区分长机与多架僚机** 进行显示与交互；与伴飞实现 `forward_fc_parsed_to_gcs`、`inject_follower_telemetry_to_gcs` 一一对应。

### 3.1 物理连接与上行数据形态

| 场景 | PC 侧连接 | 进入 MP 的数据 |
|------|-----------|----------------|
| 实验室 | USB 转串口 ↔ 长机 **GCS 专用串口** | 纯 MAVLink 字节流 |
| P900 外场 | USB 转串口 ↔ **地面 P900**；PC 上可经 `p900_gcs_bridge.py` 再虚拟一对串口给 MP | 桥接去掉成帧外壳后，仍为 **单路 MAVLink 流** |

MP **只连接一条链路**（一个 COM 口或 TCP Client 等），**不**为每架飞机单独拉线；多机信息全部在该流内用 **`sysid` / `compid`** 区分。

### 3.2 MP 内部解析模型（与拓扑无关）

MP 对串口/TCP 字节流按 MAVLink v1/v2 规则：

1. **同步与分帧**：识别帧头、长度、校验（及 v2 签名若启用）。
2. **提取路由字段**：每条消息带 **`sysid`、`compid`**。
3. **按 `msgid` 反序列化**：如 `HEARTBEAT`、`GLOBAL_POSITION_INT`、`ATTITUDE`、`PARAM_VALUE` 等。
4. **按 `sysid` 分桶**：为 **每个 system id** 维护独立状态（位置、姿态、参数表、连接健康度、地图图标等）。

因此：**同一比特流中出现多个 `sysid`** 时，MP 表现为 **多架飞行器**，无需自定义 MP 插件即可在飞机列表中看到长机与各僚机（前提是僚机侧有消息发出）。

### 3.3 长机数据：来源、消息与 MP 行为

| 项目 | 说明 |
|------|------|
| **数据来源** | 长机 **ArduPilot 飞控** 经 UART（如 `local_fc.serial_dev`）进入伴飞，`MavlinkVehicle` 解析后回调 **`forward_fc_parsed_to_gcs`**，再写到 GCS 串口或 P900 的 GCS 通道。 |
| **MAVLink `sysid`** | 与飞控 **`MAV_SYSID` 一致**，即长机在 MP 中的「本机」真实遥测。 |
| **典型消息** | 飞控实际发出的遥测均被转发（含参数、电池、EKF、`STATUSTEXT` 等，视飞控配置而定）。 |
| **节流** | 对高频遥测：`ATTITUDE`、`GLOBAL_POSITION_INT`、`LOCAL_POSITION_NED`、`VFR_HUD`、`ATTITUDE_QUATERNION` 若两次发送间隔 **&lt; 200 ms** 则丢弃本次转发，减轻窄带链路压力。 |
| **MP 侧表现** | 选中 **长机 sysid** 时，可进行 **完整飞控能力**：模式切换（与 ArduPilot 支持一致）、**任务上传/下载**、**参数读写**、详细状态树等（以 MP 与固件版本为准）。 |

### 3.4 僚机数据：来源、消息与 MP 行为

| 项目 | 说明 |
|------|------|
| **数据来源** | **非**僚机飞控直连 MP。僚机经机间链路上报 `STATE_REPORT`，长机写入 **`follower_state_cache_`**，由 **`inject_follower_telemetry_to_gcs`** 按 **各僚机 sysid** **合成** MAVLink 再发往 GCS。 |
| **MAVLink `sysid`** | 与 **`cfg.follower_sysids`** 中配置的僚机 id 一致，使 MP 能分机显示。 |
| **注入消息类型**（每架僚机一轮） | **`HEARTBEAT`**（`MAV_COMP_ID_AUTOPILOT1`）、**`GLOBAL_POSITION_INT`**、**`ATTITUDE`**。 |
| **注入频率** | 默认约 **0.5 Hz**（两次注入间隔 ≥ **500 ms**）；与编队控制、链路负载相关，可在外场调低。 |
| **拉参窗口** | MP 发起 `PARAM_REQUEST_*` 等被伴飞识别后，进入 **数秒「拉参活跃窗口」**，此期间 **暂停** 向 GCS 注入僚机合成遥测，避免与 `PARAM_VALUE` 等争抢带宽导致 MP 卡在「正在获取参数」。 |
| **MP 侧表现** | 地图/HUD 上可出现 **多架僚机位置与简易姿态**；**不具备**与 USB 直连僚机飞控相同的 **完整参数表、全量传感器、任务读写到僚机** 等能力。对僚机发 **`target_system = 僚机`** 的标准指令 **不会**被伴飞转发到僚机飞控（见 §1）。 |

### 3.5 MP 侧操作与数据处理要点

1. **连接类型**：在 MP 中选择与 PC 实际接线一致的连接（Serial / TCP 等），波特率与电台/桥接一致。
2. **多机列表**：连接成功后，应看到 **1 个长机 sysid + N 个僚机 sysid**（僚机需已在机间在线且缓存非空，故可能有短暂延迟）。
3. **选中飞机（Current/Active）**：发 **群控自定义命令、任务、参数** 时，**必须选中长机 sysid**（见 §4、§5）。在僚机 sysid 上操作标准任务/参数 **不符合**本工程转发模型。
4. **地图与航迹**：长机为真实融合位置；僚机为合成 `GLOBAL_POSITION_INT`，用于 **态势显示**，精度与更新率受机间链路与注入频率限制。
5. **告警与文本**：僚机 **`STATUSTEXT`、电池、EKF 状态** 等 **不会**经本方案自动出现在 MP，除非另行扩展伴飞转发逻辑。

### 3.6 数据流小结（MP 视角）

```
┌─────────────────────────────────────────────────────────────────┐
│  MP 单路 MAVLink 输入                                            │
├─────────────────────────────────────────────────────────────────┤
│  sysid = 长机飞控  → 真实遥测（伴飞透传 + 部分节流）              │
│  sysid = 僚机 i    → 合成遥测（HEARTBEAT + POS + ATT，~0.5 Hz）   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
              MP 按 sysid 分桶 → 地图 / HUD / 参数缓存 / 日志
```

---

## 4. 群控命令接口：`COMMAND_LONG`

使用标准 **`COMMAND_LONG`**（`msgid = 76`），**`command` 字段取值**如下（与 `MAV_CMD_USER_1`～`MAV_CMD_USER_7` 数值一致）。

| 动作 | `command`（十进制） | 符号名（便于对照） | `param1` | 其余 param | 长机行为概要 | 僚机侧影响（实现层） |
|------|---------------------|-------------------|----------|------------|--------------|----------------------|
| 开始编队 | `31010` | `MAV_CMD_USER_1` / `START_FORMATION` | 忽略 | — | `cmd_start_formation()`，长机 Guided 编队 | 周期接收 **`FOLLOWER_CMD`**（目标点） |
| 停止编队 | `31011` | `USER_2` / `STOP_FORMATION` | 忽略 | — | 停止编队，切 Loiter 等 | 停止编队目标更新 |
| 返航 | `31012` | `USER_3` / `RTL` | 忽略 | — | 群控 RTL | 依实现切 RTL |
| 降落 | `31013` | `USER_4` / `LAND` | 忽略 | — | 群控降落 | 依实现 |
| 解锁起飞 | `31014` | `USER_5` / `ARM_TAKEOFF` | **起飞高度（米）** | — | `cmd_arm_and_takeoff(param1)` | 编队/任务未起则随群控流程 |
| 设置执行模式 | `31015` | `SET_MODE` | **`1`=编队，`2`=独立** | — | 设置 `exec_mode`（FORMATION / INDEPENDENT） | 独立模式走 **TASK_*** 子任务链 |
| 启动执行 | `31016` | `START_EXEC` | 忽略 | — | 编队：`cmd_start_formation()`；独立：`distribute_independent_mission()` | 独立：接收子航点、`TASK_START` 后 AUTO |
| 停止执行 | `31017` | `STOP_EXEC` | 忽略 | — | 编队：停止编队；独立：停止独立执行流程 | 停止对应模式 |

**应答**：伴飞通过 **`COMMAND_ACK`** 回复（`result = MAV_RESULT_ACCEPTED`），经 GCS 通道回 MP（P900 下为 `write_mavlink_to_gcs`）。

**MP 操作建议**：在 MP 中配置自定义命令或使用「发送命令」插件时，将 **目标系统设为长机 sysid**，勿对僚机 sysid 发送上表命令（伴飞逻辑只认长机 `target_system`）。

### 4.1 僚机简单飞行控制代理（`CTRL_CMD` / `CTRL_PROXY`）

在 MP 中选中 **某僚机 sysid**，且消息的 **`target_system`** 为该僚机时，长机伴飞 **不再**将下列指令写入长机飞控，而是经机间链路发送 **`LinkMsgType::CTRL_CMD`**（线格式为标准 **`COMMAND_LONG`**，`command = SwarmMavCmd::CTRL_PROXY`（`31034`），载荷字段见 `LinkPacket.seq` + `ctrl_pf[]`）。

| GCS 消息 | 代理到僚机飞控的语义（节选） |
|----------|------------------------------|
| `COMMAND_LONG` | `MAV_CMD_COMPONENT_ARM_DISARM`、`NAV_TAKEOFF`、`NAV_LAND`、`NAV_RETURN_TO_LAUNCH`、`DO_SET_MODE` |
| `COMMAND_INT` | 同上（若 MP 发 `COMMAND_INT`；如 `NAV_TAKEOFF` 高度取 `z`） |
| `SET_MODE` | 映射为 `DO_SET_MODE`（`base_mode` + `custom_mode`） |

若 **错误** 向僚机发送 **群控 `USER_1`～`STOP_EXEC`** 或 **机间任务协议号**（`TASK_*` 对应命令号），长机应答 **`COMMAND_ACK`：`MAV_RESULT_DENIED`**。其余未实现的 `COMMAND_LONG`/`COMMAND_INT` 应答 **`UNSUPPORTED`**。  
**任务、参数、航线下载等** 仍以 **长机** 为对象；本代理不覆盖 `MISSION_*` / `PARAM_*` 的僚机透传。

---

## 5. 任务与航线：`MISSION_*`（面向长机飞控）

以下消息由 MP 发往 **长机飞控 sysid**（伴飞同步镜像到内部 `waypoints_`，并触发 `cmd_upload_waypoints` 等）：

- `MISSION_COUNT`
- `MISSION_ITEM_INT`
- `MISSION_CLEAR_ALL`

**条件**（代码侧）：`target_system == local_fc.sysid`，`mission_type == MAV_MISSION_TYPE_MISSION`。

**与僚机关系**：

- **编队模式**：航点用于长机导航与编队几何；僚机由长机 **`FOLLOWER_CMD`** 跟踪编队，**不是** MP 直接上传任务到僚机飞控。
- **独立模式**：在 **`SET_MODE` 为独立** 且执行 **`START_EXEC`** 后，长机将主任务 **按机均分** 为子任务，经 **`TASK_META` / `TASK_ITEM` / `TASK_COMMIT` / `TASK_ACK` / `TASK_START`** 下发各僚机，僚机本地 `upload_mission` 后切 AUTO。

---

## 6. 执行模式小结

```
MP 下发 SET_MODE(param1):  1 → FORMATION     2 → INDEPENDENT
         ↓
START_EXEC:  编队 → 开始编队控制          独立 → 拆分并下发子任务到各机
STOP_EXEC:   编队 → 停止编队              独立 → 停止独立执行相关状态
```

---

## 7. 机间载荷（非 MP 直接发送，供接口理解）

MP **不**直接组这些包；由长机伴飞生成，用于实现上表行为：

| 类型 | 用途 |
|------|------|
| `FOLLOWER_CMD` | 编队模式下周期目标点 → 僚机 Guided |
| `STATE_REPORT` | 僚机 → 长机状态，用于编队与 MP 合成遥测 |
| `TASK_META` / `TASK_ITEM` / `TASK_COMMIT` / `TASK_ACK` / `TASK_START` | 独立模式子任务传输与启动 |
| `CTRL_CMD`（`CTRL_PROXY`） | 见 **§4.1**：MP 选中僚机时的简单控制指令代理 |

空口实现可为 **UDP** 或 **P900 成帧**（`P900FramedLink::write_frame(src,dst,payload)`，`dst` 为对方 sysid）。

---

## 8. 约束与已知限制

1. **僚机在 MP 中的遥测** 多为长机 **合成注入**（见 **§3.4**），**不要指望**对僚机进行完整参数读写或等同于 USB 直连飞控的体验。
2. **多机 sysid** 必须在全网唯一。
3. **窄带 / P900**：拉参窗口内伴飞会暂停向 GCS 注入僚机合成遥测（见 **§3.4**）；外场应合理设置速率与时隙（见 `MISSION_PLANNER_PARSING_CN.md` §6、§12）。

---

## 9. 与源码对应关系

| 接口 | 主要位置 |
|------|----------|
| `SwarmMavCmd` 常量 | `include/common/types.h` |
| GCS 下行解析与转发过滤 | `SwarmController::poll_gcs_serial`、`should_forward_gcs_msg_to_local_fc` |
| 僚机简单控制代理 | `try_proxy_gcs_command_to_follower`、`try_proxy_gcs_command_int_to_follower`、`try_proxy_gcs_set_mode_to_follower`、`execute_follower_ctrl_cmd` |
| `CTRL_CMD` 编解码 | `src/comm/link_mavlink_codec.cpp`（`SwarmMavCmd::CTRL_PROXY`） |
| 群控 `COMMAND_LONG` 处理 | `SwarmController::handle_gcs_command_long` |
| 任务镜像与上传 | `handle_gcs_mission_count`、`handle_gcs_mission_item_int`、`handle_gcs_mission_clear_all` |
| 编队周期发僚机目标 | `SwarmController::run_leader_tick` → `FOLLOWER_CMD` |
| 独立任务拆分与下发 | `SwarmController::distribute_independent_mission`、`try_finish_independent_leader_auto` |
| 长机遥测 → MP | `SwarmController::forward_fc_parsed_to_gcs` |
| 僚机合成遥测 → MP | `SwarmController::inject_follower_telemetry_to_gcs` |
| P900 GCS 成帧 | `P900FramedLink`、`tools/p900_gcs_bridge.py` |

---

## 10. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-04-01 | 首版，与当前仓库 `SwarmMavCmd` 及 `SwarmController` 行为对齐 |
| 2026-04-01 | 增加 §3：MP 接收与处理长机/僚机数据方案（解析模型、数据源对比、操作要点、数据流图） |
| 2026-04-01 | 增加 §4.1 与 §7：`CTRL_CMD` 僚机简单控制代理（`COMMAND_LONG`/`COMMAND_INT`/`SET_MODE`） |
