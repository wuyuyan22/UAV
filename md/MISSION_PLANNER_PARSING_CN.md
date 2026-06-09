# Mission Planner 接收与解析说明（P900 多从网络方案）

## 1. 文档目的

说明 **Mission Planner（MP）** 在本群控架构下如何通过 **MAVLink** 接收数据、如何 **按飞机（system id）区分多机**，以及 **长机伴飞程序**（`SwarmController`）发出的报文与 MP 解析行为的对应关系。

**本文档以「僚机与地面站均经 P900 多从网络、长机电台为主设备」为推荐与目标拓扑**；机间与地链共享空口容量时的 **调度、封装与带宽策略** 在此统一描述。实验室或 SITL 下采用 **以太网/UDP 机间 + 本地串口接 MP** 的拆法，见 **§2.3 备选拓扑**。

本文档 **不** 复述 MP 内部 C# 源码，而是依据 **MAVLink 通用语义** 与本仓库实现归纳。

---

## 2. 连接拓扑

### 2.1 目标拓扑（推荐）：P900 多从网络

长机端 **P900 配置为主设备（Master）**；**地面站电台** 与 **各僚机电台** 均为 **从设备（Slave）**。空口上所有从站只与主站通信，**不存在僚机—地面站直连**。

| 节点 | 电台角色 | 与长机关系 | 数据语义 |
|------|----------|------------|----------|
| 长机伴飞计算机 | 接 **P900 Master** 模块（典型 **UART9 `/dev/ttyS9`**） | 唯一主站出口 | 汇聚/分发、调度、与飞控交互（UART8 等） |
| 地面站（MP） | Slave | 经地面电台接入主站 | MAVLink 上下行（任务、参数、群控命令、遥测显示） |
| 僚机伴飞计算机 | Slave | 经机载电台接入主站 | 机间业务：`STATE_REPORT`、`FOLLOWER_CMD`、`TASK_*` 等（与现有 `LinkPacket`→MAVLink 映射一致） |

**路径原则**：`地面站 ↔ 长机主站 ↔ 僚机`，由长机 **仲裁与路由**。

**MP 侧不变**：地面站 MP 仍解析 **单路字节流中的多 `sysid`**（长机飞控真实遥测 + 长机合成的僚机遥测等），见 §3。

### 2.2 长机伴机单串口复用模型（UART9）

目标硬件接线建议：

- **UART8**（如 `/dev/ttyS8`）：长机 **ArduPilot 飞控** MAVLink（与现实现一致）。
- **UART9**（如 `/dev/ttyS9`）：长机 **P900 Master** 数字接口（波特率与电台配置一致）。

在 **单一 UART9** 上，需承载两类逻辑流量：

1. **地面站通道**：与 MP 之间的 MAVLink 帧（等价于当前 `gcs_serial_` 收发逻辑）。
2. **僚机通道**：与各僚机之间的机间载荷（等价于当前 `InterUavLink` 经 UDP 收发的 MAVLink 封装消息）。

**实现要求**（与当前「UDP + 独立 gcs 串口」代码差异）：

- 若 P900 固件已提供 **多从站寻址 + 透明管道**（每从站独立虚拟串口语义），伴机软件仍可为 **一个物理串口**，在驱动/库层 demux，或在应用层按电台 **节点 ID / 信道** 分包。
- 若电台仅为 **半双工透明比特管道**，则必须在 **应用层** 增加 **成帧与节点字段**（例如：`[magic][ver][dst_role][dst_id][len][payload][CRC]`），由长机 **`RadioScheduler`（调度层）** 按 §12.3 时隙 **轮询各从站** 收发，避免 GCS 与僚机报文无序抢占导致解析错位。

**结论**：目标架构下，`poll_gcs_serial` / `forward_fc_parsed_to_gcs` / `inject_follower_telemetry_to_gcs` 的 **逻辑保留**，但 **物理出口** 从「独立 UDP + 独立串口」收敛为 **经 UART9 的受控复用**（或由驱动拆成多逻辑 fd，仍由长机统一调度）。

### 2.3 备选拓扑（实验室 / SITL）

| 环节 | 说明 |
|------|------|
| 机间 | 僚机—长机使用 **UDP**（`InterUavLink`，默认端口如 `19870`），不经过 P900。 |
| 地面站 | MP 经 **USB 转串口** 直连长机 **GCS 串口**（`gcs_serial_dev`），或仅 SITL 无电台。 |

此拓扑用于 **开发联调、室内网络环境**；与 §2.1 相比，**无空口带宽与半双工约束**，但 **不代表外场 P900 行为**。

---

## 3. MAVLink 帧与「解析」含义（MP 侧，与拓扑无关）

MP 与常见地面站一样，对串口字节流做：

1. **同步与分帧**：识别 MAVLink v1/v2 帧头、长度、校验（及 v2 的签名若启用）。
2. **提取路由字段**：每条消息均带 **`sysid`**、**`compid`**（组件 ID）。
3. **按 `msgid` 反序列化**：例如 `HEARTBEAT`、`GLOBAL_POSITION_INT`、`ATTITUDE`。
4. **按 sysid 分桶**：为 **每个 sysid** 维护独立的连接状态、地图位置、姿态、参数缓存等 UI 数据。

**结论**：无论底层是 **P900 多从** 还是 **UDP 实验室链路**，MP 「分机」的本质都是 **同一解析器 + 多 sysid 状态机**，无需本工程自定义 MP 侧协议。

---

## 4. 本工程发往 MP 的两类数据源（逻辑行为）

> **P900 目标拓扑下**：下列数据仍由长机伴机产生，但写入 **UART9 / 调度器「地面站时隙」**；与僚机方向的收发 **分时共享空口**，需遵守 §7、§12.6 的带宽与拉参策略。

### 4.1 长机飞控 → MP（透传/节流）

- **路径**：长机飞控串口 → `MavlinkVehicle::read_messages` 解析 → 回调 `gcs_parsed_forward` → `forward_fc_parsed_to_gcs` → **`gcs_serial_`（目标：UART9 上的 GCS 时隙）**。
- **sysid/compid**：与飞控一致（即长机在飞控上配置的 **MAV_SYSID** 等），**不修改**。
- **节流**（降低窄带拥塞）：对下列消息类型若两次发送间隔 **小于 200 ms** 则丢弃本次转发：  
  `ATTITUDE`、`GLOBAL_POSITION_INT`、`LOCAL_POSITION_NED`、`VFR_HUD`、`ATTITUDE_QUATERNION`。  
  其余 `msgid` **原样转发**（若飞控发出且被解析到）。

**MP 侧表现**：该 sysid 对应 **真实长机飞控** 的全量能力（参数、电池、EKF、STATUSTEXT 等，取决于飞控实际输出与是否被节流）。

### 4.2 僚机状态 → MP（伴飞合成）

- **路径**：机间链路汇总至 `follower_state_cache_` → `inject_follower_telemetry_to_gcs` → **同上，经 GCS 出口发送**。
- **来源**：在 §2.1 拓扑下，缓存数据来自 **P900 僚机从站时隙** 收到的 `STATE_REPORT`（等效 UDP 时的 `GLOBAL_POSITION_INT` 映射），而非以太网。
- **触发条件**：`gcs_serial_enable && role==LEADER && follower_sysids` 非空；**地面站拉参窗口**内（见 §7）**不注入**；两次注入间隔约 **500 ms**（可按链路负载降至 **0.2 Hz**，见 §12.6）。
- **编码约定**：对 `follower_sysids` 中每个已在缓存中出现的僚机，依次发送（均使用 **`MAV_COMP_ID_AUTOPILOT1`**）：
  - `HEARTBEAT`：`sysid = 僚机 sysid`
  - `GLOBAL_POSITION_INT`：位置、相对高度、速度分量、航向等
  - `ATTITUDE`：roll/pitch/yaw（来自缓存中的 `UnitState`）

**MP 侧表现**：这些 sysid 在 MP 中显示为 **独立飞机**，但数据来自 **伴飞合成**，**不是**僚机飞控串口直连的完整遥测。

---

## 5. 消息与速率小结

| 来源 | 典型 sysid | 主要消息 | 备注 |
|------|------------|----------|------|
| 长机飞控 | 飞控 `MAV_SYSID` | 飞控发出的全部被转发类型 | 部分高频遥测 200 ms 最小间隔 |
| 僚机（合成） | `cfg.follower_sysids` 各值 | HEARTBEAT、GLOBAL_POSITION_INT、ATTITUDE | 默认约 0.5 Hz 一轮；P900 窄带下建议自适应降低 |

---

## 6. 地面站拉参时的行为（带宽保护）

当 MP 发送参数相关请求（如 `PARAM_REQUEST_LIST` / `PARAM_REQUEST_READ` 等）被伴飞解析到时，会进入 **「拉参活跃窗口」**（约数秒 grace）。此期间：

- **`inject_follower_telemetry_to_gcs` 直接返回**，不向 GCS 方向注入僚机合成遥测，避免与 `PARAM_VALUE` 等争抢带宽导致 MP 卡在「正在获取参数」。

在 P900 多从网络下，建议 **同步压缩僚机时隙** 或 **暂停非关键机间上报**，把空口让给参数与任务。

---

## 7. MP → 长机（下行与指令接口）

伴飞从 **GCS 逻辑通道** 读取 MAVLink 后，**按消息级解析并转发**，而非无条件原始透传：

- 对带 `target_system` 的下行消息（如 `COMMAND_*`、`MISSION_*`、`PARAM_*`、`SET_MODE`），仅当目标为本机 `sysid` 或广播 `0` 时，才转发到本机飞控；
- 对不含 `target_system` 的链路维护类消息保持兼容转发；
- 伴飞同时本地解析群控命令与任务消息，用于群控状态机。

该机制用于多长机并存时的链路隔离，避免误把其他长机的任务/命令转发给本机飞控。

### 7.1 MP 可用群控命令（`COMMAND_LONG`）

下表为当前实现中 MP 可直接下发给伴飞节点（`target_system=长机sysid`，`target_component=0` 或 `MAV_COMP_ID_ONBOARD_COMPUTER`）的命令接口：

| 业务动作 | `command` | `param1` | 说明 |
|---|---:|---|---|
| 开始编队 | `31010` (`MAV_CMD_USER_1`) | 忽略 | 触发 `cmd_start_formation()`，切 Guided 并开始编队控制 |
| 停止编队 | `31011` (`MAV_CMD_USER_2`) | 忽略 | 触发 `cmd_stop_formation()`，停止编队并切 Loiter |
| 返航 | `31012` (`MAV_CMD_USER_3`) | 忽略 | 触发 `cmd_rtl()` |
| 降落 | `31013` (`MAV_CMD_USER_4`) | 忽略 | 触发 `cmd_land()` |
| 解锁起飞 | `31014` (`MAV_CMD_USER_5`) | 起飞高度（m） | 触发 `cmd_arm_and_takeoff(param1)` |
| 设置执行模式 | `31015` | `1=FORMATION`，`2=INDEPENDENT` | 切换 `exec_mode` |
| 启动执行 | `31016` | 忽略 | 按当前 `exec_mode` 启动（编队或独立任务） |
| 停止执行 | `31017` | 忽略 | 按当前 `exec_mode` 停止 |

> 说明：伴飞会回发 `COMMAND_ACK`（`result=ACCEPTED`）给 MP。

### 7.2 MP 任务接口（`MISSION_*`）

MP 下发标准任务消息时，伴飞支持并处理以下接口：

- `MISSION_COUNT`
- `MISSION_ITEM_INT`
- `MISSION_CLEAR_ALL`

处理行为：

1. 同步转发到本机飞控（受 `target_system` 过滤保护）；
2. Leader 侧同时镜像到伴飞内部 `waypoints_`，用于编队计算/独立拆分任务；
3. 当 `MISSION_ITEM_INT` 收齐后触发 `cmd_upload_waypoints(...)`，更新群控任务源。

### 7.3 多长机场景建议

1. 始终给 `COMMAND_LONG` 与 `MISSION_*` 设置明确 `target_system`（不要依赖广播）。
2. 全网保证 `sysid` 唯一（长机与僚机都唯一）。
3. MP 操作前确认当前选中飞机与下发目标一致。

---

## 8. Mission Planner 使用建议（P900 场景）

1. **连接类型**：地面站侧为 **P900 Slave 模块串口** 接 PC，波特率与电台一致；空口上由长机 Master 调度，MP 仍按 MAVLink 解析。
2. **多机显示**：在 MP 中应能看到 **长机 sysid** + 各 **僚机 sysid**（僚机状态经主站汇聚后由长机合成注入 GCS 通道）。
3. **选中飞机**：僚机为合成链路时，**勿期望** 能对其完成完整参数读写；必要时应 **单独维护僚机维护链路** 或扩展转发消息类型。
4. **带宽**：优先使用电台支持的 **较高串口波特率**（在误码可接受前提下）；必须保留 **遥测节流、拉参暂停注入、任务期间限流**（§12.6）。

---

## 9. 与本工程代码的对应关系

| 行为 | 代码位置（参考） | P900 目标拓扑下的演进 |
|------|------------------|------------------------|
| 打开 GCS 串口、注册飞控→MP 回调 | `SwarmController::init`，`VehicleConfig::gcs_parsed_forward` | UART9 仍为 **地面站逻辑通道**；若与机间复用，需经 **调度层** 再写物理串口 |
| 飞控遥测转发与节流 | `SwarmController::forward_fc_parsed_to_gcs` | 逻辑不变，出口纳入 **空口配额** |
| 僚机合成遥测 | `SwarmController::inject_follower_telemetry_to_gcs` | 同上 |
| 实际写出串口 | `SerialPort gcs_serial_`，`gcs_tx_mtx_` | 可能与 **`InterUavLink` 物理后端合并** 为同一 `write_path` |
| MP 下行透传飞控 | `SwarmController::poll_gcs_serial` | 从 **GCS 时隙** 取字节，解析逻辑不变 |
| 机间收发 | `InterUavLink`（`src/comm/inter_uav_link.cpp`） | **当前为 UDP**；目标为 **P900 僚机从站时隙** 或 **封装帧内的逻辑 peer** |

---

## 10. 限制说明

- 僚机在 MP 中的状态依赖 **长机主站能否稳定收到各从站上报**；P900 窄带、遮挡、半双工会放大延迟与丢包。
- 合成报文 **仅包含** HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE；与真机直连 MP 的显示丰富度不同。
- **当前仓库实现**（截至文档更新）：机间仍为 **`InterUavLink` + UDP**；GCS 为 **独立 `gcs_serial_`**。要达到 §2.1，需完成 **§12.7** 中的软件迁移。

---

## 11. P900 多从网络：协议分层与调度（目标设计）

### 11.1 设计目标

1. 长机作为 **唯一主站**，统一仲裁 **地面站从站** 与 **各僚机从站** 的链路时序与优先级。  
2. **业务层语义不变**：`LinkPacket` / 现有 MAVLink 映射（`STATE_REPORT`、`FOLLOWER_CMD`、`TASK_*`）及 MP 显示逻辑 **复用**。  
3. 在低带宽、半双工条件下保障 **安全控制** 与 **任务可靠传输**。

### 11.2 协议分层

| 层级 | 职责 |
|------|------|
| **业务层** | `LinkPacket`、群控状态机、MP 命令与任务解析（与现 `SwarmController` 一致） |
| **传输层** | **`ISwarmAirLink`**（`include/comm/inter_uav_link.h`）：`bind`/`set_target`/`send_packet`/`recv_packet`；**现状** 由 **`InterUavLink`（UDP）** 实现；目标侧增加 **`P900FramedLink`** |
| **调度层（仅长机 Master）** | `tick()`：周期轮询各从站、优先级队列、GCS 时隙与僚机时隙划分、可选 ACK 重传 |

### 11.3 主站调度建议（长机）

- **周期**：例如 **100 ms** 一级；
- **时隙分配（示例）**：**60%～70%** 僚机状态与控制；**20%～30%** 地面站 MAVLink 透传与合成遥测；**10%** 重传与管理；
- **抢占**：安全类（RTL / LAND / 急停）可 **打断** 低优先级时隙。

**消息优先级**（从高到低）：

1. 安全类（RTL/LAND/急停）  
2. 编队控制（`FOLLOWER_CMD`）  
3. 状态上报（`STATE_REPORT`）  
4. 任务分发（`TASK_META` / `TASK_ITEM` / `TASK_COMMIT` / `TASK_ACK` / `TASK_START`）  
5. 低优先级遥测与日志

### 11.4 地址与路由

- **节点标识** 建议与 MAVLink **`sysid`** 对齐，减少映射表复杂度。  
- 路由表（长机维护）：`sysid`（或电台从站 ID）→ **最近在线时间**、**丢包/超时统计**、**待重传队列**。  
- 地面站可映射为固定特殊 ID（如 `target_role=GCS`），与僚机 `sysid` 区分，便于调度器分类。

### 11.5 可靠性机制

- **关键消息**（任务、模式切换、急停）：**序列号 + ACK + 超时重传 + 去重窗口**。  
- **状态类**（高频位置）：允许丢包，通过 **下一周期上报** 恢复，不强求逐包 ACK。  
- **链路心跳与降级**：超时 → **降频** → **悬停/Loiter** → 按策略 **RTL**。

### 11.6 与 MP 相关的带宽控制

- 保留 **拉参窗口暂停僚机注入**。  
- **僚机合成注入频率** 与 **空口利用率** 联动（如 0.5 Hz → 0.2 Hz）。  
- **任务上传/参数下载** 期间，限制非关键机间与合成遥测额度。

### 11.7 迁移实施步骤（软件）

1. **抽象机间链路**：`ISwarmAirLink` 已在 `inter_uav_link.h` 定义，`InterUavLink` 已实现该接口；下一步让 **`SwarmController` 依赖 `ISwarmAirLink&` 或 `std::unique_ptr<ISwarmAirLink>`**，按配置构造 UDP 或 P900 实现。  
2. **保留 `UdpInterUavLink`**：封装现有 UDP 实现，供 §2.3 实验室模式零改动切换。  
3. **实现 `P900FramedLink`（或 Master 侧 `P900MuxPort`）**：UART9 字节流 **成帧**、CRC、从站寻址；与电台手册中的 **主从模式** 对齐。  
4. **长机侧 `RadioScheduler`**：合并 `poll_gcs_serial` 下行与 `link_.recv/send` 的 **时序**，避免半双工自撞。  
5. **僚机侧**：`ISwarmAirLink` 后端改为 **P900 Slave**：仅与主站对发，**不再** `set_target` 到 IP（或 IP 字段表示逻辑从站号，由实现解释）。  
6. **联调**：多 sysid MP 显示、群控命令、任务、编队、独立任务全链路。  
7. **外场**：按实测时延/丢包调频率、重传次数、超时阈值。

### 11.8 验收指标（建议）

- 僚机遥测在 MP 列表中 **在线率** 满足任务书（如 >99% 工况下定义）。  
- 安全控制指令 **端到端时延**（P95）满足任务要求。  
- 任务下发成功率在标定距离与遮挡条件下稳定达标。  
- MP **拉参** 不因空口拥塞长时间卡死。

### 11.9 硬件与接线清单（长机）

| 伴飞侧接口 | 连接对象 | 说明 |
|------------|----------|------|
| UART8 | 长机飞控 TELEM | MAVLink，与现 `local_fc.serial_dev` 一致 |
| UART9 | P900 **Master** | 多从网络唯一主出口；波特率与模块一致 |

### 11.10 与旧版「UDP + 串口」文档差异摘要

- **旧**：机间默认 UDP，GCS 默认独立串口，P900 为「后续可选」。  
- **现**：**外场推荐拓扑** 为 **P900 多从**；UDP 为 **开发备选**；软件上需 **调度层 + 可插拔链路** 才能一致落地。

---

## 12. 相关文档

- `MISSION_SWARM_IMPLEMENTATION_CN.md`：群控任务、MP 指令与模式。
- `README.md`：运行参数与硬件说明。
