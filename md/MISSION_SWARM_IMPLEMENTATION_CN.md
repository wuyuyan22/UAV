# Mission Planner 群控任务实现文档

## 1. 文档目标

本文档定义以下能力的工程实现方案：

- Mission Planner（以下简称 MP）只连接长机链路；
- MP 下发主任务后，系统支持两种模式：
  - 编队模式：长机执行主任务，僚机通过 GUIDED 实时跟随；
  - 非编队模式：长机拆分主任务并下发子任务，僚机通过 AUTO 独立执行；
- 明确 MP 指令结构、模块接口、状态机、时序、异常处理和开发里程碑。

本文档面向当前工程代码组织（`SwarmController` / `MavlinkVehicle` / `InterUavLink`）。

---

## 2. 当前基础能力（已具备）

- MP -> 长机飞控任务透传：`gcs_serial` 输入字节已透传到飞控串口；
- 长机伴机已可解析 `MISSION_COUNT` / `MISSION_ITEM_INT` / `MISSION_CLEAR_ALL` 并镜像到 `waypoints_`；
- 编队实时控制链路已具备：长机计算目标 -> 机间链路下发 -> 僚机执行 GUIDED 目标；
- 已有读参降载机制：读参窗口暂停僚机注入与编队下发。

---

## 3. 目标功能定义

### 3.1 编队模式（Formation）

1. MP 下发主任务（航线）给长机；
2. 长机飞控执行主任务（可 AUTO）；
3. 长机伴机实时读取长机状态和当前任务进度；
4. 长机伴机按编队参数计算每架僚机目标；
5. 僚机伴机持续发送 GUIDED 目标给各自飞控（建议 5~10Hz）；
6. 长机汇总僚机状态并回传 MP。

### 3.2 非编队模式（Independent）

1. MP 下发主任务给长机；
2. 长机伴机根据僚机数量与拆分策略生成子任务；
3. 长机伴机将子任务下发至各僚机伴机；
4. 僚机伴机上传子任务到本机飞控（`MISSION_*`）；
5. 子任务上传成功后僚机切 AUTO 执行；
6. 执行进度和失败信息回传长机再回传 MP。

---

## 4. MP 指令结构（控制面）

任务面继续使用标准 `MISSION_*`，控制面建议统一使用 `COMMAND_LONG + MAV_CMD_USER_xx`。

目标字段约定：

- `target_system = leader_sysid`
- `target_component = MAV_COMP_ID_ONBOARD_COMPUTER`（或 `MAV_COMP_ID_ALL`）

### 4.1 指令表

1) `MAV_CMD_USER_1`：设置模式

- `param1`: `1=FORMATION`, `2=INDEPENDENT`
- `param2`: 非编队拆分策略（`1=均分航点`, `2=空间聚类`, `3=自定义`）
- `param3`: 是否立即应用（`0=否`, `1=是`）

2) `MAV_CMD_USER_2`：开始执行

- `param1`: 任务版本（`0=最新`）
- `param2`: 是否强制重分配（`0/1`）

3) `MAV_CMD_USER_3`：暂停

- `param1`: 暂停动作（`1=LOITER`, `2=HOLD`）

4) `MAV_CMD_USER_4`：继续

- `param1`: 继续策略（`1=从当前位置`, `2=回最近航点`）

5) `MAV_CMD_USER_5`：终止

- `param1`: 终止动作（`1=RTL`, `2=LAND`, `3=LOITER`）

6) `MAV_CMD_USER_6`：设置编队参数

- `param1`: 编队类型（线形/V形/楔形）
- `param2`: 间距（m）
- `param3`: 高度差（m）
- `param4`: 偏航偏置（deg）
- `param5`: 最大速度（m/s）
- `param6`: 最小速度（m/s）

7) `MAV_CMD_USER_7`：设置非编队拆分参数

- `param1`: 最小子任务航点数
- `param2`: 最大子任务航段长度（m）
- `param3`: 启动方式（`1=同时`, `2=分批`）

---

## 5. 系统模块设计与接口

## 5.1 长机伴机新增模块

### A. MissionOrchestrator（任务编排核心）

职责：

- 维护模式和状态机；
- 接收主任务版本更新；
- 触发编队执行或非编队拆分/下发；
- 汇总执行状态供 MP 可视化。

建议接口（头文件级）：

```cpp
enum class MissionMode { NONE, FORMATION, INDEPENDENT };
enum class MissionState {
  IDLE, RECEIVING_MASTER, MASTER_READY,
  FORMATION_RUNNING, PLANNING_SUBTASKS, UPLOADING_SUBTASKS,
  INDEPENDENT_RUNNING, PAUSED, ABORTED, ERROR
};

struct MasterMission {
  uint32_t version = 0;
  std::vector<WayPoint> wps;
  uint64_t updated_ms = 0;
};

struct OrchestratorStatus {
  MissionMode mode = MissionMode::NONE;
  MissionState state = MissionState::IDLE;
  uint32_t mission_version = 0;
  int followers_total = 0;
  int followers_done = 0;
  int followers_failed = 0;
  std::string reason;
};

class MissionOrchestrator {
public:
  bool set_mode(MissionMode mode, int strategy);
  bool on_master_mission(const MasterMission &m);
  bool start(uint32_t version = 0, bool force_redistribute = false);
  bool pause(int pause_type);
  bool resume(int resume_type);
  bool abort(int abort_action);
  void tick();
  OrchestratorStatus status() const;
};
```

### B. MissionSplitter（非编队任务拆分）

职责：

- 将主任务拆为每架僚机子任务；
- 支持多策略；
- 输出可直接上传飞控的航点集合。

建议接口：

```cpp
enum class SplitStrategy { EVEN_WP = 1, GEO_CLUSTER = 2, CUSTOM = 3 };

struct SplitParams {
  int min_wp_per_task = 2;
  float max_leg_len_m = 3000.0f;
};

struct SubMission {
  int follower_sysid = 0;
  uint32_t mission_version = 0;
  std::vector<WayPoint> wps;
};

class MissionSplitter {
public:
  std::vector<SubMission> split(const MasterMission &master,
                                const std::vector<int> &followers,
                                SplitStrategy strategy,
                                const SplitParams &params);
};
```

### C. FollowerMissionClient（僚机任务上传会话）

职责：

- 对单僚机执行子任务上传状态机；
- 处理 `MISSION_REQUEST(_INT)` / `MISSION_ACK`；
- 超时重试与错误报告。

建议接口：

```cpp
enum class UploadState {
  IDLE, CLEARING, SEND_COUNT, SENDING_ITEMS, WAIT_ACK, DONE, FAILED
};

struct UploadResult {
  int follower_sysid = 0;
  bool ok = false;
  int mission_result = -1;
  std::string reason;
};

class FollowerMissionClient {
public:
  bool start(const SubMission &task);
  void on_rx_mavlink(const mavlink_message_t &msg);
  void tick();
  UploadState state() const;
  UploadResult result() const;
};
```

## 5.2 僚机伴机接口

僚机节点至少支持两个入口：

1) 编队目标入口（实时）

```cpp
bool on_follower_guided_target(const LinkPacket &pkt);
```

2) 子任务入口（非编队）

```cpp
bool on_subtask_begin(uint32_t mission_version, uint16_t count);
bool on_subtask_item(uint16_t seq, const WayPoint &wp);
bool on_subtask_commit();
```

提交后僚机伴机执行：

- `MISSION_CLEAR_ALL`
- `MISSION_COUNT`
- 依据 `MISSION_REQUEST(_INT)` 发送 `MISSION_ITEM_INT`
- `MISSION_ACK(ACCEPTED)` 后切 `AUTO`

---

## 6. 机间协议扩展（Leader <-> Follower）

建议扩展 `LinkMsgType`：

- `STATE_REPORT`：僚机状态上报（已有）
- `FOLLOWER_CMD`：编队实时目标（已有）
- `TASK_META`：子任务元信息（版本、数量、策略）
- `TASK_ITEM`：子任务航点条目
- `TASK_COMMIT`：子任务发送完成
- `TASK_ACK`：上传结果
- `TASK_PROGRESS`：执行进度（当前航点/完成度）
- `TASK_ABORT`：紧急中止

建议字段增加：

- `mission_version`
- `task_id`
- `seq/total`
- `result_code`
- `timestamp_ms`

---

## 7. 状态机设计

长机编排状态机：

- `IDLE`
- `RECEIVING_MASTER`
- `MASTER_READY`
- `FORMATION_RUNNING`
- `PLANNING_SUBTASKS`
- `UPLOADING_SUBTASKS`
- `INDEPENDENT_RUNNING`
- `PAUSED`
- `ABORTED`
- `ERROR`

关键转换：

- 收到完整主任务 -> `MASTER_READY`
- `SET_MODE(FORMATION)+START` -> `FORMATION_RUNNING`
- `SET_MODE(INDEPENDENT)+START` -> `PLANNING_SUBTASKS -> UPLOADING_SUBTASKS -> INDEPENDENT_RUNNING`
- `PAUSE` -> `PAUSED`
- `RESUME` -> 原运行态
- `ABORT` / 严重故障 -> `ABORTED`

---

## 8. 时序流程

### 8.1 编队模式

1. MP 上传主任务；
2. 长机伴机镜像并版本化；
3. MP 下发 `SET_MODE(FORMATION)`、`START`；
4. 长机飞控按主任务飞行；
5. 长机伴机周期计算僚机目标并下发；
6. 僚机 GUIDED 执行，周期回报状态；
7. 长机聚合状态回报 MP。

### 8.2 非编队模式

1. MP 上传主任务；
2. MP 下发 `SET_MODE(INDEPENDENT)`、`START`；
3. 长机拆分子任务；
4. 长机对每架僚机启动上传会话；
5. 全部 `MISSION_ACK(ACCEPTED)` 后下发开始执行；
6. 僚机切 AUTO 执行；
7. 周期上报进度与异常。

---

## 9. 安全与容错策略

1) 链路失联

- 僚机超过阈值未收到长机指令（编队模式） -> `LOITER` 或 `RTL`；
- 僚机子任务上传超时重试 `N=3`，失败上报长机并进入安全模式。

2) 模式互斥

- 编队实时下发与非编队任务上传必须互斥；
- 切换前清理旧状态（目标缓存、上传会话、超时计时器）。

3) 任务一致性

- 每次主任务更新递增 `mission_version`；
- 僚机执行前需校验版本一致。

4) 读参/高负载窗口

- 保持“读参期间暂停僚机注入与编队下发”；
- 可选：读参期间暂停非关键日志与高频回传。

---

## 10. 与现有代码对接建议

1. `SwarmController`

- 保留现有 `handle_gcs_mission_*` 主任务镜像；
- 新增 `MissionOrchestrator orchestrator_`；
- 在 `control_loop()` 调用 `orchestrator_.tick()`；
- `handle_gcs_command_long()` 中解析 `MAV_CMD_USER_1..7` 转发给编排器。

2. `InterUavLink`

- 扩展 `LinkMsgType` 和对应编解码；
- 增加 `TASK_*` 消息可靠传输（ACK/重发/超时）。

3. 僚机节点

- 增加子任务缓冲与上传状态机；
- 编队与非编队执行入口分离并互斥控制。

---

## 11. 开发里程碑

### M1：编队模式闭环

- 完成 MP 模式命令解析；
- 编队模式稳定执行（长机主任务 + 僚机 GUIDED）；
- 基础状态回传与安全策略。

### M2：非编队基础闭环

- 实现 `MissionSplitter(EVEN_WP)`；
- 实现 `FollowerMissionClient`；
- 僚机 AUTO 执行和进度上报。

### M3：增强能力

- 多策略拆分（空间聚类、自定义规则）；
- 失败重分配；
- MP 可视化增强（任务版本、每机进度、失败原因）。

---

## 12. 测试与验收标准

### 12.1 编队模式

- MP 上传任务成功后，长机按主任务飞行；
- 2 架及以上僚机能够持续保持编队；
- 长机切换航点时僚机轨迹连续无大幅振荡；
- 断链策略触发符合预期（RTL/LOITER）。

### 12.2 非编队模式

- 主任务可拆分为各僚机子任务；
- 所有僚机 `MISSION_ACK=ACCEPTED` 后进入 AUTO；
- 每架僚机进度可回传并聚合展示；
- 单机上传失败不会导致系统无响应，可上报并降级。

---

## 13. 版本与变更记录

- `v1.0`：初版实现文档（协议、接口、状态机、时序、里程碑）。

