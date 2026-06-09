# 分布式 Mesh 群控与机间避障实现方案

本文档定义 **UAV_Swarm_APM**（`swarm_node`）从当前「单一长机中心化」架构，演进为 **Mesh 组网 + 动态领队 + 分布式避撞 + 地面站直发任务** 的完整工程方案。内容综合当前仓库源码、业界开源实践（d-ORCA、PX4_Swarm_Controller、SwarmPilot、mavsdk_drone_show 等）及前期架构讨论，面向模块开发、SITL 联调与外场实飞。

**文档版本**：v1.0  
**适用代码基线**：`UAV` 仓库当前 main 分支  
**关联文档**：
- `无人机避障设计文档.md` — 机间避障算法与安全状态机（Phase A–D，本文在其基础上扩展为分布式实现）
- `MISSION_PLANNER_PARSING_CN.md` — MP 连接拓扑、P900 多从、GCS 命令接口
- `MISSION_SWARM_IMPLEMENTATION_CN.md` — 群控任务与执行模式
- `PROJECT_ANALYSIS_CN.md` — 项目现状与风险

---

## 1. 文档目标与范围

### 1.1 目标

1. **去中心化通信**：机间由星型拓扑演进为 Mesh 组网（优先 IP/5G VPN，保留 P900 星型降级）。
2. **动态领队**：编队时由地面站指定任意一架为领队；领队失联时集群自主选举，不依赖单一固定长机。
3. **分布式避撞**：编队意图由领队下发，**避撞由各机本地**基于邻居态势独立计算，不经领队转发。
4. **非编队直发任务**：独立飞行模式下，地面站（MP）**直接对各机**下发任务，不再依赖长机拆分与代理。
5. **最小侵入演进**：复用 `ISwarmAirLink`、`SwarmController` 双线程模型、MAVLink 标准消息映射。

### 1.2 执行模式定义（演进后）

| 模式 | 代号 | 任务来源 | 领队角色 | 避撞 |
|------|------|---------|---------|------|
| 编队协同 | `FORMATION` | GCS 航点 → 领队跟踪 → 下发队形意图 | 动态指定，可切换 | 每机本地 ORCA/APF |
| 独立飞行 | `INDEPENDENT` | GCS **直发**各机 `MISSION_*` | 无领队 | 每机本地 ORCA/APF |
| 待机/维护 | `IDLE` | — | 可选选举待命 | 仅广播态势 |

### 1.3 范围（In Scope）

| 类别 | 说明 |
|------|------|
| Mesh 机间链路 | `MeshLink` 实现 `ISwarmAirLink`，sysid 路由 + 广播 |
| 邻居态势 | `NeighborTable` + `STATE_BCAST` 全机广播 |
| 分布式避撞 | APF → VO/TTC → ORCA 分阶段 |
| 动态角色 | `RoleManager`：指定领队、心跳、选举 |
| GCS 直发任务 | 取消长机 `MissionSplitter` 代理路径（保留兼容开关） |
| 遥测去中心化 | 每机直报 GCS，逐步废弃长机合成注入 |
| SITL 回归 | 多机 Mesh + 编队 + 独立 + 领队切换用例 |

### 1.4 非范围（Out of Scope，后续单独立项）

| 类别 | 说明 |
|------|------|
| 真 Mesh 硬件选型与驱动 | 本文给出接口与阶段 6 接入点；硬件采购另文 |
| 802.11s / BATMAN 等多跳路由 | 阶段 1–5 假设 L3 可达（VPN/5G）；多跳路由阶段 6 |
| 静态障碍物 / SLAM | 飞控 OA 或传感器融合，见避障设计文档 Phase D |
| MP 插件 UI | 领队选择、Mesh 拓扑可视化 |
| 拍卖/匈牙利任务分配 | GCS 侧规划工具，非机载必需 |

---

## 2. 现状与问题

### 2.1 当前架构（中心化）

```
Mission Planner ──(5G/VPN/P900)──► 长机 swarm_node（唯一枢纽）
                                      │
              ┌───────────────────────┼───────────────────────┐
              │ FC UART               │ 机间 UDP/P900           │
              ▼                       ▼                         ▼
        长机 ArduPilot          僚机1 swarm_node           僚机N swarm_node
                                      │                         │
                                      ▼                         ▼
                                僚机1 FC                  僚机N FC
```

**长机四合一职责**（去中心化需拆解）：

| 职责 | 实现位置 | 问题 |
|------|---------|------|
| 机间路由器 | `InterUavLink::peers_` 仅长机维护 | 僚机不可达彼此 |
| GCS 网关 | P900 Master / 合成遥测注入 | 地面站依赖长机在线 |
| 编队指挥官 | `FormationCalculator` + `FOLLOWER_CMD` | 固定启动 role=LEADER |
| 任务代理 | `MissionSplitter` + `CTRL_CMD` 代理 | 独立模式仍经长机 |

### 2.2 机间协议现状

```cpp
// include/comm/inter_uav_link.h
enum class LinkMsgType : uint8_t {
    STATE_REPORT  = 0x01,  // follower -> leader（星型）
    FOLLOWER_CMD  = 0x06,  // leader -> follower
    CTRL_CMD      = 0x10,  // leader -> follower 代理 GCS
    TASK_META     = 0x20,  // leader -> follower 子任务
    ...
};
```

- `STATE_REPORT` 速度由 `vel+hdg` 合成，`vz=0`，无法做 TTC/VO。
- 无僚机间消息；无角色宣告；无广播态势。

### 2.3 与安全/协同相关的代码缺口

| 缺口 | 位置 | 影响 |
|------|------|------|
| 无机间避撞 | `run_leader_tick()` 直发 `FOLLOWER_CMD` | 转弯交叉、共点撞机 |
| 第 3 架起固定长机速度 | `formation.cpp` | 追尾风险 |
| 独立模式共点 | `mission_splitter.cpp` 衔接航点共享 | 同时到达同坐标 |
| 角色编译期固定 | `main.cpp` `role=argv[2]` | 无法动态换领队 |
| 长机合成遥测 | `inject_follower_telemetry_to_gcs()` | 非真机直连，参数不可写 |

---

## 3. 目标架构

### 3.1 设计原则

1. **意图集中、安全分布**：领队（或 GCS）给期望轨迹/速度 `v_pref`；最终控制量 `v_safe = Avoid(v_pref, neighbors)` 在**本机**计算。
2. **对称节点**：所有无人机运行同一 `swarm_node` 二进制；`NodeRole` 为**运行时状态**，非启动参数硬编码。
3. **Mesh 优先、星型兜底**：外场优先 5G/VPN 全互 ping；P900 多从保留为 `FALLBACK_STAR` 链路档位。
4. **GCS 可达任意节点**：MP 按 `target_system=sysid` 直发；不再强制经唯一长机转发。
5. **分层防御**：GCS 规划去冲突 → 本地 ORCA 运行时 → CRITICAL 本机 LOITER → 飞控 OA（可选）。

### 3.2 逻辑架构图

```
                    ┌─────────────────────────────────────┐
                    │           地面站 Mission Planner       │
                    │  SET_LEADER / MISSION_* / 群控命令    │
                    └──────────────┬──────────────────────┘
                                   │ 可达任意 sysid（Mesh/VPN）
         ┌─────────────────────────┼─────────────────────────┐
         ▼                         ▼                         ▼
   ┌───────────┐             ┌───────────┐             ┌───────────┐
   │ UAV-1     │◄──Mesh─────►│ UAV-2     │◄──Mesh─────►│ UAV-N     │
   │ swarm_node│   STATE_    │ swarm_node│   STATE_    │ swarm_node│
   │           │   BCAST     │           │   BCAST     │           │
   │ ┌───────┐ │             │ ┌───────┐ │             │ ┌───────┐ │
   │ │Role   │ │             │ │Role   │ │             │ │Role   │ │
   │ │Manager│ │             │ │Follower│             │ │Follower│ │
   │ └───┬───┘ │             │ └───┬───┘ │             │ └───┬───┘ │
   │     │     │             │     │     │             │     │     │
   │ ┌───▼───┐ │             │ ┌───▼───┐ │             │ ┌───▼───┐ │
   │ │Neighbor│             │ │Neighbor│             │ │Neighbor│ │
   │ │Table  │◄─────────────┼►│Table  │◄─────────────┼►│Table  │ │
   │ └───┬───┘ │             │ └───┬───┘ │             │ └───┬───┘ │
   │ ┌───▼───────┐           │ ┌───▼───────┐           │ ┌───▼───────┐
   │ │Collision  │           │ │Collision  │           │ │Collision  │
   │ │Avoidance  │           │ │Avoidance  │           │ │Avoidance  │
   │ └───┬───────┘           │ └───┬───────┘           │ └───┬───────┘
   │     ▼                   │     ▼                   │     ▼
   │ ArduPilot FC            │ ArduPilot FC            │ ArduPilot FC
   └───────────┘             └───────────┘             └───────────┘
```

### 3.3 编队模式数据流

```
1. GCS: SET_LEADER(sysid=L) + UPLOAD_WP + START_FORMATION
2. UAV-L: RoleManager 进入 LEADER；广播 ROLE_ANNOUNCE / ROLE_HEARTBEAT
3. UAV-L: navigate_leader() + FormationCalculator → 各机 v_pref（FOLLOWER_CMD）
4. 全体: 周期 STATE_BCAST（位置 + vn/ve/vd）
5. 各僚机: v_safe = CollisionAvoidance.solve(v_pref, NeighborTable)
6. 各机: fc->set_velocity_target_local_ned(v_safe) 或位置+速度混合
7. 领队失联: ROLE_HEARTBEAT 超时 → 僚机选举 → 新领队继续步骤 3
```

### 3.4 独立模式数据流（GCS 直发）

```
1. GCS: SET_MODE(INDEPENDENT) — 各机本地切换，不经长机
2. GCS: MISSION_COUNT/ITEM_INT → target_system=1,2,3,... 各机分别下发
3. 各机: 本地 upload_mission + AUTO（现有 MavlinkVehicle 能力）
4. 各机: 仍周期 STATE_BCAST + 本地 CollisionAvoidance（运行时兜底）
5. 不再调用: distribute_independent_mission() / MissionSplitter（可通过 --legacy-independent 保留）
```

---

## 4. 通信层设计

### 4.1 链路抽象（已有 + 扩展）

现有 `ISwarmAirLink`（`include/comm/inter_uav_link.h`）保持不变。新增实现：

| 实现类 | 场景 | 说明 |
|--------|------|------|
| `InterUavLink` | 实验室 / 星型 UDP | 保留；僚机 `set_target` 到已知 IP |
| `MeshLink` | **目标：5G/VPN 全互连** | 维护完整 `sysid→ip:port`；支持单播与广播 |
| `P900FramedLink` | P900 多从（文档 §11） | 长机 Master 调度；`FALLBACK_STAR` 档位 |

`SwarmController` 构造时按 `--link-profile` 选择后端：

| profile | 机间后端 | GCS | 说明 |
|---------|---------|-----|------|
| `lan` | MeshLink UDP | IP/串口 | SITL/局域网 |
| `5g` | MeshLink VPN | IP | 外场默认 |
| `p900-star` | P900FramedLink | P900 Master | 星型降级，无真正 Mesh |
| `hybrid` | MeshLink + P900 兜底 | IP 主 + P900 备 | LOST 时切换 |

### 4.2 MeshLink 行为

**邻居发现**：

- 任意收到机间 MAVLink 帧时 `note_peer(src_sysid, from_ip, from_port)`（复用现有逻辑）。
- 启动可选静态路由：`--mesh-peer 2=10.8.0.2:19870,3=10.8.0.3:19870`。
- 可选 UDP 组播：`--mesh-mcast 239.255.0.1:19870` 用于 `STATE_BCAST`。

**发送策略**：

```cpp
int MeshLink::send_packet(const LinkPacket &pkt) {
    if (pkt.dst_id == 0) {
        // 广播：组播或遍历 peers_ 单播
        return broadcast(pkt);
    }
    // 单播：查 peers_[dst_id]
    return unicast(pkt);
}
```

**与 P900 差异**：P900 多从下僚机仅与 Master 通信，**不能**实现对等 Mesh；该场景下 `STATE_BCAST` 可由长机**中继**（`RELAY_BCAST` 扩展，见 §4.4 可选）。

### 4.3 机间消息类型（扩展）

| 类型 | 值 | 方向 | MAVLink 映射 | 说明 |
|------|-----|------|-------------|------|
| `STATE_REPORT` | 0x01 | 机→领队 | `GLOBAL_POSITION_INT` | **兼容保留**；Mesh 下可弱化 |
| `STATE_BCAST` | **0x02** | **每机→全体** | `GLOBAL_POSITION_INT`, `dst=0` 或组播 | **v2 含 vn/ve/vd** |
| `FOLLOWER_CMD` | 0x06 | 领队→僚机 | `SET_POSITION_TARGET_GLOBAL_INT` | 队形意图 `v_pref` |
| `ROLE_ANNOUNCE` | **0x07** | 领队→全体 | `COMMAND_LONG` 自定义 | param1=leader_sysid |
| `ROLE_HEARTBEAT` | **0x08** | 领队→全体 | `HEARTBEAT` 或自定义 | 存活 + 任期 seq |
| `ROLE_ELECT` | **0x09** | 候选→全体 | `COMMAND_LONG` 自定义 | 选举投票（可选 Phase 4） |
| `CTRL_CMD` | 0x10 | 代理 | 现有 | **Mesh 模式下逐步废弃** |
| `TASK_*` | 0x20–0x24 | 长机→僚机 | 现有 | **独立直发模式下废弃** |

### 4.4 STATE_BCAST v2 载荷

**编码**（`link_mavlink_codec.cpp` 扩展）：

| 字段 | 来源 | 说明 |
|------|------|------|
| lat, lon, relative_alt | 本机 `UnitState` | 与现有一致 |
| vx, vy, vz | **真实** `vn, ve, vd`（cm/s） | 不再用 vel+hdg 合成 |
| hdg | 本机 heading | 保留 |
| 协议标记 | `gp.alt == 1002` | v2 含真实 NED 速度 |
| time_boot_ms | 本地单调 ms | 供接收方算 `state_age` |

**频率**：

| profile | STATE_BCAST | FOLLOWER_CMD | ctrl_hz |
|---------|-------------|--------------|---------|
| lan | 10 Hz | 10 Hz | 20 |
| 5g | 5 Hz | 5 Hz | 10 |
| p900-star | 2 Hz | 2 Hz | 5 |

---

## 5. 邻居态势模块

### 5.1 NeighborTable

**文件**：`include/swarm/neighbor_table.h`，`src/swarm/neighbor_table.cpp`

```cpp
struct NeighborSnapshot {
    int      sysid = 0;
    double   lat = 0, lon = 0;   // rad
    float    alt_rel = 0;        // m
    float    vn = 0, ve = 0, vd = 0;
    uint64_t last_rx_ms = 0;
    bool     valid(uint64_t now, uint64_t max_age_ms) const;
};

class NeighborTable {
public:
    void upsert(const LinkPacket &bcast, uint64_t now_ms);
    void prune(uint64_t now_ms, uint64_t max_age_ms);
    std::vector<NeighborSnapshot> snapshot(uint64_t now_ms) const;
    double min_pair_distance_h(uint64_t now_ms) const;  // 可观测
};
```

**更新来源**：收包线程收到 `STATE_BCAST`（及兼容 `STATE_REPORT`）写入；控制线程读快照。

**过期策略**：`state_age > state_max_age_ms`（默认 2000ms）的邻居不参与避撞，触发本机 `AvoidLevel::STALE` 降级。

---

## 6. 分布式避撞模块

### 6.1 职责边界

| 层级 | 职责 | 执行位置 |
|------|------|---------|
| 领队 / GCS | 队形几何、航点、任务 | 领队 `FormationCalculator` 或 GCS 规划 |
| **CollisionAvoidance** | 机间分离、TTC 预警 | **每机本地** |
| SafetyStateMachine | WARN/CRITICAL/LOITER | **每机本地** |
| ArduPilot OA | 静态障碍（可选） | 飞控 |

### 6.2 接口

**文件**：`include/swarm/collision_avoidance.h`，`src/swarm/collision_avoidance.cpp`

```cpp
enum class AvoidLevel : uint8_t {
    NONE = 0, WARN, CRITICAL, STALE
};

struct AvoidanceParams {
    bool   enable = true;
    double safe_h = 20, crit_h = 10;
    double safe_v = 15, crit_v = 8;
    double t_horizon = 3.0, t_crit = 2.0;
    double sep_gain = 10.0, max_correction_h = 25.0;
    double vel_scale_min = 0.3;
    uint64_t state_max_age_ms = 2000;
};

struct AvoidInput {
    UnitState self;
    double v_pref_n, v_pref_e, v_pref_d;  // 领队意图或航点跟踪
    std::vector<NeighborSnapshot> neighbors;
};

struct AvoidOutput {
    AvoidLevel level = AvoidLevel::NONE;
    double v_out_n, v_out_e, v_out_d;
    double min_dist_h, min_ttc;
    int    pairs_in_conflict = 0;
};

class CollisionAvoidance {
public:
    void set_params(const AvoidanceParams &p);
    AvoidOutput solve(const AvoidInput &in) const;
};
```

### 6.3 算法分阶段

| 阶段 | 算法 | 输入 | 输出 | 适用 |
|------|------|------|------|------|
| **Phase A** | APF 斥力 + 速度缩放 | 距离 | 修正位置/速度 | 首飞、SITL T1–T3 |
| **Phase B** | VO / 简化 TTC | 距离 + 速度 | 最小 Δv | 对向接近 |
| **Phase C** | ORCA 2D LP | 多邻居半平面 | 最接近 v_pref 的 v_safe | ≥3 机、工业推荐 |

**Phase A 核心**（与《无人机避障设计文档》§6.2 一致）：

```
对每个邻居 j，若 d_h < safe_h 且 d_v < safe_v:
  u_sep = normalize(p_self - p_j)
  Δp += sep_gain * (1/d_h - 1/safe_h) * u_sep，限幅 max_correction_h
v_out = scale(v_pref, f(d_min))，f ∈ [vel_scale_min, 1]
level = f(d_min, ttc)
```

**Phase C ORCA**：对每对 (i,j) 构造 ORCA 半平面，2D 线性规划求满足所有约束且最接近 `v_pref` 的速度；n≤10 时计算量可忽略。参考 [d-ORCA](https://gamma.umd.edu/pro/aerialswarm/dorca/)。

### 6.4 本机安全状态机

**文件**：`include/swarm/safety_state_machine.h`

```
NORMAL ──(d<safe 或 TTC<t_horizon)──► WARN
WARN ──(d>safe 持续3s)──► NORMAL
WARN ──(d<crit 或 TTC<t_crit)──► CRITICAL
CRITICAL ──(人工/GCS START 或 d>safe 持续5s)──► RECOVER/WARN
任意 ──(无有效邻居)──► STALE（保守降速，不盲修正）
```

| 状态 | 本机动作 |
|------|---------|
| NORMAL | 输出 `v_out ≈ v_pref` |
| WARN | ORCA 修正 + GCS `[AVOID-WARN]` |
| CRITICAL | **本机** LOITER / hold 当前点（不等领队） |
| STALE | 降速至 30%，暂停编队扩张 |

与现有机制并行：**飞控心跳 RTL**、**5G HealthMonitor LOST** 优先级高于避障恢复。

### 6.5 控制环接入

**僚机 / 非领队**（`run_follower_tick` 扩展）：

```
1. 若收到 FOLLOWER_CMD：缓存目标 lat/lon/alt/vel/hdg → 算 v_pref
2. neighbors = neighbor_table_.snapshot(now)
3. out = collision_avoidance_.solve({self, v_pref, neighbors})
4. 若 out.level >= CRITICAL: fc->set_mode(LOITER); return
5. fc->set_velocity_target_local_ned(out.v_out_n, out.v_out_e, out.v_out_d)
6. 周期 send_state_bcast()
```

**领队**（`run_leader_tick` 扩展）：

```
1. navigate_leader() + formation_.compute() → 下发 FOLLOWER_CMD（意图）
2. 领队自身同样执行步骤 2–5（对僚机做本地避撞）
3. 周期 ROLE_HEARTBEAT + send_state_bcast()
```

**接口**：`MavlinkVehicle::set_velocity_target_local_ned()` 已存在于 `mavlink_vehicle.cpp`。

---

## 7. 动态角色与领队选举

### 7.1 RoleManager

**文件**：`include/swarm/role_manager.h`，`src/swarm/role_manager.cpp`

```cpp
enum class SwarmRole : uint8_t {
    IDLE = 0, LEADER, FOLLOWER, CANDIDATE
};

class RoleManager {
public:
    void on_gcs_set_leader(int leader_sysid, int local_sysid);
    void on_role_announce(int leader_sysid, uint32_t term, uint64_t now);
    void on_role_heartbeat(int leader_sysid, uint32_t term, uint64_t now);
    void tick(uint64_t now_ms);  // 超时检测、选举

    SwarmRole role() const;
    int  leader_sysid() const;
    bool is_leader() const;
};
```

### 7.2 GCS 指定领队

新增 MAVLink 命令（`SwarmMavCmd` 扩展）：

| 命令 | command | param1 | 说明 |
|------|---------|--------|------|
| `SET_LEADER` | 31018 | leader_sysid | 全网切换领队 |
| `SET_LEADER` | 31018 | 0 | 释放领队，全体 IDLE |

**行为**：

1. 被指定 sysid 的节点：`role=LEADER`，广播 `ROLE_ANNOUNCE(term++)`。
2. 其余节点：`role=FOLLOWER`，记录 `leader_sysid`。
3. 旧领队收到后自动降为 FOLLOWER。

### 7.3 领队失联与选举

**参数**：

| 参数 | 默认 | 说明 |
|------|------|------|
| `leader_hb_timeout_ms` | 3000 | 无 ROLE_HEARTBEAT → 认为失联 |
| `election_quiet_ms` | 1000 | 选举窗口，避免抖动 |
| `election_policy` | `MIN_SYSID` | 或 `MAX_BATTERY` |

**流程**：

```
1. follower 检测 leader_hb 超时 → CANDIDATE
2. 广播 ROLE_ELECT(self_sysid, term)
3. 确定性规则选 winner（最小 sysid 且 battery 最高优先）
4. winner → LEADER，广播 ROLE_ANNOUNCE
5. 其余 → FOLLOWER
```

**迟滞**：新领队需连续 3 个周期收到多数僚机 `ROLE_ACK`（可选）后才恢复 `FOLLOWER_CMD` 下发，避免双领队。

### 7.4 与启动参数的关系

| 现参数 | 演进 |
|--------|------|
| `swarm_node <sysid> <role>` | **保留兼容**；role 仅作默认初值 |
| `--followers a,b,c` | 领队侧：编队成员列表；Mesh 下可从 GCS `SET_SWARM_MEMBERS` 动态配置 |
| `--leader-ip` | 星型 fallback；Mesh 下可选 |

---

## 8. 地面站与 Mission Planner 集成

### 8.1 遥测：取消长机合成（目标态）

| 现行为 | 目标行为 |
|--------|---------|
| 长机 `inject_follower_telemetry_to_gcs()` 代发僚机 HEARTBEAT/POS | **每机直报 GCS**（5G/VPN 或 Mesh 多跳） |
| MP 中僚机参数不可真读 | 各机维护链路时可正常 PARAM |

**迁移开关**：`--legacy-synthetic-telem` 保留旧行为至阶段 5 完成。

### 8.2 命令路由

Mesh 模式下 MP 连接**任意在线节点**或**地面 VPN 网关**，下发时：

- 必须设置 **`target_system = 目标 sysid`**（文档 §7.3 已建议）。
- 群控命令 `SET_LEADER`、`START_FORMATION` 可广播 `target_system=0` 或逐机下发。

**废弃路径**（可 `--legacy-proxy` 保留）：

- `try_proxy_gcs_command_to_follower()` → `CTRL_CMD`
- `distribute_independent_mission()` → `TASK_*`

### 8.3 独立模式操作员流程（新）

```
1. MP 连接 VPN，可见 sysid 1..N 均在线
2. 对每架选中飞机分别：规划航线 → Write Waypoints（target=该机 sysid）
3. 发送 SET_MODE(INDEPENDENT) 到各机（或单机本地切换）
4. 各机：Readiness 检查（GPS、armed、neighbor STALE 无 CRITICAL）
5. 各机：START_EXEC 或 MP 标准 AUTO 启动
6. 运行中：各机本地 ORCA；GCS 监控 min_d 日志（可选 STATUSTEXT）
```

**规划去冲突**（GCS 侧建议，非机载必需）：

- 高度分层：相邻 sysid 差 20m
- 时间错开：起飞间隔 5–8s
- 航线空间分离：平行航带 30m

---

## 9. SwarmController 改造要点

### 9.1 配置扩展（SwarmConfig）

```cpp
// 新增字段示例
enum class LinkProfile : uint8_t { LAN, G5, P900_STAR, HYBRID };
LinkProfile link_profile = LinkProfile::G5;

bool mesh_enable = true;
std::string mesh_mcast_addr;          // 可选
std::unordered_map<int, std::string> mesh_static_peers;

bool avoid_enable = true;
AvoidanceParams avoid_params;

bool legacy_independent = false;    // 启用旧 MissionSplitter 路径
bool legacy_synthetic_telem = false;
bool legacy_gcs_proxy = false;

uint64_t leader_hb_timeout_ms = 3000;
```

### 9.2 成员变量新增

```cpp
std::unique_ptr<ISwarmAirLink> air_link_;  // 替代裸指针二选一
NeighborTable                  neighbor_table_;
CollisionAvoidance             collision_avoidance_;
SafetyStateMachine             safety_sm_;
RoleManager                    role_manager_;
std::atomic<SwarmRole>         swarm_role_;
```

### 9.3 线程模型（不变）

- **控制线程**：`control_loop` → 角色 tick + 编队/独立 + 避撞 + 状态广播
- **收包线程**：`receive_loop` → 更新 NeighborTable + RoleManager + 任务包

### 9.4 关键函数变更摘要

| 函数 | 变更 |
|------|------|
| `init()` | 按 `link_profile` 构造 MeshLink / UDP / P900 |
| `run_leader_tick()` | 仅 LEADER 下发 FOLLOWER_CMD；领队本地避撞 |
| `run_follower_tick()` | v_pref + 避撞 + STATE_BCAST |
| `receive_loop()` | 处理 STATE_BCAST、ROLE_* |
| `handle_gcs_command_long()` | 解析 SET_LEADER |
| `distribute_independent_mission()` | 默认禁用；legacy 开关保留 |
| `inject_follower_telemetry_to_gcs()` | 默认禁用 |

---

## 10. 源码文件清单

### 10.1 新增

| 路径 | 职责 |
|------|------|
| `include/swarm/neighbor_table.h` | 邻居表 |
| `src/swarm/neighbor_table.cpp` | |
| `include/swarm/collision_avoidance.h` | APF/VO/ORCA |
| `src/swarm/collision_avoidance.cpp` | |
| `include/swarm/safety_state_machine.h` | 本机安全状态 |
| `src/swarm/safety_state_machine.cpp` | |
| `include/swarm/role_manager.h` | 动态领队/选举 |
| `src/swarm/role_manager.cpp` | |
| `include/comm/mesh_link.h` | Mesh ISwarmAirLink |
| `src/comm/mesh_link.cpp` | |

### 10.2 修改

| 路径 | 变更 |
|------|------|
| `include/comm/inter_uav_link.h` | 新 LinkMsgType；LinkPacket 增加 vn/ve/vd |
| `src/comm/link_mavlink_codec.cpp` | STATE_BCAST v2；ROLE_* 编解码 |
| `src/comm/inter_uav_link.cpp` | 广播发送 |
| `include/common/types.h` | SwarmMavCmd::SET_LEADER |
| `include/swarm/swarm_controller.h` | 新成员与配置 |
| `src/swarm/swarm_controller.cpp` | 控制环接入 |
| `src/swarm/formation.cpp` | 第 3 架起统一速度律 |
| `src/main.cpp` | 新 CLI 参数 |
| `CMakeLists.txt` | 新源文件 |

---

## 11. 命令行参数（规划）

```bash
./swarm_node <sysid> [role] [options]

# Mesh / 链路
  --link-profile <lan|5g|p900-star|hybrid>
  --mesh-peer <sysid=ip:port,...>
  --mesh-mcast <ip:port>
  --link-port <p>                 # 默认 19870

# 避撞
  --avoid-enable <0|1>
  --avoid-safe-h <m>
  --avoid-crit-h <m>
  --avoid-t-horizon <s>
  --avoid-algo <apf|vo|orca>      # 默认 apf，逐步开放

# 角色
  --default-role <leader|follower|idle>
  --leader-hb-timeout-ms <n>

# 兼容旧行为
  --legacy-independent
  --legacy-synthetic-telem
  --legacy-gcs-proxy

# 控制频率
  --hz <n>                        # 避障开建议 20（lan）/ 10（5g）
```

---

## 12. 默认参数速查

| 参数 | LAN/SITL | 5G 外场 | P900 星型 |
|------|----------|---------|-----------|
| ctrl_hz | 20 | 10 | 5 |
| STATE_BCAST Hz | 10 | 5 | 2 |
| safe_h / crit_h (m) | 20 / 10 | 25 / 15 | 30 / 20 |
| safe_v / crit_v (m) | 15 / 8 | 20 / 10 | 20 / 10 |
| t_horizon (s) | 3 | 4 | 5 |
| state_max_age_ms | 1000 | 2000 | 3000 |
| leader_hb_timeout_ms | 2000 | 3000 | 5000 |

---

## 13. 实施里程碑

### 总览（约 8–10 周）

```
阶段1 对称节点+MeshLink(IP)     ──► 阶段2 STATE_BCAST+NeighborTable
        │                                    │
        └──────────────┬─────────────────────┘
                       ▼
              阶段3 分布式避撞 APF→ORCA
                       │
                       ▼
              阶段4 动态领队+选举
                       │
                       ▼
              阶段5 GCS直发+去合成遥测
                       │
                       ▼
              阶段6 Mesh硬件/多跳（可选）
```

### 阶段 1：对称节点 + MeshLink（1 周）

- [ ] `SwarmRole` 运行时化；`RoleManager` 骨架
- [ ] `MeshLink` 实现 `ISwarmAirLink`（单播 + 静态 peer + 可选组播）
- [ ] `SwarmController` 依赖 `unique_ptr<ISwarmAirLink>`
- [ ] SITL：3 机 VPN 互 ping，互发测试包

**验收**：任意两机 sysid 互发单播成功；不再依赖 `--leader-ip` 才能路由。

### 阶段 2：态势广播 + 邻居表（1 周）

- [ ] `STATE_BCAST` v2 编解码（真实 vn/ve/vd）
- [ ] `NeighborTable` 接入收包/控制线程
- [ ] 全体周期广播；1Hz 日志 `min_d`、邻居数
- [ ] 保留 `STATE_REPORT` 兼容领队缓存

**验收**：每机打印邻居 sysid 与距离；包龄 < 500ms（LAN）。

### 阶段 3：分布式避撞（2–3 周）

- [ ] `CollisionAvoidance` Phase A（APF）+ `SafetyStateMachine`
- [ ] 僚机/领队控制环接入 `set_velocity_target_local_ned`
- [ ] Phase B TTC/VO；Phase C ORCA（≥3 机）
- [ ] `formation.cpp` 第 3 架起速度律修复
- [ ] SITL T1–T5（见 §14）

**验收**：小间距编队转弯 min_d > crit_h；对向接近触发 WARN/CRITICAL 本机 LOITER。

### 阶段 4：动态领队 + 选举（1.5 周）

- [ ] `SET_LEADER` GCS 命令；`ROLE_ANNOUNCE` / `ROLE_HEARTBEAT`
- [ ] 领队超时选举；双领队防护
- [ ] 编队中切换领队 SITL 用例

**验收**：领队进程退出后 5s 内新领队产生并恢复 FOLLOWER_CMD。

### 阶段 5：GCS 直发 + 去中心化遥测（1.5 周）

- [ ] 默认关闭 `inject_follower_telemetry` 与 `distribute_independent_mission`
- [ ] 文档更新 MP 多连接/VPN 操作说明
- [ ] `--legacy-*` 开关与迁移指南

**验收**：无长机进程时，GCS 仍可对各机写任务并 AUTO；各机直报 MP。

### 阶段 6：Mesh 硬件 / 多跳（可选，2+ 周）

- [ ] 对接 Mesh 电台 SDK 或 802.11s
- [ ] 多跳路由或应用层中继 `RELAY_BCAST`
- [ ] 外场带宽/时延标定

---

## 14. 测试方案

### 14.1 SITL 环境

```bash
# 示例：4 实例，VPN 或 localhost 不同端口
# UAV1: sysid=1, --link-profile lan --mesh-peer 2=127.0.0.1:19871,...
# 各实例 FC UDP 14550/14560/...
```

### 14.2 用例表

| ID | 场景 | 步骤 | 通过标准 |
|----|------|------|----------|
| M1 | Mesh 连通 | 3 机互发 STATE_BCAST | 每机邻居数=2，包龄<500ms |
| M2 | 指定领队 | SET_LEADER(2) + 编队 | sysid2 发 FOLLOWER_CMD，其余跟随 |
| M3 | 领队切换 | 编队中 SET_LEADER(3) | 5s 内切换无 CRITICAL |
| M4 | 领队崩溃 | kill 领队进程 | 10s 内选举新领队，编队恢复或 LOITER 安全 |
| A1 | 正常编队 | delta_dis=100 直线 | min_d>safe_h，level=NONE |
| A2 | 小间距 | delta_dis=30 直线 | WARN 无碰撞 |
| A3 | 转弯交叉 | 90° 转弯 | min_d>crit_h 或 CRITICAL 本机 LOITER |
| A4 | 对向接近 | 独立模式交叉航线 | 本机 ORCA 避让 |
| A5 | 状态过期 | drop 50% BCAST | STALE，不盲修正 |
| G1 | GCS 直发任务 | 无领队，各机 MISSION | 各机 AUTO 独立执行 |
| G2 | 兼容 legacy | --legacy-independent | 与现网 TASK_* 行为一致 |

### 14.3 实飞前置

1. SITL 全用例通过  
2. GPS 3D Fix，sat≥10  
3. 首飞 2 机、高度分层 25m、`--avoid-algo apf`  
4. 空域隔离与应急断链预案  

---

## 15. 风险与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| P900 无法真 Mesh | 仍星型 | `p900-star` profile + 长机中继 BCAST（可选） |
| 广播带宽 O(n²) | 大机群拥塞 | 组播 + 降频；n>10 分区 |
| 选举抖动 | 双领队/无领队 | 任期 term +  quiet 窗口 + GCS 显式 SET_LEADER |
| ORCA 与 GUIDED 滞后 | 修正振荡 | max_correction 限幅；ctrl_hz 20 |
| GCS 多机带宽 | 遥测拥塞 | 令牌桶 + 拉参暂停（现有逻辑复用） |
| 与旧 MP 流程不兼容 | 操作员习惯 | `--legacy-*` 三开关并行一个版本周期 |

---

## 16. 与现有文档关系

| 文档 | 关系 |
|------|------|
| `无人机避障设计文档.md` | 本文 §6 为其 **分布式落地版**；原 Phase A–D 算法仍有效，执行位置从「仅长机」改为「每机」 |
| `MISSION_PLANNER_PARSING_CN.md` | §8 遥测/命令路由在阶段 5 后更新；P900 §2.1 降为 fallback |
| `MISSION_SWARM_IMPLEMENTATION_CN.md` | 执行模式枚举保留；独立模式「长机拆分」改为「GCS 直发」 |
| `PROJECT_ANALYSIS_CN.md` | 路由/可观测性项在本方案阶段 1–5 中覆盖 |

---

## 17. 术语表

| 术语 | 含义 |
|------|------|
| Mesh | 机间对等组网，各节点可互寻址 |
| v_pref | 领队/GCS 给出的期望速度或隐含速度 |
| v_safe / v_out | 避撞模块输出后的安全速度 |
| ORCA | Optimal Reciprocal Collision Avoidance，互惠最优避障 |
| TTC | Time To Collision |
| term | 领队任期序号，防旧心跳干扰 |
| legacy 开关 | 保留旧中心化行为的编译/运行选项 |

---

## 18. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-06-09 | 初版：Mesh 组网、动态领队、分布式避撞、GCS 直发任务、分阶段实施与测试 |
