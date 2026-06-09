# 5G 数图传一体改造详细实现方案

| 项目 | 说明 |
|------|------|
| 文档版本 | v1.2 |
| 适用工程 | UAV_Swarm_APM（鲁班猫0 + ArduPilot 群控伴飞） |
| 关联现状 | P900 串口成帧（`P900FramedLink`）+ 可选 UDP 机间（`InterUavLink`） |
| 目标 | **以 5G 蜂窝 IP 全面替代 P900 电台**，承载机间群控、地面遥测/指令、图传三业务；**P900 路径标 deprecated，仅作可选硬退化备份**；**地面 Mission Planner 采用每机直连 + 定制接收/处理多机数据** |
| 范围变化 | 删除 P900 在主流程中的默认依赖；新增「GCS over IP」抽象与会话模型；新增链路健康监测/降级；附 systemd / tc / 代码改动清单 |

---

## 1. 背景与目标

### 1.1 背景

当前外场推荐拓扑为 **P900 电台**：机间与（长机）地面站可经 **UART9 成帧复用**（`P900FramedLink`）；地面端依赖 `tools/p900_gcs_bridge.py` 将 MAVLink 封装为应用层帧格式。该方案带宽受串口波特率限制，**无法在同一链路上承载高清图传**，且：

- 同口复用导致 **GCS 下行 / 僚机上报 / 长机回传 / ACK** 全部抢占同一 UART，突发易拥塞。
- 长机做 GCS 网关，需把僚机状态 **合成假 MAVLink** 注入 GCS（`SwarmController::inject_follower_telemetry_to_gcs`），存在 **双源/伪造遥测** 与改参不可达的工程妥协。
- 拓扑刚性，扩机数受限。

5G 模组在伴飞计算机上通常提供 **以太网类接口（RNDIS/CDC-NCM）或 PPP**，获得 **Mbps 级 IP 连通性**，具备「数传 + 图传」一体化物理条件。需要工程上解决的核心问题：**NAT、抖动、多机寻址、带宽竞争、链路抖动/中断的可观测与降级**。

### 1.2 建设目标（v1.2 调整：5G 全面替代 P900）

1. **承载切换**：**全部** 业务（机间群控 / GCS 遥测指令 / 图传）默认承载在 5G + VPN 之上；**默认配置不启用 P900**。`P900FramedLink` 与 `tools/p900_gcs_bridge.py` 标记为 **deprecated**，仅作弱信号外场的可选硬退化备份保留代码。
2. **数传 - 机间**：保持长机中心化群控逻辑，复用 `ISwarmAirLink / InterUavLink`（UDP）承载机间帧，仅替换底层 IP 网络（UART → 5G IP）。
3. **数传 - GCS**：与地面 Mission Planner 改为 **每架伴飞直连**（不再经长机二次转发，**默认关闭 `inject_follower_telemetry_to_gcs`**）。新增 **`IGcsLink` / `GcsIpLink`** 抽象（UDP 优先、TCP 可选）替换原 `gcs_serial_` 与 `link_p900_.write_mavlink_to_gcs` 路径。
4. **图传**：与数传 **同模组、不同进程、不同端口、不同 QoS class**；GStreamer RTP/UDP 或 RTSP，单机码率自适应。
5. **地面站**：Mission Planner **定制开发**——单端口多源接入、按 `sysid` 维护多架状态、按 `sysid` 路由下行指令至「会话表」中对应伴飞 IP。
6. **可观测与降级**：引入 5G 链路健康监测（RSSI/RTT/丢包/累计字节），失联或超阈值时自动降级（本机 RTL/LOITER）；保留 `--p900-uart9` 入口作为 **可插拔回退**。
7. **工程约束**：尽量复用现有 `ISwarmAirLink / InterUavLink`、`MavlinkManager` 与 `enqueue_gcs_*` 队列结构；新增代码集中在 `comm/gcs_ip_link.{h,cpp}` 与 `SwarmController` 配置/分支，避免侵入控制环。

### 1.3 非目标（本阶段可不实现）

- 图传与 MAVLink 在同一 UDP 端口内自定义复用帧（可做二期优化）。
- 运营商 QoS 签约级 SLA 保障（依赖商务与专网，本方案以应用层 token + Linux tc 为主）。
- 替代飞控侧硬件数传口的工业认证方案（需单独合规评估）。
- MAVLink 应用层加密 / 签名（一期依赖 VPN 通道，远期再引入 MAVLink2 signing）。

---

## 2. 现状分析（与代码对应）

### 2.1 链路抽象

- **`ISwarmAirLink`**（`include/comm/inter_uav_link.h`）：机间发送/接收 `LinkPacket`，底层经 MAVLink 编解码。  
- **`InterUavLink`**：UDP 套接字，`bind(listen_ip, link_listen_port)`，`set_target(link_target_ip, port)`。  
- **`P900FramedLink`**：串口 + 成帧（魔数、版本、CRC、src/dst），长机侧 **`write_mavlink_to_gcs` / `pop_gcs_byte`** 与 GCS 逻辑通道复用。

### 2.2 进程启动参数（`src/main.cpp`）

| 参数 | 作用 |
|------|------|
| `--p900-uart9` | 打开 P900 成帧模式；长机自动 `gcs_serial_enable` |
| `--p900-dev` / `--p900-baud` | P900 串口设备与波特率 |
| `--link-port` | 机间 UDP 监听端口（默认 19870） |
| `--leader-ip` | 僚机指向长机 IP（UDP 模式） |
| `--gcs-serial` / `--gcs-baud` | 独立 GCS 串口（非 P900 复用时） |

### 2.3 5G 改造时的关键差异

| 维度 | P900（现状） | 5G IP（目标） |
|------|------|--------|
| 寻址 | 帧内 `src/dst`（sysid 映射） | IP + 端口；通过 VPN 固定私网地址，或公网 + 中继云 |
| 拓扑 | 总线复用，长机做网关 | **每机直连地面 + 机间长机中心**；P 平面拆分 |
| GCS | 成帧 + 桥接脚本（`tools/p900_gcs_bridge.py`） | 标准 **UDP/TCP MAVLink**；单端口多源 + 会话表 |
| 带宽估算 | `gcs_tx_token_rate_bps_` 按串口波特率 × 0.7 | 按 **`--gcs-udp-tx-kbps` 配置 + 链路实测** 自适应 |
| 时延/抖动 | 单跳 ms 级、抖动小但带宽固定 | 数十 ms ~ 数百 ms 抖动、带宽可达 Mbps 但波动大 |
| RTT 典型值 | 串口 5–20 ms（波特率敏感） | 5G NSA 30–80 ms / SA 20–50 ms / VPN 再 +5–20 ms |
| 半双工 | 需调度避免自撞 | 全双工为主，仍需防拥塞与排队累积 |
| 失效模式 | 信号衰减线性 | 阶跃式断流（基站切换/限速/QoS 抢占） |
| 多机扩展 | 总线性能瓶颈，难超 4–6 机 | 仅地面入口聚合带宽与 MP 性能受限 |
| 安全 | 物理空口，难嗅探 | **必须** 走 VPN（WG/Tailscale），禁止公网裸暴露 |

---

## 3. 总体架构

### 3.1 逻辑分层（v1.2）

```
┌────────────────────────────────────────────────────────────────────┐
│  地面：定制 Mission Planner（C#）                                    │
│   ├ 单端口 UDP 14550 接收多机 MAVLink；按 sysid 维护多架 MAVState     │
│   ├ 会话表：sysid → last remote endpoint（UDP）→ 指令回包路由          │
│   └ 视频播放：GStreamer / VLC （独立进程）                             │
└──────────▲───────────▲───────────▲─────────▲───────────────────────┘
           │UDP 14550  │           │         │ RTP UDP 5000+sysid
   ┌───────┴────┐ ┌────┴────┐  ┌───┴────┐    │
   │长机伴飞 #1  │ │僚机伴飞 #2│  │僚机伴飞 N│   │
   │ swarm proc │ │ swarm   │  │ swarm   │   │
   │ + GcsIpLink│ │+GcsIpLink│ │+GcsIpLink│   │
   │ +InterUav  │ │+InterUav │ │+InterUav │   │
   └─┬──┬──┬────┘ └──┬──┬────┘ └──┬──┬────┘   │
     │  │  └───── 机间 UDP 19870（汇聚至长机）──┘   │
     │  │                                       │
     │  └── FC MAVLink （/dev/ttyS8 串口） ──────┤
     │                                          │
     └── GStreamer RTP（图传，独立进程）─────────┘

────────────────── 以下为伴飞主机内部分层 ──────────────────────────────
│  应用层：SwarmController / MavlinkManager / 群控业务                  │
│  数传层：IGcsLink(GcsIpLink) ‖ ISwarmAirLink(InterUavLink)            │
│         （※ P900FramedLink 仅作 deprecated 回退保留）                  │
│  媒体层：GStreamer 进程（与 swarm 进程解耦）                           │
│  网络层：WireGuard/Tailscale；tc HTB + DSCP；策略路由                  │
│  承载层：5G 模组（wwan0/usb0/ppp0）；ModemManager / systemd           │
│  健康层：5GHealthMonitor（RSSI/RSRP/RTT/丢包）→ 触发降级              │
```

**原则**：

1. **业务面拆分**：机间（`InterUavLink`/UDP 19870）、GCS（`GcsIpLink`/UDP 14550）、图传（GStreamer/UDP 5000+sysid）三条独立 UDP 流，**端口、进程、QoS class 三维都隔离**；任何一条拥塞不会拖垮另两条。
2. **控制面 vs 数据面分离**：群控（`SwarmController`）只关心 `ISwarmAirLink`；GCS 收发只关心 `IGcsLink`；底层从 P900/UDP/未来云中继的切换对上层透明。
3. **每机直连 GCS**：地面只看真实飞控来源 `sysid`，避免长机合成假遥测；指令按 `sysid` 路由回正确伴飞 IP。
4. **失效隔离**：5G 主链路异常 → `5GHealthMonitor` 触发降级（本机 LOITER/RTL，可选 P900 fallback），不阻塞控制循环。

### 3.2 推荐拓扑：VPN 全网可达 + 机间长机中心 + GCS 每机直连 MP

1. **VPN**：每台伴飞与地面站接入同一虚拟私网（推荐 **WireGuard**：性能高、内核态、易于在嵌入式部署；Tailscale 适合无运维场景）。
2. **固定私网地址**（示例）：

   | 节点 | VPN IP | 备注 |
   |------|--------|------|
   | 长机伴飞 | `10.8.0.1` | 兼任机间 UDP 汇聚 |
   | 僚机伴飞 i | `10.8.0.(1+i)` | 1 ≤ i ≤ N |
   | 地面 MP 主机 | `10.8.0.100` | 单端口监听 14550/UDP |
   | 视频接收主机 | `10.8.0.100` 或独立 | 端口 5000+sysid |

3. **机间（群控）**：**默认禁用** `--p900-uart9`；走 `InterUavLink` UDP，僚机 `--leader-ip 10.8.0.1 --link-port 19870`，长机仅监听。
4. **GCS（每机直连）**：每架伴飞通过 `GcsIpLink` **主动发送** 本机飞控 MAVLink 至 `10.8.0.100:14550`；下行依赖 GCS 侧按 `sysid` 维护的「会话表」回包到对应伴飞 IP（详见 §4.4）。
5. **图传**：GStreamer 进程独立 systemd 服务，端口 `5000+sysid`，**不进入** `SwarmController` 内部队列。
6. **降级**（v1.2 新增）：`5GHealthMonitor` 周期性采样 5G 接口状态与到 `10.8.0.100` 的 RTT/丢包；超阈值（连续 3s RTT > 1s 或 5s 完全无回包）触发：
   - 立即在 GCS 上发 `STATUSTEXT` 告警 + `RC_CHANNELS_OVERRIDE` 透传保护（可选）。
   - 控制环切「保位悬停 / RTL」（按编队策略可配置）。
   - 若启用 `--p900-uart9-fallback`，把 `air_link_` 与 `IGcsLink` 切回 P900 路径（仅低速兜底，禁用图传）。

**与 v1.1 差异**：

- 默认明确 **5G + VPN**，P900 标 deprecated 不在默认启动参数里。
- 新增 `IGcsLink` / `GcsIpLink` 接口与会话模型（§4.4、§5.2）。
- 新增 `5GHealthMonitor` 与降级路径（§3.1 健康层 / §11）。

---

## 4. 网络详细设计

### 4.1 地址与端口规划（每机直连 MP，示例）

| 角色 | VPN IP | 用途 | 端口（示例） |
|------|--------|------|----------------|
| 长机伴飞 | 10.8.0.1 | 机间 UDP 监听 | 19870 |
| 僚机 i | 10.8.0.(1+i) | 机间 → 长机 | 目的 10.8.0.1:19870 |
| 地面站 PC | 10.8.0.100 | **MP 监听** GCS MAVLink（全机发往此地址） | **14550 UDP**（或 TCP 服务端口） |
| 长机伴飞 | 10.8.0.1 | GCS：向 `10.8.0.100:14550` **主动发** 本机飞控 MAVLink | 源端口任意 |
| 僚机伴飞 | 10.8.0.2… | 同上，各自 sysid | 源端口任意 |
| 各机 | 各 IP | 图传 RTP/UDP | **5000+sysid** 或与 MP 解耦的独立播放器端口 |

**说明**：

- **推荐**：地面 **单一 UDP 监听口**（如 14550），各机伴飞向 `10.8.0.100:14550` 发送 MAVLink；报文内 **`sysid` 区分飞机**。定制 MP 在 **一个套接字** 上解析多源（或内核已把多源归一到同一 socket 的模型下按包处理）。  
- **备选**：每机不同目标端口 `14550+sysid`，MP 内开多个 `UdpClient`/`TcpListener`，由统一调度器汇总到「多机状态机」。

防火墙：允许 **VPN 网段 → 地面站 14550/UDP**（及图传端口段）。

### 4.2 路由策略

- **默认路由**：5G 接口上网；若同时存在 Wi‑Fi/以太网，用 **metric** 或 **policy routing** 保证 **VPN 对端走 5G**（避免 VPN 隧道本身走错接口）。  
- **飞控串口**：仍走 `local_fc.serial_dev`（如 `/dev/ttyS8`），与 5G 无关。

### 4.3 NAT / 无公网场景

若无法建立机到机直连 VPN（例如仅运营商私网）：

- **方案 A**：各机 MAVLink **出站至云主机 mavlink-router**，地面 MP 只连云上一端口（与「每机直连 MP」等价于「每机直连云 + MP 连云」）。  
- **方案 B**：**MQTT/自定义隧道** 仅作信令，数据面仍尽量 UDP（列为远期）。

一期优先 **全员 VPN 可达地面站**；无法直达时再采用 **方案 A**。

### 4.4 NAT 保活与会话表（v1.2 新增）

**问题**：MP 单端口（14550 UDP）接收多机 MAVLink，回包时如何把指令路由到正确伴飞 IP？

**模型**：

1. **伴飞侧（`GcsIpLink`）**：
   - 启动后 **主动周期性向 GCS 发送 HEARTBEAT**（≥1 Hz），即使没有遥测可发，也维持 NAT 会话与「最近活跃时间戳」。
   - 维护本机连出 socket，记录 `(local_sysid, gcs_endpoint)`；从同一 socket 接收下行 MAVLink，避免被运营商 NAT 关闭通道。

2. **地面侧（定制 MP）**：

   ```
   sessions: dict[sysid, (remote_ip, remote_port, last_seen_ms, mav_status)]
   
   on_recv(packet, src_ip, src_port):
       msg = parse_mavlink(packet)
       sessions[msg.sysid] = (src_ip, src_port, now(), ...)
       dispatch_to_uav_state(msg)
   
   on_send(target_sysid, msg):
       (ip, port, _, _) = sessions[target_sysid]
       socket.sendto(encode(msg), (ip, port))
   ```

   要点：
   - 单 socket、单端口；按 **包内 `sysid`** 区分多源、按 **会话表** 路由下行。
   - 会话超时（建议 10 s 无收）→ 标记 `OFFLINE`，UI 灰显该机；不立即清除会话以容忍短暂抖动。
   - 同 sysid 多源（理论上不应出现）：按 `last_seen_ms` 取新；记录冲突日志便于排错。

3. **DNS / 漂移容忍**：伴飞使用主机名 `gcs.swarm.local`（写入 `/etc/hosts` 或 VPN DNS）；MP 主机 IP 变化时仅需更新一处。

**故障处理**：

- 5 s 未收到伴飞 HEARTBEAT → MP 自动重发 `REQUEST_DATA_STREAM` / 在 UI 标红。
- 伴飞 `GcsIpLink` 检测到自身连续发送失败（如 `EHOSTUNREACH`）→ 触发 `5GHealthMonitor.on_send_fail()`。

---

## 5. 数传实现要点

### 5.1 机间（群控）

1. 全节点：`--no-gcs-serial` 或不传 `--p900-uart9`。  
2. 长机：`--link-port 19870`，不设 `--leader-ip`（或代码侧仅 follower 使用）。  
3. 僚机：`--leader-ip 10.8.0.1 --link-port 19870`。  
4. `link_listen_ip` 默认 `0.0.0.0` 即可绑定全接口（含 VPN）。

与 `src/swarm/swarm_controller.cpp` 中 **`air_link_` 非 P900 时使用 UDP** 的分支一致。

### 5.2 地面站（GCS）：每机直连 Mission Planner — `IGcsLink` 抽象（v1.2）

**数据路径**：

- 每架 **伴飞 ↔ 飞控**：经串口 `/dev/ttyS8`（与现网一致，不变）。
- 每架 **伴飞 ↔ 地面 MP**：经 5G + VPN，**伴飞主动维护** 至 `gcs_host:gcs_port` 的 UDP（或 TCP）会话；**长机不再合成/转发僚机遥测**。

**伴飞侧模块化（建议工程实现）**：

1. **抽象接口**（建议放 `include/comm/gcs_link.h`）：

   ```cpp
   class IGcsLink {
   public:
       virtual ~IGcsLink() = default;
       virtual bool open()  = 0;
       virtual void close() = 0;
       virtual int  write_mavlink(const uint8_t *wire, uint16_t len) = 0;
       virtual bool read_byte(uint8_t *out) = 0;     // 单字节供 mavlink_parse_char
       virtual bool is_open() const = 0;
       virtual void on_tick(uint64_t now_ms) {}      // 心跳/保活/统计
   };
   ```

2. **三个具体实现**：

   | 类 | 角色 | 备注 |
   |------|------|------|
   | `GcsSerialLink` | 现状回退 | 直接包 `gcs_serial_`，与现行串口 GCS 等价 |
   | `GcsP900Link`   | deprecated 回退 | 包 `link_p900_.write_mavlink_to_gcs / pop_gcs_byte` |
   | **`GcsIpLink`** | **5G 主路径** | UDP（默认）/TCP；内部维持 NAT 保活、会话计数、RTT 统计 |

3. **`SwarmController` 改造**：
   - 新增成员 `std::unique_ptr<IGcsLink> gcs_link_;` 替代直接持有 `SerialPort gcs_serial_`。
   - `enqueue_gcs_bytes / flush_gcs_tx_queue` 分支统一调用 `gcs_link_->write_mavlink(...)`；删除原 `cfg_.p900_uart9_mode` 与 `gcs_serial_` 的 if/else（保留为 `GcsP900Link / GcsSerialLink` 内部）。
   - `poll_gcs_serial()` 重命名为 `poll_gcs_link()`，从 `gcs_link_->read_byte()` 取字节，喂 `mavlink_parse_char(MAVLINK_COMM_2, ...)`。

4. **配置项（CLI 与 `SwarmConfig` 同步新增）**：

   ```
   --gcs-link <ip|serial|p900>      # 默认 ip（5G 主路径）
   --gcs-udp-target <ip:port>       # 默认 10.8.0.100:14550
   --gcs-udp-bind   <ip:port>       # 默认 0.0.0.0:0（系统选源端口）
   --gcs-udp-tx-kbps <n>            # 默认 384，替代原 baud×0.7 估算
   --gcs-heartbeat-hz <n>           # 默认 1，NAT 保活与活性
   --no-follower-synthetic-telem-gcs  # 默认在 ip/p900-leader 之外都为 true
   --p900-uart9-fallback            # 仅在主链断开后切到 P900（可选）
   ```

5. **关闭僚机合成遥测**：当 `--gcs-link=ip` 时，**默认设置** `follower_synthetic_telem_to_gcs=false`，跳过 `inject_follower_telemetry_to_gcs()`。仅 `--gcs-link=p900` 长机模式（兜底）保留 v1.1 行为。

6. **指令下行路由**：依赖地面 MP 维护的会话表（§4.4），伴飞侧 **无须** 维护「我能从哪个端点接到 MP」，只需 **把所有收到的下行字节** 全部送入飞控（与现 `gcs_serial_` 相同语义）。

**与 P900 解耦**：默认不再使用 `tools/p900_gcs_bridge.py`；GCS 链路上仅为 **标准 MAVLink over IP**。`P900FramedLink` 留在 `comm/` 下作为 `GcsP900Link / SwarmAir P900` 回退实现，不在主路径加载。

### 5.3 Mission Planner 侧程序改造要点

Mission Planner 开源（C#）。在「每机直连」架构下，建议将改造范围收敛为可交付模块：

| 模块 | 内容 |
|------|------|
| **多路接入** | 单端口 UDP `ReceiveFrom` 多源，或多端口监听；统一送入 MAVLink 解析器（`MAVLinkInterface` 同类管线）。 |
| **按机维护** | 以 `(sysid, compid)` 为键维护 **多架** `MAVState` / 当前选中飞机；UI 飞机列表与地图图标与之一致。 |
| **去重与一致性** | 若短时存在双路径，按 **优先真实 FC 源** 或时间戳丢弃重复 `sysid` 包。 |
| **指令路由** | 用户在某架 UI 上操作（改模式、任务、参数）时，发送的 MAVLink **target_system** 为该架 sysid，且 **从对应 UDP 会话发出** 至该机伴飞 IP。 |
| **群控扩展（可选）** | 群状态面板、编队指令聚合显示；与伴飞 `COMMAND_LONG` 代理逻辑对齐（参见仓库内 MP 群控相关文档）。 |

**不改 MP 的过渡方案**：地面运行 **mavlink-router**，各机连 router，router 再 **单 TCP** 接 MP——可快速验证网络，但与「直接改 MP 接收多机」相比，运维多一跳。

### 5.4 令牌桶与速率（v1.2 必改）

现有 `gcs_tx_token_rate_bps_` 在 P900 模式下按 **波特率 × 0.7** 估算（`swarm_controller.cpp:282-294`）。**切到 5G 后必须改造**：

1. **配置驱动**：新增 `cfg_.gcs_tx_max_kbps`（CLI `--gcs-udp-tx-kbps`，默认 **384 kbps**）。
   ```cpp
   // init() 中根据 link 类型选择速率
   uint32_t budget_bps = (cfg_.gcs_link == GcsLinkType::IP)
       ? cfg_.gcs_tx_max_kbps * 1000u
       : (gcs_baud / 10) * 7 / 10;   // 串口/P900 沿用旧逻辑
   gcs_tx_token_rate_bps_ = std::max<uint32_t>(128, budget_bps);
   ```

2. **运行时自适应**（可选，二期）：根据 `5GHealthMonitor` 的 RTT/丢包样本：
   - `loss > 5%` 或 `rtt_p95 > 500ms`：把 `gcs_tx_token_rate_bps_` 暂降到 50%；
   - 持续 30 s 健康 → 缓升回上限；
   - 任何下调都要通过 `STATUSTEXT` 上报 GCS。

3. **辅以内核层 tc**（推荐同时启用）：见 §6.5 模板，对 14550/UDP 单独打 DSCP，蜂窝出口侧做 `ceil` 保护。

---

## 6. 图传实现要点

### 6.1 技术选型

| 方案 | 优点 | 缺点 |
|------|------|------|
| RTP + UDP + H.264 | 延迟低、实现简单 | 需处理丢包花屏 |
| RTSP | 生态好、可 VLC 直接看 | 略增握手与缓冲 |
| WebRTC | NAT 穿透强 | 集成与调试复杂 |

**一期建议**：**GStreamer RTP/UDP** 或 **RTSP Server（机端轻量）**。

### 6.2 机端示例（GStreamer，需按硬件编码器调整）

```bash
# 示例：USB 摄像头 + x264 软编（生产环境优先硬件编码）
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! \
  rtph264pay ! udpsink host=10.8.0.100 port=5002
```

端口建议 **`5000 + sysid`**，避免多机图传冲突。地面端：`udpsrc` + `rtph264depay` + `avdec_h264` + `autovideosink` 或写入录制管道。

### 6.3 带宽与 QoS

- **每机直连 MP 时**：地面入口总流量 ≈ **Σ(各机 MAVLink 上行) + Σ(各机图传)**，需在 **地面带宽** 与 **各机 5G 上行** 两侧分别做容量规划。  
- 图传 **CBR 或 capped VBR**，单机码率 **低于该机 5G 实测上行 × 0.6**，为 MAVLink 留余量。  
- 使用 **`tc HTB`（各机伴飞出口）**：父类总带宽限制在实测值 90%；子类 1 为 GCS MAVLink，子类 2 为图传 UDP（`ceil` 固定）。

### 6.4 与飞控相机关系

- **独立 USB/CSI 相机**：图传管道与飞控无关，最简单。  
- **飞控转发图像**：需 MAVLink 相机消息或 companion 从飞控取流，复杂度高，建议二期。

### 6.5 tc HTB + DSCP 模板（v1.2 新增）

伴飞主机（5G 出口接口假设为 `wwan0`，用 VPN 时改为 `wg0`）：

```bash
#!/usr/bin/env bash
# /usr/local/sbin/swarm-qos.sh  —— 系统启动后 systemd 调起
IFACE="${1:-wwan0}"
LINK_KBIT="${2:-4000}"          # 该机 5G 上行实测 × 0.85
TELEM_KBIT="${3:-512}"          # MAVLink 高优先级
VIDEO_KBIT="${4:-2500}"         # 图传次优先级
GCS_PORT="${5:-14550}"
RTP_PORT="${6:-5000}"

tc qdisc del dev $IFACE root 2>/dev/null
tc qdisc add dev $IFACE root handle 1: htb default 30 r2q 1
tc class add dev $IFACE parent 1: classid 1:1  htb rate ${LINK_KBIT}kbit ceil ${LINK_KBIT}kbit
tc class add dev $IFACE parent 1:1 classid 1:10 htb rate ${TELEM_KBIT}kbit ceil ${LINK_KBIT}kbit prio 0
tc class add dev $IFACE parent 1:1 classid 1:20 htb rate ${VIDEO_KBIT}kbit ceil ${LINK_KBIT}kbit prio 1
tc class add dev $IFACE parent 1:1 classid 1:30 htb rate 64kbit            ceil ${LINK_KBIT}kbit prio 7

tc qdisc add dev $IFACE parent 1:10 handle 10: fq_codel target 5ms
tc qdisc add dev $IFACE parent 1:20 handle 20: fq_codel target 20ms
tc qdisc add dev $IFACE parent 1:30 handle 30: fq_codel

# u32 按目的端口/DSCP 标记落 class
tc filter add dev $IFACE parent 1: protocol ip prio 1 u32 \
   match ip dport $GCS_PORT 0xffff flowid 1:10
tc filter add dev $IFACE parent 1: protocol ip prio 1 u32 \
   match ip sport $GCS_PORT 0xffff flowid 1:10
tc filter add dev $IFACE parent 1: protocol ip prio 2 u32 \
   match ip dport $RTP_PORT  0xfff0 flowid 1:20
```

伴飞应用层可同时给 GCS UDP socket 设 `IP_TOS = 0x88` (DSCP AF41)，便于运营商承载侧识别（视专网策略而定）。

地面侧 `wg0` 入向若需限速对各机做隔离，可用 **IFB + ingress** 镜像后做与上同结构的 HTB。

---

## 7. 5G 模组与系统侧集成

### 7.1 驱动与接口形态

常见形态：

- **Qualcomm / 移远等**：`qmi_wwan` + ModemManager，或厂商 **GobiNet**。  
- **RNDIS**：出现 `usb0`，DHCP 获取地址。  
- **CDC-NCM**：类似以太网接口。

验收：**`ip link` 可见接口、`ping 8.8.8.8` 或专网 APN 网关成功**。

### 7.2 拨号与保活

- 使用 **ModemManager** 或厂商脚本；**systemd** 单元 `After=network-online.target`。  
- **断线重连**：MM 自带；应用层 MAVLink **发送失败重试**与已有接收超时配合。  
- **DNS**：VPN 内建议使用 **私网 DNS 或 hosts** 固定长机名。

### 7.3 天线与功耗

外场文档中单独记录：**RSSI/RSRP 与图传码率、数传频率** 的对应表，便于现场快速降载。

### 7.4 systemd 单元模板（v1.2 新增）

伴飞主机三件套：拨号 → VPN → swarm。

```ini
# /etc/systemd/system/swarm-modem.service
[Unit]
Description=5G modem dial via ModemManager
After=network.target
Before=wg-quick@wg0.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/usr/bin/mmcli -m any --enable
ExecStart=/usr/bin/mmcli -m any --simple-connect="apn=cmnet,ip-type=ipv4"
ExecStop=/usr/bin/mmcli -m any --simple-disconnect

[Install]
WantedBy=multi-user.target
```

```ini
# /etc/systemd/system/swarm-qos.service
[Unit]
Description=Apply HTB QoS for swarm uplink
After=swarm-modem.service wg-quick@wg0.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/swarm-qos.sh wg0 4000 512 2500
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

```ini
# /etc/systemd/system/swarm-controller@.service   （%i = sysid）
[Unit]
Description=UAV Swarm Controller (sysid=%i)
After=swarm-qos.service
Requires=wg-quick@wg0.service

[Service]
Type=simple
Environment=GCS_HOST=10.8.0.100 GCS_PORT=14550
ExecStart=/opt/swarm/bin/swarm_controller %i %I_ROLE \
          --link-port 19870 --leader-ip 10.8.0.1 --hz 10 \
          --gcs-link ip --gcs-udp-target ${GCS_HOST}:${GCS_PORT} \
          --gcs-udp-tx-kbps 384 --no-follower-synthetic-telem-gcs
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

启用：`systemctl enable --now swarm-modem wg-quick@wg0 swarm-qos swarm-controller@2`（僚机 sysid=2 示例；`%I_ROLE` 由 EnvironmentFile 区分长/僚机）。

---

## 8. 软件改造清单（v1.2 重排：5G 优先 / P900 deprecated）

### 8.1 必选 — 伴飞 `UAV_Swarm_APM` 代码改造

- [ ] **新增** `include/comm/gcs_link.h` 抽象接口 `IGcsLink`。
- [ ] **新增** `src/comm/gcs_ip_link.{h,cpp}`：UDP 实现（sendto/recvfrom + bind + RTT 统计 + NAT 心跳）。
- [ ] **重构** `SwarmController`：
  - 把 `SerialPort gcs_serial_` 与 `link_p900_.write_mavlink_to_gcs` 路径统一到 `std::unique_ptr<IGcsLink> gcs_link_`；
  - 删除 `flush_gcs_tx_queue / poll_gcs_serial` 中 `cfg_.p900_uart9_mode && role==LEADER` 的分支判断，改由 `IGcsLink` 的具体实现承担；
  - **默认禁用** `inject_follower_telemetry_to_gcs()`，仅当 `cfg_.gcs_link_type == LEGACY_P900_LEADER` 时保留。
- [ ] **新增 CLI** 与 `SwarmConfig`：`--gcs-link`、`--gcs-udp-target`、`--gcs-udp-bind`、`--gcs-udp-tx-kbps`、`--gcs-heartbeat-hz`、`--no-follower-synthetic-telem-gcs`、`--p900-uart9-fallback`。
- [ ] **修改令牌桶**：`gcs_tx_token_rate_bps_` 在 IP 模式下按 `gcs_tx_max_kbps` 配置，移除波特率耦合。
- [ ] **新增** `src/comm/health_monitor.{h,cpp}`：周期采样 RSSI/RSRP（`mmcli`）、到 `gcs_host` 的 RTT/丢包；提供 `is_degraded()` 接口；触发 `STATUSTEXT` 与控制环降级。
- [ ] **机间** UDP：保持 `InterUavLink`，确认 `bind 0.0.0.0` 能在 VPN 接口接收；`--leader-ip` 使用 VPN 私网地址。
- [ ] 图传 GStreamer 独立 systemd 服务，端口 `5000+sysid`。

### 8.2 必选 — 系统/运维

- [ ] 部署 VPN（WireGuard 优先），固定 IP 表，确认 `ping` 通 `GCS_HOST`。
- [ ] 配置 `swarm-modem / wg-quick@wg0 / swarm-qos / swarm-controller@<sysid>` systemd 链路。
- [ ] 运行 `swarm-qos.sh` 给 5G 出口（或 wg0）打 HTB；MAVLink 14550 / 19870 高优先级、RTP 5000+sysid 次优先级。
- [ ] 防火墙：仅开放 VPN 网段；公网入口默认拒绝。

### 8.3 必选 — 地面 Mission Planner 定制

- [ ] 单端口 UDP 14550 多源接入层；按 `(sysid, compid)` 维护 **多架 `MAVState`**。
- [ ] 维护「会话表」`sysid → (remote_ip, remote_port, last_seen_ms)`；下行按目标 sysid 路由回包。
- [ ] UI：多机列表 / 当前飞机切换 / 参数·任务·模式互不串机。
- [ ] 心跳 5 s 无收 → 标灰；同 sysid 多源冲突告警。

### 8.4 可选 — 伴飞增强（二期）

- [ ] `5GHealthMonitor` 自适应回压：根据 RTT/丢包动态调 `gcs_tx_token_rate_bps_`。
- [ ] `CloudRelayLink`：无 VPN 直达时走云上中继。
- [ ] MAVLink2 signing；GCS 链路 TLS（DTLS 或 stunnel）。
- [ ] 5G 模组指标周期性 `STATUSTEXT` 透传 GCS。

### 8.5 可选 — 不改 MP 源码的过渡方案

- [ ] 地面运行 **mavlink-router**：N 个 UDP server endpoint（每机一个）+ 1 个 TCP/UDP 输出至 MP。
- [ ] 优势：免改 MP；劣势：会话路由能力弱、多机 sysid 冲突需人为规避。

### 8.6 标记 deprecated（保留代码、默认禁用）

- `P900FramedLink`：**保留**，通过 `--p900-uart9-fallback` 显式启用为备份。
- `tools/p900_gcs_bridge.py`：**保留**，仅 P900 fallback 路径使用。
- `inject_follower_telemetry_to_gcs()`：**保留**，仅 P900 长机 fallback 时激活。

---

## 9. 分阶段实施计划（v1.2 补 KPI 目标值）

| 阶段 | 内容 | KPI 目标 / 产出 |
|------|------|------|
| **P0** | 单机 5G 拨号 + VPN + 地面 ping 通 | RTT P95 ≤ 80 ms（运营商专网 ≤ 50 ms）；30 min 不掉线；网络基线报告 |
| **P1** | `IGcsLink/GcsIpLink` 实现；单机 GCS over UDP 替换 P900 GCS 路径 | MP 单机 HEARTBEAT 1 Hz 稳定 ≥ 60 min；参数列表读取成功率 ≥ 99%；任务上传 100 wp 成功；切回 P900 fallback 验证 |
| **P2** | UDP 机间 + 长机 / 双僚机群控（无图传） | 群控指令端到端时延 P95 ≤ 200 ms；STATE_REPORT 丢包 ≤ 2% |
| **P3** | **每机** MAVLink 经 VPN 至地面；定制 MP 多机展示；关闭合成遥测注入 | 多机 MP 列表稳定、**无 sysid 冲突**；任意指定机改参成功率 ≥ 99%；切机延迟 ≤ 200 ms |
| **P3b** | 定制 MP：单端口多源接入 + 会话表 + 群控 UI | 可发布 MP 分支或安装包；3 机并发 1 h 不串 |
| **P4** | 图传 + 多机并发 MAVLink + tc HTB | 单机 1080p@30fps 2 Mbps 稳定，MAVLink 时延 P95 不劣化超过 30%；地面入口聚合带宽报告 |
| **P5** | `5GHealthMonitor` + 自动降级 | 主动断 5G 可在 5 s 内切换 LOITER；启用 P900 fallback 时控制不中断 |
| **P6** | 外场昼夜 / 弱信号 / 基站切换 / 断线重连 | 单机 4 h 飞行无干预；自动恢复 ≥ 95% 抖动事件；最终验收签字数据 |

每阶段结束填写 **时延 P95 / 丢包率 / 任务成功率 / 链路恢复时长** 四类核心指标。

---

## 10. 测试与验收

### 10.1 数传

- **机间**：指令经长机至僚机飞控的时延 P95（群控路径不变）。  
- **GCS 每机直连**：各机遥测至 MP 的时延、丢包；MP 侧 **多机切换**、参数列表是否与真机一致。  
- **无重复 sysid**：确认关闭 `inject_follower_telemetry_to_gcs` 后，MP 不再出现同一飞机两路矛盾数据。  
- **并发图传** 时是否触发 GUID 超时、任务 ACK 失败。

### 10.2 图传

- 主观卡顿、花屏频率；客观 **FPS、码率、UDP 丢包**。

### 10.3 安全

- VPN 密钥轮换流程；**禁止 MAVLink 裸暴露公网**（仅 VPN 内端口）。

---

## 11. 风险与应对（v1.2 补降级路径）

| 风险 | 触发判据 | 应对 |
|------|----------|------|
| 蜂窝抖动大 | RTT P95 > 500 ms 持续 5 s | 降数据流频率（`SET_MESSAGE_INTERVAL`）；降 `gcs_tx_token_rate_bps_` 50%；外场可降 `ctrl_hz` |
| 短时基站切换断流 | `5GHealthMonitor` 检测发送失败 ≥ 3 次 / 2 s 无收 | 控制环维持上一目标 + 触发 LOITER；3 s 内未恢复发 STATUSTEXT 告警 |
| 长时 5G 失联 | 连续 5 s RTT 无回 | 控制环切 RTL（可配置）；启用 `--p900-uart9-fallback` 时 `air_link_/gcs_link_` 切到 `P900FramedLink`，禁用图传 |
| NAT 无法点对点 | VPN 握手失败 | 上云 mavlink-router 或 WireGuard 中心节点 |
| 图传占满上行 | tc 计数 1:20 ceil 命中 | tc rate 自动收紧 + 应用层降码率；关键模式（起降/RTL）暂停图传 |
| 模块发热降频 | 模组温度告警 | 散热设计；图传分辨率自动降级 |
| 同 sysid 双源 | MP 端检测到「同一 sysid 多 endpoint 同时活跃」 | 优先真实 FC 源（按消息率/抖动判定）；记录冲突日志；提示运维 |
| 合规 | — | 使用已认证频段与功率；空域与无线电法规单独评审 |
| MAVLink 公网暴露 | 入向流量来自非 VPN 网段 | 防火墙拒绝；GCS UDP 端口仅监听 VPN 接口 |

---

## 12. 附录

### 12.1 长机 / 僚机启动参数示例（5G + VPN + 每机 GCS 目标）

以下为 **目标形态**（v1.2）；`--gcs-link / --gcs-udp-target / --gcs-udp-tx-kbps / --no-follower-synthetic-telem-gcs / --p900-uart9-fallback` 需在伴飞程序实现后启用（见 §8.1）。

```bash
# 地面 MP 监听 10.8.0.100:14550；GCS_HOST=10.8.0.100

# 长机 sysid=1
./swarm_controller 1 1 --link-port 19870 --followers 2,3 --hz 10 \
  --gcs-link ip --gcs-udp-target 10.8.0.100:14550 \
  --gcs-udp-tx-kbps 384 --gcs-heartbeat-hz 1 \
  --no-follower-synthetic-telem-gcs

# 僚机 sysid=2
./swarm_controller 2 2 --link-port 19870 --leader-ip 10.8.0.1 --hz 10 \
  --gcs-link ip --gcs-udp-target 10.8.0.100:14550 \
  --gcs-udp-tx-kbps 384 --gcs-heartbeat-hz 1

# 弱信号外场可加 P900 fallback：
#   --p900-uart9-fallback --p900-dev /dev/ttyS9 --p900-baud 57600
```

在 **未实现 UDP GCS** 前，可临时用 **mavlink-router** 将各机飞控串口桥到地面，但正式方案仍以 **伴飞内聚 `GcsIpLink`** 为准。

### 12.2 相关源文件索引（v1.2 增量）

| 文件 | 状态 | 备注 |
|------|------|------|
| `include/comm/inter_uav_link.h` | 不变 | `ISwarmAirLink / LinkPacket / InterUavLink` |
| `include/comm/p900_framed_link.h` | **deprecated** | 保留，仅 fallback 使用 |
| `include/comm/gcs_link.h` | **新增** | `IGcsLink` 抽象 |
| `include/comm/gcs_ip_link.h` | **新增** | `GcsIpLink`：UDP 实现 + 心跳 + 统计 |
| `include/comm/gcs_serial_link.h` | **新增** | `GcsSerialLink`：包旧串口路径 |
| `include/comm/gcs_p900_link.h` | **新增** | `GcsP900Link`：包 P900 fallback |
| `include/comm/health_monitor.h` | **新增** | `5GHealthMonitor` |
| `include/swarm/swarm_controller.h` | **改** | `SwarmConfig` 新字段；`gcs_serial_` → `gcs_link_` |
| `src/main.cpp` | **改** | 新增 CLI；默认 `--gcs-link ip` |
| `src/swarm/swarm_controller.cpp` | **改** | `init/poll_gcs_serial/flush_gcs_tx_queue/inject_follower_telemetry_to_gcs` 分支重排 |
| `tools/p900_gcs_bridge.py` | **deprecated** | 仅 P900 fallback 路径使用 |
| `scripts/swarm-qos.sh` | **新增** | 见 §6.5 |
| `systemd/*.service` | **新增** | 见 §7.4 |

### 12.3 代码改动地图（v1.2 新增）

```
include/
├── comm/
│   ├── gcs_link.h         (NEW)  IGcsLink 抽象
│   ├── gcs_ip_link.h      (NEW)
│   ├── gcs_serial_link.h  (NEW, wraps existing SerialPort)
│   ├── gcs_p900_link.h    (NEW, wraps P900FramedLink)
│   └── health_monitor.h   (NEW)
├── swarm/swarm_controller.h
│       新增字段：
│         enum class GcsLinkType { IP, SERIAL, P900_LEADER, NONE };
│         GcsLinkType gcs_link_type = GcsLinkType::IP;
│         std::string gcs_udp_target_ip;  int gcs_udp_target_port = 14550;
│         std::string gcs_udp_bind_ip = "0.0.0.0"; int gcs_udp_bind_port = 0;
│         uint32_t gcs_tx_max_kbps = 384;
│         int      gcs_heartbeat_hz = 1;
│         bool     follower_synthetic_telem_to_gcs = false;
│         bool     p900_uart9_fallback = false;
│       成员替换：SerialPort gcs_serial_  →  std::unique_ptr<IGcsLink> gcs_link_;
│       新增：std::unique_ptr<HealthMonitor> health_;
src/
├── comm/
│   ├── gcs_ip_link.cpp     (NEW)
│   ├── gcs_serial_link.cpp (NEW)
│   ├── gcs_p900_link.cpp   (NEW)
│   └── health_monitor.cpp  (NEW)
├── swarm/swarm_controller.cpp
│       init():           按 gcs_link_type 工厂构造 gcs_link_；移除波特率耦合的令牌桶
│       poll_gcs_serial → poll_gcs_link
│       flush_gcs_tx_queue: gcs_link_->write_mavlink(...)
│       inject_follower_telemetry_to_gcs: 仅 type==P900_LEADER 时调用
│       receive_loop: 调 health_->tick()；触发降级
└── main.cpp
        新增 CLI：--gcs-link / --gcs-udp-target / --gcs-udp-bind /
                  --gcs-udp-tx-kbps / --gcs-heartbeat-hz /
                  --no-follower-synthetic-telem-gcs / --p900-uart9-fallback
```

### 12.4 文档修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-04-20 | 初稿：与当前仓库实现对齐 |
| v1.1 | 2026-04-20 | 改为 **每机直连 MP**；补充 MP 定制模块与关闭合成遥测注入；更新端口与阶段计划 |
| **v1.2** | **2026-04-27** | **5G 全面替代 P900**：P900 标 deprecated；新增 `IGcsLink/GcsIpLink/HealthMonitor` 模块化设计；新增会话表与 NAT 保活、tc HTB / DSCP / systemd 模板、KPI 目标值、降级路径与代码改动地图 |

---

**结论（v1.2）**：在 **5G + VPN** 前提下，**所有业务面**（机间群控 / GCS 遥测指令 / 图传）默认承载在 5G 模组上；P900 路径被收敛为可选硬退化备份。机间继续走 `InterUavLink`（UDP）+ 长机汇聚；GCS 改为各伴飞 **直连地面 MP**，由新建 `IGcsLink/GcsIpLink` 抽象统一收发，并配合 **会话表** 完成单端口多源接入与 sysid 路由；伴飞默认 **关闭 `inject_follower_telemetry_to_gcs`**；图传独立 GStreamer 进程 + **tc HTB + DSCP**；新增 `5GHealthMonitor` 与降级路径，确保 5G 抖动 / 断流时控制环可控；地面侧需要做 MP 定制（多源接入 + 会话表 + 多机 UI）或暂用 mavlink-router 过渡。
