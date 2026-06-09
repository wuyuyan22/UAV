# UAV_Swarm_APM 项目分析与优化建议（结合当前代码）

## 1. 项目定位

`UAV_Swarm_APM` 是一个面向 ArduPilot 的 C++17 群控原型，当前已实现以下核心闭环能力：

- 伴机与本机飞控的 MAVLink 通信（串口或 UDP/SITL）；
- Leader/Follower 双角色调度；
- 机间链路基于 UDP 承载 MAVLink 标准消息；
- 基础编队计算、Guided 保活、心跳失联安全 RTL。

从代码状态看，它已经不是“纯概念 Demo”，而是“可联调的工程原型”，但距离稳定实飞工程仍有若干关键缺口。

---

## 2. 代码结构（已按当前实现校准）

```text
UAV_Swarm_APM/
├─ CMakeLists.txt
├─ README.md
├─ include/
│  ├─ common/                # 数据结构、数学工具、模式/命令常量
│  ├─ hal/                   # Linux 串口/UDP 抽象（Windows 为桩）
│  ├─ mavlink_adapter/       # 单机 MAVLink 会话 + 管理器
│  ├─ comm/                  # 机间链路（InterUavLink）
│  ├─ swarm/                 # SwarmController + FormationCalculator
│  └─ mavlink/               # 仓内 MAVLink 头（ardupilotmega 方言）
├─ src/
│  ├─ main.cpp
│  ├─ common/
│  ├─ hal/
│  ├─ mavlink_adapter/
│  ├─ comm/
│  └─ swarm/
└─ tools/sitl_test.sh
```

模块关系：

- `main.cpp`：参数解析、`SwarmConfig` 组装、生命周期管理；
- `SwarmController`：双线程调度（控制 + 收包）、角色逻辑、安全策略；
- `MavlinkVehicle/MavlinkManager`：飞控会话与状态管理；
- `InterUavLink`：机间消息收发（MAVLink 标准消息映射）；
- `HAL`：Linux 下实际可用，Windows 当前为占位实现。

---

## 3. 关键数据结构与协议事实

### 3.1 业务数据结构

主要位于 `include/common/types.h`：

- `UnitState`：机体状态快照（经纬高、姿态、速度、模式、解锁、电池、NED）；
- `WayPoint`：航点（经纬高、速度、航向、模式）；
- `FormationParams`：编队参数（间距/高度差/速度边界等）；
- `SwarmConfig`（定义在 `swarm_controller.h`）：角色、频率、链路、本机飞控配置。

### 3.2 机间协议（已更新为标准 MAVLink 承载）

`LinkPacket` 是内部统一表示，不直接裸结构体上网。实际传输映射如下：

- `STATE_REPORT`：编码为 `GLOBAL_POSITION_INT`（follower -> leader）；
- `FOLLOWER_CMD`：编码为 `SET_POSITION_TARGET_GLOBAL_INT`（leader -> follower）。

这意味着旧版“结构体直拷贝跨平台风险”在当前实现里已经明显降低，后续风险重点转向“字段语义一致性与路由机制”。

---

## 4. 运行流程（与源码一致）

## 4.1 启动流程

1. `main()` 读取 `swarm_node <sysid> <role> [options]`；
2. 组装 `SwarmConfig`（默认 `/dev/ttyS8`、`57600`、`ctrl_hz=10`）；
3. `SwarmController::init()`：
   - 限制 `ctrl_hz >= 2`；
   - 连接本机飞控；
   - 注册 GCS 命令处理（`MAV_CMD_USER_1..5`）；
   - 初始化机间 UDP 链路；
4. `start()` 启动控制线程与收包线程；
5. 接收信号后 `stop()` 安全退出。

## 4.2 线程模型

- 控制线程 `control_loop()`：
  - 周期心跳；
  - 心跳健康检测（超时触发 RTL）；
  - 按角色执行 leader/follower tick；
  - Guided 保活；
- 收包线程 `receive_loop()`：
  - 持续 `mav_mgr_.read_all()`；
  - follower 接收并执行 leader 下发目标；
  - leader 接收 follower 状态并写入缓存。

## 4.3 控制命令映射（GCS -> 节点）

基于 `COMMAND_LONG.command = MAV_CMD_USER_1..5`：

- `USER_1`：开始编队；
- `USER_2`：停止编队（切 Loiter）；
- `USER_3`：RTL；
- `USER_4`：LAND；
- `USER_5`：解锁 + 起飞（`param1` 为高度）。

---

## 5. 构建与运行

## 5.1 构建

```bash
mkdir build
cd build
cmake ..
make -j4
```

生成：`swarm_node`  
CMake 使用 C++17，并链接 `pthread`；MAVLink 头已内置于仓库。

## 5.2 常用运行参数

```bash
swarm_node <sysid> <role> [options]
```

- `role`: `1=LEADER`, `2=FOLLOWER`
- 常用 options:
  - `--serial <dev>`
  - `--baud <rate>`
  - `--udp <ip:port>`
  - `--type <copter|plane>`
  - `--link-port <p>`
  - `--leader-ip <ip>`
  - `--hz <n>`
  - `--followers <a,b,c>`
  - `--sim`

---

## 6. 当前实现亮点

- 分层边界清晰，控制/通信/MAVLink/HAL 解耦合理；
- 针对 Guided 模式加入频率下限与保活机制（工程上很关键）；
- 安全动作明确（失联 RTL、停止编队 Loiter）；
- GCS 命令入口已经接到控制器，具备远程触发能力；
- 机间通信改为 MAVLink 标准消息，便于后续工具链兼容。

---

## 7. 当前问题与风险（按优先级）

## 7.1 Leader 下发链路目标未配置（高优先级，功能性）

在 `SwarmController::init()` 中，Follower 分支调用了 `link_.set_target(...)`，但 Leader 分支没有设置任何目标地址。  
`run_leader_tick()` 虽然持续 `link_.send_packet(FOLLOWER_CMD)`，但默认 `target_ip_` 为空，导致发送路径不可达或失败，可能使 follower 收不到编队指令。

> 影响：Leader->Follower 关键控制链路存在“看起来在发，实际不达”的风险。

## 7.2 任务闭环仍依赖外部触发（高优先级，易用性）

当前默认未加载航点，`wp_num_ == 0` 时 leader 逻辑直接返回；若未通过外部接口上传航点并启动编队，系统只会维持“在线但不执行任务”状态。

## 7.3 裸指针暴露与跨线程访问窗口（中高优先级，稳定性）

`MavlinkManager::get_vehicle()` 返回裸指针，调用方在锁外长期使用；多线程下存在生命周期与并发访问窗口。短期可用，长期工程化建议收敛为快照或受控回调。

## 7.4 Plane/Copter 安全保活判定不一致（中优先级，行为一致性）

`send_guided_keepalive()` 中使用了 `CopterMode::GUIDED` 判断。对于固定翼分支，建议按 `vehicle_type` 分别判定 `PlaneMode::GUIDED`，避免模式识别偏差。

## 7.5 可观测性不足（中优先级，调试效率）

缺少关键指标输出：链路发送成功率、follower 最近包龄、编队误差、命令执行时延。联调时问题定位成本偏高。

## 7.6 平台能力不对称（中优先级，部署边界）

`serial_port` / `udp_socket` 的 Windows 路径为桩实现，当前完整能力依赖 Linux 环境（鲁班猫/WSL/Ubuntu）。

## 7.7 P900 全链路带宽与时延约束（中高优先级，架构性）

若“地面站-长机-僚机”全部采用 P900，链路会面临低带宽与时延抖动约束。当前设计若按高频全量状态上报/目标下发运行，存在控制通道被遥测挤占的风险。

> 影响：命令到达延迟增大、批量控制一致性下降，极端情况下出现“有连接但不可控”。

## 7.8 缺少链路退化降级状态机（中高优先级，安全性）

当前逻辑已有心跳失联 RTL，但尚未形成“链路退化 -> 降级控制 -> 安全退出”的分级状态机，难以应对 P900 场景下的连续弱链路状态。

---

## 8. 建议优化路线（结合现状可落地）

## 阶段 1（立即修复，1-2 天）

- 修复 Leader 目标路由：
  - 方案 A：支持 `sysid -> ip:port` 路由表并按 follower 定向发送；
  - 方案 B：先支持单目标 `--follower-ip` 快速打通链路；
- 增加发送返回值检查与失败日志；
- 启动时打印链路配置摘要（listen/target/role）。
- 增加链路配置档位（例如 `--link-profile p900`），统一加载默认频率/重试/超时参数。

## 阶段 2（稳定性，2-4 天）

- 将车辆访问从裸指针改为受控接口（快照、lambda 访问器或 `shared_ptr`）；
- 给 `UnitState` 读取提供一致性快照接口；
- Plane/Copter 分开做 Guided 保活判定。
- 增加消息优先级队列（控制命令高优先，遥测低优先），避免窄带链路拥塞。
- 完成命令幂等与 ACK 去重逻辑（`seq` + 时间窗）。

## 阶段 3（任务闭环，3-5 天）

- 增加 `--wp-file`（JSON/CSV）加载航点；
- 增加 `--auto-start`，实现“上电后自动进入任务”；
- 命令回执：对 GCS 指令输出 ACK/NACK 与原因码。
- 引入链路退化状态机：`NORMAL -> DEGRADED -> SAFE_RTL`，并定义触发阈值（丢包率/ACK 超时/心跳年龄）。

## 阶段 4（工程化，持续）

- 补全单测：编队计算、参数解析、协议映射；
- 补全 SITL 回归脚本（copter/plane、多 follower、丢包场景）；
- 输出 Prometheus/日志指标，支持长期飞行复盘。
- 增加 P900 专项压测：限带宽、注入时延/抖动/丢包，验证批量命令一致性与降级可靠性。

## 8.1 P900 运行参数建议（首版基线）

建议在“全 P900 或 P900 主链路”场景使用以下初始参数：

- GCS -> Leader 命令频率：`1~2 Hz`；
- Leader -> Follower 目标下发：`2~5 Hz`（按链路质量动态调节）；
- Follower -> Leader 状态回传：`1~2 Hz` + 异常事件即时上报；
- 命令超时：`500~1200 ms` 分级；
- 重试次数：`2~3` 次（安全命令可更高）；
- 遥测限流：保证控制命令始终有保底带宽。

---

## 9. 适用场景与边界

### 适用场景

- 教学与算法验证；
- 小规模 Leader-Follower 编队实验；
- 作为 ArduPilot 伴机群控的二次开发底座。

### 当前边界

- 未覆盖复杂避障与大规模编队；
- 未形成完备任务管理平台接口；
- 工业级高可靠部署仍需进一步工程化。

---

## 10. 结论

当前 `UAV_Swarm_APM` 的架构和基础控制能力是扎实的，特别是 Guided 安全保活与标准 MAVLink 化方向正确。  
最优先应先修复“Leader 下发目标路由”与“任务闭环入口”两项，否则会持续影响联调效率与实飞可用性。完成这两项后，再推进并发安全与可观测性建设，项目就能较快进入稳定迭代阶段。
