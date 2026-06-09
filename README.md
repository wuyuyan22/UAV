# UAV Swarm Controller for LubanCat0 + ArduPilot

适配鲁班猫0（LubanCat 0, RK356X）机载伴机与 ArduPilot 飞控的多旋翼/固定翼无人机群控系统。

## 一、整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                 鲁班猫0 (Companion Computer)                 │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │               SwarmController (调度核心)               │   │
│  │  ┌─────────────┐ ┌──────────────┐ ┌───────────────┐ │   │
│  │  │ Formation    │ │ MavlinkMgr   │ │ InterUavLink  │ │   │
│  │  │ Calculator   │ │  ┌─────────┐ │ │  (UDP)        │ │   │
│  │  │ (编队算法)   │ │  │Vehicle  │ │ └──────┬────────┘ │   │
│  │  └─────────────┘ │  └─────────┘ │        │          │   │
│  │                   └──────┬───────┘        │          │   │
│  └──────────────────────────┼────────────────┼──────────┘   │
│                              │ MAVLink v2     │ UDP          │
│  ┌──────────────────────────┴───────┐        │              │
│  │  HAL: SerialPort / UdpSocket     │        │              │
│  └──────────────┬───────────────────┘        │              │
└─────────────────┼────────────────────────────┼──────────────┘
         UART8    │                            │ ETH0/WiFi
    ┌─────────────┴──────────┐          ┌──────┴───────┐
    │  ArduPilot 飞控         │          │ 其他鲁班猫节点  │
    │  (Pixhawk / APM)       │          │ (Leader/Follower)│
    │  运行 ArduCopter/Plane  │          └──────────────┘
    └────────────┬───────────┘
                 │ PWM/DShot
    ┌────────────┴───────────┐
    │  电机 / 电调 / 舵机      │
    └────────────────────────┘
```

## 二、硬件连接

### 2.1 鲁班猫0 ↔ 飞控接线

| 鲁班猫0 侧 | 飞控侧 | 说明 |
|------------|--------|------|
| **UART8 TX** (40pin 第8脚) | TELEM1 RX | MAVLink 通信 |
| **UART8 RX** (40pin 第10脚) | TELEM1 TX | MAVLink 通信 |
| **GND** (40pin 第6/9/14脚任一) | TELEM1 GND | 共地 |

> 鲁班猫0 UART 电平 3.3V TTL，与 Pixhawk TELEM 端口兼容，可直连。
> 设备节点: `/dev/ttyS8`（需在 `fire-config` 中使能 UART8）。
> 也可用 USB 转串口: `/dev/ttyUSB0` 或 USB 直连: `/dev/ttyACM0`。

### 2.2 飞控参数设置（Mission Planner / QGC）

```
SERIAL1_PROTOCOL = 2        # MAVLink2
SERIAL1_BAUD     = 57       # 57600 (或 115 = 115200)
GUID_TIMEOUT     = 3        # Guided 无指令超时秒数（3秒无指令则悬停）
GUID_OPTIONS     = 0        # 默认; 若用姿态控制: bit3=8 使推力为0~1
```

### 2.3 机间通信

| 接口 | 用途 |
|------|------|
| **ETH0** | 有线组网（推荐, 延迟低） |
| **WiFi** | 无线组网（灵活, 需注意丢包） |
| **USB 4G** | 远距离通信（延迟较高, 备用） |

## 三、模块说明

| 模块 | 目录 | 职责 |
|------|------|------|
| **types / math_utils** | `include/common/` | 数据结构、坐标变换、航迹数学（RK4/Haversine） |
| **SerialPort / UdpSocket** | `include/hal/` | Linux POSIX 串口 + BSD UDP 封装 |
| **MavlinkVehicle** | `include/mavlink_adapter/` | 单机 MAVLink 会话: 心跳、模式、解锁、GUIDED 位置/速度/姿态 |
| **MavlinkManager** | 同上 | 多机管理: 批量收发、状态聚合、心跳监测 |
| **FormationCalculator** | `include/swarm/` | 编队几何: 左翼/右翼目标点、渐变间距、速度约束 |
| **SwarmController** | 同上 | Leader/Follower 状态机、航点导航、安全保活 |
| **InterUavLink** | `include/comm/` | 机间 MAVLink 消息承载（GLOBAL_POSITION_INT / SET_POSITION_TARGET_GLOBAL_INT 映射） |

## 四、控制逻辑

### 4.1 Guided 模式控制（核心）

多旋翼自动控制使用 **GUIDED 模式**，伴机持续发送目标：

| 控制方式 | MAVLink 消息 | 适用场景 |
|----------|-------------|----------|
| **全球位置** | `SET_POSITION_TARGET_GLOBAL_INT` | 编队导航、航点跟踪 |
| **本地NED位置** | `SET_POSITION_TARGET_LOCAL_NED` | 近距编队、精确相对定位 |
| **本地NED速度** | 同上 (type_mask=USE_VELOCITY) | 速度跟随、平滑过渡 |
| **姿态+推力** | `SET_ATTITUDE_TARGET` | 底层姿态控制 (Guided_NoGPS) |

**关键约束**: 必须 **>=2Hz** 持续发送目标，否则飞控在 `GUID_TIMEOUT` 秒后悬停！

### 4.2 典型飞行流程

```
1. 建立 MAVLink 连接 (串口 /dev/ttyS8 @ 57600)
2. 等待飞控心跳 → 确认通信正常
3. 切换 GUIDED 模式 → MAV_CMD_DO_SET_MODE
4. 解锁 → MAV_CMD_COMPONENT_ARM_DISARM (param1=1)
5. 起飞 → MAV_CMD_NAV_TAKEOFF (param7=目标高度)
6. 编队飞行 → 循环发送 SET_POSITION_TARGET_GLOBAL_INT (>=2Hz)
7. 降落 → MAV_CMD_NAV_LAND 或 RTL
```

### 4.3 安全机制

| 保护 | 实现方式 |
|------|----------|
| **Guided 保活** | 即使无新目标，也以 >2Hz 重发当前位置，防止 GUID_TIMEOUT 触发 |
| **心跳丢失 RTL** | 飞控心跳丢失 >5s → 自动触发 RTL + 停止编队 |
| **编队停止 → Loiter** | 停止编队时切 Loiter 模式悬停，而非直接 disarm |
| **GCS 失效保护** | 建议在飞控配置 FS_GCS_ENABLE=1 (断连后 RTL) |

### 4.4 MAVLink 命令对照表（GCS -> 群控节点）
当前群控节点通过 `COMMAND_LONG` 接收来自 GCS 的控制命令；`COMMAND_LONG.command` 取值使用 `MAV_CMD_USER_1..5`。命令会被分发到“目标 sysid 为本机的群控节点（swarm_node 参数 `<sysid>`）”。

| Swarm 业务 | MAV_CMD_USER_# | `COMMAND_LONG.param1` | 作用于 |
|---|---:|---:|---|
| 开始编队 | `MAV_CMD_USER_1` | 忽略 | 触发 `cmd_start_formation()`（Leader/Follower 都会响应，但 Leader 才会真正下发目标） |
| 停止编队 | `MAV_CMD_USER_2` | 忽略 | 触发 `cmd_stop_formation()`（停止下发并切到 Loiter） |
| 返航 RTL | `MAV_CMD_USER_3` | 忽略 | 触发 `cmd_rtl()` |
| 下降着陆 | `MAV_CMD_USER_4` | 忽略 | 触发 `cmd_land()` |
| 解锁 + 起飞 | `MAV_CMD_USER_5` | 起飞高度 `alt`（m） | 触发 `cmd_arm_and_takeoff(alt)` |

参数路由约束（避免误触发）：
- `COMMAND_LONG.target_system` 必须等于 `swarm_node <sysid>` 的 `<sysid>`。
- `COMMAND_LONG.target_component` 建议填 `MAV_COMP_ID_ONBOARD_COMPUTER`（或 `MAV_COMP_ID_ALL`）。

## 五、构建

```bash
# 在鲁班猫0 上原生编译
mkdir build && cd build
cmake .. && make -j4

# 交叉编译 (PC 上, 需 aarch64 工具链)
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/aarch64-toolchain.cmake
make -j$(nproc)
```

## 六、运行

```bash
# ── 多旋翼 Leader (sysid=1, UART8) ──
./swarm_node 1 1 --serial /dev/ttyS8 --baud 57600 --type copter --hz 10

# ── 多旋翼 Leader + MP/P900 命令串口 (UART9) ──
./swarm_node 1 1 --serial /dev/ttyS8 --baud 57600 --type copter \
    --gcs-serial /dev/ttyS9 --gcs-baud 57600 --hz 10

# ── 多旋翼 Follower (sysid=2) ──
./swarm_node 2 2 --serial /dev/ttyS8 --baud 57600 --type copter \
    --leader-ip 192.168.1.1 --hz 10

# ── SITL 测试 (UDP, 无需硬件) ──
./swarm_node 1 1 --udp 127.0.0.1:14550 --type copter --hz 10 --sim

# ── 固定翼 Leader ──
./swarm_node 1 1 --serial /dev/ttyS8 --baud 115200 --type plane --hz 10

# ── 指定 follower 列表 ──
./swarm_node 1 1 --udp 127.0.0.1:14550 --followers 2,3,4 --hz 10
```

## 七、推荐开发步骤

1. **SITL 验证**: 在 PC 上跑 ArduPilot SITL + 本程序 (`--udp`)，确认心跳、模式、起飞、GUIDED 位置下发正常
2. **单机上板**: 鲁班猫0 接飞控 (`--serial /dev/ttyS8 --baud 57600`)，确认通信、模式切换、解锁起飞
3. **编队仿真**: SITL 多实例 + 多个 swarm_node，验证 leader 计算 + follower 跟随
4. **双机实飞**: 两块鲁班猫0 通过网线互联，低高度小范围验证编队
5. **外场放开**: 逐步增大编队间距、速度范围，加入异常恢复测试

## 八、从原有工程迁移说明

| 原 UAV_BZ_L | 新 UAV_Swarm_APM |
|-------------|-------------------|
| CHS40 自定义帧 (0xEB 0x90) | MAVLink v2 标准协议 |
| `send_wp_cmd()` 写串口 | `set_position_target_global()` GUIDED |
| `decode_frame1/2/3()` 手动解析 | MAVLink 自动解码 + 回调 |
| P840 链路 (串口) | UDP 机间通信 |
| 单文件 3500 行 CNode | 7 模块分层解耦 |
| 硬编码 uav1-uav6 | `unordered_map<sysid>` 动态管理 |
| Timer 回调 + 全局对象 | 专用线程 + mutex + 安全保活 |
| 仅固定翼 | 支持多旋翼 (COPTER) + 固定翼 (PLANE) |

## 九、参考文档

- [ArduPilot Guided Mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html)
- [Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [MAVLink SET_POSITION_TARGET](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [ArduPilot Companion Computers](https://ardupilot.org/dev/docs/companion-computers.html)
- [野火鲁班猫文档](https://doc.embedfire.com)
