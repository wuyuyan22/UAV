# 长机 VPN + 开机自启盘点（10.8.0.1）

> 本机当前已是长机（leader），WireGuard `wg0=10.8.0.1/24` 已通过 `wg-quick@wg0` 自启。本文档只补**剩余的优化点**与对应的 systemd 配置，不动现有运行中的连接。

## 1. 当前已就绪的部分

| 组件 | 状态 |
|------|------|
| `/etc/wireguard/wg0.conf` Address=10.8.0.1/24 | ✅ |
| 服务端 `47.95.252.224:51820` + `AllowedIPs=10.8.0.0/24` | ✅ |
| `PersistentKeepalive=25`（NAT 保活） | ✅ |
| `wg-quick@wg0.service` enabled + active | ✅ |
| `ModemManager` 拨号 `wwan0=100.221.252.67/29`，access tech 5gnr | ✅ |
| `ping 10.8.0.100` RTT ~80 ms，0% loss | ✅ |

## 2. 仍需优化的三件事

### 2.1 wireguard-go 用户态 → 内核态 WireGuard（性能/CPU）

当前日志：
```
[!] Missing WireGuard kernel module. Falling back to slow userspace implementation.
[#] wireguard-go wg0
```

5G + 图传 + MAVLink 混跑时，用户态 wg 会明显拉高 CPU 与延迟抖动。修复 **二选一**：

#### 方案 A：升级到 6.12 内核（bookworm-backports 已自带 WG）

```bash
sudo apt update
sudo apt install -t bookworm-backports \
    linux-image-6.12.43+deb12-cloud-arm64-unsigned
sudo update-initramfs -u
# RK356x BSP 驱动兼容性确认后再 reboot
```

> ⚠️ RK356x 板载外设（5G 模组、Wi-Fi）驱动可能依赖 OEM 6.1 内核。**先在副板验证**，再切主控板。

#### 方案 B：保留 6.1 内核，仅装 wireguard 内核模块（DKMS）

```bash
sudo apt install wireguard-dkms linux-headers-$(uname -r)
sudo modprobe wireguard
sudo systemctl restart wg-quick@wg0
wg show wg0   # 看是否还有 wireguard-go 进程
```

如果 `linux-headers-$(uname -r)` 在仓库里找不到（OEM 内核常见），就只能走方案 A。

### 2.2 wg-quick@wg0 严格 After 5G 拨号（启动竞争）

现状：`After=network-online.target`，与 5G 默认路由就绪有几百毫秒空窗，日志里出现 `RTNETLINK answers: Network is unreachable`。

仓库已就位 drop-in：

```bash
sudo install -Dm644 systemd/wg-quick@wg0.service.d/10-after-modem.conf \
     /etc/systemd/system/wg-quick@wg0.service.d/10-after-modem.conf
sudo systemctl daemon-reload
sudo systemctl restart wg-quick@wg0
journalctl -u wg-quick@wg0 -b 0 | tail -10   # 不再出现 unreachable
```

### 2.3 swarm 主程序挂到 wg0 + 5G 启动链

部署 `swarm_node` 后让其严格在 wg0 之后启动：

```bash
# 1) 安装 swarm_node 与 QoS 脚本
sudo install -Dm755 build/swarm_node            /opt/swarm/bin/swarm_node
sudo install -Dm755 scripts/swarm-qos.sh        /usr/local/sbin/swarm-qos.sh

# 2) 安装本机 systemd 单元（QoS + 控制器）
sudo install -Dm644 systemd/swarm-qos.service           /etc/systemd/system/swarm-qos.service
sudo install -Dm644 systemd/swarm-controller@.service   /etc/systemd/system/swarm-controller@.service

# 3) 安装长机环境文件（仓库内 /etc/default/swarm-1.env 已写好）
sudo install -Dm644 systemd/etc-default-swarm-1.env  /etc/default/swarm-1.env

# 4) 启用
sudo systemctl daemon-reload
sudo systemctl enable --now swarm-qos
sudo systemctl enable --now swarm-controller@1
journalctl -u swarm-controller@1 -f
```

> 注意：仓库内 `swarm-controller@.service` 已含 `Requires=wg-quick@wg0.service` + `After=swarm-qos.service`，无需额外 drop-in。

## 3. 最终启动顺序示意

```
boot
 │
 ├─ ModemManager.service ─────────► 5G 拨号 (wwan0)
 │                                        │
 ├─ NetworkManager-wait-online.service ◄──┘
 │                                        │
 └─ network-online.target ◄───────────────┘
                  │
                  ▼
          wg-quick@wg0.service  (drop-in: After=ModemManager + 默认路由就绪)
                  │
                  ▼
          swarm-qos.service     (tc HTB + DSCP)
                  │
                  ▼
          swarm-controller@1.service  (长机 swarm_node)
```

## 4. 验证清单

```bash
# 启动顺序
systemd-analyze critical-chain swarm-controller@1.service

# VPN
wg show
ping -c3 10.8.0.100        # GCS
ping -c3 10.8.0.3          # 僚机（sysid=2 对应 10.8.0.3）

# QoS
sudo tc -s qdisc show dev wg0

# swarm
ss -upn | grep -E '14550|14551|19870'
journalctl -u swarm-controller@1 -f | grep -E '5GHealth|gcs_ip|tx_token'
```

## 5. 长机 swarm_node 关键启动参数（已在 /etc/default/swarm-1.env 内）

| 参数 | 长机取值 | 说明 |
|------|----------|------|
| `sysid` | 1 | 与 wg0 IP 10.8.0.1 对应 |
| `--link-port 19870` | 监听机间 UDP | 僚机以 `--leader-ip 10.8.0.1 --link-port 19870` 连入 |
| `--gcs-link ip` | 默认 | 走 5G + VPN |
| `--gcs-udp-target 10.8.0.100:14550` | 主控 MP 入口 | 与方案 §4.1 端口表一致 |
| `--gcs-udp-bind 10.8.0.1:14551` | 显式绑定 wg0 | 避免内核选源端口走 wwan0 漏经 VPN |
| `--gcs-heartbeat-hz 1` | NAT 保活 | 配合 wg `PersistentKeepalive=25` |
| `--followers 2,3` | 长机管的僚机 | 与 §3.2 一致 |
| `--p900-uart9-fallback`（可选） | P900 兜底 | 弱信号外场启用 |

## 6. 不建议改的地方

- **不要** 把 `AllowedIPs` 改成 `0.0.0.0/0`：会把公网流量也吸进 VPN，5G 上行被 swarm 服务端吃掉。
- **不要** 改 `wg0 MTU=1420`：5G NSA 帧损一般 ≥1400，1420 已是 wg over IPv4 的默认安全值。
- **不要** 取消 `PersistentKeepalive`：5G 运营商 NAT 表 30s 不收即老化，丢这个长机就会被僚机/MP 找不到。
