# systemd 部署模板（5G 数图传一体）

对应方案 §7.4、§8.2。三件套：拨号 → VPN → QoS → swarm。

## 安装步骤

```bash
# 1) 二进制
sudo install -Dm755 build/swarm_node /opt/swarm/bin/swarm_node

# 2) QoS 脚本（参数见脚本头注释）
sudo install -Dm755 scripts/swarm-qos.sh /usr/local/sbin/swarm-qos.sh

# 3) systemd 单元
sudo install -Dm644 systemd/swarm-modem.service       /etc/systemd/system/swarm-modem.service
sudo install -Dm644 systemd/swarm-qos.service         /etc/systemd/system/swarm-qos.service
sudo install -Dm644 systemd/swarm-controller@.service /etc/systemd/system/swarm-controller@.service

# 4) （可选）按机覆盖参数
sudo tee /etc/default/swarm-2.env > /dev/null <<EOF
ROLE=2
GCS_HOST=10.8.0.100
LEADER_IP=10.8.0.1
EXTRA_ARGS=--p900-uart9-fallback --p900-dev /dev/ttyS9 --p900-baud 57600
EOF

# 5) WireGuard（不在本仓库管理；按 /etc/wireguard/wg0.conf 配置后）
sudo systemctl enable --now wg-quick@wg0

# 6) 拉起整套
sudo systemctl daemon-reload
sudo systemctl enable --now swarm-modem swarm-qos
sudo systemctl enable --now swarm-controller@2     # 该机 sysid=2
```

## 关键启动顺序

```
swarm-modem.service        (mmcli 拨号)
   ↓ Before=
wg-quick@wg0.service       (WireGuard 隧道)
   ↓ After=
swarm-qos.service          (HTB+fq_codel+DSCP 打标)
   ↓ After=
swarm-controller@<sysid>   (主程序，IP 主链 + 健康监视 + 可选 P900 fallback)
```

## 验证

```bash
sudo systemctl status swarm-controller@2
sudo journalctl -u swarm-controller@2 -f
sudo tc -s qdisc show dev wg0
ping -c3 10.8.0.100
ss -upn | grep 14550
```

## 故障速查

| 现象 | 排查点 |
|------|--------|
| `wg-quick@wg0` 起不来 | `wg show` 看握手；APN 是否拨上、运营商防火墙 |
| MP 一直没看到飞机 | `gcs-udp-target` 是否指向 MP 主机的 VPN IP；MP 监听端口防火墙 |
| 频繁 `[5GHealth] DEGRADED` | `--health-warn-ms` 放大、`tc class` 给图传更小的 ceil |
| 一切正常但图传卡顿 | `tc -s` 看 1:20 是否触顶；机端码率 < 实测上行 × 0.6 |
