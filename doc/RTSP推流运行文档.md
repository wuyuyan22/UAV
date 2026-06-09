# RTSP 推流运行文档

本文档说明在 **鲁班猫 4（RK3588）** 机载 Linux 上，使用 **FFmpeg + MediaMTX** 完成摄像头采集、RK 硬件 H.264 编码、RTSP 推流，以及在机载本机或地面主机上拉流播放的完整流程。适用于数图传联调、单机图传验证，以及与 [声网直播推流运行文档.md](./声网直播推流运行文档.md)、地面站等下游对接前的本地 RTSP 环回测试。

---

## 1. 架构概览

```
┌─────────────┐    RTSP RECORD     ┌──────────────┐    RTSP PLAY / HLS    ┌─────────────────┐
│   FFmpeg    │ ────────────────► │   MediaMTX   │ ────────────────────► │ VLC / ffplay    │
│ v4l2+       │  rtsp://:8554/live│  (机载服务)   │  rtsp://<机载IP>:8554 │ （机载或地面 PC）│
│ h264_rkmpp  │                   │  mediamtx.yml │  http://:8888/...   │                 │
└─────────────┘                   └──────────────┘                       └─────────────────┘
      推流端 (Publisher)                 中继/分发                           播放端 (Reader)
```

| 组件 | 路径/版本 | 作用 |
|------|-----------|------|
| MediaMTX | `/root/agora/mediamtx/mediamtx` v1.18.2 | RTSP/RTMP/HLS/WebRTC 流媒体服务器 |
| 配置文件 | `/root/agora/mediamtx/mediamtx.yml` | 路径、鉴权、端口；**启动时必须显式传入** |
| FFmpeg | 带 `--enable-rkmpp` 构建（如 76cd3b5） | `v4l2` 采集、`h264_rkmpp` 硬编、RTSP 客户端推流 |

默认推流地址（机载环回）：**`rtsp://127.0.0.1:8554/live`**。地面或 PC 拉流时将 `127.0.0.1` 换为机载局域网/VPN IP（见 §6）。

---

## 2. 平台与摄像头

### 2.0 硬件说明

| 项目 | 本环境 |
|------|--------|
| 开发板 | 鲁班猫 4（LubanCat 4） |
| SoC | **RK3588**（非 RK3566） |
| 媒体栈 | Rockchip MPP，FFmpeg `h264_rkmpp` / GStreamer `mpph264enc` |
| 算力 | RK3588 可稳定硬编 **1080p@30**；数传带宽紧张时优先 **720p@30** 或 **1080p@15** |

**帧率档位（按需选用，§4）**

| 档位 | 分辨率@帧率 | 典型码率 | 适用场景 |
|------|-------------|----------|----------|
| 省带宽 | 720p@15 | 2 Mbps | 5G/数传联调、多机并发 |
| 默认 | 1080p@15 | 4 Mbps | 环回验证、延迟不敏感 |
| **高帧率直播** | **1080p@30** | 6～8 Mbps | 鲁班猫 4 机载直播、地面 VLC 流畅观看 |
| 平衡 | 720p@30 | 3～4 Mbps | 要高帧率但上行有限 |

本文档命令在 RK3588 机载已验证；若移植到 RK3566 等其它 RK 板卡，设备节点与最大分辨率需用 `v4l2-ctl` 重新确认。

### 2.1 设备节点（鲁班猫 4）

| 节点 | 说明 |
|------|------|
| `/dev/video11` | `rkisp_mainpath`，ISP 输出，**推流用这个** |
| `/dev/video0` | `rkcif` 原始 Bayer，勿用于 UYVY/H.264 直播 |

```bash
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video11 --list-formats-ext | head -20
```

`/dev/video11` 常见采集格式：`UYVY`、`NV12` 等（**无** `nv24`）。推流命令须使用 **`uyvy422`**（或与 `v4l2-ctl` 查询结果一致）。

### 2.2 FFmpeg 与硬编

```bash
ffmpeg -version | head -1
ffmpeg -hide_banner -encoders 2>/dev/null | grep rkmpp
# 应看到 h264_rkmpp
```

### 2.3 MediaMTX 与端口

```bash
ls -l /root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml
ss -tlnp | grep -E '8554|8888'
```

若已安装到系统路径，亦可使用 `/usr/local/bin/mediamtx` 与 `/etc/mediamtx.yml`，**下文命令中的路径按本机实际替换**。

### 2.4 摄像头占用

推流前确认无其他进程独占设备：

```bash
fuser -v /dev/video11
```

---

## 3. 启动 MediaMTX（必须带配置文件）

### 3.1 常见错误

直接执行 `./mediamtx` 且未传入 yml 时，程序可能在**当前工作目录**查找 `mediamtx.yml`。未找到则使用**空配置**，日志类似：

```text
WAR configuration file not found ... using an empty configuration
```

此时路径 `live` 未注册，推流端可能失败或行为异常。

### 3.2 正确启动方式

**前台（调试用）：**

```bash
pkill -f mediamtx 2>/dev/null || true
/root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml
```

**后台：**

```bash
/root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml &
```

**systemd 示例**（路径按部署修改）：

```bash
sudo tee /etc/systemd/system/mediamtx.service > /dev/null <<'EOF'
[Unit]
Description=MediaMTX RTSP server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx
sudo systemctl status mediamtx
```

### 3.3 启动成功标志

- 日志中**不再出现** `using an empty configuration`
- 出现：`[RTSP] listener opened on :8554`
- `ss -tlnp` 显示 `*:8554`（对所有网卡监听）
- FFmpeg 推流后出现：`[path live] ... is publishing to path 'live'`

---

## 4. FFmpeg RTSP 推流

推流前须已启动 MediaMTX（§3）。**`-framerate`、`-fps_mode cfr -r`、`-g` 三者帧率数值须一致**（例如均为 30）；`-g` 建议为帧率的 1～2 倍（30fps 时常用 `-g 30` 或 `-g 60`）。

### 4.1 默认联调（1080p@15fps）

```bash
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1920x1080 -framerate 15 \
  -i /dev/video11 \
  -c:v h264_rkmpp -rc_mode CBR -b:v 4000k -g 30 \
  -fps_mode cfr -r 15 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

| 参数 | 说明 |
|------|------|
| `-input_format uyvy422` | 与 `video11` 实际像素格式一致 |
| `-framerate 15` | 向 V4L2 申请帧率（rkisp 可能不支持 `VIDIOC_G_PARM`，见 §8） |
| **`-fps_mode cfr -r 15`** | **必加**：固定输出帧率与时间戳，避免误判为 0.07fps 导致 MediaMTX 读超时断流 |
| `h264_rkmpp` | RK3588 MPP 硬件 H.264；支持 `uyvy422`，无需 `-vf format=yuv420p` |
| `-rtsp_transport tcp` | RTSP 走 TCP，与 MediaMTX、地面 VLC 拉流更稳定 |
| **不要** `-listen 1` | 该选项让 FFmpeg 当 RTSP **服务端**；MediaMTX 已在 8554 监听，应作**客户端**推流 |

推流前可选预设格式：

```bash
v4l2-ctl --device=/dev/video11 \
  --set-fmt-video=width=1920,height=1080,pixelformat=UYVY
```

### 4.2 高帧率直播（1080p@30fps，推荐）

RK3588 算力充足时，用于**流畅直播**（地面 VLC / ffplay）。上行建议 ≥ **8 Mbps**（含协议开销）。

```bash
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1920x1080 -framerate 30 \
  -i /dev/video11 \
  -c:v h264_rkmpp -rc_mode CBR -b:v 6000k -g 30 \
  -fps_mode cfr -r 30 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

| 参数 | 30fps 取值 | 说明 |
|------|------------|------|
| `-framerate` / `-r` | `30` | 采集与输出帧率对齐，避免 `dup`/`drop` 异常 |
| `-g` | `30`（1s 一个 IDR）或 `60`（2s） | 越小观众进流越快，码率略升 |
| `-b:v` | `6000k`～`8000k` | 卡顿则升码率；5G 紧张则降到 `5000k` |

**成功标志**：终端 `15 fps` 应变为约 **`25～30 fps`**（或 `frame=` 每秒增加 ~30），且持续数分钟无 `Broken pipe`。

### 4.3 高帧率平衡（720p@30fps）

在带宽有限但仍要 **30fps** 时使用：

```bash
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1280x720 -framerate 30 \
  -i /dev/video11 \
  -c:v h264_rkmpp -rc_mode CBR -b:v 3500k -g 30 \
  -fps_mode cfr -r 30 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

### 4.4 省带宽（720p@15fps，数传联调）

```bash
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1280x720 -framerate 15 \
  -i /dev/video11 \
  -c:v h264_rkmpp -rc_mode CBR -b:v 2000k -g 30 \
  -fps_mode cfr -r 15 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

数传场景建议单机视频码率低于 5G 实测上行 × 0.6，为 MAVLink 留余量（参见 `5G数图传一体改造实现方案_CN.md` §6.3）。

### 4.5 可选：降低延迟（1080p@30）

```bash
ffmpeg -fflags nobuffer -flags low_delay \
  -f v4l2 -input_format uyvy422 -video_size 1920x1080 -framerate 30 \
  -use_wallclock_as_timestamps 1 -i /dev/video11 \
  -c:v h264_rkmpp -rc_mode CBR -b:v 6000k -g 30 \
  -fps_mode cfr -r 30 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

### 4.6 环回自检（排除摄像头问题）

仅用 FFmpeg 验证 MediaMTX + RTSP 链路：

```bash
ffmpeg -f lavfi -i testsrc=size=1280x720:rate=15 \
  -c:v h264_rkmpp -b:v 2000k -g 30 -fps_mode cfr -r 15 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live_test
```

### 4.7 推流成功标志

**FFmpeg 终端：**

```text
Stream #0:0: Video: h264 (High) ... 30 fps    # 高帧率档
frame=  600+ fps=25~30 ... time=00:00:20.00 ...
```

15fps 档输出行显示 `15 fps`，`frame=` 每秒约增加 15。

不应长期停留在 `0.07 fps`、`frame=2` 且 `drop` 持续暴涨。

**MediaMTX 日志：**

```text
INF [path live] stream is available and online, 1 track (H264)
INF [RTSP] [session ...] is publishing to path 'live'
```

`RTP packets are too big ... remuxing` 为自动拆包提示，一般可忽略。

---

## 5. 路径与多机命名

`mediamtx.yml` 中：

- `pathDefaults.source: publisher` — 允许 RTSP/RTMP 等客户端推流
- `paths.all_others` — 未单独配置的路径名（如 `live`、`uav2`）均套用默认策略

**不必**单独写 `paths.live`，加载完整 yml 后即可使用 `rtsp://<host>:8554/<路径名>`。

| 机号 sysid | 推流 URL（机载） |
|------------|------------------|
| 1 | `rtsp://127.0.0.1:8554/uav1` |
| 2 | `rtsp://127.0.0.1:8554/uav2` |

FFmpeg 中将 URL 末尾改为 `/uav1`、`/uav2` 即可。地面拉流 URL 中的主机地址改为机载 IP（§6）。

---

## 6. 拉流播放

### 6.1 地址说明

| 场景 | RTSP URL 示例 |
|------|----------------|
| 机载本机环回 | `rtsp://127.0.0.1:8554/live` |
| 地面 PC / 同网段 | `rtsp://<机载IP>:8554/live` |
| VPN（示例） | `rtsp://10.8.0.3:8554/live` |

查询机载 IP：`hostname -I` 或 `ip -4 addr`。推流进程可继续使用 `127.0.0.1`；**仅拉流端在其它主机上时需写机载 IP**。

MediaMTX 其它协议（以 `mediamtx.yml` 为准）：

| 协议 | 地址示例（机载环回） |
|------|----------------------|
| RTSP | `rtsp://127.0.0.1:8554/live` |
| HLS | `http://127.0.0.1:8888/live/index.m3u8` |
| WebRTC | `http://127.0.0.1:8889/live` |

### 6.2 机载 ffplay

```bash
ffplay -rtsp_transport tcp rtsp://127.0.0.1:8554/live
```

### 6.3 地面主机 VLC

1. **媒体** → **打开网络串流**（`Ctrl+N`）
2. 填入（IP 换成主机 `ping` 通的机载地址）：

   ```text
   rtsp://192.168.137.230:8554/live
   ```

3. 若卡顿、花屏：在 **工具 → 偏好设置 → 全部 → 输入/编解码器** 中勾选 **「使用 RTP over RTSP (TCP)」**，或命令行：

   ```bash
   vlc --rtsp-tcp rtsp://192.168.137.230:8554/live
   ```

**前提**：FFmpeg 推流保持运行，且 MediaMTX 日志中有 `publishing to path 'live'`。

### 6.4 地面主机 ffplay

```bash
ffplay -rtsp_transport tcp rtsp://<机载IP>:8554/live
```

### 6.5 HLS 备选（防火墙对 RTSP 不友好时）

VLC 打开：

```text
http://<机载IP>:8888/live/index.m3u8
```

延迟通常略高于 RTSP。

### 6.6 主机侧连通性检查

在 **PC 上**执行：

```bash
ping <机载IP>
nc -zv <机载IP> 8554    # 有 nc 时
```

---

## 7. 端口与防火墙

| 端口 | 用途 |
|------|------|
| 8554 | RTSP（TCP）；FFmpeg `-rtsp_transport tcp` 时拉流主要依赖此端口 |
| 8000 / 8001 | RTP / RTCP（UDP，RTSP UDP 模式时使用） |
| 1935 | RTMP |
| 8888 | HLS（HTTP） |
| 8889 | WebRTC 信令（HTTP） |
| 8189 | WebRTC ICE（UDP） |
| 8890 | SRT（UDP） |

若启用 `ufw`，TCP 推流/拉流至少放行：

```bash
sudo ufw allow 8554/tcp
sudo ufw allow 8888/tcp   # 仅用 HLS 时
```

---

## 8. 故障速查

| 现象 | 原因 | 处理 |
|------|------|------|
| `empty configuration` / `path 'live' is not configured` | 未加载 yml | 启动时显式传入 `mediamtx.yml`（§3） |
| FFmpeg 显示 `0.07 fps`，约 10s 后 `Broken pipe` | rkisp 时间戳异常，输出极稀疏 | 增加 **`-fps_mode cfr -r 15`**（§4.1） |
| MediaMTX `read tcp ... i/o timeout` | 推流端长时间无数据 | 同上；确认 `frame=` 持续增长 |
| `Device or resource busy` | 摄像头被占用 | `fuser -v /dev/video11`，结束其它 `ffmpeg`/预览进程 |
| 指定 `nv24` 或错误 `video0` | 格式/节点错误 | 使用 `video11` + `uyvy422` |
| 使用 `-listen 1` | FFmpeg 与 MediaMTX 角色冲突 | **去掉** `-listen 1` |
| 地面 VLC 无画面 | URL 仍写 `127.0.0.1` 或推流已停 | 改为机载 IP；保持 FFmpeg 运行 |
| 拉流卡顿、花屏 | 码率过高或 UDP 丢包 | 降 `-b:v`；VLC/ffplay 使用 TCP |
| `8554` 端口占用 | 旧 MediaMTX 未退出 | `pkill -f mediamtx` 后重启 |
| `dup`/`drop` 较多 | 采集帧率与 `-r` 不一致 | 将 `-framerate`、`-r`、`-g` 对齐；高帧率改 720p@30 或降码率 |
| 30fps 仍显示 ~15fps | 未改 `-r 30` 或码率/CPU 瓶颈 | 确认 §4.2 全套 30 参数；`top` 看 CPU，必要时 `-b:v 5000k` |
| 高帧率卡顿、马赛克 | 码率不足或链路带宽不够 | 提高 `-b:v`；或改 §4.3 720p@30 |

**可忽略的日志：**

- `ioctl(VIDIOC_G_PARM): Inappropriate ioctl`（rkisp 常见）
- `RTP packets are too big ... remuxing`（MediaMTX 自动处理）

**日志查看：**

```bash
# MediaMTX 前台日志；systemd 部署时：
sudo journalctl -u mediamtx -f

# FFmpeg 提高日志级别（按需）
ffmpeg -loglevel debug ... 2>&1 | tee /tmp/ffmpeg-rtsp.log
```

---

## 9. 典型联调顺序

1. 启动 MediaMTX：`/root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml`
2. 先 §4.1 **1080p@15** 确认链路，再切 §4.2 **1080p@30** 高帧率直播
3. 机载环回：`ffplay -rtsp_transport tcp rtsp://127.0.0.1:8554/live`
4. 地面 PC：VLC 打开 `rtsp://<机载IP>:8554/live`（§6.3）
5. 环回正常后，对接 VPN、地面站或 [声网直播推流运行文档.md](./声网直播推流运行文档.md) 等下游（公网高帧率见该文档 §4.2～4.4）

---

## 10. 与项目其他文档的关系

| 文档 | 关系 |
|------|------|
| [声网直播推流运行文档.md](./声网直播推流运行文档.md) | 公网/声网 RTC 推流；RTSP 用于本机/局域网环回，与之互补 |
| `5G数图传一体改造实现方案_CN.md` §6 | 图传选型、码率与 QoS 原则 |
| `systemd/README.md` | 伴飞 systemd 部署；可将 `mediamtx.service` 纳入开机流程 |
| `通信链路详细分析文档.md` | 端到端通信链路；RTSP 可作为机端本地分发节点 |

---

## 11. 快速命令备忘

```bash
# 1) 起 MediaMTX
/root/agora/mediamtx/mediamtx /root/agora/mediamtx/mediamtx.yml &

# 2a) 推流 — 默认 1080p@15
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1920x1080 -framerate 15 \
  -i /dev/video11 -c:v h264_rkmpp -rc_mode CBR -b:v 4000k -g 30 \
  -fps_mode cfr -r 15 -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live

# 2b) 推流 — 高帧率直播 1080p@30（推荐流畅观看）
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1920x1080 -framerate 30 \
  -i /dev/video11 -c:v h264_rkmpp -rc_mode CBR -b:v 6000k -g 30 \
  -fps_mode cfr -r 30 -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live

# 2c) 推流 — 高帧率平衡 720p@30
ffmpeg -f v4l2 -input_format uyvy422 -video_size 1280x720 -framerate 30 \
  -i /dev/video11 -c:v h264_rkmpp -rc_mode CBR -b:v 3500k -g 30 \
  -fps_mode cfr -r 30 -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/live

# 3) 机载拉流
ffplay -rtsp_transport tcp rtsp://127.0.0.1:8554/live

# 4) 地面 VLC（IP 按本机修改）
vlc --rtsp-tcp rtsp://192.168.137.230:8554/live
```

---

*文档版本：v2.2 | 平台：鲁班猫 4 / RK3588 / MediaMTX 1.18.2 / FFmpeg（rkmpp）*
