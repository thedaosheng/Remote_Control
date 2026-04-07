# Cloud — LiveKit Server (阿里云)

部署在阿里云 ECS 上的 LiveKit Server，作为 WebRTC SFU 中继 Linux ↔ Vision Pro 的视频流。

## 服务器信息

- **IP**: 39.102.113.104
- **LiveKit 版本**: 1.10.1
- **配置文件**: `/etc/livekit.yaml`（也备份在本目录 `livekit.yaml`）

## 端口

| 端口 | 协议 | 用途 | 阿里云安全组 |
|------|------|------|-------------|
| 7880 | TCP | LiveKit HTTP/WebSocket（信令）| **必须开** |
| 7881 | TCP | RTC TCP fallback | 建议开 |
| 50000-60000 | UDP | RTC UDP 媒体数据 | **必须开** |
| 3478 | UDP/TCP | TURN（备用，LiveKit 内置）| 可选 |

## 安装

```bash
# 1. 下载（中国大陆推荐用代理）
cd /usr/local/bin
wget "https://ghfast.top/https://github.com/livekit/livekit/releases/download/v1.10.1/livekit_1.10.1_linux_amd64.tar.gz" -O livekit.tar.gz
tar xzf livekit.tar.gz
chmod +x livekit-server

# 2. 写配置（见 livekit.yaml）
sudo cp livekit.yaml /etc/livekit.yaml

# 3. 验证安装
livekit-server --version
```

## 启动

```bash
# 前台启动（调试用）
livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104

# 后台启动
nohup livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104 > /tmp/livekit.log 2>&1 &

# 检查是否在监听
ss -tlnp | grep 7880
```

## 验证

启动后，从客户端测试连接：

```bash
# 简单的 HTTP 探测
curl http://39.102.113.104:7880
# 返回 "OK" 表示 server 在运行

# Python SDK 测试
python3 -c "
import asyncio
from livekit import rtc, api

async def test():
    token = api.AccessToken('teleop_key', 'teleop_secret_key_2026')
    token.with_identity('test-client')
    token.with_grants(api.VideoGrants(room_join=True, room='teleop-room'))
    room = rtc.Room()
    await room.connect('ws://39.102.113.104:7880', token.to_jwt())
    print(f'Connected! Room: {room.name}')
    await room.disconnect()

asyncio.run(test())
"
```

## 配置说明（livekit.yaml）

```yaml
port: 7880                    # WebSocket 端口
rtc:
  port_range_start: 50000     # UDP 起始端口
  port_range_end: 60000       # UDP 结束端口
  use_external_ip: true       # 使用 STUN 自动检测公网 IP
  tcp_port: 7881              # TCP fallback
keys:
  teleop_key: teleop_secret_key_2026   # API key: secret
turn:
  enabled: true
  domain: 39.102.113.104
  udp_port: 3478
```

## 常见问题

### 客户端连不上 7880
检查阿里云安全组是否放行了 TCP 7880。LiveKit 的 HTTP server 在本机能 `curl` 通就说明 server 没问题。

### 视频走 TCP fallback 而不是 UDP
检查 UDP 50000-60000 安全组是否放行。LiveKit 优先 UDP，UDP 不通才回退 TCP。

### Linux 端是 Clash TUN 用户连不上
Clash TUN 会劫持 UDP 流量。需要在 Clash Verge 的 `Merge.yaml` 中排除 LiveKit 服务器 IP：
```yaml
tun:
  inet4-route-exclude-address:
    - 39.102.113.104/32
```

## 监控

```bash
# 查看实时日志
tail -f /tmp/livekit.log

# 查看连接的房间和参与者
grep -E "participant active|room created" /tmp/livekit.log | tail -10
```
