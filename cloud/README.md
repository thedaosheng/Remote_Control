# Cloud - Signaling Server + TURN

WebSocket signaling server and coturn TURN relay, hosted on Alibaba Cloud.

## Server: `39.102.113.104`

| Service | Port | Protocol |
|---------|------|----------|
| Signaling (WS) | 8765 | TCP |
| Signaling (WSS) | 8766 | TCP |
| TURN | 3478 | UDP + TCP |
| TURN relay | 49152-65535 | UDP |

## Signaling Server (v9)

Handles role-based message routing between sender and receiver.

### Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `register` | client → server | Register as `sender` / `receiver` |
| `request_offer` | receiver → sender | Request sender to create offer |
| `offer` | sender → receivers | SDP offer (cached for late-joining receivers) |
| `answer` | receiver → senders | SDP answer |
| `ice` | bidirectional | ICE candidate exchange |
| `ping/pong` | bidirectional | Heartbeat + RTT measurement |
| `pose` | receiver → sender | Head tracking data (position + quaternion) |

### Start

```bash
python3 -u signaling_server.py
```

## coturn TURN Server

### Config (`/etc/turnserver.conf`)

Key settings:
- Long-term credentials: `user=remote:Wawjxyz3!`
- External IP mapping: `external-ip=39.102.113.104/172.21.124.70`
- Relay ports: 49152-65535

### Alibaba Cloud Security Group

Must allow:
- TCP 8765-8766 inbound (signaling)
- UDP 3478 inbound (TURN)
- UDP 49152-65535 inbound (TURN relay)
- All UDP outbound
