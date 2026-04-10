# 🇺🇸 如何让一台电脑变成"美国电脑" —— 链式代理部署指南

> **目标**：让任何一台电脑的所有网络流量都通过美国静态住宅 IP 出口，在外界看来就像一台真正位于美国的电脑。
>
> **已验证效果**：在 ipdata.co、scamalytics.com、ippure.com、ip-api.com 等 IP 评测网站上全部显示为美国纯净住宅 IP，非 VPN、非数据中心、非代理。

## 1. 架构总览

```
你的电脑 ──▶ Clash TUN ──▶ 🇭🇰 HK-Relay (香港IEPL) ──▶ 🇺🇸 US-Residential (SOCKS5) ──▶ 目标网站
```

### 核心原理：链式代理（Chained Proxy）

流量经过两跳才到达目标网站：
1. **第一跳 - 香港 IEPL 专线**：Clash TUN 劫持所有流量发到香港。IEPL 不经过公网 GFW，延迟低（~50ms）且稳定。
2. **第二跳 - 美国住宅 SOCKS5**：香港节点转发到 iProyal 美国静态住宅代理。SOCKS5 做透明 TCP/UDP 转发。

**安全保障**：任何一段链路断开 → 连接失败 → **绝不回落直连泄露真实 IP**。

### 为什么别人看不出你是谁？

| 层级 | 检查内容 | 为什么我们能过 |
|------|---------|--------------|
| IP 归属 | 是否数据中心/VPN？ | 我们的 IP 注册在住宅 ISP (NB NETWORKS GROUP LLC, ASN54339) |
| 行为历史 | IP 是否被大量共享？ | 静态住宅 IP 只有你一个人用，历史干净 |
| 连接特征 | 是否有代理特征头？ | SOCKS5 透明转发，不修改 TLS/HTTP 头，不插入 X-Forwarded-For |

你的真实 IP → 只有第一跳（香港）知道。iProyal → 只看到香港 IP。目标网站 → 只看到美国住宅 IP。

## 2. 代理服务商

**iProyal 静态住宅代理**

- 官网：https://iproyal.com （中文站 https://iproyal.cn）
- 控制面板：https://dashboard.iproyal.com
- 快速上手：https://iproyal.cn/quick-start-guides/static-residential-proxies/

**当前代理信息：**

| 项目 | 值 |
|------|-----|
| IP 地址 | `95.134.206.39` |
| 地区 | 美国纽约州 Kaser |
| ISP | NB NETWORKS GROUP LLC |
| ASN | AS54339 |
| SOCKS5 端口 | `12324` |
| HTTP 端口 | `12323`（链式模式下有 TLS 兼容问题，**建议用 SOCKS5**） |
| 用户名 | `14a5bdb857e9f` |
| 密码 | `125b6a6f29` |

## 3. 前置条件

- **Clash Verge Rev** 2.4.x+（内置 mihomo/Meta 内核 >= v1.19.17）
- 机场订阅包含**香港 IEPL 专线节点**
- mihomo >= v1.19.17（`relay` 已移除，必须用 `dialer-proxy`）

## 4. Step by Step 配置

### Step 1：安装 Clash Verge Rev 并导入机场订阅

- macOS: GitHub Releases 下载 DMG
- Linux: `sudo dpkg -i clash-verge_x.x.x_amd64.deb`

### Step 2：找到脚本增强文件

Clash Verge → 订阅 → 当前订阅右侧「脚本」(Script) → 编辑

文件路径：
```bash
# macOS
~/Library/Application Support/io.github.clash-verge-rev.clash-verge-rev/profiles/<script_uid>.js

# Linux
~/.local/share/io.github.clash-verge-rev.clash-verge-rev/profiles/<script_uid>.js
```

脚本 UID 在 `profiles.yaml` 中，看当前活跃配置绑定的 `script` 字段。

### Step 3：写入链式代理脚本

**核心配置，一个文件搞定所有。** 将脚本文件替换为 [`clash-chained-proxy.js`](./scripts/clash-chained-proxy.js)

### Step 4：配置全局 Merge（TUN 路由排除）

```yaml
profile:
  store-selected: true

tun:
  route-exclude-address:
    - 169.254.0.0/16
    - fe80::/10
    - 224.0.0.0/4
    - ff00::/8
  inet4-route-exclude-address:
    - 192.168.0.0/16
    - 10.0.0.0/8
    - 172.16.0.0/12
    - 127.0.0.0/8
```

### Step 5：重启并切换

1. 重启 Clash Verge Rev
2. 确认模式为 **Rule**（不是 Global）
3. 在 Proxies 页面，把 `🔰 选择节点` 和 `🐟 漏网之鱼` 都切到 `🇺🇸 US-Residential`
4. 打开 https://ippure.com/ 确认 IP 为 `95.134.206.39`

## 5. 系统环境配置（可选，进一步降低风险）

```bash
# 时区改为美东（匹配纽约出口）
sudo systemsetup -settimezone "America/New_York"  # macOS
sudo timedatectl set-timezone America/New_York     # Linux

# DNS 改为 Google DNS
sudo networksetup -setdnsservers Wi-Fi 8.8.8.8 1.1.1.1  # macOS
```

## 6. 验证

| 网站 | 检查项 | 期望结果 |
|------|--------|---------|
| https://ippure.com | IP | `95.134.206.39` |
| https://ipdata.co | IP Type | 非 VPN / 非代理 / 非数据中心 |
| https://scamalytics.com/ip/95.134.206.39 | Fraud Score | Low |
| https://ip-api.com | ISP / Hosting | NB NETWORKS GROUP LLC / None |
| https://meowvps.com/tools/ip-check/ | 综合检查 | 低风险 |

终端验证：
```bash
curl https://api.ipify.org        # 应返回 95.134.206.39
curl http://ip-api.com/json       # Country: United States, Hosting: false
```

## 7. 回退方法

**快速回退（不改文件）**：在 Clash Verge GUI 里把策略组切回其他节点，立即生效。

**彻底回退**：将脚本文件替换为：
```javascript
function main(config, profileName) { return config; }
```
然后重启 Clash Verge。

## 8. 已部署设备

| 设备 | IP | 用户名 | 脚本文件 | 状态 |
|------|-----|--------|---------|------|
| Linux Desktop | 192.168.0.181 | rhz | `sGxvdaDiQpwX.js` | ✅ 已验证 |
| Mac Mini | 192.168.0.177 | edward | `skjP6XIedprF.js` | ✅ 已验证 |
| MacBook Air M4 | 192.168.0.212 | ZHUANZ | `shZ1WdPGyWav.js` | ✅ 已验证 |

## 9. 注意事项

- **为什么用 SOCKS5 不用 HTTP？** iProyal HTTP 代理在链式模式下有 TLS 握手兼容问题（`Proxy CONNECT aborted`），SOCKS5 透明转发无此问题。
- **为什么用 `dialer-proxy` 不用 `relay`？** mihomo v1.19.17 已移除 `relay` 类型 proxy-group，必须用 `dialer-proxy` 字段。
- **为什么选香港 IEPL 做中继？** IEPL 是专线，不经 GFW，延迟低且稳定。直连美国住宅 IP 容易因跨太平洋链路不稳而断连。
- **iProyal 白名单**：建议在 dashboard.iproyal.com 中将公网 IP 加入白名单。

---

**文档版本**：v2.0（从 v1 升级为链式代理架构）
**创建日期**：2026-04-11
