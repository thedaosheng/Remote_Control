# Clash 新电脑配置：Claude 专用美国住宅 IP 分流

> 目的：所有设备访问 Claude/Anthropic 时走独享美国住宅 IP，其他流量走 Ikuuu 节点。

## 背景

Anthropic 风控系统会检测 IP/ASN、浏览器指纹、DNS 泄露、支付信息等多层信号。使用机场节点（如 Ikuuu）访问 Claude 风险极高（ASN 一眼代理商）。解决方案是购买独享美国住宅 IP，仅让 Claude 流量走这个干净出口。

## 代理信息

| 项目 | 值 |
|------|-----|
| 供应商 | IPRoyal (独享 ISP 代理) |
| 类型 | HTTP Proxy |
| 服务器 | `95.134.206.39` |
| 端口 | `12323` |
| 用户名 | `14a5bdb857e9f` |
| 密码 | `125b6a6f29` |
| 出口 IP 地区 | 美国纽约 Kaser |
| ASN | AS54339 NB NETWORKS GROUP LLC |
| ip-api proxy 标记 | false |
| ip-api hosting 标记 | false |

## 配置方式

### 方式一：Clash Verge Rev（推荐，通过 Script 注入）

适用于所有使用 Clash Verge Rev 的设备。

**步骤：**

1. 打开 Clash Verge → **Profiles（配置）**
2. 找到当前使用的 Ikuuu 订阅卡片
3. 点卡片右上角 **编辑图标** → 切到 **Script** 标签页
4. 把下面的 JS 完整粘贴进去（替换原有内容）
5. 保存 → 点订阅的刷新按钮重载配置
6. **重要：模式切到 Rule（规则模式），不要用 Global**

```javascript
// Define main function (script entry)

function main(config, profileName) {
  // === Claude 专用美国住宅 IP ===
  const claudeProxy = {
    name: "🇺🇸 US-Residential-Claude",
    type: "http",
    server: "95.134.206.39",
    port: 12323,
    username: "14a5bdb857e9f",
    password: "125b6a6f29",
  };

  // 添加代理节点
  if (!config.proxies) config.proxies = [];
  config.proxies.unshift(claudeProxy);

  // 添加 Claude 策略组
  const claudeGroup = {
    name: "🤖 Claude",
    type: "select",
    proxies: ["🇺🇸 US-Residential-Claude"],
  };
  if (!config["proxy-groups"]) config["proxy-groups"] = [];
  config["proxy-groups"].unshift(claudeGroup);

  // Claude 分流规则：域名 + 进程双重保障
  const claudeRules = [
    // 已知域名（精确匹配，优先级最高）
    "DOMAIN-SUFFIX,claude.ai,🤖 Claude",
    "DOMAIN-SUFFIX,anthropic.com,🤖 Claude",
    "DOMAIN-SUFFIX,claude.com,🤖 Claude",
    "DOMAIN-KEYWORD,anthropic,🤖 Claude",
    // 进程兜底：不管访问什么域名，只要是 Claude 进程发出的都走住宅 IP
    "PROCESS-NAME,Claude,🤖 Claude",
    "PROCESS-NAME,claude,🤖 Claude",
    "PROCESS-NAME,Claude Helper,🤖 Claude",
    "PROCESS-NAME,Claude Helper (GPU),🤖 Claude",
    "PROCESS-NAME,Claude Helper (Renderer),🤖 Claude",
  ];
  if (!config.rules) config.rules = [];
  config.rules = claudeRules.concat(config.rules);

  return config;
}
```

### 方式二：手动编辑配置文件（ClashX / Clash for Android / Stash 等）

在配置文件的对应位置手动插入：

**proxies 段最前面加：**
```yaml
- name: "🇺🇸 US-Residential-Claude"
  type: http
  server: 95.134.206.39
  port: 12323
  username: "14a5bdb857e9f"
  password: "125b6a6f29"
```

**proxy-groups 段最前面加：**
```yaml
- name: "🤖 Claude"
  type: select
  proxies:
    - "🇺🇸 US-Residential-Claude"
```

**rules 段最前面加：**
```yaml
# 域名精确匹配
- DOMAIN-SUFFIX,claude.ai,🤖 Claude
- DOMAIN-SUFFIX,anthropic.com,🤖 Claude
- DOMAIN-SUFFIX,claude.com,🤖 Claude
- DOMAIN-KEYWORD,anthropic,🤖 Claude
# 进程兜底（防止 Anthropic 新增域名漏网）
- PROCESS-NAME,Claude,🤖 Claude
- PROCESS-NAME,claude,🤖 Claude
- PROCESS-NAME,Claude Helper,🤖 Claude
- PROCESS-NAME,Claude Helper (GPU),🤖 Claude
- PROCESS-NAME,Claude Helper (Renderer),🤖 Claude
```

## 配置完成后验证

### 1. 检查分流是否生效

在 Clash 的连接日志中，访问 claude.ai 后应该看到：

```
claude.ai:443 → ['🇺🇸 US-Residential-Claude', '🤖 Claude']
api.anthropic.com:443 → ['🇺🇸 US-Residential-Claude', '🤖 Claude']
```

### 2. 通过 Clash API 查看（Clash Verge）

```bash
curl -s http://127.0.0.1:9090/connections \
  -H "Authorization: Bearer <你的secret>" | \
  python3 -c "
import json,sys
data=json.load(sys.stdin)
for c in data.get('connections',[]):
    meta = c.get('metadata',{})
    host = meta.get('host','')
    if 'claude' in host.lower() or 'anthropic' in host.lower():
        print(f'{host}:{meta.get(\"destinationPort\",\"\")} → {c.get(\"chains\",[])}')
"
```

## 注意事项

1. **必须用 Rule 模式** — Global 模式会把所有流量扔到 IPRoyal，HTTP 代理不支持 UDP 且并发有限，会导致全部超时
2. **一个 Claude 账号不要多设备同时用** — 同一 IP 上多个活跃会话可能触发账号密度风控
3. **不要频繁切换 IP** — 保持稳定使用同一个出口

## 新电脑系统环境配置（必做）

配完 Clash 分流后，还需要修改系统环境以匹配美国 IP，否则浏览器指纹层会暴露矛盾。

### macOS 一键配置

```bash
# 1. 时区改为美东（匹配 IPRoyal 纽约出口）
sudo systemsetup -settimezone "America/New_York"

# 2. DNS 改为 Google DNS（去掉 114.114.114.114 中国 DNS）
sudo networksetup -setdnsservers Wi-Fi 8.8.8.8 1.1.1.1
# 如果用有线网：
sudo networksetup -setdnsservers "USB 10/100/1000 LAN" 8.8.8.8 1.1.1.1

# 3. 验证
date +'%z %Z'              # 应显示 -0400 EDT 或 -0500 EST
scutil --dns | grep nameserver  # 应显示 8.8.8.8
```

### 手动操作（需要 GUI）

- **系统语言**：系统设置 → 通用 → 语言与地区 → 添加 English (US) 并拖到第一位
- **浏览器语言**：Chrome 设置 → 语言 → 将 English (US) 设为首选
- **WebRTC 防泄漏**：Chrome 安装 [WebRTC Leak Shield](https://chromewebstore.google.com/detail/webrtc-leak-shield/bppamachkoflopbagkdoflbgfjflfnfl) 扩展

### Clash Verge 强制 Rule 模式

如果订阅默认是 Global 模式，在对应的 **Merge 文件** 中加一行：

```yaml
mode: rule
```

这样即使配置刷新也会保持 Rule 模式。

### 验证清单

| 检查项 | 命令 | 期望结果 |
|--------|------|---------|
| 时区 | `date +'%z %Z'` | `-0400 EDT` 或 `-0500 EST` |
| DNS | `scutil --dns \| grep nameserver` | `8.8.8.8` |
| 系统语言 | `defaults read -g AppleLocale` | `en_US` |
| Claude 出口 | Clash 连接日志 | `claude.ai → 🤖 Claude → 🇺🇸 US-Residential-Claude` |
| Clash 模式 | Clash Verge 界面 | Rule（非 Global） |

## 流量架构

```
设备
 └── 所有流量 → Clash (TUN + Fake-IP, Rule 模式)
                 │
                 ├── claude.ai / anthropic.com / claude.com
                 │     └── 🤖 Claude → 🇺🇸 US-Residential-Claude
                 │            └── 出口: 95.134.206.39 (美国纽约, 独享住宅 IP)
                 │
                 └── 其他流量
                       └── 🔰 选择节点 → Ikuuu 节点
```

## IP 续费

- 供应商：[IPRoyal](https://iproyal.com/isp-proxies/)
- 当前套餐：$17.82/月（含城市定位 + Scamalytics 0 risk 筛选）
- 续费/管理：登录 IPRoyal Dashboard
