// 链式代理脚本 - 所有流量通过香港 IEPL 中继到美国住宅 IP
// 用法：Clash Verge Rev → 订阅 → 脚本增强 → 粘贴此内容
// 版本：v2.0 (2026-04-11)

function main(config, profileName) {

  // ==========================================
  // 1. 香港 IEPL 中继组（自动选延迟最低的香港 IEPL 节点）
  // ==========================================
  const hkRelayGroup = {
    name: "🇭🇰 HK-Relay",
    type: "url-test",
    proxies: [],
    url: "http://www.gstatic.com/generate_204",
    interval: 300,
    tolerance: 50,
  };
  if (config.proxies) {
    config.proxies.forEach(function(p) {
      if (p.name && p.name.indexOf("香港") !== -1 && p.name.indexOf("IEPL") !== -1) {
        hkRelayGroup.proxies.push(p.name);
      }
    });
  }
  if (hkRelayGroup.proxies.length === 0 && config.proxies) {
    config.proxies.forEach(function(p) {
      if (p.name && p.name.indexOf("香港") !== -1) {
        hkRelayGroup.proxies.push(p.name);
      }
    });
  }

  // ==========================================
  // 2. 美国住宅代理节点（SOCKS5, 通过香港中继）
  // ==========================================
  const residentialProxy = {
    name: "🇺🇸 US-Residential",
    type: "socks5",
    server: "95.134.206.39",
    port: 12324,
    username: "14a5bdb857e9f",
    password: "125b6a6f29",
    "dialer-proxy": "🇭🇰 HK-Relay",
  };
  if (!config.proxies) config.proxies = [];
  config.proxies.unshift(residentialProxy);

  // ==========================================
  // 3. 策略组
  // ==========================================
  if (!config["proxy-groups"]) config["proxy-groups"] = [];

  var claudeGroup = {
    name: "🤖 Claude",
    type: "select",
    proxies: ["🇺🇸 US-Residential"],
  };

  var globalExitGroup = {
    name: "🌐 全局出口",
    type: "select",
    proxies: ["🇺🇸 US-Residential"],
  };

  config["proxy-groups"].unshift(globalExitGroup);
  config["proxy-groups"].unshift(claudeGroup);
  config["proxy-groups"].unshift(hkRelayGroup);

  // ==========================================
  // 4. 劫持所有订阅策略组，前插住宅节点
  // ==========================================
  config["proxy-groups"].forEach(function(group) {
    var name = group.name || "";
    if (name === "🤖 Claude" || name === "🌐 全局出口" || name === "🇭🇰 HK-Relay") return;
    if (name.indexOf("国内") !== -1 || name.indexOf("直连") !== -1) return;
    if (group.proxies && Array.isArray(group.proxies)) {
      if (group.proxies.indexOf("🇺🇸 US-Residential") === -1) {
        group.proxies.unshift("🇺🇸 US-Residential");
      }
    }
  });

  // ==========================================
  // 5. 分流规则
  // ==========================================
  var claudeRules = [
    "DOMAIN-SUFFIX,claude.ai,🤖 Claude",
    "DOMAIN-SUFFIX,anthropic.com,🤖 Claude",
    "DOMAIN-SUFFIX,claude.com,🤖 Claude",
    "DOMAIN-KEYWORD,anthropic,🤖 Claude",
    "PROCESS-NAME,Claude,🤖 Claude",
    "PROCESS-NAME,claude,🤖 Claude",
    "PROCESS-NAME,Claude Helper,🤖 Claude",
    "PROCESS-NAME,Claude Helper (GPU),🤖 Claude",
    "PROCESS-NAME,Claude Helper (Renderer),🤖 Claude",
  ];

  var directRules = [
    "DOMAIN-SUFFIX,bambulab.com,DIRECT",
    "DOMAIN-SUFFIX,bambu.com,DIRECT",
    "PROCESS-NAME,BambuStudio,DIRECT",
    "IP-CIDR,192.168.0.0/16,DIRECT,no-resolve",
    "IP-CIDR,10.0.0.0/8,DIRECT,no-resolve",
    "IP-CIDR,172.16.0.0/12,DIRECT,no-resolve",
    "IP-CIDR,127.0.0.0/8,DIRECT,no-resolve",
  ];

  if (!config.rules) config.rules = [];
  config.rules = config.rules.filter(function(r) {
    return r.indexOf("MATCH") !== 0;
  });
  config.rules = claudeRules
    .concat(directRules)
    .concat(config.rules)
    .concat(["MATCH,🌐 全局出口"]);

  return config;
}
