# SenseGlove Nova — Linux 串口协议完整逆向文档

> **官方声明 Linux 不支持 Nova 手套，但我们通过逆向 SGConnect/SGCore 二进制库，完整破解了串口协议，实现了在 Ubuntu 22.04 上直接驱动 Nova 手套——无需 SenseCom、无需 Windows、无需 Docker。**

## 一、连接方式

Nova 手套通过经典蓝牙 SPP (Serial Port Profile) 通信，UUID `00001101-0000-1000-8000-00805f9b34fb`。

### 蓝牙配对 + rfcomm 串口

```bash
# 1. 扫描手套（手套开机后蓝灯闪烁）
bluetoothctl scan on
# 看到 NOVA-XXXX-L 或 NOVA-XXXX-R

# 2. 配对（PIN 确认直接 yes，Nova 无屏幕）
# 推荐用 pexpect 自动确认：
python3 -c "
import pexpect, time
child = pexpect.spawn('bluetoothctl', encoding='utf-8', timeout=15)
child.sendline('agent DisplayYesNo')
child.sendline('default-agent')
child.sendline('pair <MAC>')
child.expect('Confirm passkey')
child.sendline('yes')
child.expect('Pairing successful')
child.sendline('trust <MAC>')
child.sendline('quit')
child.close()
"

# 3. 建立 rfcomm 虚拟串口（需要 sudo）
sudo rfcomm connect /dev/rfcomm0 <MAC> 1
# 显示 "Connected" 后保持终端不关

# 4. 现在 /dev/rfcomm0 就是手套的串口
```

### 注意事项
- 手套闲置几分钟会自动关机，需要重新开机配对
- `rfcomm connect` 必须保持前台运行，关闭即断连
- USB-CAN 适配器等其他 ttyACM 设备不受影响

---

## 二、串口协议

### 逆向方法

通过 `dlopen` 加载 `libSGConnect.so`，读取 `SGConnect::Connection` 类的静态成员变量：

```c
// pingCmd    @ .bss 0x34fda0 = "{Q}"
// startSensors @ .bss 0x34fd60 = "{S}"
// endSensors   @ .bss 0x34fd80 = "{s}"
// constRequest @ .bss 0x34fd40 = "{C}"
// idRequest    @ .bss 0x34fd20 = "{I}"
```

力反馈命令格式通过反汇编 `libSGCoreCpp.so` 中的 `ToNovaCommand` / `ToStreamCommand` 函数获得。

### 命令列表

| 命令 | 格式 | 说明 |
|------|------|------|
| 开始数据流 | `{S}` | 手套开始以 ~66Hz 发送传感器数据 |
| 停止数据流 | `{s}` | 停止传感器数据 |
| Ping | `{Q}` | 设备识别（Nova 不回复，但不影响） |
| 常量请求 | `{C}` | 请求设备常量/校准参数 |
| ID 请求 | `{I}` | 请求设备 ID |
| 力反馈 (Nova1) | `{Z<ffb×4><wrist><buzz×2>}` | 制动 + 振动 |
| 力反馈 (Nova2) | `{Z<ffb×4><wrist>}` | 仅制动 + 腕部 |

### 值编码

```python
def float_to_sg_byte(f):
    """将 0.0~1.0 的 float 转为 SG 协议单字节
    
    编码: chr(round(f * 100) + 1)
    0.0 → chr(1)  = 0x01
    0.5 → chr(51) = 0x33  
    1.0 → chr(101) = 0x65
    """
    return chr(round(f * 100) + 1)
```

### 传感器数据格式

每行一帧，文本格式：
```
[sensor1;sensor2|sensor3|sensor4|sensor5:qw;qx;qy;qz:battery;status]
```

| 字段 | 说明 |
|------|------|
| `sensor1;sensor2` | 拇指传感器（`;` 分隔 = 同组） |
| `sensor3\|sensor4\|sensor5` | 食指/中指/无名指+小指（`\|` 分隔） |
| `qw;qx;qy;qz` | IMU 四元数 |
| `battery` | 电池电量 (0-100) |
| `status` | 状态码 (0=正常) |

### 力反馈命令详解

#### Nova 1 格式 (9 字节)
```
{Z<thumb><index><middle><ring><wrist><buzz0><buzz1>}
```
- `thumb/index/middle/ring`: FFB 制动力 0.0~1.0 → `float_to_sg_byte()`
- `wrist`: 腕部挤压 0~100 整数 → `chr(value + 1)`
- `buzz0/buzz1`: 振动马达 0.0~1.0 → `float_to_sg_byte()`

#### Nova 2 流式格式 (7 字节)
```
{Z<thumb><index><middle><ring><wrist>}
```
- 同上但无 buzz 字段，振动通过单独的波形命令发送

---

## 三、Python 驱动库

```python
"""SenseGlove Nova Linux 驱动 — 直接串口通信，无需 SenseCom"""

import serial
import time
import threading


class NovaGlove:
    """SenseGlove Nova 手套驱动
    
    用法:
        glove = NovaGlove('/dev/rfcomm0')
        glove.start()
        
        # 读传感器
        data = glove.get_sensor_data()
        print(data['sensors'])  # [768, 1936, -1575, 856, 1874]
        print(data['imu'])      # [-1.6551, 2.5318, 0.7764, -0.3415]
        print(data['battery'])  # 93
        
        # 力反馈: 制动食指
        glove.set_force_feedback([0.0, 1.0, 0.0, 0.0])
        
        # 释放
        glove.set_force_feedback([0.0, 0.0, 0.0, 0.0])
        
        glove.stop()
    """
    
    def __init__(self, port='/dev/rfcomm0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.running = False
        self.lock = threading.Lock()
        self.latest_data = None
        self._reader_thread = None
    
    @staticmethod
    def _sg_byte(f):
        """0.0~1.0 → SG 协议字节"""
        v = max(0, min(100, round(f * 100)))
        return chr(v + 1)
    
    def start(self):
        """开始数据流"""
        self.ser.reset_input_buffer()
        self.ser.write(b'{S}')
        self.running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()
    
    def stop(self):
        """停止数据流 + 释放力反馈"""
        self.set_force_feedback([0.0, 0.0, 0.0, 0.0])
        self.running = False
        self.ser.write(b'{s}')
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        self.ser.close()
    
    def _read_loop(self):
        """后台读取传感器数据"""
        buf = ''
        while self.running:
            try:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                buf += chunk.decode('ascii', errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('[') and line.endswith(']'):
                        parsed = self._parse_frame(line)
                        if parsed:
                            with self.lock:
                                self.latest_data = parsed
            except Exception:
                if self.running:
                    time.sleep(0.01)
    
    @staticmethod
    def _parse_frame(line):
        """解析传感器帧
        
        格式: [s1;s2|s3|s4|s5:qw;qx;qy;qz:battery;status]
        """
        try:
            inner = line[1:-1]
            parts = inner.split(':')
            if len(parts) < 3:
                return None
            
            # 传感器值
            sensor_str = parts[0]
            # 分割: 先按 ; 和 | 拆分
            sensor_vals = []
            for tok in sensor_str.replace(';', '|').split('|'):
                sensor_vals.append(int(tok))
            
            # IMU 四元数
            imu_parts = parts[1].split(';')
            imu = [float(x) for x in imu_parts]
            
            # 电池和状态
            misc = parts[2].split(';')
            battery = int(misc[0])
            status = int(misc[1]) if len(misc) > 1 else 0
            
            return {
                'sensors': sensor_vals,
                'imu': imu,
                'battery': battery,
                'status': status,
            }
        except (ValueError, IndexError):
            return None
    
    def get_sensor_data(self):
        """获取最新传感器数据"""
        with self.lock:
            return self.latest_data
    
    def set_force_feedback(self, ffb, wrist=0.0):
        """设置力反馈制动
        
        Args:
            ffb: [thumb, index, middle, ring] 每个 0.0~1.0
            wrist: 腕部挤压 0.0~1.0
        """
        cmd = '{Z'
        for i in range(4):
            cmd += self._sg_byte(ffb[i] if i < len(ffb) else 0.0)
        cmd += self._sg_byte(wrist)
        cmd += '}'
        self.ser.write(cmd.encode('latin-1'))
    
    def set_vibration(self, buzz, wrist=0.0):
        """设置振动 (Nova 1 格式)
        
        Args:
            buzz: [motor0, motor1] 每个 0.0~1.0
            wrist: 腕部振动 0~100 整数
        """
        cmd = '{Z'
        # FFB 全零
        cmd += self._sg_byte(0.0) * 4
        # 腕部
        cmd += chr(int(wrist) + 1)
        # 振动
        for i in range(2):
            cmd += self._sg_byte(buzz[i] if i < len(buzz) else 0.0)
        cmd += '}'
        self.ser.write(cmd.encode('latin-1'))


# ============== 快速测试 ==============
if __name__ == '__main__':
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/rfcomm0'
    
    print('连接 SenseGlove Nova @ %s' % port)
    glove = NovaGlove(port)
    glove.start()
    
    print('读取数据 5 秒...')
    for _ in range(50):
        time.sleep(0.1)
        data = glove.get_sensor_data()
        if data:
            print('传感器: %s  电池: %d%%' % (data['sensors'], data['battery']))
    
    print('\n制动全部手指 3 秒...')
    glove.set_force_feedback([1.0, 1.0, 1.0, 1.0])
    time.sleep(3)
    
    print('释放')
    glove.set_force_feedback([0.0, 0.0, 0.0, 0.0])
    time.sleep(0.5)
    
    glove.stop()
    print('完成')
```

---

## 四、蓝牙连接自动化脚本

```bash
#!/bin/bash
# connect_nova.sh — 自动配对连接 SenseGlove Nova
# 用法: sudo bash connect_nova.sh

MAC=$(timeout 8 bluetoothctl scan on 2>&1 | grep -oP '(?<=Device )[0-9A-F:]+(?= NOVA)' | head -1)
if [ -z "$MAC" ]; then
    echo "未找到 Nova 手套，确认手套已开机（蓝灯闪烁）"
    exit 1
fi
echo "找到: $MAC"

# 自动确认 PIN 配对
python3 -c "
import pexpect, time
c = pexpect.spawn('bluetoothctl', encoding='utf-8', timeout=15)
c.sendline('agent DisplayYesNo'); time.sleep(0.3)
c.sendline('default-agent'); time.sleep(0.3)
c.sendline('pair $MAC')
try:
    idx = c.expect(['Confirm passkey', 'Pairing successful', 'AlreadyExists'], timeout=10)
    if idx == 0: c.sendline('yes'); c.expect('Pairing successful', timeout=10)
except: pass
c.sendline('trust $MAC'); time.sleep(0.5)
c.sendline('quit'); c.close()
"

# rfcomm 串口
rfcomm connect /dev/rfcomm0 "$MAC" 1
```

---

## 五、已知限制

1. **固件版本**: 此协议适用于 Nova 1 和 Nova 2 v1.x 固件（经典蓝牙 SPP）。Nova 2 v2.x 固件改用 BLE GATT，协议不同
2. **手套关机**: 闲置几分钟自动关机，需要重新开机 + rfcomm 连接
3. **rfcomm 占用**: `rfcomm connect` 必须保持前台运行
4. **官方不支持**: SenseCom Linux 版明确写 "not yet compatible with Nova Gloves"，我们绕过了它

---

## 六、逆向工具代码

用于从 `libSGConnect.so` 提取协议命令的 C 程序：

```c
// read_sg_protocol.c
// 编译: gcc -o read_sg_protocol read_sg_protocol.c -ldl
// 运行: LD_LIBRARY_PATH=<path_to_libSGConnect> ./read_sg_protocol

#include <stdio.h>
#include <dlfcn.h>

int main() {
    void *lib = dlopen("libSGConnect.so", RTLD_NOW);
    if (!lib) { printf("dlopen: %s\n", dlerror()); return 1; }
    
    const char *syms[] = {
        "_ZN9SGConnect10Connection7pingCmdB5cxx11E",
        "_ZN9SGConnect10Connection12startSensorsB5cxx11E",
        "_ZN9SGConnect10Connection10endSensorsB5cxx11E",
        "_ZN9SGConnect10Connection12constRequestB5cxx11E",
        "_ZN9SGConnect10Connection9idRequestB5cxx11E",
        NULL
    };
    
    for (int i = 0; syms[i]; i++) {
        void *s = dlsym(lib, syms[i]);
        if (s) {
            char **ptr = (char**)s;
            unsigned long *sz = (unsigned long*)((char*)s + 8);
            if (*ptr && *sz > 0 && *sz < 100)
                printf("%s = '%.*s'\n", syms[i], (int)*sz, *ptr);
        }
    }
    dlclose(lib);
    return 0;
}
```
