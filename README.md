# ToF无线麦克风

靠近嘴巴自动启动录音的无线麦克风系统。

## 功能

**发送端（tof_wireless_mic）**
- ToF传感器检测距离，靠近时自动录音
- I2S麦克风采集音频
- ESP-NOW无线传输
- 手动录音按键（GPIO9）
- 振动/LED反馈（GPIO21）

**接收端（tof_wireless_mic_rx）**
- 接收无线音频数据
- ADPCM解压缩
- USB音频设备（UAC）输出到电脑

## 硬件需求

| 组件 | 规格 |
|------|------|
| 发送端 | ESP32-S3 |
| 接收端 | ESP32-S3 |
| 麦克风 | I2S接口 |
| 传感器 | VL6180X ToF |
| 其他 | WS2812 LED，按键, 振动马达|

## 快速开始

### 前置条件
- ESP-IDF 5.0+
- 两块ESP32-S3开发板

### 1. 编译发送端
```bash
cd tof_wireless_mic
idf.py build
idf.py flash -p COM3
```

### 2. 编译接收端
```bash
cd ../tof_wireless_mic_rx
idf.py build
idf.py flash -p COM4
```

### 3. 修改MAC地址

**发送端** - 修改文件 [tof_wireless_mic/main/main.c](tof_wireless_mic/main/main.c#L23)
```c
// 改成你接收端的MAC地址
static const uint8_t peer_mac[6] = {0x98, 0xa3, 0x16, 0xf0, 0xb4, 0x34};
```

查看接收端MAC地址：
```bash
# 接收端刚上电时的日志会显示 MAC address 信息
idf.py monitor -p COM4
```

也可在 [tof_wireless_mic/main/audio_espnow.h](tof_wireless_mic/main/audio_espnow.h#L11) 修改：
- `ESPNOW_CHANNEL` - 确保发送端和接收端一致

## 配置说明

| 文件 | 配置项 | 说明 |
|------|--------|------|
| main.c | `peer_mac[6]` | 接收端MAC地址 |
| main.c | `FORCE_RECORD_GPIO` | 强制录音按键(GPIO9) |
| main.c | `VIBRATE_LED_GPIO` | 振动/LED引脚(GPIO21) |
| main.c | `VIBRATE_HOLD_MS` | 振动时长(毫秒) |
| ToF.h | `TOF_SDA_GPIO` | ToF I2C_SDA(GPIO6) |
| ToF.h | `TOF_SCL_GPIO` | ToF I2C_SCL(GPIO7) |
| audio_espnow.h | `ESPNOW_CHANNEL` | 通讯信道(1-14) |

## 工作原理

1. 发送端ToF传感器持续检测距离
2. 距离<50mm时自动启动I2S录音
3. 音频数据压缩(ADPCM)后通过ESP-NOW传输
4. 接收端解压并输出到USB音频设备
5. 电脑识别为USB麦克风，直接使用

## 故障排查

**接收不到音频**
- 检查两端MAC地址是否匹配
- 确认信道(ESPNOW_CHANNEL)一致
- 查看日志: `idf.py monitor`

**ToF不工作**
- 检查I2C接线(GPIO6/GPIO7)
- 确认传感器地址为0x29

**接收端无法被电脑识别**
- 检查USB线是否是数据线
- 重新拔插USB
- 查看设备管理器是否出现USB设备
