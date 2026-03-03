#ifndef AUDIO_WIFI_H
#define AUDIO_WIFI_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * 简化发送端无线接口，使用普通 Wi‑Fi + UDP 广播代替 ESP‑NOW。
 * 这样音频包直接发到 IP 层，任意在同一局域网监听的接收端都能收到，
 * 不再需要手工填写 MAC 地址，音质也可以借助常规协议优化。
 */

/* 日志标签 */
#define AUDIO_TAG            "WIFI_AUDIO_TX"

/* I2S 配置 */
#define MIC_I2S_PORT         I2S_NUM_0

/* 音频缓冲相关 */
#define AUDIO_BUF_SIZE       2048   // 32bit samples

/* UDP 目标端口 */
#define AUDIO_WIFI_PORT      12345

/* 默认目标 IP：如果接收端运行在 ESP32 AP 模式则应使用 AP 的地址，
 * 本例中默认接收端 AP 固定在 192.168.4.1。必须在调用
 * `audio_wifi_init` 之前用 `audio_wifi_set_dest_ip` 修改为其它地址。
 */
#define AUDIO_WIFI_DEFAULT_IP "192.168.4.1"

/* 接收端 AP 默认配置，与对端接收机代码保持一致 */
#define AUDIO_WIFI_RX_SSID     "ESP32_AUDIO_AP"
#define AUDIO_WIFI_RX_PASSWORD "12345678"


/**
 * @brief 可选：设置 UDP 目标地址（在调用 audio_wifi_init 之前设置）
 * @param ip C 字符串形式的 IPv4 地址
 */
void audio_wifi_set_dest_ip(const char *ip);

/**
 * @brief 初始化 I2S 音频采集 (与 espnow 版本一致)
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_i2s_init(void);

/**
 * @brief 初始化 Wi-Fi 模块并建立 UDP socket
 *        本例程以 STA 模式连接由 `CONFIG_WIFI_SSID`/`CONFIG_WIFI_PASSWORD`
 *        指定的热点，连接成功后创建 UDP socket 并将目的地址设置为
 *        上述广播地址或通过 `audio_wifi_set_dest_ip` 指定的地址。
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_wifi_init(void);

/**
 * @brief 可选：在运行时设置 STA 模式的 SSID/密码（init 之前调用）
 */
void audio_wifi_set_sta_credentials(const char *ssid, const char *password);

/**
 * @brief 查询当前 Wi‑Fi 连接状态
 * @return true 已连接 | false 未连接
 */
bool audio_wifi_is_connected(void);

/**
 * @brief 获取累计 UDP 发送统计
 */
void audio_wifi_get_counters(uint32_t *sent, uint32_t *failed);

/**
 * @brief 启动音频采集并发送任务
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_send_task_start(void);

/**
 * @brief 设置音频传输开关
 */
void audio_set_transmission_enabled(uint8_t enable);

/**
 * @brief 读取音频传输开关状态
 */
uint8_t audio_get_transmission_enabled(void);

#endif // AUDIO_WIFI_H
