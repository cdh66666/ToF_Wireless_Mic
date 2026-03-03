#ifndef AUDIO_ESPNOW_H
#define AUDIO_ESPNOW_H

#include <stdint.h>
#include "esp_err.h"

// 配置参数（可根据需要修改）
#define AUDIO_TAG           "ESPNOW_AUDIO_TX"
#define MIC_I2S_PORT        I2S_NUM_0
#define ESPNOW_CHANNEL      6   // ESP-NOW信道（接收端需一致）
#define AUDIO_BUF_SIZE      2048 // 音频缓冲区大小（字节）
#define ESPNOW_PAYLOAD_LEN  250  // ESP-NOW单包数据长度（<=250）

// 新增：监控统计相关配置
#define MONITOR_INTERVAL_MS 1000 // 每1秒输出一次统计信息
#define MAX_SEND_ERROR_CNT  100  // 发送失败计数上限

/**
 * @brief 设置ESP-NOW接收端MAC地址
 * @param mac 接收端6字节MAC地址
 */
void audio_espnow_set_peer_mac(const uint8_t mac[6]);

/**
 * @brief 初始化I2S音频采集
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_i2s_init(void);

/**
 * @brief 初始化ESP-NOW传输
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_espnow_init(void);

/**
 * @brief 启动音频采集并发送任务
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t audio_send_task_start(void);

/**
 * @brief 设置音频传输启用标志位
 * @param enable 1=启用传输，0=禁用传输
 */
void audio_set_transmission_enabled(uint8_t enable);

/**
 * @brief 获取音频传输启用标志位
 * @return uint8_t 1=传输已启用，0=传输已禁用
 */
uint8_t audio_get_transmission_enabled(void);

#endif // AUDIO_ESPNOW_H