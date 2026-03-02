#ifndef AUDIO_ESPNOW_H
#define AUDIO_ESPNOW_H

#include <stdint.h>
#include "esp_err.h"

// 配置参数（可根据需要修改）
#define TAG                 "ESPNOW_AUDIO_TX"
#define MIC_I2S_PORT        I2S_NUM_0
#define ESPNOW_CHANNEL      1   // ESP-NOW信道（接收端需一致）
#define AUDIO_BUF_SIZE      2048 // 音频缓冲区大小（字节）
#define ESPNOW_PAYLOAD_LEN  250  // ESP-NOW单包数据长度（<=250）

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

#endif // AUDIO_ESPNOW_H