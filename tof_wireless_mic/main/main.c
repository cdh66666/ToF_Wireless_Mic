#include "audio_espnow.h"
#include "esp_log.h"

// 接收端MAC地址（替换为你的实际MAC）
static const uint8_t peer_mac[6] = {0x98, 0xa3, 0x16, 0xf0, 0xb4, 0x34};

void app_main(void)
{
    // 1. 设置接收端MAC地址（可选，使用默认则注释）
    audio_espnow_set_peer_mac(peer_mac);

    // 2. 初始化I2S
    if (audio_i2s_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2S初始化失败");
        return;
    }
    ESP_LOGI(TAG, "I2S初始化完成");

    // 3. 初始化ESP-NOW
    if (audio_espnow_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW初始化失败");
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW初始化完成");

    // 4. 启动音频发送任务
    if (audio_send_task_start() != ESP_OK) {
        ESP_LOGE(TAG, "音频发送任务启动失败");
        return;
    }
    ESP_LOGI(TAG, "音频发送任务启动成功");
}