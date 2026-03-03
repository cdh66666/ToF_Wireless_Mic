#include "audio_espnow.h"
#include "ToF.h"
#include "ws2812.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";

// 强制录音按键（内部上拉，按下接地）
#define FORCE_RECORD_GPIO GPIO_NUM_9

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

    // 5. 初始化WS2812 RGB灯
    if (ws2812_init() != ESP_OK) {
        ESP_LOGE(TAG, "WS2812初始化失败");
        return;
    }
    ESP_LOGI(TAG, "WS2812初始化完成");

    // 6. 初始化ToF传感器
    esp_err_t ret = tof050c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ToF初始化失败");
        return;
    }
    ESP_LOGI(TAG, "ToF初始化完成");

    // 7. 配置强制录音按键为输入并启用内部上拉
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FORCE_RECORD_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 8. 循环测距并控制RGB灯和音频传输（100ms/次）
    while (1) {
        uint16_t distance = tof050c_read_distance_mm();

        // 优先检测强制录音按键（按下接地）
        int force_level = gpio_get_level(FORCE_RECORD_GPIO);
        if (force_level == 0) {
            // 用户按住按键：强制录音传输（不受ToF限制）
            ws2812_set_red(255);
            audio_set_transmission_enabled(1);
            ESP_LOGI(TAG, "强制录音按键按下 - 强制錄音傳输");
        } else {
            // 根据距离状态控制RGB灯和音频传输
            if (distance > 150) {
                // 距离超过15cm：关灯，禁止音频传输
                ws2812_off();
                audio_set_transmission_enabled(0);
                ESP_LOGI(TAG, "距离: %d mm (>15cm) - 关灯，停止音频传输", distance);
            }
            else if (distance < 50) {
                // 距离小于5cm：红灯全亮，启动音频传输
                ws2812_set_red(255);
                audio_set_transmission_enabled(1);
                ESP_LOGI(TAG, "距离: %d mm (<5cm) - 红灯，启动音频传输", distance);
            }
            else {
                // 距离在5-15cm范围：绿灯，亮度随距离减少而增加，启动音频传输
                uint8_t brightness = (uint8_t)(((150 - distance) * 255) / 100);
                ws2812_set_green(brightness);
                audio_set_transmission_enabled(1);
                ESP_LOGD(TAG, "距离: %d mm (5-15cm) - 绿灯，亮度: %d", distance, (150-distance)*255/100);
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


 