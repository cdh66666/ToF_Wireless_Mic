#include "audio_espnow.h"
#include "ToF.h"
#include "ws2812.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_system.h" // 新增：用于获取内存信息
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

// 强制录音按键（内部上拉，按下接地）
#define FORCE_RECORD_GPIO GPIO_NUM_9
// 震动马达/LED控制引脚（GPIO21，默认低电平）
#define VIBRATE_LED_GPIO  GPIO_NUM_21
// 高电平保持时间（毫秒）
#define VIBRATE_HOLD_MS   150

// 全局变量：记录上一次的音频传输状态，用于检测状态变化
static bool s_last_transmission_enabled = false;

// 接收端MAC地址（替换为你的实际MAC）
// 接收端MAC地址（替换为你的实际MAC）
// static const uint8_t peer_mac[6] = {0x98, 0xa3, 0x16, 0xf0, 0xb4, 0x34};
static const uint8_t peer_mac[6] = {0xac, 0xa7, 0x04, 0xed, 0x96, 0x50};//白色
// static const uint8_t peer_mac[6] = {0xac, 0xa7, 0x04, 0xee, 0x51, 0x60};

/**
 * @brief 控制震动/LED引脚：置高并保持指定时间后恢复低电平
 * @note 该函数会创建临时延时，不阻塞主循环（通过独立延时实现）
 */
static void vibrate_led_trigger(void)
{
    // 置为高电平（LED亮/马达震动）
    gpio_set_level(VIBRATE_LED_GPIO, 1);
    ESP_LOGD(TAG, "震动/LED引脚(GPIO21)置高，保持%dms", VIBRATE_HOLD_MS);
    
    // 保持高电平指定时间（不使用vTaskDelay避免阻塞主循环的ToF检测）
    uint32_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < VIBRATE_HOLD_MS / portTICK_PERIOD_MS) {
        // 空循环等待，仅占用少量CPU
        taskYIELD();
    }
    
    // 恢复低电平（LED灭/马达停止）
    gpio_set_level(VIBRATE_LED_GPIO, 0);
    ESP_LOGD(TAG, "震动/LED引脚(GPIO21)恢复低电平");
}

void app_main(void)
{
    // ========== 新增：初始化震动/LED控制引脚 ==========
    gpio_config_t vibrate_io_conf = {
        .pin_bit_mask = (1ULL << VIBRATE_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,       // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE, // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE,  // 禁用中断
    };
    gpio_config(&vibrate_io_conf);
    // 默认置为低电平
    gpio_set_level(VIBRATE_LED_GPIO, 0);
    ESP_LOGI(TAG, "震动/LED引脚(GPIO21)初始化完成，默认低电平");

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

    // 初始化上一次传输状态为关闭
    s_last_transmission_enabled = false;

    // 8. 循环测距并控制RGB灯和音频传输（100ms/次）
    while (1) {
        uint16_t distance = tof050c_read_distance_mm();
        bool current_transmission_enabled = s_last_transmission_enabled; // 默认保持上一次状态，除非条件触发改变

        // 优先检测强制录音按键（按下接地）
        int force_level = gpio_get_level(FORCE_RECORD_GPIO);
        if (force_level == 0) {
            // 用户按住按键：强制录音传输（不受ToF限制）
            ws2812_set_red(255);
            current_transmission_enabled = true;
            // ESP_LOGI(TAG, "强制录音按键按下 - 强制錄音傳输");
        } else {
            // 根据距离状态控制RGB灯和音频传输
            if (distance > 150) {
                // 距离超过15cm：关灯，禁止音频传输
                ws2812_off();
                current_transmission_enabled = false;
                // ESP_LOGI(TAG, "距离: %d mm (>15cm) - 关灯，停止音频传输", distance);
            }
            else if (distance < 50) {
                // 距离小于5cm：红灯全亮，启动音频传输
                ws2812_set_red(255);
                current_transmission_enabled = true;
                // ESP_LOGI(TAG, "距离: %d mm (<5cm) - 红灯，启动音频传输", distance);
            }
            else {
                // 距离在5-15cm范围：绿灯，亮度随距离减少而增加，启动音频传输
                uint8_t brightness = (uint8_t)(((150 - distance) * 255) / 100);
                ws2812_set_green(brightness);
                
                // ESP_LOGD(TAG, "距离: %d mm (5-15cm) - 绿灯，亮度: %d", distance, brightness);
            }
        }

        // ========== 核心逻辑：检测录音状态变化并触发震动/LED ==========
        if (current_transmission_enabled != s_last_transmission_enabled) {
            // 状态发生变化（开启/关闭），触发震动/LED
            ESP_LOGI(TAG, "录音状态变化：%s → %s，触发震动/LED",
                     s_last_transmission_enabled ? "开启" : "关闭",
                     current_transmission_enabled ? "开启" : "关闭");
            
            // 更新音频传输状态
            audio_set_transmission_enabled(current_transmission_enabled);
            
            // 触发震动/LED（置高0.5秒）
            vibrate_led_trigger();
            
            // 更新上一次状态
            s_last_transmission_enabled = current_transmission_enabled;
        } else {
            // 状态未变化，仅更新音频传输状态（防止意外丢失）
            audio_set_transmission_enabled(current_transmission_enabled);
        }
        
        // 优化：只每500ms打印一次内存信息，减少日志开销
        static uint32_t last_mem_print = 0;
        uint32_t current_tick = xTaskGetTickCount();
        if (current_tick - last_mem_print > 500 / portTICK_PERIOD_MS) {
            // ESP_LOGI("MEM", "剩余堆内存：%d bytes", esp_get_free_heap_size());
            last_mem_print = current_tick;
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}