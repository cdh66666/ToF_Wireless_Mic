#include "ws2812.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

static const char *TAG = "WS2812";

// 有些平台头文件包含路径/宏可能导致函数原型不可见，显式声明以下API以避免隐式声明错误
extern esp_err_t led_strip_clear(led_strip_handle_t strip);
extern esp_err_t led_strip_refresh(led_strip_handle_t strip);
extern esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

// 静态全局变量
static led_strip_handle_t s_led_strip = NULL;

// ===================== 初始化函数 =====================
esp_err_t ws2812_init(void)
{
    // 1. LED Strip配置
    led_strip_config_t strip_config = {0};
    strip_config.strip_gpio_num = WS2812_GPIO_PIN;
    strip_config.max_leds = WS2812_LED_COUNT;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = 0;

    // 2. RMT配置
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建LED Strip失败: %d", ret);
        return ret;
    }

    // 3. 初始化LED为关闭状态
    ret = led_strip_clear(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "清除LED失败: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "WS2812 RGB灯初始化成功");
    return ESP_OK;
}

// ===================== 核心控制函数 =====================
esp_err_t ws2812_set_color(ws2812_color_t color)
{
    if (s_led_strip == NULL) {
        ESP_LOGE(TAG, "LED Strip未初始化");
        return ESP_FAIL;
    }

    esp_err_t ret = led_strip_set_pixel(s_led_strip, 0, color.r, color.g, color.b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置LED颜色失败: %d", ret);
        return ret;
    }

    ret = led_strip_refresh(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "刷新LED失败: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ws2812_off(void)
{
    if (s_led_strip == NULL) {
        ESP_LOGE(TAG, "LED Strip未初始化");
        return ESP_FAIL;
    }

    esp_err_t ret = led_strip_clear(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "关闭LED失败: %d", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ws2812_set_red(uint8_t brightness)
{
    ws2812_color_t color = {brightness, 0, 0};
    return ws2812_set_color(color);
}

esp_err_t ws2812_set_green(uint8_t brightness)
{
    ws2812_color_t color = {0, brightness, 0};
    return ws2812_set_color(color);
}

// ===================== 智能距离控制函数 =====================
esp_err_t ws2812_set_by_distance(uint16_t distance_mm)
{
    /*
     * 距离状态控制逻辑：
     * > 150mm (15cm):  不发光
     * 50-150mm (5-15cm): 绿光，亮度随距离减少而增加
     * < 50mm (5cm):    红色，全亮状态
     */

    if (distance_mm > 150) {
        // 距离超过15cm，关灯
        return ws2812_off();
    }
    else if (distance_mm < 50) {
        // 距离小于5cm，红灯全亮
        return ws2812_set_red(255);
    }
    else {
        // 距离在5-15cm范围，绿灯，亮度随距离减少而增加
        // 距离50mm时亮度为0，距离150mm时亮度为255
        // 亮度计算：(150-distance) / 100 * 255
        uint8_t brightness = (uint8_t)(((150 - distance_mm) * 255) / 100);
        return ws2812_set_green(brightness);
    }
}
