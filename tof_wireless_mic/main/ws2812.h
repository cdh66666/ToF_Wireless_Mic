#ifndef __WS2812_H__
#define __WS2812_H__

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>

// ===================== 硬件配置 =====================
#define WS2812_GPIO_PIN          GPIO_NUM_48      // RGB灯控制引脚
#define WS2812_LED_COUNT         1                 // LED数量（单个RGB灯）
#define WS2812_RMT_CHANNEL       RMT_CHANNEL_0    // RMT通道

// RGB颜色结构体
typedef struct {
    uint8_t r;   // 红色成分 (0-255)
    uint8_t g;   // 绿色成分 (0-255)
    uint8_t b;   // 蓝色成分 (0-255)
} ws2812_color_t;

/**
 * @brief 初始化WS2812 RGB灯驱动
 * @return ESP_OK=成功，其他=失败
 */
esp_err_t ws2812_init(void);

/**
 * @brief 设置RGB灯颜色
 * @param color 颜色结构体，包含R、G、B成分(0-255)
 * @return ESP_OK=成功，其他=失败
 */
esp_err_t ws2812_set_color(ws2812_color_t color);

/**
 * @brief 关闭RGB灯
 * @return ESP_OK=成功，其他=失败
 */
esp_err_t ws2812_off(void);

/**
 * @brief 设置纯红色灯
 * @param brightness 亮度(0-255)
 * @return ESP_OK=成功，其他=失败
 */
esp_err_t ws2812_set_red(uint8_t brightness);

/**
 * @brief 设置纯绿色灯
 * @param brightness 亮度(0-255)
 * @return ESP_OK=成功，其他=失败
 */
esp_err_t ws2812_set_green(uint8_t brightness);

/**
 * @brief 根据距离自动设置RGB灯状态
 * @param distance_mm 距离值（毫米）
 * @return ESP_OK=成功，其他=失败
 * 
 * 距离范围说明：
 * - distance > 150mm (15cm): 不亮
 * - 50mm-150mm (5-15cm): 绿色，亮度随距离减少而增加
 * - distance < 50mm (5cm): 红色，最亮状态
 */
esp_err_t ws2812_set_by_distance(uint16_t distance_mm);

#endif // __WS2812_H__
