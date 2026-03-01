/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_device_uac.h"
#include "driver/i2s_std.h"


#define MIC_I2S_PORT      I2S_NUM_0


i2s_chan_handle_t rx_chan;                      // I2S rx channel handler
 
static void i2s_example_init_std_simplex()      //I2S的初始化函数
{
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg=
        {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_512,
            .sample_rate_hz = 44100,            
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = GPIO_NUM_17,
            .ws   = GPIO_NUM_18,
            .dout = I2S_GPIO_UNUSED,
            .din  = GPIO_NUM_16,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

// 仅新增：32bit转16bit核心函数（最小改动）
static void convert_32to16(const uint8_t *in_32, uint8_t *out_16, size_t in_len, size_t *out_len)
{
    *out_len = 0;
    if (in_32 == NULL || out_16 == NULL || in_len < 4) return;

    const uint32_t *in_samples = (const uint32_t *)in_32;
    int16_t *out_samples = (int16_t *)out_16;
    size_t sample_num = in_len / 4;  // 32bit=4字节/样本

    // 提取高16位（INMP441有效数据），若无效可改为 (sample & 0xFFFF) 取低16位
    for (size_t i = 0; i < sample_num; i++) {
        out_samples[i] = (int16_t)(in_samples[i] >> 16);
        // 可选：音量放大（声音小时取消注释）
        // out_samples[i] *= 2;
    }
    *out_len = sample_num * 2;  // 16bit=2字节/样本
}

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    // 步骤1：临时缓冲区存储32bit原始数据（长度=len*2，确保能读取足够32bit数据）
    uint8_t i2s_32b_buf[len * 2];
    size_t i2s_bytes = 0;

    // 步骤2：读取32bit原始数据（阻塞模式）
    esp_err_t ret = i2s_channel_read(rx_chan, i2s_32b_buf, len * 2, &i2s_bytes, portMAX_DELAY);
    if (ret != ESP_OK || i2s_bytes == 0) {
        *bytes_read = 0;
        return ret;
    }

    // 步骤3：32bit转16bit，结果写入UAC的buf
    convert_32to16(i2s_32b_buf, buf, i2s_bytes, bytes_read);

    return ESP_OK;
}
 

void app_main(void)
{
    i2s_example_init_std_simplex();
    uac_device_config_t config = {
        .output_cb = NULL,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };

    uac_device_init(&config);
}
