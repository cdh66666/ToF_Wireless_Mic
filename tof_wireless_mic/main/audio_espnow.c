#include "audio_espnow.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

// 全局变量
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0x98, 0xa3, 0x16, 0xf0, 0xb4, 0x34}; // 默认MAC
static i2s_chan_handle_t rx_chan;

// ==================== ADPCM 压缩核心模块 ====================
typedef struct {
    int16_t prev_sample;  // 上一个采样值
    int8_t step_index;    // 步长索引
} adpcm_state_t;

static const int16_t adpcm_step_table[] = {
    7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
    37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66,
    67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
    82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96,
    97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
    112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126,
    127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141,
    142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156,
    157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171,
    172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186,
    187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201,
    202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216,
    217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231,
    232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246,
    247, 248, 249, 250, 251, 252, 253, 254, 255
};

static const int8_t adpcm_index_table[] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static void adpcm_init_state(adpcm_state_t *state) {
    state->prev_sample = 0;
    state->step_index = 0;
}

static void adpcm_encode(const int16_t *in, size_t in_len, uint8_t *out, size_t *out_len, adpcm_state_t *state) {
    if (in == NULL || out == NULL || state == NULL || in_len % 2 != 0) {
        *out_len = 0;
        return;
    }

    size_t sample_num = in_len / 2;
    size_t out_idx = 0;
    int16_t prev_sample = state->prev_sample;
    int8_t step_index = state->step_index;

    if (step_index < 0) step_index = 0;
    if (step_index > 88) step_index = 88;

    for (size_t i = 0; i < sample_num; i += 2) {
        uint8_t nibble1 = 0, nibble2 = 0;
        int16_t diff = in[i] - prev_sample;
        int16_t step = adpcm_step_table[step_index];

        int8_t sign = (diff < 0) ? 8 : 0;
        if (sign) diff = -diff;
        int8_t delta = 0;
        int16_t temp = step;
        if (diff >= temp) { delta = 4; diff -= temp; }
        temp >>= 1;
        if (diff >= temp) { delta |= 2; diff -= temp; }
        temp >>= 1;
        if (diff >= temp) { delta |= 1; diff -= temp; }
        nibble1 = sign | delta;

        step_index += adpcm_index_table[nibble1];
        if (step_index < 0) step_index = 0;
        if (step_index > 88) step_index = 88;
        int16_t delta_val = step >> 3;
        if (delta & 4) delta_val += step;
        if (delta & 2) delta_val += step >> 1;
        if (delta & 1) delta_val += step >> 2;
        if (sign) prev_sample -= delta_val;
        else prev_sample += delta_val;

 
        if (i + 1 < sample_num) {
            diff = in[i+1] - prev_sample;
            step = adpcm_step_table[step_index];
            sign = (diff < 0) ? 8 : 0;
            if (sign) diff = -diff;
            delta = 0;
            temp = step;
            if (diff >= temp) { delta = 4; diff -= temp; }
            temp >>= 1;
            if (diff >= temp) { delta |= 2; diff -= temp; }
            temp >>= 1;
            if (diff >= temp) { delta |= 1; diff -= temp; }
            nibble2 = sign | delta;

            step_index += adpcm_index_table[nibble2];
            if (step_index < 0) step_index = 0;
            if (step_index > 88) step_index = 88;
            delta_val = step >> 3;
            if (delta & 4) delta_val += step;
            if (delta & 2) delta_val += step >> 1;
            if (delta & 1) delta_val += step >> 2;
            if (sign) prev_sample -= delta_val;
            else prev_sample += delta_val;

 
        }

        out[out_idx++] = (nibble1 << 4) | (nibble2 & 0x0F);
    }

    state->prev_sample = prev_sample;
    state->step_index = step_index;
    *out_len = out_idx;
}

// ==================== 工具函数 ====================
static void convert_32to16(const uint8_t *in_32, uint8_t *out_16, size_t in_len, size_t *out_len)
{
    *out_len = 0;
    if (in_32 == NULL || out_16 == NULL || in_len < 4) return;

    const uint32_t *in_samples = (const uint32_t *)in_32;
    int16_t *out_samples = (int16_t *)out_16;
    size_t sample_num = in_len / 4;

    for (size_t i = 0; i < sample_num; i++) {
        out_samples[i] = (int16_t)(in_samples[i] >> 16);
        // 可选：音量放大
        // out_samples[i] *= 2;
    }
    *out_len = sample_num * 2;
}

// ==================== 接口实现 ====================
void audio_espnow_set_peer_mac(const uint8_t mac[6])
{
    if (mac != NULL) {
        memcpy(s_peer_mac, mac, ESP_NOW_ETH_ALEN);
    }
}

esp_err_t audio_i2s_init(void)
{
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg= {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_512,
            .sample_rate_hz = CONFIG_UAC_SAMPLE_RATE,            
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
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
    
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (ret != ESP_OK) return ret;
    
    ret = i2s_channel_init_std_mode(rx_chan, &rx_std_cfg);
    if (ret != ESP_OK) return ret;
    
    ret = i2s_channel_enable(rx_chan);
    return ret;
}

esp_err_t audio_espnow_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ret = esp_now_init();
    if (ret != ESP_OK) return ret;
    
    ret = esp_now_set_pmk((uint8_t *)"1234567890123456");
    if (ret != ESP_OK) return ret;
 
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    
    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "ESP-NOW初始化完成，接收端MAC: "MACSTR, MAC2STR(s_peer_mac));
    return ESP_OK;
}

// ==================== 音频发送任务 ====================
static void audio_send_task(void *arg)
{
    uint8_t i2s_32b_buf[AUDIO_BUF_SIZE] = {0};
    int16_t audio_16b_buf[AUDIO_BUF_SIZE / 2] = {0};
    uint8_t adpcm_buf[AUDIO_BUF_SIZE / 4] = {0};
    uint8_t espnow_buf[ESPNOW_PAYLOAD_LEN] = {0};

    size_t i2s_read_len = 0;
    size_t audio_16b_len = 0;
    size_t adpcm_len = 0;
    size_t send_offset = 0;

    adpcm_state_t adpcm_enc_state;
    adpcm_init_state(&adpcm_enc_state);

    const size_t max_i2s_read = 512;

    while (1) {
        esp_err_t ret = i2s_channel_read(rx_chan, i2s_32b_buf, max_i2s_read, &i2s_read_len, portMAX_DELAY);
        if (ret != ESP_OK || i2s_read_len == 0) {
            ESP_LOGE(TAG, "读取I2S数据失败");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        convert_32to16(i2s_32b_buf, (uint8_t*)audio_16b_buf, i2s_read_len, &audio_16b_len);
        if (audio_16b_len == 0) {
            continue;
        }

        adpcm_encode(audio_16b_buf, audio_16b_len, adpcm_buf, &adpcm_len, &adpcm_enc_state);
        if (adpcm_len == 0) {
            continue;
        }
        ESP_LOGD(TAG, "ADPCM压缩: %d字节 → %d字节", audio_16b_len, adpcm_len);

        send_offset = 0;
        while (send_offset < adpcm_len) {
            size_t send_len = (adpcm_len - send_offset) > (ESPNOW_PAYLOAD_LEN - 2) ? (ESPNOW_PAYLOAD_LEN - 2) : (adpcm_len - send_offset);
            
            espnow_buf[0] = (send_len >> 8) & 0xFF;
            espnow_buf[1] = send_len & 0xFF;
            memcpy(espnow_buf + 2, adpcm_buf + send_offset, send_len);

            ret = esp_now_send(s_peer_mac, espnow_buf, 2 + send_len);
            if (ret != ESP_OK) {
                //ESP_LOGE(TAG, "ESP-NOW发送失败: %d", ret);
                vTaskDelay(2 / portTICK_PERIOD_MS);
                continue;
            }
            send_offset += send_len;

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}

esp_err_t audio_send_task_start(void)
{
    BaseType_t ret = xTaskCreate(audio_send_task, "audio_send_task", 8192, NULL, 5, NULL);
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}