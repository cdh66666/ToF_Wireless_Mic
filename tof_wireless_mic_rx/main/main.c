#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "usb_device_uac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

// 手动定义MIN/MAX宏
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// ==================== ADPCM 解压核心模块 ====================
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

static void adpcm_decode(const uint8_t *in, size_t in_len, int16_t *out, size_t *out_len, adpcm_state_t *state) {
    if (in == NULL || out == NULL || state == NULL) {
        *out_len = 0;
        return;
    }

    size_t out_idx = 0;
    int16_t prev_sample = state->prev_sample;
    int8_t step_index = state->step_index;

    if (step_index < 0) step_index = 0;
    if (step_index > 88) step_index = 88;

    for (size_t i = 0; i < in_len; i++) {
        uint8_t byte = in[i];
        uint8_t nibble1 = (byte >> 4) & 0x0F;
        uint8_t nibble2 = byte & 0x0F;

        int16_t step = adpcm_step_table[step_index];
        int8_t sign = nibble1 & 8;
        int8_t delta = nibble1 & 7;
        int16_t delta_val = step >> 3;
        if (delta & 4) delta_val += step;
        if (delta & 2) delta_val += step >> 1;
        if (delta & 1) delta_val += step >> 2;
        if (sign) prev_sample -= delta_val;
        else prev_sample += delta_val;

 
        out[out_idx++] = prev_sample;

        step_index += adpcm_index_table[nibble1];
        if (step_index < 0) step_index = 0;
        if (step_index > 88) step_index = 88;

        step = adpcm_step_table[step_index];
        sign = nibble2 & 8;
        delta = nibble2 & 7;
        delta_val = step >> 3;
        if (delta & 4) delta_val += step;
        if (delta & 2) delta_val += step >> 1;
        if (delta & 1) delta_val += step >> 2;
        if (sign) prev_sample -= delta_val;
        else prev_sample += delta_val;

 
        out[out_idx++] = prev_sample;

        step_index += adpcm_index_table[nibble2];
        if (step_index < 0) step_index = 0;
        if (step_index > 88) step_index = 88;
    }

    state->prev_sample = prev_sample;
    state->step_index = step_index;
    *out_len = out_idx * 2;
}

// ==================== 配置参数 ====================
#define TAG                 "ESPNOW_AUDIO_RX"
#define ESPNOW_CHANNEL      1   // 与发送端一致
#define AUDIO_QUEUE_SIZE    100  // 音频缓存队列大小
#define ESPNOW_PAYLOAD_LEN  250 // 与发送端一致
#define USB_AUDIO_BUF_LEN   2048 // USB UAC回调缓冲区大小

// 带长度的音频数据结构体
typedef struct {
    uint8_t *data;
    size_t len;
} audio_packet_t;

// 音频缓存队列
static QueueHandle_t s_audio_queue = NULL;

// ==================== ESP-NOW接收回调 ====================
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 2) {
        ESP_LOGE(TAG, "接收回调参数错误/数据过短");
        return;
    }

    size_t adpcm_data_len = (data[0] << 8) | data[1];
    if (adpcm_data_len > len - 2) {
        ESP_LOGE(TAG, "数据长度异常: %d > %d", adpcm_data_len, len - 2);
        return;
    }
    const uint8_t *adpcm_data = data + 2;

    audio_packet_t *packet = malloc(sizeof(audio_packet_t));
    if (packet == NULL) {
        ESP_LOGE(TAG, "内存分配失败(packet)");
        return;
    }
    packet->data = malloc(adpcm_data_len);
    if (packet->data == NULL) {
        ESP_LOGE(TAG, "内存分配失败(data)");
        free(packet);
        return;
    }
    
    memcpy(packet->data, adpcm_data, adpcm_data_len);
    packet->len = adpcm_data_len;

    if (xQueueSend(s_audio_queue, &packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "音频队列满，丢弃数据");
        free(packet->data);
        free(packet);
    }

    ESP_LOGD(TAG, "接收ESP-NOW ADPCM数据: %d字节", adpcm_data_len);
}

// ==================== ESP-NOW初始化 ====================
static void espnow_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"1234567890123456"));

    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "接收端MAC地址: "MACSTR, MAC2STR(mac_addr));
}

// ==================== USB UAC音频输入回调 ====================
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    static adpcm_state_t adpcm_dec_state;
    static uint8_t adpcm_dec_buf[1024] = {0};
    static size_t adpcm_dec_buf_len = 0;

    *bytes_read = 0;
    if (buf == NULL || len == 0) return ESP_FAIL;

    size_t copied = 0;
    int16_t *pcm_buf = (int16_t *)buf;
    size_t pcm_sample_needed = len / 2;
    size_t pcm_sample_copied = 0;

    static bool adpcm_dec_init = false;
    if (!adpcm_dec_init) {
        adpcm_init_state(&adpcm_dec_state);
        adpcm_dec_init = true;
    }

    while (pcm_sample_copied < pcm_sample_needed) {
        if (adpcm_dec_buf_len > 0) {
            size_t dec_len = 0;
            size_t dec_sample_max = pcm_sample_needed - pcm_sample_copied;
            size_t dec_adpcm_max = (dec_sample_max + 1) / 2;
            
            if (dec_adpcm_max > adpcm_dec_buf_len) {
                dec_adpcm_max = adpcm_dec_buf_len;
            }

            adpcm_decode(adpcm_dec_buf, dec_adpcm_max, pcm_buf + pcm_sample_copied, &dec_len, &adpcm_dec_state);
            size_t dec_sample = dec_len / 2;

            if (dec_sample > 0) {
                pcm_sample_copied += dec_sample;
                adpcm_dec_buf_len -= dec_adpcm_max;

                if (adpcm_dec_buf_len > 0) {
                    memmove(adpcm_dec_buf, adpcm_dec_buf + dec_adpcm_max, adpcm_dec_buf_len);
                }
                continue;
            }
        }

        audio_packet_t *packet = NULL;
        if (xQueueReceive(s_audio_queue, &packet, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (adpcm_dec_buf_len + packet->len < sizeof(adpcm_dec_buf)) {
                memcpy(adpcm_dec_buf + adpcm_dec_buf_len, packet->data, packet->len);
                adpcm_dec_buf_len += packet->len;
            } else {
                ESP_LOGW(TAG, "解码缓冲区满，丢弃部分ADPCM数据");
            }

            free(packet->data);
            free(packet);
        } else {
            size_t fill_sample = MIN(32, pcm_sample_needed - pcm_sample_copied);
            memset(pcm_buf + pcm_sample_copied, 0, fill_sample * 2);
            pcm_sample_copied += fill_sample;
        }
    }

    *bytes_read = pcm_sample_copied * 2;
    return ESP_OK;
}

// ==================== 主函数 ====================
void app_main(void)
{
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_packet_t *));
    if (s_audio_queue == NULL) {
        ESP_LOGE(TAG, "创建队列失败");
        abort();
    }

    espnow_init();
    ESP_LOGI(TAG, "ESP-NOW接收端初始化完成");

    uac_device_config_t uac_config = {
        .output_cb = NULL,
        .input_cb = uac_input_cb,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };
    uac_device_init(&uac_config);
    ESP_LOGI(TAG, "USB UAC初始化完成，已作为音频设备接入电脑");
}