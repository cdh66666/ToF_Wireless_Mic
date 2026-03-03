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

/* socket headers */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// 手动定义MIN/MAX宏
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// ==================== ADPCM 解压核心模块（保留不变）====================
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
    *out_len = out_idx; // 修复：返回正确的采样数
}

// ==================== 配置参数（修改AP相关）====================
#define TAG                 "UDP_AUDIO_RX_AP"
#define AUDIO_QUEUE_SIZE    100  // 音频缓存队列大小
#define USB_AUDIO_BUF_LEN   2048 // USB UAC回调缓冲区大小

// AP热点配置（可自定义）
#define AP_SSID       "ESP32_AUDIO_AP"  // 热点名称
#define AP_PASSWORD   "12345678"        // 热点密码（至少8位）
#define AP_CHANNEL    1                 // 热点信道
#define AP_MAX_STA    4                 // 最大连接数

// 带长度的音频数据结构体
typedef struct {
    uint8_t *data;
    size_t len;
} audio_packet_t;

// 音频缓存队列
static QueueHandle_t s_audio_queue = NULL;

/* AP + UDP 接收 */
#define AUDIO_UDP_PORT    12345
static int s_udp_sock = -1;

// ==================== WiFi AP 初始化（核心修改）====================
static esp_err_t wifi_ap_init(void)
{
    // 1. 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 3. 创建默认AP接口（关键：替代原来的STA接口）
    esp_netif_create_default_wifi_ap();

    // 4. 初始化WiFi驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 配置AP参数
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASSWORD,
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_STA,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK, // 加密方式
            .pmf_cfg = {
                .required = false
            }
        },
    };
    // 如果密码为空，设置为开放热点
    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // 6. 设置WiFi模式为AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    // 7. 配置AP参数
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    // 8. 启动WiFi AP
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP启动成功");
    ESP_LOGI(TAG, "热点名称: %s", AP_SSID);
    ESP_LOGI(TAG, "热点密码: %s", strlen(AP_PASSWORD) > 0 ? AP_PASSWORD : "无");
    ESP_LOGI(TAG, "ESP32 AP IP地址: 192.168.4.1"); // AP模式默认IP

    return ESP_OK;
}

// ==================== UDP 初始化（绑定AP接口）====================
static esp_err_t udp_server_init(void)
{
    // 1. 创建UDP套接字
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "创建UDP套接字失败: errno %d", errno);
        return ESP_FAIL;
    }

    // 2. 设置套接字可重用（避免端口占用）
    int opt = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 3. 绑定到AP的IP和端口（关键：INADDR_ANY表示监听所有接口）
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY), // 监听所有连接的IP
        .sin_port = htons(AUDIO_UDP_PORT),
    };
    if (bind(s_udp_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "绑定UDP端口失败: errno %d", errno);
        close(s_udp_sock);
        s_udp_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP服务器启动成功，监听端口: %d", AUDIO_UDP_PORT);
    return ESP_OK;
}

// ==================== UDP 接收任务（保留逻辑，优化栈大小）====================
static void udp_recv_task(void *arg)
{
    uint8_t buf[1024];
    ESP_LOGI(TAG, "UDP接收任务启动");

    while (1) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        // 接收UDP数据
        int len = recvfrom(s_udp_sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&from, &fromlen);
        if (len <= 0) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        // 打印发送端信息（调试用）
        ESP_LOGD(TAG, "收到来自 %s:%d 的数据，长度: %d",
                 inet_ntoa(from.sin_addr), ntohs(from.sin_port), len);

        // 解析ADPCM数据（前2字节为长度）
        if (len <= 2) continue;
        size_t adpcm_len = (buf[0] << 8) | buf[1];
        if (adpcm_len > (size_t)(len - 2)) continue;
        uint8_t *adpcm_data = buf + 2;

        // 分配内存缓存数据
        audio_packet_t *packet = malloc(sizeof(audio_packet_t));
        if (!packet) continue;
        packet->data = malloc(adpcm_len);
        if (!packet->data) { 
            free(packet); 
            continue; 
        }
        memcpy(packet->data, adpcm_data, adpcm_len);
        packet->len = adpcm_len;

        // 放入队列（非阻塞）
        if (xQueueSend(s_audio_queue, &packet, 0) != pdTRUE) {
            ESP_LOGW(TAG, "音频队列满，丢弃数据");
            free(packet->data);
            free(packet);
        }
    }
}

// ==================== USB UAC音频输出回调（修复解码长度）====================
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    static adpcm_state_t adpcm_dec_state;
    static uint8_t adpcm_dec_buf[1024] = {0};
    static size_t adpcm_dec_buf_len = 0;

    *bytes_read = 0;
    if (buf == NULL || len == 0) return ESP_FAIL;

    int16_t *pcm_buf = (int16_t *)buf;
    size_t pcm_sample_needed = len / 2; // 需要的PCM采样数（16位）
    size_t pcm_sample_copied = 0;

    // 初始化解码状态
    static bool adpcm_dec_init = false;
    if (!adpcm_dec_init) {
        adpcm_init_state(&adpcm_dec_state);
        adpcm_dec_init = true;
    }

    // 填充PCM数据
    while (pcm_sample_copied < pcm_sample_needed) {
        if (adpcm_dec_buf_len > 0) {
            size_t dec_len = 0;
            size_t dec_sample_max = pcm_sample_needed - pcm_sample_copied;
            size_t dec_adpcm_max = (dec_sample_max + 1) / 2; // 1字节ADPCM=2个采样
            
            if (dec_adpcm_max > adpcm_dec_buf_len) {
                dec_adpcm_max = adpcm_dec_buf_len;
            }

            // 解码ADPCM为PCM
            adpcm_decode(adpcm_dec_buf, dec_adpcm_max, pcm_buf + pcm_sample_copied, &dec_len, &adpcm_dec_state);
            size_t dec_sample = dec_len; // 修复：直接使用解码后的采样数

            if (dec_sample > 0) {
                pcm_sample_copied += dec_sample;
                adpcm_dec_buf_len -= dec_adpcm_max;

                // 移动剩余数据
                if (adpcm_dec_buf_len > 0) {
                    memmove(adpcm_dec_buf, adpcm_dec_buf + dec_adpcm_max, adpcm_dec_buf_len);
                }
                continue;
            }
        }

        // 从队列读取ADPCM数据
        audio_packet_t *packet = NULL;
        if (xQueueReceive(s_audio_queue, &packet, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (adpcm_dec_buf_len + packet->len < sizeof(adpcm_dec_buf)) {
                memcpy(adpcm_dec_buf + adpcm_dec_buf_len, packet->data, packet->len);
                adpcm_dec_buf_len += packet->len;
            } else {
                ESP_LOGW(TAG, "解码缓冲区满，丢弃部分ADPCM数据");
            }
            // 释放内存
            free(packet->data);
            free(packet);
        } else {
            // 无数据时填充静音
            size_t fill_sample = MIN(32, pcm_sample_needed - pcm_sample_copied);
            memset(pcm_buf + pcm_sample_copied, 0, fill_sample * 2);
            pcm_sample_copied += fill_sample;
        }
    }

    *bytes_read = pcm_sample_copied * 2;
    return ESP_OK;
}

// ==================== 主函数（调整初始化顺序）====================
void app_main(void)
{
    // 1. 创建音频队列
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_packet_t *));
    if (s_audio_queue == NULL) {
        ESP_LOGE(TAG, "创建音频队列失败");
        abort();
    }

    // 2. 初始化WiFi AP
    if (wifi_ap_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP初始化失败");
        abort();
    }

    // 3. 初始化UDP服务器
    if (udp_server_init() != ESP_OK) {
        ESP_LOGE(TAG, "UDP服务器初始化失败");
        abort();
    }

    // 4. 启动UDP接收任务（栈大小调整为8192）
    xTaskCreate(udp_recv_task, "udp_recv_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "UDP接收任务启动");

    // 5. 初始化USB UAC
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