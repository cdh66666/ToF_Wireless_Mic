#include "audio_wifi.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "esp_err.h"

/*
 * 若 CONFIG_WIFI_SSID/PASSWORD 未定义，则使用接收端 AP 的默认
 * SSID/密码，这样发端可以直接连接对端自带的热点。用户可在
 * 运行时调用 audio_wifi_set_sta_credentials() 覆盖。
 */
#ifndef CONFIG_WIFI_SSID
#warning "CONFIG_WIFI_SSID not defined; using receiver AP SSID"
#define CONFIG_WIFI_SSID AUDIO_WIFI_RX_SSID
#endif
#ifndef CONFIG_WIFI_PASSWORD
#warning "CONFIG_WIFI_PASSWORD not defined; using receiver AP password"
#define CONFIG_WIFI_PASSWORD AUDIO_WIFI_RX_PASSWORD
#endif
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// 全局变量
static struct sockaddr_in s_dest_addr;
static int s_sock = -1;
static uint8_t s_audio_transmission_enabled = 0;
static EventGroupHandle_t s_wifi_event_group;

// STA 模式的凭据，可在运行时调整
static char s_sta_ssid[32] = CONFIG_WIFI_SSID;
static char s_sta_password[64] = CONFIG_WIFI_PASSWORD;

// 统计数据
static uint32_t s_send_count = 0;
static uint32_t s_send_fail = 0;

// I2S receive channel handle (defined here so multiple functions can access it)
static i2s_chan_handle_t rx_chan;

/* Wi‑Fi 事件位 */
#define WIFI_CONNECTED_BIT BIT0

/* ADPCM 压缩/转换函数与之前完全相同 */

typedef struct {
    int16_t prev_sample;
    int8_t step_index;
} adpcm_state_t;

static const int16_t adpcm_step_table[] = { /* same as espnow file */
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

// convert 32‑bit to 16‑bit helper
static void convert_32to16(const uint8_t *in_32, uint8_t *out_16, size_t in_len, size_t *out_len)
{
    *out_len = 0;
    if (in_32 == NULL || out_16 == NULL || in_len < 4) return;

    const uint32_t *in_samples = (const uint32_t *)in_32;
    int16_t *out_samples = (int16_t *)out_16;
    size_t sample_num = in_len / 4;

    for (size_t i = 0; i < sample_num; i++) {
        out_samples[i] = (int16_t)(in_samples[i] >> 16);
    }
    *out_len = sample_num * 2;
}

// ==================== 接口实现 ====================
void audio_wifi_set_dest_ip(const char *ip)
{
    if (ip != NULL && strlen(ip) > 0) {
        s_dest_addr.sin_addr.s_addr = inet_addr(ip);
    }
}

void audio_wifi_set_sta_credentials(const char *ssid, const char *password)
{
    if (ssid && strlen(ssid) < sizeof(s_sta_ssid)) {
        strcpy(s_sta_ssid, ssid);
    }
    if (password && strlen(password) < sizeof(s_sta_password)) {
        strcpy(s_sta_password, password);
    }
    ESP_LOGI(AUDIO_TAG, "STA credentials set to '%s'/'%s'", s_sta_ssid, s_sta_password[0] ? "<hidden>" : "(empty)");
}

bool audio_wifi_is_connected(void)
{
    if (s_wifi_event_group == NULL) return false;
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

void audio_wifi_get_counters(uint32_t *sent, uint32_t *failed)
{
    if (sent) *sent = s_send_count;
    if (failed) *failed = s_send_fail;
}

// convert numeric disconnect reason to human‑readable string
static const char *wifi_reason_to_str(int reason)
{
    switch (reason) {
    case WIFI_REASON_UNSPECIFIED:            return "unspecified";
    case WIFI_REASON_AUTH_EXPIRE:            return "auth expire";
    case WIFI_REASON_AUTH_LEAVE:             return "auth leave";
    case WIFI_REASON_ASSOC_EXPIRE:           return "assoc expire";
    case WIFI_REASON_ASSOC_TOOMANY:          return "assoc too many";
    case WIFI_REASON_NOT_AUTHED:             return "not authed";
    case WIFI_REASON_NOT_ASSOCED:            return "not assoced";
    case WIFI_REASON_ASSOC_LEAVE:            return "assoc leave";
    case WIFI_REASON_ASSOC_NOT_AUTHED:       return "assoc not authed";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:    return "pwrcap bad";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:   return "supchan bad";
    case WIFI_REASON_IE_INVALID:             return "IE invalid";
    case WIFI_REASON_MIC_FAILURE:            return "MIC failure";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4-way handshake timeout";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:      return "group key timeout";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:     return "IE differs";
    case WIFI_REASON_GROUP_CIPHER_INVALID:   return "group cipher invalid";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:return "pairwise cipher invalid";
    case WIFI_REASON_AKMP_INVALID:           return "AKM invalid";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:  return "RSN IE version";
    case WIFI_REASON_INVALID_RSN_IE_CAP:     return "RSN IE cap";
    case WIFI_REASON_802_1X_AUTH_FAILED:     return "802.1x auth failed";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:  return "cipher rejected";
    case WIFI_REASON_BEACON_TIMEOUT:         return "beacon timeout";
    case WIFI_REASON_NO_AP_FOUND:            return "no AP found";
    case WIFI_REASON_AUTH_FAIL:              return "auth fail";
    case WIFI_REASON_ASSOC_FAIL:             return "assoc fail";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:      return "handshake timeout";
    default:                                 return "unknown";
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(AUDIO_TAG, "Wi-Fi STA start, initiating connect (ssid='%s')",
                 s_sta_ssid);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *discon = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(AUDIO_TAG, "Wi-Fi disconnected, reason %d (%s)", discon->reason,
                 wifi_reason_to_str(discon->reason));
        /* automatic reconnect with a brief pause to avoid hammering AP when auth fails */
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip_info = (ip_event_got_ip_t *)event_data;
        /* esp_ip4_addr_t is compatible with lwIP ip4_addr_t; cast to satisfy prototype */
        ESP_LOGI(AUDIO_TAG, "Got IP: %s",
                 ip4addr_ntoa((const ip4_addr_t *)&ip_info->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* 每隔几秒输出连接/统计信息，帮助判断传输是否正常 */
static void wifi_status_task(void *arg)
{
    uint32_t prev_sent = 0, prev_fail = 0;
    while (1) {
        bool conn = audio_wifi_is_connected();
        uint32_t cur_sent = s_send_count, cur_fail = s_send_fail;
        ESP_LOGI(AUDIO_TAG, "Wi-Fi %s, total sent=%u/%u failed, delta sent=%u failed=%u",
                 conn ? "connected" : "disconnected",
                 cur_sent, cur_fail,
                 cur_sent - prev_sent, cur_fail - prev_fail);
        prev_sent = cur_sent;
        prev_fail = cur_fail;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t audio_wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
            /* require WPA2-PSK only; avoids negotiation to unsupported ciphers */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, s_sta_ssid, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char *)wifi_config.sta.password, s_sta_password, sizeof(wifi_config.sta.password)-1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(AUDIO_TAG, "Connecting to SSID '%s'...", s_sta_ssid);
    /* wait for connection */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(AUDIO_TAG, "Wi-Fi connected");

    /* create UDP socket */
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        ESP_LOGE(AUDIO_TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    int broadcast = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    /* enlarge send buffer to reduce ENOMEM occurrences when network is slow */
    int sndbuf = 16384;
    setsockopt(s_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(AUDIO_WIFI_PORT);
    s_dest_addr.sin_addr.s_addr = inet_addr(AUDIO_WIFI_DEFAULT_IP);

    /* start helper task to report status periodically */
    xTaskCreate(wifi_status_task, "wifi_status_task", 4096, NULL, 5, NULL);

    ESP_LOGI(AUDIO_TAG, "UDP socket ready, port %d, destination %s", AUDIO_WIFI_PORT,
             inet_ntoa(s_dest_addr.sin_addr));
    return ESP_OK;
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

/* simple checksum for debug: sum of bytes */
    static uint32_t debug_pkt_count = 0;
    static uint32_t debug_last_sum = 0;

    static void audio_send_task(void *arg)
{
    uint8_t i2s_32b_buf[AUDIO_BUF_SIZE] = {0};
    int16_t audio_16b_buf[AUDIO_BUF_SIZE / 2] = {0};
    uint8_t adpcm_buf[AUDIO_BUF_SIZE / 4] = {0};
    uint8_t udp_buf[AUDIO_BUF_SIZE / 4 + 2] = {0};

    size_t i2s_read_len = 0;
    size_t audio_16b_len = 0;
    size_t adpcm_len = 0;
    size_t send_offset = 0;
    static uint32_t pkt_seq = 0;

    adpcm_state_t adpcm_enc_state;
    adpcm_init_state(&adpcm_enc_state);
    const size_t max_i2s_read = 512;

    while (1) {
        if (!audio_wifi_is_connected()) {
            // If not connected, skip encoding/sending to avoid memory errors
            vTaskDelay(20 / portTICK_PERIOD_MS);
            continue;
        }

        esp_err_t ret = i2s_channel_read(rx_chan, i2s_32b_buf, max_i2s_read, &i2s_read_len, portMAX_DELAY);
        if (ret != ESP_OK || i2s_read_len == 0) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        /* compute checksum when we have an i2s read before encoding */
        /* (checksum computed later on encoded data) */

        if (!s_audio_transmission_enabled) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        convert_32to16(i2s_32b_buf, (uint8_t*)audio_16b_buf, i2s_read_len, &audio_16b_len);
        if (audio_16b_len == 0) continue;

        adpcm_encode(audio_16b_buf, audio_16b_len, adpcm_buf, &adpcm_len, &adpcm_enc_state);
        if (adpcm_len == 0) continue;
        ESP_LOGD(AUDIO_TAG, "ADPCM压缩: %d字节 → %d字节", audio_16b_len, adpcm_len);

        /* debug: compute simple sum of bytes to verify nonzero content every 100 frames */
        debug_pkt_count++;
        if ((debug_pkt_count & 0x3F) == 0) { /* every 64 packets */
            uint32_t sum = 0;
            for (size_t i = 0; i < adpcm_len; i++) sum += adpcm_buf[i];
            ESP_LOGD(AUDIO_TAG, "ADPCM checksum pkt %u len %u sum %u",
                     debug_pkt_count, adpcm_len, sum);
        }

        send_offset = 0;
        const size_t max_payload = 256; /* smaller packets to avoid fragmentation */
        while (send_offset < adpcm_len) {
            size_t send_len = adpcm_len - send_offset;
            if (send_len > max_payload) send_len = max_payload;
            udp_buf[0] = (send_len >> 8) & 0xFF;
            udp_buf[1] = send_len & 0xFF;
            memcpy(udp_buf + 2, adpcm_buf + send_offset, send_len);

            if (s_sock >= 0) {
                ret = sendto(s_sock, udp_buf, send_len + 2, 0,
                             (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
                if (ret < 0) {
                    s_send_fail++;
                    if (errno == ENOMEM) {
                        ESP_LOGW(AUDIO_TAG, "UDP send failed ENOMEM, pausing");
                        vTaskDelay(pdMS_TO_TICKS(5));
                    } else {
                        ESP_LOGW(AUDIO_TAG, "UDP send failed: errno %d", errno);
                    }
                } else {
                    s_send_count++;
                }
                pkt_seq++;
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

void audio_set_transmission_enabled(uint8_t enable)
{
    s_audio_transmission_enabled = (enable != 0) ? 1 : 0;
    ESP_LOGI(AUDIO_TAG, "音频传输%s", s_audio_transmission_enabled ? "已启用" : "已禁用");
}

uint8_t audio_get_transmission_enabled(void)
{
    return s_audio_transmission_enabled;
}
