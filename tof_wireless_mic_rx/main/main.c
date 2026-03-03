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
#include "esp_system.h" // 获取CPU和内存信息

// 手动定义MIN/MAX宏
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// ==================== ADPCM 解压核心模块（优化效率） ====================
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

// 优化后的ADPCM解码函数（减少CPU占用，提升效率）
static void adpcm_decode(const uint8_t *in, size_t in_len, int16_t *out, size_t *out_len, adpcm_state_t *state) {
    if (in == NULL || out == NULL || state == NULL) {
        *out_len = 0;
        return;
    }

    size_t out_idx = 0;
    int16_t prev_sample = state->prev_sample;
    int8_t step_index = state->step_index;

    // 提前限制step_index，避免循环内重复判断
    step_index = step_index < 0 ? 0 : (step_index > 88 ? 88 : step_index);

    for (size_t i = 0; i < in_len; i++) {
        uint8_t byte = in[i];
        uint8_t nibble1 = (byte >> 4) & 0x0F;
        uint8_t nibble2 = byte & 0x0F;

        // 解码nibble1（优化重复计算）
        int16_t step = adpcm_step_table[step_index];
        int16_t delta_val = step >> 3;
        if (nibble1 & 4) delta_val += step;
        if (nibble1 & 2) delta_val += step >> 1;
        if (nibble1 & 1) delta_val += step >> 2;
        prev_sample += (nibble1 & 8) ? -delta_val : delta_val;
        out[out_idx++] = prev_sample;
        
        step_index += adpcm_index_table[nibble1];
        step_index = step_index < 0 ? 0 : (step_index > 88 ? 88 : step_index);

        // 解码nibble2（优化重复计算）
        step = adpcm_step_table[step_index];
        delta_val = step >> 3;
        if (nibble2 & 4) delta_val += step;
        if (nibble2 & 2) delta_val += step >> 1;
        if (nibble2 & 1) delta_val += step >> 2;
        prev_sample += (nibble2 & 8) ? -delta_val : delta_val;
        out[out_idx++] = prev_sample;
        
        step_index += adpcm_index_table[nibble2];
        step_index = step_index < 0 ? 0 : (step_index > 88 ? 88 : step_index);
    }

    state->prev_sample = prev_sample;
    state->step_index = step_index;
    *out_len = out_idx * 2;
}

// ==================== 配置参数（关键优化） ====================
#define TAG                 "ESPNOW_AUDIO_RX"
#define ESPNOW_CHANNEL      6   // 与发送端一致
#define AUDIO_QUEUE_SIZE    200  // 队列大小从100增大到200，避免满队列
#define ESPNOW_PAYLOAD_LEN  250 // 与发送端一致
#define USB_AUDIO_BUF_LEN   2048 // USB UAC回调缓冲区大小
#define MONITOR_INTERVAL_MS 1000 // 每1秒打印一次监控信息
#define ADPCM_DEC_BUF_SIZE  4096 // 解码缓冲区从1024增大到4096字节
#define MAX_FRAME_ID        65535 // 帧ID最大值（循环自增上限，和发送端匹配）

// 带长度的音频数据结构体
typedef struct {
    uint8_t *data;
    size_t len;
} audio_packet_t;

// 音频缓存队列
static QueueHandle_t s_audio_queue = NULL;

// ==================== 新增：监控统计变量 ====================
static uint32_t s_total_recv_packets = 0;        // 累计接收包数
static uint32_t s_total_recv_bytes = 0;          // 累计接收字节数
static uint32_t s_lost_packets = 0;              // 累计丢包数
static uint32_t s_invalid_packets = 0;           // 累计无效包数
static uint32_t s_queue_full_discard = 0;        // 队列满丢弃数
static uint32_t s_last_frame_id = 0;             // 上一帧ID（用于丢包检测）
static uint32_t s_last_monitor_time = 0;         // 上一次统计时间（ms）
static uint32_t s_decode_empty_fill = 0;         // 解码时填充空数据次数

// ==================== 新增：打印监控统计信息 ====================
static void print_recv_monitor_stats(void)
{
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - s_last_monitor_time < MONITOR_INTERVAL_MS) {
        return;
    }

    // 计算关键指标
    float recv_speed = (float)s_total_recv_bytes / (MONITOR_INTERVAL_MS / 1000.0f) / 1024.0f; // KB/s
    float loss_rate = (s_total_recv_packets + s_lost_packets > 0) ? 
                     (float)s_lost_packets / (s_total_recv_packets + s_lost_packets) * 100 : 0;
    uint32_t free_heap = esp_get_free_heap_size();
    /* cpu usage API not available in this IDF version, use placeholder */
    uint8_t cpu_usage = 0;
    uint32_t queue_size = uxQueueMessagesWaiting(s_audio_queue); // 当前队列中的包数

    // 打印统计信息（核心监控）
    ESP_LOGI(TAG, "========== 接收端监控统计 (每%d秒) ==========", MONITOR_INTERVAL_MS/1000);
    ESP_LOGI(TAG, "📊 传输状态：接收速率=%.2f KB/s | 累计接收包=%lu | 队列当前包数=%lu",
             recv_speed, s_total_recv_packets, queue_size);
    ESP_LOGI(TAG, "⚠️  异常统计：丢包数=%lu | 丢包率=%.1f%% | 无效包=%lu | 队列丢弃=%lu",
             s_lost_packets, loss_rate, s_invalid_packets, s_queue_full_discard);
    ESP_LOGI(TAG, "🔊 解码状态：填充空数据次数=%lu | 剩余堆内存=%lu B | CPU占用率=%d%%",
             s_decode_empty_fill, free_heap, cpu_usage);
    ESP_LOGI(TAG, "===========================================\n");

    // 重置统计（保留累计丢包/无效包，用于长期监控）
    s_total_recv_packets = 0;
    s_total_recv_bytes = 0;
    s_queue_full_discard = 0;
    s_decode_empty_fill = 0;
    s_last_monitor_time = current_time;
}

// ==================== ESP-NOW接收回调（修复队列发送+帧ID溢出） ====================
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // 1. 基础参数校验（适配发送端的帧ID格式：3字节帧ID + 2字节长度）
    if (recv_info == NULL || data == NULL || len <= 5) {
        s_invalid_packets++;
        ESP_LOGE(TAG, "❌ 接收参数错误/数据过短！长度：%d | 累计无效包：%lu", len, s_invalid_packets);
        return;
    }

    // 2. 解析帧ID（修复整数溢出导致的假丢包）
    uint32_t current_frame_id = (data[0] << 16) | (data[1] << 8) | data[2];
    if (s_last_frame_id != 0) {
        uint32_t lost = 0;
        if (current_frame_id > s_last_frame_id) {
            // 正常递增：当前帧ID > 上一帧ID
            lost = current_frame_id - s_last_frame_id - 1;
        } else if (current_frame_id < s_last_frame_id && s_last_frame_id < MAX_FRAME_ID) {
            // 未到最大值却变小，是真丢包
            lost = current_frame_id + (MAX_FRAME_ID - s_last_frame_id) - 1;
        }
        // 到最大值后回零（current_frame_id < s_last_frame_id 且 s_last_frame_id >= MAX_FRAME_ID），lost=0
        
        if (lost > 0 && lost < 100) { // 过滤超大丢包数（溢出值）
            s_lost_packets += lost;
            ESP_LOGE(TAG, "❌ 检测到丢包！丢失帧数：%lu | 当前帧ID：%lu | 上一帧ID：%lu",
                     lost, current_frame_id, s_last_frame_id);
        } else if (lost > 100) {
            // 超大丢包数，判定为帧ID回零，仅打印调试信息
            ESP_LOGD(TAG, "📌 帧ID循环：上一帧ID=%lu → 当前帧ID=%lu（非丢包）",
                     s_last_frame_id, current_frame_id);
        }
    }
    s_last_frame_id = current_frame_id;

    // 3. 解析ADPCM数据长度（位置从data[3-4]开始，适配发送端）
    size_t adpcm_data_len = (data[3] << 8) | data[4];
    if (adpcm_data_len > len - 5 || adpcm_data_len == 0) { // 数据长度异常
        s_invalid_packets++;
        ESP_LOGE(TAG, "❌ 数据长度异常！解析长度：%lu | 实际剩余长度：%d | 累计无效包：%lu",
                 adpcm_data_len, len - 5, s_invalid_packets);
        return;
    }
    const uint8_t *adpcm_data = data + 5;

    // 4. 分配内存存储数据包
    audio_packet_t *packet = malloc(sizeof(audio_packet_t));
    if (packet == NULL) {
        s_invalid_packets++;
        ESP_LOGE(TAG, "❌ packet内存分配失败！剩余堆内存：%lu B | 累计无效包：%lu",
                 esp_get_free_heap_size(), s_invalid_packets);
        return;
    }
    packet->data = malloc(adpcm_data_len);
    if (packet->data == NULL) {
        s_invalid_packets++;
        free(packet);
        ESP_LOGE(TAG, "❌ data内存分配失败！剩余堆内存：%lu B | 累计无效包：%lu",
                 esp_get_free_heap_size(), s_invalid_packets);
        return;
    }

    // 5. 拷贝数据并写入队列（关键修复）
    memcpy(packet->data, adpcm_data, adpcm_data_len);
    packet->len = adpcm_data_len;

    // 修复1：超时从0改为5ms，增加队列写入成功率
    // 修复2：确保发送的是指针本身（队列元素大小匹配）
    if (xQueueSend(s_audio_queue, &packet, pdMS_TO_TICKS(5)) != pdTRUE) {
        s_queue_full_discard++;
        free(packet->data);
        free(packet);
        ESP_LOGW(TAG, "⚠️  音频队列满，丢弃数据包！帧ID：%lu | 累计丢弃：%lu",
                 current_frame_id, s_queue_full_discard);
    } else {
        // 6. 更新接收统计
        s_total_recv_packets++;
        s_total_recv_bytes += adpcm_data_len;
        ESP_LOGD(TAG, "✅ 接收成功 | 帧ID：%lu | 数据长度：%lu B | 累计接收包：%lu",
                 current_frame_id, adpcm_data_len, s_total_recv_packets);
    }

    // 7. 定期打印监控信息
    print_recv_monitor_stats();
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

    // 初始化监控时间
    s_last_monitor_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "接收端MAC地址: "MACSTR, MAC2STR(mac_addr));
}

// ==================== USB UAC音频输入回调（核心优化） ====================
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    static adpcm_state_t adpcm_dec_state;
    static uint8_t adpcm_dec_buf[ADPCM_DEC_BUF_SIZE] = {0}; // 扩容到4096字节
    static size_t adpcm_dec_buf_len = 0;

    *bytes_read = 0;
    if (buf == NULL || len == 0) return ESP_FAIL;

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
        // 关键修复：超时从2ms增大到10ms，减少空数据填充
        if (xQueueReceive(s_audio_queue, &packet, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (adpcm_dec_buf_len + packet->len < sizeof(adpcm_dec_buf)) {
                memcpy(adpcm_dec_buf + adpcm_dec_buf_len, packet->data, packet->len);
                adpcm_dec_buf_len += packet->len;
            } else {
                ESP_LOGW(TAG, "⚠️  解码缓冲区满，丢弃部分ADPCM数据");
            }

            free(packet->data);
            free(packet);
        } else {
            // 统计填充空数据的次数（说明接收数据不足）
            s_decode_empty_fill++;
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
    // 队列元素大小为audio_packet_t*（指针），和存入的类型严格匹配
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