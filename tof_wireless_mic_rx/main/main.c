#include <stdio.h>
#include <stdint.h>
#include <string.h>
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
// 新增：手动定义MIN/MAX宏（兜底，避免未定义错误）
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
// ==================== 配置参数 ====================
#define TAG                 "ESPNOW_AUDIO_RX"
#define ESPNOW_CHANNEL      1   // 与发送端一致
#define AUDIO_QUEUE_SIZE    30  // 音频缓存队列大小
#define ESPNOW_PAYLOAD_LEN  256 // 与发送端一致
#define USB_AUDIO_BUF_LEN   512 // USB UAC回调缓冲区大小

// 新增：带长度的音频数据结构体（解决二进制数据长度问题）
typedef struct {
    uint8_t *data;   // 音频数据缓冲区
    size_t len;      // 数据长度（关键：不再用strlen计算）
} audio_packet_t;

// 音频缓存队列（存储带长度的音频数据包）
static QueueHandle_t s_audio_queue = NULL;

// ==================== ESP-NOW接收回调 ====================
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "接收回调参数错误");
        return;
    }

    // 分配缓冲区存储音频数据和长度
    audio_packet_t *packet = malloc(sizeof(audio_packet_t));
    if (packet == NULL) {
        ESP_LOGE(TAG, "内存分配失败(packet)");
        return;
    }
    packet->data = malloc(len);
    if (packet->data == NULL) {
        ESP_LOGE(TAG, "内存分配失败(data)");
        free(packet);
        return;
    }
    
    memcpy(packet->data, data, len);
    packet->len = len;

    // 放入音频队列（满了则丢弃旧数据）
    if (xQueueSend(s_audio_queue, &packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "音频队列满，丢弃数据");
        free(packet->data);
        free(packet);
    }

    ESP_LOGD(TAG, "接收ESP-NOW数据: %d字节", len);
}

// ==================== ESP-NOW初始化 ====================
static void espnow_init()
{
    // 1. 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // 3. 初始化ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));  // 注册接收回调
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"1234567890123456"));

    // 打印自身MAC（用于配置发送端）
    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "接收端MAC地址: "MACSTR, MAC2STR(mac_addr));
}

// ==================== USB UAC音频输入回调 ====================


// 优化 uac_input_cb：增加短暂阻塞等待，避免立刻填充静音
static esp_err_t uac_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    *bytes_read = 0;
    if (buf == NULL || len == 0) return ESP_FAIL;

    size_t copied = 0;
    while (copied < len) {
        audio_packet_t *packet = NULL;
        // 从 0ms 非阻塞 → 2ms 阻塞等待，给队列一点时间填充
        if (xQueueReceive(s_audio_queue, &packet, pdMS_TO_TICKS(2)) == pdTRUE) {
            size_t to_copy = (packet->len < (len - copied)) ? packet->len : (len - copied);
            memcpy(buf + copied, packet->data, to_copy);
            copied += to_copy;
            free(packet->data);
            free(packet);
        } else {
            // 队列空时，只填充少量静音，避免突然断流
            size_t fill_len = MIN(64, len - copied);
            memset(buf + copied, 0, fill_len);
            copied += fill_len;
        }
    }
    *bytes_read = copied;
    return ESP_OK;
}
// ==================== 主函数 ====================
void app_main(void)
{
    // 1. 创建音频缓存队列（存储audio_packet_t指针）
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_packet_t *));
    if (s_audio_queue == NULL) {
        ESP_LOGE(TAG, "创建队列失败");
        abort();
    }

    // 2. 初始化ESP-NOW
    espnow_init();
    ESP_LOGI(TAG, "ESP-NOW接收端初始化完成");

    // 3. 初始化USB UAC
    uac_device_config_t uac_config = {
        .output_cb = NULL,
        .input_cb = uac_input_cb,  // 绑定音频输入回调
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };
    uac_device_init(&uac_config);
    ESP_LOGI(TAG, "USB UAC初始化完成，已作为音频设备接入电脑");
}