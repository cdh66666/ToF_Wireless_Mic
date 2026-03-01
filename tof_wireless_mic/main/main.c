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

// ==================== 配置参数 ====================
#define TAG                 "ESPNOW_AUDIO_TX"
#define MIC_I2S_PORT        I2S_NUM_0
#define ESPNOW_CHANNEL      1   // ESP-NOW信道（接收端需一致）
#define AUDIO_BUF_SIZE      1024 // 音频缓冲区大小（字节）
#define ESPNOW_PAYLOAD_LEN  256  // ESP-NOW单包数据长度（<=250）

// 接收端MAC地址（替换为你的接收端实际MAC！）
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0x98, 0xa3, 0x16, 0xf0, 0xb4, 0x34};
// I2S句柄
static i2s_chan_handle_t rx_chan;

// ==================== 工具函数 ====================
// 32bit转16bit（适配INMP441和USB UAC）
static void convert_32to16(const uint8_t *in_32, uint8_t *out_16, size_t in_len, size_t *out_len)
{
    *out_len = 0;
    if (in_32 == NULL || out_16 == NULL || in_len < 4) return;

    const uint32_t *in_samples = (const uint32_t *)in_32;
    int16_t *out_samples = (int16_t *)out_16;
    size_t sample_num = in_len / 4;  // 32bit=4字节/样本

    // 提取高16位（INMP441有效数据）
    for (size_t i = 0; i < sample_num; i++) {
        out_samples[i] = (int16_t)(in_samples[i] >> 16);
        // 可选：音量放大（声音小时取消注释）
        // out_samples[i] *= 2;
    }
    *out_len = sample_num * 2;  // 16bit=2字节/样本
}

// ==================== I2S初始化（复用你的代码） ====================
static void i2s_init()
{
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg= {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_512,
            .sample_rate_hz = 16000,            
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
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // STA模式即可
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // 3. 初始化ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"1234567890123456"));  // 加密主密钥（可选）
 
    // 4. 添加接收端为对等节点
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;  // 简化测试，先不加密
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "添加对等节点失败！");
        abort();
    }

    ESP_LOGI(TAG, "ESP-NOW初始化完成，接收端MAC: "MACSTR, MAC2STR(s_peer_mac));
}

// ==================== 音频发送任务 ====================
static void audio_send_task(void *arg)
{
    uint8_t i2s_32b_buf[AUDIO_BUF_SIZE] = {0};
    uint8_t audio_16b_buf[AUDIO_BUF_SIZE / 2] = {0};
    uint8_t espnow_buf[ESPNOW_PAYLOAD_LEN] = {0};

    size_t i2s_read_len = 0;
    size_t audio_16b_len = 0;
    size_t send_offset = 0;

    // 新增：限制I2S读取量，避免一次读太多导致发送过载
    const size_t max_i2s_read = 512; // 44.1kHz下，512字节≈5.8ms音频

    while (1) {
        // 1. 限制单次I2S读取长度（不要读满AUDIO_BUF_SIZE）
        esp_err_t ret = i2s_channel_read(rx_chan, i2s_32b_buf, max_i2s_read, &i2s_read_len, portMAX_DELAY);
        if (ret != ESP_OK || i2s_read_len == 0) {
            ESP_LOGE(TAG, "读取I2S数据失败");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // 2. 转换为16bit音频数据
        convert_32to16(i2s_32b_buf, audio_16b_buf, i2s_read_len, &audio_16b_len);
        if (audio_16b_len == 0) {
            continue;
        }

        // 3. 分块发送（ESP-NOW单包最大250字节）
        send_offset = 0;
        while (send_offset < audio_16b_len) {
            size_t send_len = (audio_16b_len - send_offset) > ESPNOW_PAYLOAD_LEN ? ESPNOW_PAYLOAD_LEN : (audio_16b_len - send_offset);
            memcpy(espnow_buf, audio_16b_buf + send_offset, send_len);

            // 发送ESP-NOW数据包，失败则短暂等待后重试
            ret = esp_now_send(s_peer_mac, espnow_buf, send_len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ESP-NOW发送失败: %d", ret);
                vTaskDelay(2 / portTICK_PERIOD_MS); // 失败后等待2ms再试
                continue;
            }
            send_offset += send_len;

            // 新增：每包之间加微小延时，避免发送太快
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}

// ==================== 主函数 ====================
void app_main(void)
{
    // 初始化I2S（采集音频）
    i2s_init();
    ESP_LOGI(TAG, "I2S初始化完成");

    // 初始化ESP-NOW（无线发送）
    espnow_init();
    ESP_LOGI(TAG, "ESP-NOW初始化完成");

    // 创建音频发送任务（高优先级）
    xTaskCreate(audio_send_task, "audio_send_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "音频发送任务启动");
}