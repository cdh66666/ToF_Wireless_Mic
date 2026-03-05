#include <stdio.h>
#include "esp_err.h"          // 错误处理
#include "esp_system.h"       // esp_read_mac 所需头文件
#include "esp_efuse.h"        // esp_efuse_mac_get_default 所需头文件
#include "esp_mac.h"          // ESP_MAC_WIFI_STA 宏定义所需头文件

void app_main(void)
{
    // 定义存储MAC地址的数组
    uint8_t mac_addr[6];
    
    // 方式1：获取WiFi STA模式的MAC地址（最常用）
    esp_err_t err = esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    if (err == ESP_OK) {
        printf("WiFi STA MAC地址: %02x:%02x:%02x:%02x:%02x:%02x\n",
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        printf("读取WiFi STA MAC失败 (错误码: %d)\n", err);
    }

    // 方式2：获取出厂默认MAC（EFUSE中固化）
    err = esp_efuse_mac_get_default(mac_addr);
    if (err == ESP_OK) {
        printf("出厂默认MAC地址: %02x:%02x:%02x:%02x:%02x:%02x\n",
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        printf("读取出厂默认MAC失败 (错误码: %d)\n", err);
    }

 
}