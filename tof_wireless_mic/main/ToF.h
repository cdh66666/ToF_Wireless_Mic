#ifndef TOF_H
#define TOF_H

// 1. 引入基础标准头文件（解决uint8_t/bool/NULL未定义）
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 2. 引入ESP-IDF核心头文件
#include "esp_err.h"          // ESP_OK/esp_err_t
#include "esp_log.h"          // ESP_LOGI/ESP_LOGE
#include "esp_attr.h"         // IRAM_ATTR
#include "driver/i2c_master.h"// I2C master驱动（ESP-IDF v5.x）
#include "driver/gpio.h"      // GPIO相关
#include "freertos/FreeRTOS.h"// FreeRTOS核心
#include "freertos/task.h"    // vTaskDelay/portTICK_PERIOD_MS

// 日志TAG定义
#define TAG "VL6180X"

// I2C端口定义（根据你的硬件修改，默认I2C_NUM_0）
#define I2C_MASTER_NUM I2C_NUM_0

// ToF050c I2C地址（7位地址）
#define VL6180X_I2C_ADDR      0x29        // 7位I2C地址

// ToF050c寄存器地址
#define IDENTIFICATION__MODEL_ID                0x000
#define IDENTIFICATION__MODEL_REV_MAJOR       0x001
#define IDENTIFICATION__MODEL_REV_MINOR       0x002
#define IDENTIFICATION__MODULE_REV_MAJOR      0x003
#define IDENTIFICATION__MODULE_REV_MINOR      0x004
#define IDENTIFICATION__DATE_HI               0x006
#define IDENTIFICATION__DATE_LO               0x007
#define IDENTIFICATION__TIME                  0x008
#define SYSTEM__MODE_GPIO0                    0x010
#define SYSTEM__MODE_GPIO1                    0x011
#define SYSTEM__HISTORY_CTRL                  0x012
#define SYSTEM__INTERRUPT_CONFIG_GPIO         0x014
#define SYSTEM__INTERRUPT_CLEAR               0x015
#define SYSTEM__FRESH_OUT_OF_RESET            0x016
#define SYSTEM__GROUPED_PARAMETER_HOLD        0x017
#define SYSRANGE__START                       0x018
#define SYSRANGE__THRESH_HIGH                 0x019
#define SYSRANGE__THRESH_LOW                  0x01A
#define SYSRANGE__INTERMEASUREMENT_PERIOD     0x01B
#define SYSRANGE__MAX_CONVERGENCE_TIME        0x01C
#define SYSRANGE__CROSSTALK_COMPENSATION_RATE 0x01E
#define SYSRANGE__CROSSTALK_VALID_HEIGHT      0x021
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  0x022
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET   0x024
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   0x025
#define SYSRANGE__RANGE_IGNORE_THRESHOLD      0x026
#define SYSRANGE__MAX_AMBIENT_LEVEL_MULT      0x02C
#define SYSRANGE__RANGE_CHECK_ENABLES         0x02D
#define SYSRANGE__VHV_RECALIBRATE             0x02E
#define SYSRANGE__VHV_REPEAT_RATE             0x031
#define SYSALS__START                         0x038
#define SYSALS__THRESH_HIGH                   0x03A
#define SYSALS__THRESH_LOW                    0x03C
#define SYSALS__INTERMEASUREMENT_PERIOD       0x03E
#define SYSALS__ANALOGUE_GAIN                 0x03F
#define SYSALS__INTEGRATION_PERIOD            0x040
#define RESULT__RANGE_STATUS                  0x04D
#define RESULT__ALS_STATUS                    0x04E
#define RESULT__INTERRUPT_STATUS_GPIO         0x04F
#define RESULT__ALS_VAL                       0x050
#define RESULT__HISTORY_BUFFER_0              0x052
#define RESULT__HISTORY_BUFFER_1              0x054
#define RESULT__HISTORY_BUFFER_2              0x056
#define RESULT__HISTORY_BUFFER_3              0x058
#define RESULT__HISTORY_BUFFER_4              0x05A
#define RESULT__HISTORY_BUFFER_5              0x05C
#define RESULT__HISTORY_BUFFER_6              0x05E
#define RESULT__HISTORY_BUFFER_7              0x060
#define RESULT__RANGE_VAL                     0x062
#define RESULT__RANGE_RAW                     0x064
#define RESULT__RANGE_RETURN_RATE             0x066
#define RESULT__RANGE_REFERENCE_RATE          0x068
#define RESULT__RANGE_RETURN_SIGNAL_COUNT     0x06C
#define RESULT__RANGE_REFERENCE_SIGNAL_COUNT  0x070
#define RESULT__RANGE_RETURN_AMB_COUNT        0x074
#define RESULT__RANGE_REFERENCE_AMB_COUNT     0x078
#define RESULT__RANGE_RETURN_CONV_TIME        0x07C
#define RESULT__RANGE_REFERENCE_CONV_TIME     0x080
#define RANGE_SCALER                          0x096
#define READOUT__AVERAGING_SAMPLE_PERIOD      0x10A
#define FIRMWARE__BOOTUP                      0x119
#define FIRMWARE__RESULT_SCALER               0x120
#define I2C_SLAVE__DEVICE_ADDRESS             0x212
#define INTERLEAVED_MODE__ENABLE              0x2A3

// 函数声明（统一命名，修复拼写不一致）
uint8_t VL6180X_Start_Range(void);
uint8_t VL6180X_Poll_Range(void);
uint8_t VL6180X_Read_Range(void);  // 修正为VL6180X前缀，与定义一致
uint16_t VL6180X_ReadRangeSingleMillimeters(void);
void VL6180X_Clear_Interrupt(void);

uint8_t VL6180X_Init(void);
void VL6180X_ConfigureDefault(void);
void VL6180X_SetScaling(uint8_t new_scaling);
void VL6180X_SetTimeout(uint16_t timeout);

 

// 外部声明I2C总线句柄（供C文件使用）
extern i2c_master_bus_handle_t bsp_i2c_bus_handle;

#endif // TOF_H