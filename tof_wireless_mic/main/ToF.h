#ifndef __TOF050C_H__
#define __TOF050C_H__

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"

// ===================== 硬件配置（仅改这里适配你的硬件） =====================
#define I2C_MASTER_NUM        I2C_NUM_0    // I2C端口
#define TOF_SDA_GPIO          GPIO_NUM_6   // SDA引脚
#define TOF_SCL_GPIO          GPIO_NUM_7   // SCL引脚

// ===================== 官方库宏定义 =====================
#define VL6180X_DEFAULT_I2C_ADDR 0x29 ///< 官方默认I2C地址

///! 设备模型识别号
#define VL6180X_REG_IDENTIFICATION_MODEL_ID 0x000
///! 中断配置
#define VL6180X_REG_SYSTEM_INTERRUPT_CONFIG 0x014
///! 中断清除位
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR 0x015
///! 复位状态位
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET 0x016
///! 启动测距
#define VL6180X_REG_SYSRANGE_START 0x018
///! 批次间偏移量
#define VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET 0x024
///! 启动ALS测光
#define VL6180X_REG_SYSALS_START 0x038
///! ALS增益
#define VL6180X_REG_SYSALS_ANALOGUE_GAIN 0x03F
///! ALS积分周期高字节
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI 0x040
///! ALS积分周期低字节
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO 0x041
///! 测距状态/错误码
#define VL6180X_REG_RESULT_RANGE_STATUS 0x04d
///! 中断状态
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO 0x04f
///! ALS测量值
#define VL6180X_REG_RESULT_ALS_VAL 0x050
///! 测距结果值
#define VL6180X_REG_RESULT_RANGE_VAL 0x062
///! I2C从机地址
#define VL6180X_REG_SLAVE_DEVICE_ADDRESS 0x212

///! ALS增益选项
#define VL6180X_ALS_GAIN_1 0x06    ///< 1x增益
#define VL6180X_ALS_GAIN_1_25 0x05 ///< 1.25x增益
#define VL6180X_ALS_GAIN_1_67 0x04 ///< 1.67x增益
#define VL6180X_ALS_GAIN_2_5 0x03  ///< 2.5x增益
#define VL6180X_ALS_GAIN_5 0x02    ///< 5x增益
#define VL6180X_ALS_GAIN_10 0x01   ///< 10x增益
#define VL6180X_ALS_GAIN_20 0x00   ///< 20x增益
#define VL6180X_ALS_GAIN_40 0x07   ///< 40x增益

///! 官方错误码定义
#define VL6180X_ERROR_NONE 0        ///< 成功
#define VL6180X_ERROR_SYSERR_1 1    ///< 系统错误1
#define VL6180X_ERROR_SYSERR_5 5    ///< 系统错误5
#define VL6180X_ERROR_ECEFAIL 6     ///< 早期收敛估计失败
#define VL6180X_ERROR_NOCONVERGE 7  ///< 未检测到目标
#define VL6180X_ERROR_RANGEIGNORE 8 ///< 阈值检查失败
#define VL6180X_ERROR_SNR 11        ///< 环境光过强
#define VL6180X_ERROR_RAWUFLOW 12   ///< 原始测距下溢
#define VL6180X_ERROR_RAWOFLOW 13   ///< 原始测距上溢
#define VL6180X_ERROR_RANGEUFLOW 14 ///< 测距下溢
#define VL6180X_ERROR_RANGEOFLOW 15 ///< 测距上溢

///! 官方新增寄存器（应用笔记）
#define SYSRANGE__INTERMEASUREMENT_PERIOD 0x001b ///< 连续模式测量间隔

// ===================== 优化配置参数 =====================
#define VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME 0x01C  // 最大收敛时间
#define VL6180X_REG_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x023 // 串扰补偿率
#define VL6180X_REG_SYSRANGE_CROSSTALK_VALID_HEIGHT 0x025 // 串扰有效高度
#define VL6180X_REG_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE 0x026 // 早期收敛估计
#define VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD 0x01B // 采样周期

// ===================== 错误返回值定义 =====================
#define TOF_ERROR_RETURN_VALUE 255  // 报错时返回的距离值

// ===================== 对外核心API =====================
/**
 * @brief 初始化VL6180X传感器
 * @return esp_err_t ESP_OK=成功，其他=失败
 */
esp_err_t tof050c_init(void);

/**
 * @brief 读取单次有效测距值（毫米）
 * @return uint16_t 有效距离（255=无效/错误，其他=毫米值）
 */
uint16_t tof050c_read_distance_mm(void);

 

#endif