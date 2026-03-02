#include "Tof.h"

// 静态全局变量（I2C设备句柄+参数）
static i2c_master_dev_handle_t tof050c_dev_handle = NULL;
static i2c_port_t i2c_port = I2C_MASTER_NUM;
static uint8_t ptp_offset = 0;
static uint8_t scaling = 0;
static uint16_t io_timeout = 2;
static uint16_t timeoutcnt = 0;  // 轮询超时计数器

 
// I2C总线句柄定义（全局）
i2c_master_bus_handle_t bsp_i2c_bus_handle = NULL;

// 辅助函数：将16位寄存器地址拆分为高字节和低字节
static void split_register_address(uint16_t reg, uint8_t *high_byte, uint8_t *low_byte)
{
    *high_byte = (reg >> 8) & 0xFF;
    *low_byte = reg & 0xFF;
}

// 写1字节数据到指定寄存器
esp_err_t VL6180X_WR_CMD(uint16_t reg, uint8_t data)
{
    esp_err_t ret = ESP_OK;
    uint8_t high_byte, low_byte;
    split_register_address(reg, &high_byte, &low_byte);

    // 准备写入数据：地址高字节 + 地址低字节 + 数据
    uint8_t write_buf[3] = {high_byte, low_byte, data};

    // 执行I2C写入操作（超时1000ms）
    ret = i2c_master_transmit(tof050c_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command to register 0x%04X: %d", reg, ret);
    }
    return ret;
}

// 写2字节数据到指定寄存器
esp_err_t VL6180X_WR_CMD2(uint16_t reg, uint16_t data)
{
    esp_err_t ret = ESP_OK;
    uint8_t high_byte, low_byte;
    split_register_address(reg, &high_byte, &low_byte);

    // 准备写入数据：地址高字节 + 地址低字节 + 数据高字节 + 数据低字节
    uint8_t write_buf[4] = {high_byte, low_byte, (data >> 8) & 0xFF, data & 0xFF};

    // 执行I2C写入操作
    ret = i2c_master_transmit(tof050c_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command2 to register 0x%04X: %d", reg, ret);
    }
    return ret;
}

// 从指定寄存器读取1字节数据
uint8_t VL6180X_ReadByte(uint16_t reg)
{
    esp_err_t ret = ESP_OK;
    uint8_t high_byte, low_byte;
    split_register_address(reg, &high_byte, &low_byte);

    // 第一步：发送寄存器地址（写操作）
    uint8_t write_buf[2] = {high_byte, low_byte};
    ret = i2c_master_transmit(tof050c_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send register address 0x%04X: %d", reg, ret);
        return 0;
    }

    // 第二步：读取数据（读操作）
    uint8_t read_data = 0;
    ret = i2c_master_receive(tof050c_dev_handle, &read_data, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from register 0x%04X: %d", reg, ret);
        return 0;
    }

    return read_data;
}

/**
 * @brief 初始化VL6180X传感器
 * @return uint8_t 1表示成功完成初始化(检测到复位状态)，0表示无需初始化
 */
uint8_t VL6180X_Init()
{
    // 初始化I2C总线（如果未初始化）
    if (bsp_i2c_bus_handle == NULL) {
        i2c_master_bus_config_t i2c_bus_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_port,
            .sda_io_num = GPIO_NUM_4,  // 替换为你的SDA引脚
            .scl_io_num = GPIO_NUM_5,  // 替换为你的SCL引脚
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true, // 启用内部上拉
        };
        esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bsp_i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C master bus: %d", ret);
            return 0;
        }
    }

    // 创建I2C设备配置
    i2c_device_config_t dev_cfg = {
        .device_address = VL6180X_I2C_ADDR, // 7位地址
        .scl_speed_hz = 100000,             // 100KHz
    };

    // 添加I2C设备到总线
    esp_err_t ret = i2c_master_bus_add_device(bsp_i2c_bus_handle, &dev_cfg, &tof050c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %d", ret);
        return 0;
    }

    // 初始化参数
    ptp_offset = 0;
    scaling = 0;
    io_timeout = 2;

    // 读取偏移量和复位状态
    ptp_offset = VL6180X_ReadByte(SYSRANGE__PART_TO_PART_RANGE_OFFSET);
    uint8_t reset = VL6180X_ReadByte(SYSTEM__FRESH_OUT_OF_RESET); // 使用宏替换魔法数0x016

    if (reset == 1) {
        // 执行出厂校准和默认配置
        VL6180X_WR_CMD(0X0207, 0X01);
        VL6180X_WR_CMD(0X0208, 0X01);
        VL6180X_WR_CMD(0X0096, 0X00);
        VL6180X_WR_CMD(0X0097, 0XFD);
        VL6180X_WR_CMD(0X00E3, 0X00);
        VL6180X_WR_CMD(0X00E4, 0X04);
        VL6180X_WR_CMD(0X00E5, 0X02);
        VL6180X_WR_CMD(0X00E6, 0X01);
        VL6180X_WR_CMD(0X00E7, 0X03);
        VL6180X_WR_CMD(0X00F5, 0X02);
        VL6180X_WR_CMD(0X00D9, 0X05);
        VL6180X_WR_CMD(0X00DB, 0XCE);
        VL6180X_WR_CMD(0X02DC, 0X03);
        VL6180X_WR_CMD(0X00DD, 0XF8);
        VL6180X_WR_CMD(0X009F, 0X00);
        VL6180X_WR_CMD(0X00A3, 0X3C);
        VL6180X_WR_CMD(0X00B7, 0X00);
        VL6180X_WR_CMD(0X00BB, 0X3C);
        VL6180X_WR_CMD(0X00B2, 0X09);
        VL6180X_WR_CMD(0X00CA, 0X09);
        VL6180X_WR_CMD(0X0198, 0X01);
        VL6180X_WR_CMD(0X01B0, 0X17);
        VL6180X_WR_CMD(0X01AD, 0X00);
        VL6180X_WR_CMD(0X00FF, 0X05);
        VL6180X_WR_CMD(0X0100, 0X05);
        VL6180X_WR_CMD(0X0199, 0X05);
        VL6180X_WR_CMD(0X01A6, 0X1B);
        VL6180X_WR_CMD(0X01AC, 0X3E);
        VL6180X_WR_CMD(0X01A7, 0X1F);
        VL6180X_WR_CMD(0X0030, 0X00);

        VL6180X_WR_CMD(0X0011, 0X10);
        VL6180X_WR_CMD(0X010A, 0X30);
        VL6180X_WR_CMD(0X003F, 0X46);
        VL6180X_WR_CMD(0X0031, 0XFF);
        VL6180X_WR_CMD(0X0040, 0X63);
        VL6180X_WR_CMD(0X002E, 0X01);
        VL6180X_WR_CMD(0X001B, 0X09);
        VL6180X_WR_CMD(0X003E, 0X31);
        VL6180X_WR_CMD(0X0014, 0X24);

        VL6180X_WR_CMD(SYSTEM__FRESH_OUT_OF_RESET, 0x00); // 清除复位标志

        return 1;
    }
    return 0;
}

/**
 * @brief 设置测距缩放因子
 * @param new_scaling 新的缩放因子(1-3)
 */
const uint16_t ScalerValues[] = {0, 253, 127, 84};
void VL6180X_SetScaling(uint8_t new_scaling)
{
    uint8_t const DefaultCrosstalkValidHeight = 20; // 默认CROSSTALK_VALID_HEIGHT值

    // 无效的缩放因子则直接返回
    if (new_scaling < 1 || new_scaling > 3) {
        return;
    }

    scaling = new_scaling;
    VL6180X_WR_CMD2(RANGE_SCALER, ScalerValues[scaling]);

    // 对part-to-part偏移应用缩放
    VL6180X_WR_CMD(SYSRANGE__PART_TO_PART_RANGE_OFFSET, ptp_offset / scaling);

    // 对CrosstalkValidHeight应用缩放
    VL6180X_WR_CMD(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling);

    // 仅在1x缩放时启用早期收敛估计
    uint8_t rce = VL6180X_ReadByte(SYSRANGE__RANGE_CHECK_ENABLES);
    VL6180X_WR_CMD(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling == 1));
}

/**
 * @brief 配置VL6180X为默认设置
 */
void VL6180X_ConfigureDefault()
{
    VL6180X_WR_CMD(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
    VL6180X_WR_CMD(SYSALS__ANALOGUE_GAIN, 0x46);
    VL6180X_WR_CMD(SYSRANGE__VHV_REPEAT_RATE, 0xFF);
    VL6180X_WR_CMD2(SYSALS__INTEGRATION_PERIOD, 0x0063);
    VL6180X_WR_CMD(SYSRANGE__VHV_RECALIBRATE, 0x01);
    VL6180X_WR_CMD(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
    VL6180X_WR_CMD(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);
    VL6180X_WR_CMD(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);
    VL6180X_WR_CMD(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);
    VL6180X_WR_CMD(INTERLEAVED_MODE__ENABLE, 0);
    VL6180X_SetScaling(1);
}

/**
 * @brief 设置VL6180X超时时间
 * @param timeout 超时时间(单位：ms)
 */
void VL6180X_SetTimeout(uint16_t timeout)
{
    io_timeout = timeout;
}

/**
 * @brief 启动测距
 * @return uint8_t 总是返回0(保留原始函数签名)
 */
uint8_t VL6180X_Start_Range()
{
    VL6180X_WR_CMD(SYSRANGE__START, 0x01);
    return 0;
}

/**
 * @brief 轮询等待测距完成
 * @return uint8_t 总是返回0(保留原始函数签名)
 */
uint8_t VL6180X_Poll_Range()
{
    uint8_t status;
    uint8_t range_status;
    timeoutcnt = 0; // 每次轮询重置超时计数器

    status = VL6180X_ReadByte(RESULT__INTERRUPT_STATUS_GPIO);
    range_status = status & 0x07;

    while (range_status != 0x04) {
        timeoutcnt++;
        if (timeoutcnt > io_timeout) {
            ESP_LOGE(TAG, "Range poll timeout (cnt: %d)", timeoutcnt);
            break;
        }
        status = VL6180X_ReadByte(RESULT__INTERRUPT_STATUS_GPIO);
        range_status = status & 0x07;
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms延时（替换1/portTICK_PERIOD_MS，更直观）
    }
    return 0;
}

/**
 * @brief 读取测距结果(原始值)
 * @return uint8_t 测距原始值
 */
uint8_t VL6180X_Read_Range() // 修正函数名，与声明一致
{
    uint8_t range;
    range = VL6180X_ReadByte(RESULT__RANGE_VAL);
    return range;
}

/**
 * @brief 清除中断状态
 */
void VL6180X_Clear_Interrupt()
{
    VL6180X_WR_CMD(SYSTEM__INTERRUPT_CLEAR, 0x07);
}

/**
 * @brief 执行单次测距并返回距离(毫米)
 * @return uint16_t 测距结果(毫米)
 */
uint16_t VL6180X_ReadRangeSingleMillimeters()
{
    // 启动单次测量模式
    VL6180X_Start_Range();

    // 等待测量完成
    VL6180X_Poll_Range();
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms延时（VL6180X单次测距最小耗时）

    // 清除中断
    VL6180X_Clear_Interrupt();

    // 读取并返回距离(应用缩放因子)
    uint16_t distance = (uint16_t)scaling * VL6180X_Read_Range();
    return distance;
}

 