#include "tof.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 静态全局句柄
static i2c_master_dev_handle_t tof050c_dev_handle = NULL;
static i2c_master_bus_handle_t bsp_i2c_bus_handle = NULL;

// 模块TAG
static const char *TAG = "VL6180X";

 
#define MAX_RANGE_LIMIT 255     // 最大有效测距值
#define MIN_RANGE_LIMIT 0       // 最小有效测距值

// ===================== 官方库核心函数 =====================
uint8_t VL6180X_read8(uint16_t address)
{
    uint8_t buffer[2] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };
    esp_err_t ret = i2c_master_transmit(tof050c_dev_handle, buffer, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "发送寄存器地址失败 0x%04X: %d", address, ret);
        return 0;
    }
    
    uint8_t data = 0;
    ret = i2c_master_receive(tof050c_dev_handle, &data, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取字节失败 0x%04X: %d", address, ret);
        return 0;
    }
    return data;
}

void VL6180X_write8(uint16_t address, uint8_t data)
{
    uint8_t buffer[3] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), data };
    esp_err_t ret = i2c_master_transmit(tof050c_dev_handle, buffer, 3, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写入字节失败 0x%04X: %d", address, ret);
    }
}

// 优化版配置加载
void VL6180X_loadSettings(void)
{
    // 官方私有配置
    VL6180X_write8(0x0207, 0x01);
    VL6180X_write8(0x0208, 0x01);
    VL6180X_write8(0x0096, 0x00);
    VL6180X_write8(0x0097, 0xFD);
    VL6180X_write8(0x00E3, 0x00);
    VL6180X_write8(0x00E4, 0x04);
    VL6180X_write8(0x00E5, 0x02);
    VL6180X_write8(0x00E6, 0x01);
    VL6180X_write8(0x00E7, 0x03);
    VL6180X_write8(0x00F5, 0x02);
    VL6180X_write8(0x00D9, 0x05);
    VL6180X_write8(0x00DB, 0xCE);
    VL6180X_write8(0x02DC, 0x03);
    VL6180X_write8(0x00DD, 0xF8);
    VL6180X_write8(0x009F, 0x00);
    VL6180X_write8(0x00A3, 0x3C);
    VL6180X_write8(0x00B7, 0x00);
    VL6180X_write8(0x00BB, 0x3C);
    VL6180X_write8(0x00B2, 0x09);
    VL6180X_write8(0x00CA, 0x09);
    VL6180X_write8(0x0198, 0x01);
    VL6180X_write8(0x01B0, 0x17);
    VL6180X_write8(0x01AD, 0x00);
    VL6180X_write8(0x00FF, 0x05);
    VL6180X_write8(0x0100, 0x05);
    VL6180X_write8(0x0199, 0x05);
    VL6180X_write8(0x01A6, 0x1B);
    VL6180X_write8(0x01AC, 0x3E);
    VL6180X_write8(0x01A7, 0x1F);
    VL6180X_write8(0x0030, 0x00);

    // 优化配置：解决ECEFAIL错误
    VL6180X_write8(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, 0x04);
    VL6180X_write8(VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME, 0x30);
    VL6180X_write8(VL6180X_REG_SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x20);
    VL6180X_write8(VL6180X_REG_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x00);
    VL6180X_write8(VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);
    VL6180X_write8(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x10);

    // 关闭ALS（减少干扰）
    VL6180X_write8(VL6180X_REG_SYSALS_ANALOGUE_GAIN, VL6180X_ALS_GAIN_1);
    VL6180X_write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0x00);
    VL6180X_write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 0x01);

    // 官方推荐配置
    VL6180X_write8(0x010A, 0x30);
    VL6180X_write8(0x003F, 0x46);
    VL6180X_write8(0x0031, 0xFF);
    VL6180X_write8(0x0041, 0x63);
    VL6180X_write8(0x002E, 0x01);
}

 
 
// ===================== 对外API =====================
esp_err_t tof050c_init(void)
{
    // 1. 配置GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOF_SDA_GPIO) | (1ULL << TOF_SCL_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // 增加GPIO驱动能力
    gpio_set_drive_capability(TOF_SDA_GPIO, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(TOF_SCL_GPIO, GPIO_DRIVE_CAP_3);
    ESP_LOGI(TAG, "GPIO[%d/SDA, %d/SCL] 配置完成", TOF_SDA_GPIO, TOF_SCL_GPIO);

    // 2. 创建I2C总线
    if (bsp_i2c_bus_handle == NULL)
    {
        i2c_master_bus_config_t bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_NUM,
            .sda_io_num = TOF_SDA_GPIO,
            .scl_io_num = TOF_SCL_GPIO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bsp_i2c_bus_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "创建I2C总线失败: %d", ret);
            return ret;
        }
    }

    // 3. 添加设备（降低I2C速度）
    if (tof050c_dev_handle == NULL)
    {
        i2c_device_config_t dev_cfg = {
            .device_address = VL6180X_DEFAULT_I2C_ADDR,
            .scl_speed_hz = 50000, // 50KHz提升稳定性
        };
        esp_err_t ret = i2c_master_bus_add_device(bsp_i2c_bus_handle, &dev_cfg, &tof050c_dev_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "添加I2C设备失败: %d", ret);
            return ret;
        }
    }

    // 4. 验证ID
    uint8_t model_id = VL6180X_read8(VL6180X_REG_IDENTIFICATION_MODEL_ID);
    if (model_id != 0xB4) {
        ESP_LOGE(TAG, "设备ID验证失败: 0x%02X (预期0xB4)", model_id);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "设备ID验证成功: 0x%02X", model_id);

    // 5. 加载配置
    VL6180X_loadSettings();
    VL6180X_write8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
    ESP_LOGI(TAG, "加载优化配置完成");

    ESP_LOGI(TAG, "VL6180X初始化完成（无滤波，报错返回255）");
    return ESP_OK;
}

// 单次测距（无重试，超时/报错直接返回500）
uint16_t tof050c_read_distance_mm(void)
{
    uint16_t distance = 0;
 

    // 1. 清除之前的中断
    VL6180X_write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // 2. 启动测距
    VL6180X_write8(VL6180X_REG_SYSRANGE_START, 0x01);

    // 3. 等待测距完成（100ms超时）
    int poll_timeout = 0;
    while (!(VL6180X_read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04)) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
        poll_timeout++;
        if (poll_timeout > 100) { // 100ms超时
       
            // 超时直接返回255，不复位、不重试
            return TOF_ERROR_RETURN_VALUE;
        }
    }

    // 4. 读取结果和状态
    distance = VL6180X_read8(VL6180X_REG_RESULT_RANGE_VAL);
 
    
    // 5. 清除中断
    VL6180X_write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
    return distance; // 成功，返回有效值
 
}
 