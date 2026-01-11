#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ===== ログタグ ===== */
static const char *TAG = "PEDO";

/* ===== ESP32-C6-DevKit I2C ピン（LP I2C対応） ===== */
#define I2C_PORT    I2C_NUM_0
#define I2C_SDA     GPIO_NUM_6    // DevKit LP I2C SDA
#define I2C_SCL     GPIO_NUM_7    // DevKit LP I2C SCL
#define I2C_FREQ    100000        // 100kHz

/* ===== ADXL367 ===== */
#define ADXL367_ADDR        0x1D
#define REG_DEVID_AD        0x00
#define REG_XDATA           0x08   // 8ビットデータレジスタ
#define REG_SOFT_RESET      0x1F
#define REG_FILTER_CTL      0x2C
#define REG_POWER_CTL       0x2D

#define I2C_TIMEOUT_MS      100
#define SCALE_8BIT          0.0156f  // 8ビット ±2g: 1g ≈ 64 LSB

/* ===== I2C ハンドル ===== */
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* ===== I2C 初期化 ===== */
static esp_err_t i2c_init(void)
{
    esp_err_t err;
    
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_LOGI(TAG, "Initializing I2C bus on SDA=%d, SCL=%d", I2C_SDA, I2C_SCL);
    err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C bus created successfully");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL367_ADDR,
        .scl_speed_hz = I2C_FREQ,
    };
    
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C device added at address 0x%02X", ADXL367_ADDR);
    
    return ESP_OK;
}

/* ===== レジスタ書き込み ===== */
static esp_err_t adxl367_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(dev_handle, buf, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

/* ===== レジスタ読み取り ===== */
static esp_err_t adxl367_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, val, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

/* ===== ADXL367 初期化 ===== */
static esp_err_t adxl367_init(void)
{
    esp_err_t err;
    
    ESP_LOGI(TAG, "Starting ADXL367 initialization...");
    
    // ソフトリセット
    ESP_LOGI(TAG, "Sending soft reset...");
    err = adxl367_write_reg(REG_SOFT_RESET, 0x52);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // デバイスID確認
    ESP_LOGI(TAG, "Reading device ID...");
    uint8_t id = 0;
    err = adxl367_read_reg(REG_DEVID_AD, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return err;
    }
    ESP_LOGI(TAG, "Device ID: 0x%02X (expected 0xAD)", id);
    if (id != 0xAD) {
        ESP_LOGW(TAG, "Unexpected device ID!");
    }

    // FILTER_CTL: ODR=100Hz, ±2g
    err = adxl367_write_reg(REG_FILTER_CTL, 0x13);
    if (err != ESP_OK) return err;
    
    // Measurement mode
    err = adxl367_write_reg(REG_POWER_CTL, 0x02);
    if (err != ESP_OK) return err;
    
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "ADXL367 initialized (8-bit mode)");
    return ESP_OK;
}

/* ===== XYZ 読み取り (8ビット) ===== */
static esp_err_t adxl367_read_xyz(int8_t *x, int8_t *y, int8_t *z)
{
    uint8_t buf[3];
    uint8_t reg = REG_XDATA;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    
    *x = (int8_t)buf[0];
    *y = (int8_t)buf[1];
    *z = (int8_t)buf[2];
    return ESP_OK;
}

/* ===== app_main ===== */
void app_main(void)
{
    esp_err_t err;
    
    err = i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed, halting.");
        return;
    }
    
    err = adxl367_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL367 init failed, halting.");
        ESP_LOGE(TAG, "Please check wiring: SDA->GPIO6, SCL->GPIO7, VCC->3.3V, GND->GND");
        return;
    }

    while (1) {
        int8_t x, y, z;
        if (adxl367_read_xyz(&x, &y, &z) == ESP_OK) {
            float gx = x * SCALE_8BIT;
            float gy = y * SCALE_8BIT;
            float gz = z * SCALE_8BIT;
            ESP_LOGI(TAG, "g=(%.3f, %.3f, %.3f)", gx, gy, gz);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz出力
    }
}
