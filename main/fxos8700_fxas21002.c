/**
 * @file fxos8700_fxas21002.c
 * @brief Minimal I2C driver implementation for the Adafruit 3463
 *        (FXOS8700 accelerometer/magnetometer + FXAS21002 gyroscope).
 */
#include "fxos8700_fxas21002.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "imu_fxos8700_fxas21002";

/* ------------------------------------------------------------------------
 * FXOS8700 registers
 * ---------------------------------------------------------------------- */
#define FXOS8700_REG_STATUS        0x00
#define FXOS8700_REG_OUT_X_MSB     0x01
#define FXOS8700_REG_WHOAMI        0x0D
#define FXOS8700_REG_XYZ_DATA_CFG  0x0E
#define FXOS8700_REG_CTRL_REG1     0x2A

#define FXOS8700_WHOAMI_VALUE      0xC7

/* Accelerometer full scale range: +-2g -> sensitivity 4096 counts/g. */
#define FXOS8700_ACCEL_MG_LSB      (1.0f / 4096.0f)

/* ------------------------------------------------------------------------
 * FXAS21002 registers
 * ---------------------------------------------------------------------- */
#define FXAS21002_REG_STATUS       0x00
#define FXAS21002_REG_OUT_X_MSB    0x01
#define FXAS21002_REG_WHOAMI       0x0C
#define FXAS21002_REG_CTRL_REG0    0x0D
#define FXAS21002_REG_CTRL_REG1    0x13

#define FXAS21002_WHOAMI_VALUE     0xD7

/* Gyroscope full scale range: 250 dps -> sensitivity 0.0078125 dps/LSB. */
#define FXAS21002_DPS_PER_DIGIT    0.0078125f
#define DEG_TO_RAD                 0.017453292519943295f

static i2c_port_t s_i2c_port = I2C_NUM_0;

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(s_i2c_port, dev_addr, buf, sizeof(buf),
                                       pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(s_i2c_port, dev_addr, &reg, 1, data, len,
                                         pdMS_TO_TICKS(100));
}

static int16_t combine_be16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((msb << 8) | lsb);
}

esp_err_t imu_init(const imu_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_port = config->i2c_port;

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->clk_speed_hz,
    };

    esp_err_t err = i2c_param_config(s_i2c_port, &i2c_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(s_i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Verify FXOS8700 identity. */
    uint8_t whoami = 0;
    err = i2c_read_regs(FXOS8700_I2C_ADDR, FXOS8700_REG_WHOAMI, &whoami, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FXOS8700 WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }
    if (whoami != FXOS8700_WHOAMI_VALUE) {
        ESP_LOGW(TAG, "Unexpected FXOS8700 WHO_AM_I: 0x%02X", whoami);
    }

    /* Verify FXAS21002 identity. */
    err = i2c_read_regs(FXAS21002_I2C_ADDR, FXAS21002_REG_WHOAMI, &whoami, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FXAS21002 WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }
    if (whoami != FXAS21002_WHOAMI_VALUE) {
        ESP_LOGW(TAG, "Unexpected FXAS21002 WHO_AM_I: 0x%02X", whoami);
    }

    /* --- Configure FXOS8700 --- */
    /* Put into standby to allow configuration. */
    ESP_ERROR_CHECK(i2c_write_reg(FXOS8700_I2C_ADDR, FXOS8700_REG_CTRL_REG1, 0x00));
    /* Full scale range +-2g. */
    ESP_ERROR_CHECK(i2c_write_reg(FXOS8700_I2C_ADDR, FXOS8700_REG_XYZ_DATA_CFG, 0x00));
    /* Active mode, ODR = 100 Hz (bits 5:3 = 000), normal (no low noise). */
    ESP_ERROR_CHECK(i2c_write_reg(FXOS8700_I2C_ADDR, FXOS8700_REG_CTRL_REG1, 0x01));

    /* --- Configure FXAS21002 --- */
    /* Reset to standby before configuration. */
    ESP_ERROR_CHECK(i2c_write_reg(FXAS21002_I2C_ADDR, FXAS21002_REG_CTRL_REG1, 0x00));
    /* Full scale range 250 dps (bits 1:0 = 11). */
    ESP_ERROR_CHECK(i2c_write_reg(FXAS21002_I2C_ADDR, FXAS21002_REG_CTRL_REG0, 0x03));
    /* Active mode, ODR = 100 Hz (bits 4:2 = 010). */
    ESP_ERROR_CHECK(i2c_write_reg(FXAS21002_I2C_ADDR, FXAS21002_REG_CTRL_REG1, 0x0E));

    ESP_LOGI(TAG, "FXOS8700 + FXAS21002 IMU initialized on I2C port %d (SDA=%d, SCL=%d)",
             s_i2c_port, config->sda_gpio, config->scl_gpio);

    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t accel_raw[6] = { 0 };
    esp_err_t err = i2c_read_regs(FXOS8700_I2C_ADDR, FXOS8700_REG_OUT_X_MSB, accel_raw, sizeof(accel_raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t gyro_raw[6] = { 0 };
    err = i2c_read_regs(FXAS21002_I2C_ADDR, FXAS21002_REG_OUT_X_MSB, gyro_raw, sizeof(gyro_raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope: %s", esp_err_to_name(err));
        return err;
    }

    /* FXOS8700 accel data is left-justified 14-bit, so shift right by 2. */
    int16_t ax = combine_be16(accel_raw[0], accel_raw[1]) >> 2;
    int16_t ay = combine_be16(accel_raw[2], accel_raw[3]) >> 2;
    int16_t az = combine_be16(accel_raw[4], accel_raw[5]) >> 2;

    sample->accel.x = (float)ax * FXOS8700_ACCEL_MG_LSB * IMU_STANDARD_GRAVITY;
    sample->accel.y = (float)ay * FXOS8700_ACCEL_MG_LSB * IMU_STANDARD_GRAVITY;
    sample->accel.z = (float)az * FXOS8700_ACCEL_MG_LSB * IMU_STANDARD_GRAVITY;

    int16_t gx = combine_be16(gyro_raw[0], gyro_raw[1]);
    int16_t gy = combine_be16(gyro_raw[2], gyro_raw[3]);
    int16_t gz = combine_be16(gyro_raw[4], gyro_raw[5]);

    sample->gyro.x = (float)gx * FXAS21002_DPS_PER_DIGIT * DEG_TO_RAD;
    sample->gyro.y = (float)gy * FXAS21002_DPS_PER_DIGIT * DEG_TO_RAD;
    sample->gyro.z = (float)gz * FXAS21002_DPS_PER_DIGIT * DEG_TO_RAD;

    return ESP_OK;
}
