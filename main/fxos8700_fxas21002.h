/**
 * @file fxos8700_fxas21002.h
 * @brief Minimal I2C driver for the Adafruit 3463 breakout board, which
 *        combines an FXOS8700 (accelerometer + magnetometer) and an
 *        FXAS21002 (gyroscope) in a single 9-DoF IMU.
 *
 * This driver only exposes what is needed to publish sensor_msgs/Imu data:
 * linear acceleration (from the FXOS8700 accelerometer) and angular
 * velocity (from the FXAS21002 gyroscope).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default 7-bit I2C address of the FXOS8700 (SA0 = GND). */
#define FXOS8700_I2C_ADDR      0x1F
/** Default 7-bit I2C address of the FXAS21002 (SA0 = GND). */
#define FXAS21002_I2C_ADDR     0x21

/** Standard gravity, used to convert accelerometer counts to m/s^2. */
#define IMU_STANDARD_GRAVITY   9.80665f

/**
 * @brief Configuration parameters required to initialize the IMU driver.
 */
typedef struct {
    i2c_port_t i2c_port;      /*!< I2C peripheral to use (I2C_NUM_0). */
    int sda_gpio;             /*!< GPIO number used for SDA. */
    int scl_gpio;             /*!< GPIO number used for SCL. */
    uint32_t clk_speed_hz;    /*!< I2C bus clock speed in Hz. */
} imu_config_t;

/**
 * @brief A single 3-axis sample expressed in SI units.
 */
typedef struct {
    float x;
    float y;
    float z;
} imu_vector3_t;

/**
 * @brief Combined IMU sample: linear acceleration [m/s^2] and angular
 *        velocity [rad/s].
 */
typedef struct {
    imu_vector3_t accel;      /*!< Linear acceleration in m/s^2. */
    imu_vector3_t gyro;       /*!< Angular velocity in rad/s. */
} imu_sample_t;

/**
 * @brief Initialize the I2C bus and configure both the FXOS8700 and the
 *        FXAS21002 in active/ready mode.
 *
 * @param config I2C bus configuration (port, SDA/SCL pins, clock speed).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t imu_init(const imu_config_t *config);

/**
 * @brief Read a combined accelerometer + gyroscope sample.
 *
 * @param[out] sample Destination for the converted sample.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t imu_read(imu_sample_t *sample);

#ifdef __cplusplus
}
#endif
