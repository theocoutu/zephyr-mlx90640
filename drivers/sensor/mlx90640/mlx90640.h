/*
 * Copyright (c) 2023 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90640_MLX90640_H_
#define ZEPHYR_DRIVERS_SENSOR_MLX90640_MLX90640_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>

/*
 * MLX90640_API.h should be available in your include path.
 * It contains the paramsMLX90640 struct definition and function prototypes
 * from the manufacturer's API.
 */
#include "MLX90640_API.h" // Assuming MLX90640_API.h is accessible

// Define sensor-specific channels if not already standard
// For pixel data, we might need a custom channel or use SENSOR_CHAN_ALL
// For simplicity, we'll fetch all pixels and ambient temp.
// Individual pixels can be accessed from the data struct after fetch.

// Sensor resolution (pixels)
#define MLX90640_PIXELS_W 32
#define MLX90640_PIXELS_H 24
#define MLX90640_NUM_PIXELS (MLX90640_PIXELS_W * MLX90640_PIXELS_H) // 768

// EEPROM data size (words)
#define MLX90640_EEPROM_SIZE 832
// Frame data size (words), includes control and status words
#define MLX90640_FRAME_DATA_SIZE (MLX90640_NUM_PIXELS + MLX90640_AUX_NUM + 2) // 768 + 64 + 2 = 834


/*
// Custom sensor attributes
//< MLX90640 specific attribute for ADC resolution (0-3 for 16-19 bit).
#define SENSOR_ATTR_MLX90640_RESOLUTION (SENSOR_ATTR_PRIV_START + 0)
//< MLX90640 specific attribute for measurement mode (0=interleaved, 1=chess).
#define SENSOR_ATTR_MLX90640_MODE (SENSOR_ATTR_PRIV_START + 1)
*/


/**
 * @brief MLX90640 configuration data.
 *
 * This structure contains all the configuration parameters for the MLX90640 sensor
 * derived from the device tree. This configuration is typically constant after init.
 */
struct mlx90640_config {
    struct i2c_dt_spec i2c; // I2C bus specification from DT
    uint8_t refresh_rate;   // Initial refresh rate (0-7 for 0.5Hz to 64Hz)
    uint8_t resolution;     // Initial ADC resolution (0-3 for 16-bit to 19-bit)
    uint8_t mode;           // Initial measurement mode (0=interleaved, 1=chess)
    float emissivity;       // Initial emissivity of the target (0.1 - 1.0) from DT
};

/**
 * @brief MLX90640 runtime data.
 *
 * This structure holds the runtime data for the MLX90640 sensor, including
 * calibration parameters, frame buffers, processed temperature data, and
 * runtime-configurable settings like emissivity.
 */
struct mlx90640_data {
    paramsMLX90640 mlx90640_params; // Calibration parameters from EEPROM
    uint16_t ee_data[MLX90640_EEPROM_SIZE]; // Buffer for EEPROM data
    uint16_t frame_data[MLX90640_FRAME_DATA_SIZE]; // Buffer for raw frame data (can hold one subpage)
    float temperatures[MLX90640_NUM_PIXELS]; // Processed object temperatures for the full 32x24 array
    float ta; // Ambient temperature
    float vdd; // Sensor supply voltage
    uint8_t current_subpage; // Last subpage read (0 or 1)
    float runtime_emissivity; // Emissivity that can be changed at runtime
};

#endif /* ZEPHYR_DRIVERS_SENSOR_MLX90640_MLX90640_H_ */