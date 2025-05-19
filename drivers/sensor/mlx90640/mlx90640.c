/*
 * Copyright (c) 2023 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90640

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <math.h> // For NAN and isnan

#include "mlx90640.h"

// MLX90640_API.c and MLX90640_I2C_Driver.h should be part of your build.
// MLX90640_API.c contains the core processing logic.
// MLX90640_I2C_Driver.h declares I2C functions that we will implement here
// for the Melexis API to use.
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

LOG_MODULE_REGISTER(MLX90640, CONFIG_SENSOR_LOG_LEVEL);

// Global pointer to the I2C device tree specification for the current sensor instance.
// This is used by the MLX90640_I2CRead and MLX90640_I2CWrite functions
// which are called by the Melexis API.
static const struct i2c_dt_spec *mlx90640_i2c_spec_global;


/*
 * Implementation of I2C driver functions required by MLX90640_API.c
 * These functions will use Zephyr's I2C API.
 */

void MLX90640_I2CInit(void)
{
    // This function is called by some Melexis examples.
    // In Zephyr, I2C device initialization is handled by the I2C driver and DT.
    // We just ensure mlx90640_i2c_spec_global is set before API usage.
    if (mlx90640_i2c_spec_global == NULL) {
        LOG_ERR("I2C spec not set for MLX90640_I2CInit (mlx90640_i2c_spec_global is NULL)");
    }
}

void MLX90640_I2CFreqSet(int freq_khz)
{
    // Zephyr's I2C API configures frequency at the bus controller level,
    // typically not dynamically per device or per transaction easily.
    LOG_WRN("MLX90640_I2CFreqSet(%d kHz) called, but dynamic frequency change is not implemented. "
            "Configure I2C bus frequency in devicetree.", freq_khz);
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    if (mlx90640_i2c_spec_global == NULL) {
        LOG_ERR("I2C spec not available for MLX90640_I2CRead (mlx90640_i2c_spec_global is NULL)");
        return -1; // Expected error code by Melexis API for I2C issues
    }
    // The slaveAddr argument is part of the i2c_dt_spec, but the Melexis API passes it.
    // We should verify it matches, though i2c_write_read_dt will use the one from spec.
    if (slaveAddr != mlx90640_i2c_spec_global->addr) {
        LOG_WRN("MLX90640_I2CRead: slaveAddr (0x%X) differs from DT spec (0x%X). Using DT spec.",
                slaveAddr, mlx90640_i2c_spec_global->addr);
    }

    uint8_t write_buf[2];
    // Ensure read_buf is large enough for the maximum possible read (EEPROM dump)
    if (nMemAddressRead > MLX90640_EEPROM_SIZE) {
         LOG_ERR("nMemAddressRead (%u) exceeds max buffer (%d)", nMemAddressRead, MLX90640_EEPROM_SIZE);
         return -1;
    }
    uint8_t read_buf[MLX90640_EEPROM_SIZE * 2];
    int ret;

    // MSB first for address
    write_buf[0] = (startAddress >> 8) & 0xFF;
    write_buf[1] = startAddress & 0xFF;

    ret = i2c_write_read_dt(mlx90640_i2c_spec_global, write_buf, sizeof(write_buf), read_buf, nMemAddressRead * 2);
    if (ret != 0) {
        LOG_ERR("MLX90640_I2CRead: i2c_write_read_dt failed: %d. Reg 0x%04X", ret, startAddress);
        return -1; // MLX90640 API expects -1 for NACK/error
    }

    // Convert byte array to uint16_t array (MSB first as per MLX90640)
    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        data[i] = ((uint16_t)read_buf[2 * i] << 8) | read_buf[2 * i + 1];
    }

    return 0; // Success
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data_val)
{
    if (mlx90640_i2c_spec_global == NULL) {
        LOG_ERR("I2C spec not available for MLX90640_I2CWrite (mlx90640_i2c_spec_global is NULL)");
        return -1; // Expected error code
    }
    if (slaveAddr != mlx90640_i2c_spec_global->addr) {
        LOG_WRN("MLX90640_I2CWrite: slaveAddr (0x%X) differs from DT spec (0x%X). Using DT spec.",
                slaveAddr, mlx90640_i2c_spec_global->addr);
    }

    uint8_t buf[4]; // 2 bytes for address, 2 bytes for data
    int ret;

    // MSB first for address
    buf[0] = (writeAddress >> 8) & 0xFF;
    buf[1] = writeAddress & 0xFF;
    // MSB first for data
    buf[2] = (data_val >> 8) & 0xFF;
    buf[3] = data_val & 0xFF;

    ret = i2c_write_dt(mlx90640_i2c_spec_global, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("MLX90640_I2CWrite: i2c_write_dt failed: %d. Reg 0x%04X, Data 0x%04X", ret, writeAddress, data_val);
        return -1; // MLX90640 API expects -1 for NACK
    }

    k_sleep(K_MSEC(1)); // Delay for register write settling or EEPROM write time.

    uint16_t read_data;
    // MLX90640_I2CRead itself uses mlx90640_i2c_spec_global.
    // The slaveAddr passed here will be checked against the global spec's address.
    ret = MLX90640_I2CRead(slaveAddr, writeAddress, 1, &read_data);
    if (ret != 0) {
        LOG_ERR("MLX90640_I2CWrite: Read-back failed for verification: %d", ret);
        return -1; // Error during read-back
    }

    if (read_data != data_val) {
        LOG_ERR("MLX90640_I2CWrite: Verification failed. Wrote 0x%04X, Read 0x%04X from Reg 0x%04X", data_val, read_data, writeAddress);
        return -2; // MLX90640 API expects -2 for verify error
    }

    return 0; // Success
}

int MLX90640_I2CGeneralReset(void) {
    if (mlx90640_i2c_spec_global == NULL || mlx90640_i2c_spec_global->bus == NULL) {
        LOG_ERR("I2C spec or bus not available for MLX90640_I2CGeneralReset");
        return -1;
    }
    // The I2C general call reset command is address 0x00 followed by byte 0x06.
    // i2c_write_dt is for a specific device. For general call, use i2c_write.
    uint8_t reset_cmd = 0x06;
    int ret = i2c_write(mlx90640_i2c_spec_global->bus, &reset_cmd, 1, 0x00); // Target address 0x00 for general call
    if (ret != 0) {
        LOG_ERR("MLX90640_I2CGeneralReset: i2c_write failed: %d", ret);
        return -1;
    }
    k_sleep(K_MSEC(2));
    return 0;
}


static int mlx90640_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct mlx90640_data *data = dev->data;
    const struct mlx90640_config *config = dev->config;
    // slave_addr is now part of mlx90640_i2c_spec_global
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_VOLTAGE) {
        LOG_WRN("Unsupported channel for fetch: %d. Fetching all data.", chan);
    }

    // Set the global I2C device tree specification for the Melexis API I2C calls.
    mlx90640_i2c_spec_global = &config->i2c;

    int current_op_mode = MLX90640_GetCurMode(config->i2c.addr); // GetCurMode still needs slave_addr directly
    if (current_op_mode < 0) {
        LOG_ERR("Failed to get current sensor mode: %d", current_op_mode);
        mlx90640_i2c_spec_global = NULL; // Clear global on error path
        return -EIO;
    }

    float tr_calc;

    if (current_op_mode == 1) { // Chess pattern mode
        LOG_DBG("Chess mode: Fetching two subpages.");
        ret = MLX90640_GetFrameData(config->i2c.addr, data->frame_data);
        if (ret < 0) {
            LOG_ERR("Failed to get frame data (subpage 1/2): %d", ret);
            if (ret == -MLX90640_FRAME_DATA_ERROR) {
                 LOG_ERR("Frame data error. I2C speed might be too low or sensor issue.");
            }
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        int subpage_first = ret;
        LOG_DBG("Fetched subpage %d (1/2)", subpage_first);

        float current_ta_sub1 = MLX90640_GetTa(data->frame_data, &data->mlx90640_params);
        if (isnan(current_ta_sub1)) {
            LOG_WRN("Failed to get Ta from subpage %d, using previous or default Tr", subpage_first);
            tr_calc = (isnan(data->ta) ? 25.0f : data->ta) - 8.0f;
        } else {
            data->ta = current_ta_sub1;
            tr_calc = data->ta - 8.0f;
        }
        
        MLX90640_CalculateTo(data->frame_data, &data->mlx90640_params, data->runtime_emissivity, tr_calc, data->temperatures);

        ret = MLX90640_GetFrameData(config->i2c.addr, data->frame_data);
        if (ret < 0) {
            LOG_ERR("Failed to get frame data (subpage 2/2): %d", ret);
             if (ret == -MLX90640_FRAME_DATA_ERROR) {
                 LOG_ERR("Frame data error. I2C speed might be too low or sensor issue.");
            }
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        int subpage_second = ret;
        LOG_DBG("Fetched subpage %d (2/2)", subpage_second);

        if (subpage_second == subpage_first) {
            LOG_WRN("Fetched same subpage %d again in chess mode.", subpage_first);
        }
        
        MLX90640_CalculateTo(data->frame_data, &data->mlx90640_params, data->runtime_emissivity, tr_calc, data->temperatures);
        
        data->vdd = MLX90640_GetVdd(data->frame_data, &data->mlx90640_params);
        float current_ta_sub2 = MLX90640_GetTa(data->frame_data, &data->mlx90640_params);
        if (!isnan(current_ta_sub2)) {
            data->ta = current_ta_sub2;
        }
        if (isnan(data->vdd)) LOG_WRN("VDD calculation resulted in NaN from subpage %d", subpage_second);
        if (isnan(data->ta)) LOG_WRN("Ta calculation resulted in NaN from subpage %d", subpage_second);

    } else { // Interleaved mode
        LOG_DBG("Interleaved mode: Fetching one subpage.");
        ret = MLX90640_GetFrameData(config->i2c.addr, data->frame_data);
        if (ret < 0) {
            LOG_ERR("Failed to get frame data (interleaved): %d", ret);
            if (ret == -MLX90640_FRAME_DATA_ERROR) {
                 LOG_ERR("Frame data error. I2C speed might be too low or sensor issue.");
            }
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        data->current_subpage = ret;
        LOG_DBG("Fetched subpage %d for interleaved mode", data->current_subpage);

        data->ta = MLX90640_GetTa(data->frame_data, &data->mlx90640_params);
        data->vdd = MLX90640_GetVdd(data->frame_data, &data->mlx90640_params);

        if (isnan(data->ta)) {
            LOG_WRN("Failed to get Ta (interleaved), using default Tr");
            tr_calc = 25.0f - 8.0f;
        } else {
             tr_calc = data->ta - 8.0f;
        }
        if (isnan(data->vdd)) LOG_WRN("VDD calculation resulted in NaN (interleaved)");

        MLX90640_CalculateTo(data->frame_data, &data->mlx90640_params, data->runtime_emissivity, tr_calc, data->temperatures);
    }

    MLX90640_BadPixelsCorrection(data->mlx90640_params.brokenPixels, data->temperatures, current_op_mode, &data->mlx90640_params);
    MLX90640_BadPixelsCorrection(data->mlx90640_params.outlierPixels, data->temperatures, current_op_mode, &data->mlx90640_params);
    LOG_DBG("Applied bad pixel correction for mode %d.", current_op_mode);
    
    LOG_DBG("Sample fetch complete. Ta: %.2f C, Vdd: %.2f V", data->ta, data->vdd);
    if (!isnan(data->temperatures[0])) {
        LOG_DBG("Temp[0]: %.2f C", data->temperatures[0]);
    }

    mlx90640_i2c_spec_global = NULL; // Clear global after use
    return 0;
}

static int mlx90640_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct mlx90640_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        if (isnan(data->ta)) {
            LOG_WRN("Ambient temperature (Ta) is NaN.");
            return -ENODATA;
        }
        val->val1 = (int32_t)data->ta;
        val->val2 = (int32_t)((data->ta - val->val1) * 1000000);
        break;
    case SENSOR_CHAN_VOLTAGE: // Used for VDD
        if (isnan(data->vdd)) {
            LOG_WRN("Supply voltage (Vdd) is NaN.");
            return -ENODATA;
        }
        val->val1 = (int32_t)data->vdd;
        val->val2 = (int32_t)((data->vdd - val->val1) * 1000000);
        break;
    default:
        LOG_ERR("Unsupported channel for get: %d", chan);
        return -ENOTSUP;
    }

    return 0;
}

static int mlx90640_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
    struct mlx90640_data *data = dev->data;
    const struct mlx90640_config *config = dev->config;
    // slave_addr is now part of mlx90640_i2c_spec_global, but Melexis API takes it directly.
    uint8_t slave_addr_direct = config->i2c.addr;
    int ret;

    // Set the global I2C spec for Melexis API I2C calls.
    mlx90640_i2c_spec_global = &config->i2c;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_WRN("attr_set currently supported only on SENSOR_CHAN_ALL for this driver.");
    }

    switch (attr) {
    case SENSOR_ATTR_SAMPLING_FREQUENCY:
        uint8_t new_refresh_rate_code;
        if (val->val1 == 0 && val->val2 >= 500000) new_refresh_rate_code = 0;
        else if (val->val1 < 1) new_refresh_rate_code = 0;
        else if (val->val1 == 1) new_refresh_rate_code = 1;
        else if (val->val1 == 2) new_refresh_rate_code = 2;
        else if (val->val1 <= 3) new_refresh_rate_code = 3; // Max 4Hz
        else if (val->val1 <= 7) new_refresh_rate_code = 4; // Max 8Hz
        else if (val->val1 <= 15) new_refresh_rate_code = 5; // Max 16Hz
        else if (val->val1 <= 31) new_refresh_rate_code = 6; // Max 32Hz
        else new_refresh_rate_code = 7; // Max 64Hz

        LOG_DBG("Attempting to set refresh rate to %d.%06d Hz (code %u)", val->val1, val->val2, new_refresh_rate_code);
        ret = MLX90640_SetRefreshRate(slave_addr_direct, new_refresh_rate_code);
        if (ret != 0) {
            LOG_ERR("Failed to set refresh rate (code %u): %d", new_refresh_rate_code, ret);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        LOG_INF("Set refresh rate to code %u", new_refresh_rate_code);
        break;

    case SENSOR_ATTR_MLX90640_RESOLUTION:
        if (val->val1 < 0 || val->val1 > 3) {
            LOG_ERR("Invalid resolution code %d. Must be 0-3.", val->val1);
            mlx90640_i2c_spec_global = NULL; return -EINVAL;
        }
        uint8_t new_resolution_code = (uint8_t)val->val1;
        LOG_DBG("Attempting to set resolution code to %u", new_resolution_code);
        ret = MLX90640_SetResolution(slave_addr_direct, new_resolution_code);
        if (ret != 0) {
            LOG_ERR("Failed to set resolution to code %u: %d", new_resolution_code, ret);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        LOG_INF("Set resolution to code %u", new_resolution_code);
        break;

    case SENSOR_ATTR_MLX90640_MODE:
        if (val->val1 < 0 || val->val1 > 1) {
            LOG_ERR("Invalid mode code %d. Must be 0 (interleaved) or 1 (chess).", val->val1);
            mlx90640_i2c_spec_global = NULL; return -EINVAL;
        }
        uint8_t new_mode_code = (uint8_t)val->val1;
        LOG_DBG("Attempting to set mode code to %u", new_mode_code);
        if (new_mode_code == 0) {
            ret = MLX90640_SetInterleavedMode(slave_addr_direct);
        } else {
            ret = MLX90640_SetChessMode(slave_addr_direct);
        }
        if (ret != 0) {
            LOG_ERR("Failed to set mode to code %u: %d", new_mode_code, ret);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        LOG_INF("Set mode to code %u (%s)", new_mode_code, new_mode_code == 1 ? "Chess" : "Interleaved");
        break;
    
    case SENSOR_ATTR_EMISSIVITY:
        float new_emissivity = (float)val->val1 + (float)val->val2 / 1000000.0f;
        if (new_emissivity < 0.1f || new_emissivity > 1.0f) {
            LOG_ERR("Invalid emissivity value %.3f. Must be between 0.1 and 1.0.", new_emissivity);
            mlx90640_i2c_spec_global = NULL; return -EINVAL;
        }
        data->runtime_emissivity = new_emissivity;
        LOG_INF("Set runtime emissivity to %.3f", data->runtime_emissivity);
        break;

    default:
        LOG_ERR("Unsupported attribute for set: %d", attr);
        mlx90640_i2c_spec_global = NULL; return -ENOTSUP;
    }

    mlx90640_i2c_spec_global = NULL; // Clear global after use
    return 0;
}

static int mlx90640_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
    const struct mlx90640_data *data = dev->data;
    const struct mlx90640_config *config = dev->config;
    uint8_t slave_addr_direct = config->i2c.addr; // Melexis API functions need direct address
    int sensor_raw_val;

    // Set the global I2C spec for Melexis API I2C calls that might happen indirectly.
    // Though for GetCurMode, GetRefreshRate etc., they might do their own I2C reads.
    mlx90640_i2c_spec_global = &config->i2c;


    if (chan != SENSOR_CHAN_ALL) {
        LOG_WRN("attr_get currently supported only on SENSOR_CHAN_ALL for this driver.");
    }

    switch (attr) {
    case SENSOR_ATTR_SAMPLING_FREQUENCY:
        sensor_raw_val = MLX90640_GetRefreshRate(slave_addr_direct);
        if (sensor_raw_val < 0) {
            LOG_ERR("Failed to get refresh rate from sensor: %d", sensor_raw_val);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        if (sensor_raw_val == 0) { val->val1 = 0; val->val2 = 500000; }
        else if (sensor_raw_val > 0 && sensor_raw_val <= 7) { val->val1 = 1 << (sensor_raw_val - 1); val->val2 = 0; }
        else { LOG_ERR("Unknown refresh rate code from sensor: %d", sensor_raw_val); mlx90640_i2c_spec_global = NULL; return -EIO; }
        break;
    
    case SENSOR_ATTR_MLX90640_RESOLUTION:
        sensor_raw_val = MLX90640_GetCurResolution(slave_addr_direct);
         if (sensor_raw_val < 0) {
            LOG_ERR("Failed to get resolution from sensor: %d", sensor_raw_val);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        if (sensor_raw_val < 0 || sensor_raw_val > 3) {
            LOG_ERR("Unknown resolution code from sensor: %d", sensor_raw_val); mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        val->val1 = sensor_raw_val;
        val->val2 = 0;
        break;

    case SENSOR_ATTR_MLX90640_MODE:
        sensor_raw_val = MLX90640_GetCurMode(slave_addr_direct);
        if (sensor_raw_val < 0) {
            LOG_ERR("Failed to get mode from sensor: %d", sensor_raw_val);
            mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        if (sensor_raw_val < 0 || sensor_raw_val > 1) {
             LOG_ERR("Unknown mode code from sensor: %d", sensor_raw_val); mlx90640_i2c_spec_global = NULL; return -EIO;
        }
        val->val1 = sensor_raw_val;
        val->val2 = 0;
        break;

    case SENSOR_ATTR_EMISSIVITY:
        val->val1 = (int32_t)data->runtime_emissivity;
        val->val2 = (int32_t)((data->runtime_emissivity - val->val1) * 1000000);
        break;

    default:
        LOG_ERR("Unsupported attribute for get: %d", attr);
        mlx90640_i2c_spec_global = NULL; return -ENOTSUP;
    }

    mlx90640_i2c_spec_global = NULL; // Clear global after use
    return 0;
}


static int mlx90640_init(const struct device *dev)
{
    struct mlx90640_data *data = dev->data;
    const struct mlx90640_config *config = dev->config;
    int ret;
    uint8_t slave_addr_direct = config->i2c.addr; // For Melexis API calls not using our wrappers directly

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus device not ready (DT: %s, addr: 0x%X)", config->i2c.bus->name, config->i2c.addr);
        return -ENODEV;
    }
    // Set the global I2C device tree specification for the Melexis API I2C calls.
    mlx90640_i2c_spec_global = &config->i2c;

    //k_timeout_t delay = K_TICKS(3646); // 0.03125s+0.08s
    k_timeout_t delay = K_TICKS(CONFIG_SYS_CLOCK_TICKS_PER_SEC * (0.03125f + 0.08f));
    k_sleep(delay); // Wait for sensor to stabilize after power on

    LOG_INF("Initializing MLX90640 sensor at I2C addr 0x%X on bus %s", config->i2c.addr, config->i2c.bus->name);
    LOG_INF("Initial DT Config: Refresh Rate Code: %u, Resolution Code: %u, Mode Code: %u, Emissivity: %.3f",
            config->refresh_rate, config->resolution, config->mode, config->emissivity);

    int retries = 3;
    while(retries > 0) {
        // MLX90640_DumpEE uses our MLX90640_I2CRead wrapper, which uses mlx90640_i2c_spec_global
        ret = MLX90640_DumpEE(slave_addr_direct, data->ee_data);
        if (ret == 0) break;
        LOG_WRN("Failed to dump EEPROM (attempt %d/%d, error %d). Retrying...", (3-retries+1), 3, ret);
        k_sleep(K_MSEC(100));
        retries--;
    }
    if (ret != 0) {
        LOG_ERR("Failed to dump EEPROM after multiple attempts: %d. Check sensor connection and I2C.", ret);
        mlx90640_i2c_spec_global = NULL; return -EIO;
    }
    LOG_INF("EEPROM dumped successfully.");

    ret = MLX90640_ExtractParameters(data->ee_data, &data->mlx90640_params);
    if (ret != 0) {
        LOG_ERR("Failed to extract parameters from EEPROM: %d", ret);
        if (ret == -MLX90640_EEPROM_DATA_ERROR) {
            LOG_ERR("EEPROM data seems corrupted or not a valid MLX90640 EEPROM.");
        }
        mlx90640_i2c_spec_global = NULL; return -EIO;
    }
    LOG_INF("Calibration parameters extracted successfully.");

    // Apply initial sensor configuration from devicetree/defaults
    // These calls use the Melexis API, which will use our I2C wrappers.
    ret = MLX90640_SetRefreshRate(slave_addr_direct, config->refresh_rate);
    if (ret != 0) LOG_WRN("Failed to set initial refresh rate to %u: %d", config->refresh_rate, ret);
    else LOG_DBG("Initial refresh rate set to code %u", config->refresh_rate);
    
    ret = MLX90640_SetResolution(slave_addr_direct, config->resolution);
    if (ret != 0) LOG_WRN("Failed to set initial resolution to %u: %d", config->resolution, ret);
    else LOG_DBG("Initial resolution set to code %u", config->resolution);

    if (config->mode == 0) {
        ret = MLX90640_SetInterleavedMode(slave_addr_direct);
    } else {
        ret = MLX90640_SetChessMode(slave_addr_direct);
    }
    if (ret != 0) LOG_WRN("Failed to set initial mode to %u: %d", config->mode, ret);
    else LOG_DBG("Initial mode set to code %u", config->mode);
    
    data->runtime_emissivity = config->emissivity;

    for (int i = 0; i < MLX90640_NUM_PIXELS; i++) {
        data->temperatures[i] = NAN;
    }
    data->ta = NAN;
    data->vdd = NAN;

    mlx90640_i2c_spec_global = NULL; // Clear global after init
    LOG_INF("MLX90640 initialized successfully on %s.", config->i2c.bus->name);
    return 0;
}

static const struct sensor_driver_api mlx90640_driver_api = {
    .sample_fetch = mlx90640_sample_fetch,
    .channel_get = mlx90640_channel_get,
    .attr_set = mlx90640_attr_set,
    .attr_get = mlx90640_attr_get,
};

#define MLX90640_INIT_DEVICE(inst)                                                              \
    static struct mlx90640_data mlx90640_data_##inst;                                           \
    static const struct mlx90640_config mlx90640_config_##inst = {                              \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                      \
        .refresh_rate = DT_INST_PROP_OR(inst, refresh_rate, 7),                                 \
        .resolution = DT_INST_PROP_OR(inst, resolution, 0),                                     \
        .mode = DT_INST_PROP_OR(inst, mode, 1),                                                 \
        .emissivity = DT_INST_PROP_OR(inst, emissivity, 0.95f),                                 \
    };                                                                                          \
    DEVICE_DT_INST_DEFINE(inst, mlx90640_init, NULL,                                            \
                          &mlx90640_data_##inst, &mlx90640_config_##inst,                       \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                             \
                          &mlx90640_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90640_INIT_DEVICE)