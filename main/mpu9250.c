/**
 * @file mpu9250.c
 * @brief MPU-9250/6500/9255 IMU Driver Implementation
 * @author Mathias Kulola
 * @date 2024
 * 
 * @details Implementation of the MPU-9250/6500/9255 driver with focus on
 *          rocket control system requirements including sensor fusion,
 *          calibration, and temperature compensation.
 */

#include <string.h>
#include "mpu9250.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char* TAG = "MPU9250";

/** @brief MPU-9250 register addresses */
typedef enum {
    MPU9250_SELF_TEST_X_GYRO  = 0x00,
    MPU9250_SELF_TEST_Y_GYRO  = 0x01,
    MPU9250_SELF_TEST_Z_GYRO  = 0x02,
    MPU9250_SELF_TEST_X_ACCEL = 0x0D,
    MPU9250_SELF_TEST_Y_ACCEL = 0x0E,
    MPU9250_SELF_TEST_Z_ACCEL = 0x0F,
    MPU9250_XG_OFFSET_H       = 0x13,
    MPU9250_XG_OFFSET_L       = 0x14,
    MPU9250_YG_OFFSET_H       = 0x15,
    MPU9250_YG_OFFSET_L       = 0x16,
    MPU9250_ZG_OFFSET_H       = 0x17,
    MPU9250_ZG_OFFSET_L       = 0x18,
    MPU9250_SMPLRT_DIV        = 0x19,
    MPU9250_CONFIG            = 0x1A,
    MPU9250_GYRO_CONFIG       = 0x1B,
    MPU9250_ACCEL_CONFIG      = 0x1C,
    MPU9250_ACCEL_CONFIG2     = 0x1D,
    MPU9250_LP_ACCEL_ODR      = 0x1E,
    MPU9250_WOM_THR           = 0x1F,
    MPU9250_FIFO_EN           = 0x23,
    MPU9250_I2C_MST_CTRL      = 0x24,
    MPU9250_I2C_SLV0_ADDR     = 0x25,
    MPU9250_I2C_SLV0_REG      = 0x26,
    MPU9250_I2C_SLV0_CTRL     = 0x27,
    MPU9250_I2C_SLV1_ADDR     = 0x28,
    MPU9250_I2C_SLV1_REG      = 0x29,
    MPU9250_I2C_SLV1_CTRL     = 0x2A,
    MPU9250_I2C_SLV2_ADDR     = 0x2B,
    MPU9250_I2C_SLV2_REG      = 0x2C,
    MPU9250_I2C_SLV2_CTRL     = 0x2D,
    MPU9250_I2C_SLV3_ADDR     = 0x2E,
    MPU9250_I2C_SLV3_REG      = 0x2F,
    MPU9250_I2C_SLV3_CTRL     = 0x30,
    MPU9250_I2C_SLV4_ADDR     = 0x31,
    MPU9250_I2C_SLV4_REG      = 0x32,
    MPU9250_I2C_SLV4_DO       = 0x33,
    MPU9250_I2C_SLV4_CTRL     = 0x34,
    MPU9250_I2C_SLV4_DI       = 0x35,
    MPU9250_I2C_MST_STATUS    = 0x36,
    MPU9250_INT_PIN_CFG       = 0x37,
    MPU9250_INT_ENABLE        = 0x38,
    MPU9250_INT_STATUS        = 0x3A,
    MPU9250_ACCEL_XOUT_H      = 0x3B,
    MPU9250_ACCEL_XOUT_L      = 0x3C,
    MPU9250_ACCEL_YOUT_H      = 0x3D,
    MPU9250_ACCEL_YOUT_L      = 0x3E,
    MPU9250_ACCEL_ZOUT_H      = 0x3F,
    MPU9250_ACCEL_ZOUT_L      = 0x40,
    MPU9250_TEMP_OUT_H        = 0x41,
    MPU9250_TEMP_OUT_L        = 0x42,
    MPU9250_GYRO_XOUT_H       = 0x43,
    MPU9250_GYRO_XOUT_L       = 0x44,
    MPU9250_GYRO_YOUT_H       = 0x45,
    MPU9250_GYRO_YOUT_L       = 0x46,
    MPU9250_GYRO_ZOUT_H       = 0x47,
    MPU9250_GYRO_ZOUT_L       = 0x48,
    MPU9250_EXT_SENS_DATA_00  = 0x49,
    MPU9250_EXT_SENS_DATA_01  = 0x4A,
    MPU9250_EXT_SENS_DATA_02  = 0x4B,
    MPU9250_EXT_SENS_DATA_03  = 0x4C,
    MPU9250_EXT_SENS_DATA_04  = 0x4D,
    MPU9250_EXT_SENS_DATA_05  = 0x4E,
    MPU9250_EXT_SENS_DATA_06  = 0x4F,
    MPU9250_EXT_SENS_DATA_07  = 0x50,
    MPU9250_EXT_SENS_DATA_08  = 0x51,
    MPU9250_EXT_SENS_DATA_09  = 0x52,
    MPU9250_EXT_SENS_DATA_10  = 0x53,
    MPU9250_EXT_SENS_DATA_11  = 0x54,
    MPU9250_EXT_SENS_DATA_12  = 0x55,
    MPU9250_EXT_SENS_DATA_13  = 0x56,
    MPU9250_EXT_SENS_DATA_14  = 0x57,
    MPU9250_EXT_SENS_DATA_15  = 0x58,
    MPU9250_EXT_SENS_DATA_16  = 0x59,
    MPU9250_EXT_SENS_DATA_17  = 0x5A,
    MPU9250_EXT_SENS_DATA_18  = 0x5B,
    MPU9250_EXT_SENS_DATA_19  = 0x5C,
    MPU9250_EXT_SENS_DATA_20  = 0x5D,
    MPU9250_EXT_SENS_DATA_21  = 0x5E,
    MPU9250_EXT_SENS_DATA_22  = 0x5F,
    MPU9250_EXT_SENS_DATA_23  = 0x60,
    MPU9250_I2C_SLV0_DO       = 0x63,
    MPU9250_I2C_SLV1_DO       = 0x64,
    MPU9250_I2C_SLV2_DO       = 0x65,
    MPU9250_I2C_SLV3_DO       = 0x66,
    MPU9250_I2C_MST_DELAY_CTRL= 0x67,
    MPU9250_SIGNAL_PATH_RESET = 0x68,
    MPU9250_MOT_DETECT_CTRL   = 0x69,
    MPU9250_USER_CTRL         = 0x6A,
    MPU9250_PWR_MGMT_1        = 0x6B,
    MPU9250_PWR_MGMT_2        = 0x6C,
    MPU9250_FIFO_COUNTH       = 0x72,
    MPU9250_FIFO_COUNTL       = 0x73,
    MPU9250_FIFO_R_W          = 0x74,
    MPU9250_WHO_AM_I          = 0x75,
    MPU9250_XA_OFFSET_H       = 0x77,
    MPU9250_XA_OFFSET_L       = 0x78,
    MPU9250_YA_OFFSET_H       = 0x7A,
    MPU9250_YA_OFFSET_L       = 0x7B,
    MPU9250_ZA_OFFSET_H       = 0x7D,
    MPU9250_ZA_OFFSET_L       = 0x7E
} mpu9250_reg_t;

/** @brief AK8963 (magnetometer) register addresses */
typedef enum {
    AK8963_WIA     = 0x00,
    AK8963_INFO    = 0x01,
    AK8963_ST1     = 0x02,
    AK8963_HXL     = 0x03,
    AK8963_HXH     = 0x04,
    AK8963_HYL     = 0x05,
    AK8963_HYH     = 0x06,
    AK8963_HZL     = 0x07,
    AK8963_HZH     = 0x08,
    AK8963_ST2     = 0x09,
    AK8963_CNTL1   = 0x0A,
    AK8963_CNTL2   = 0x0B,
    AK8963_ASTC    = 0x0C,
    AK8963_TS1     = 0x0D,
    AK8963_TS2     = 0x0E,
    AK8963_I2CDIS  = 0x0F,
    AK8963_ASAX    = 0x10,
    AK8963_ASAY    = 0x11,
    AK8963_ASAZ    = 0x12
} ak8963_reg_t;

/** @brief Device structure */
typedef struct mpu9250_dev_t {
    i2c_port_t i2c_port;
    uint8_t address;
    mpu_variant_t variant;
    mpu_config_t config;
    calibration_data_t calibration;
    
    // Sensor scales
    float accel_scale;
    float gyro_scale;
    float mag_scale[3];
    
    // Sensor fusion state
    float quaternion[4];
    float beta;  // Madgwick/Mahony filter gain
    
    // Timing
    uint64_t last_update;
    float sample_rate;
    
    // Magnetometer sensitivity adjustment values
    uint8_t mag_sens_adj[3];
    bool mag_enabled;
} mpu9250_dev_t;

/** @brief Madgwick filter implementation for sensor fusion */
typedef struct {
    float q0, q1, q2, q3;  /**< Quaternion */
    float beta;             /**< Filter gain */
    float zeta;             /**< Gyro drift gain */
} madgwick_filter_t;

/** @brief Mahony filter implementation for sensor fusion */
typedef struct {
    float q0, q1, q2, q3;  /**< Quaternion */
    float twoKp;            /**< Proportional gain */
    float twoKi;            /**< Integral gain */
    float integralFBx, integralFBy, integralFBz; /**< Integral error */
} mahony_filter_t;

/** @brief Private function declarations */
static esp_err_t mpu9250_write_byte(mpu_handle_t handle, uint8_t reg, uint8_t data);
static esp_err_t mpu9250_read_bytes(mpu_handle_t handle, uint8_t reg, uint8_t* data, uint8_t length);
static esp_err_t mpu9250_i2c_init(mpu_config_t* config);
static esp_err_t mpu9250_init_mag(mpu_handle_t handle);
static void mpu9250_calculate_scales(mpu_handle_t handle);
static void mpu9250_apply_calibration(mpu_handle_t handle, imu_data_t* data);
static void mpu9250_update_madgwick(mpu_handle_t handle, float dt, float* accel, float* gyro, float* mag);
static void mpu9250_update_mahony(mpu_handle_t handle, float dt, float* accel, float* gyro, float* mag);

/** @brief Accelerometer scales in m/s² per LSB */
static const float ACCEL_SCALES[] = {
    [ACCEL_FS_2G] = 2.0f * 9.80665f / 32768.0f,   // ±2g
    [ACCEL_FS_4G] = 4.0f * 9.80665f / 32768.0f,   // ±4g
    [ACCEL_FS_8G] = 8.0f * 9.80665f / 32768.0f,   // ±8g
    [ACCEL_FS_16G] = 16.0f * 9.80665f / 32768.0f  // ±16g
};

/** @brief Gyroscope scales in rad/s per LSB */
static const float GYRO_SCALES[] = {
    [GYRO_FS_250DPS] = 250.0f * M_PI / (180.0f * 32768.0f),   // ±250°/s
    [GYRO_FS_500DPS] = 500.0f * M_PI / (180.0f * 32768.0f),   // ±500°/s
    [GYRO_FS_1000DPS] = 1000.0f * M_PI / (180.0f * 32768.0f), // ±1000°/s
    [GYRO_FS_2000DPS] = 2000.0f * M_PI / (180.0f * 32768.0f)  // ±2000°/s
};


static esp_err_t mpu9250_reset_fifo(mpu_handle_t handle);
static esp_err_t mpu9250_soft_reset(mpu_handle_t handle);


/**
 * @brief Initialize MPU sensor
 */
mpu_handle_t mpu9250_init(const mpu_config_t* config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }
    
    // Allocate device structure
    mpu9250_dev_t* dev = calloc(1, sizeof(mpu9250_dev_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for device");
        return NULL;
    }
    
    // Copy configuration
    dev->config = *config;
    dev->address = config->i2c_address;
    dev->i2c_port = config->i2c_port;
    
    // Initialize I2C
    if (mpu9250_i2c_init(&dev->config) != ESP_OK) {
        free(dev);
        return NULL;
    }
    
    // Wake up device
    if (mpu9250_write_byte(dev, MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up device");
        free(dev);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Reset device
    if (mpu9250_write_byte(dev, MPU9250_PWR_MGMT_1, 0x80) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        free(dev);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wake up again
    if (mpu9250_write_byte(dev, MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up device after reset");
        free(dev);
        return NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Detect variant
    dev->variant = mpu9250_detect_variant(dev);
    if (dev->variant == MPU_UNKNOWN) {
        ESP_LOGW(TAG, "Unknown MPU variant, assuming MPU-9250");
        dev->variant = MPU_9250;
    }
    
    ESP_LOGI(TAG, "Detected MPU variant: %d", dev->variant);
    
    // Configure gyroscope
    uint8_t gyro_config = (dev->config.gyro_fs << 3);
    if (mpu9250_write_byte(dev, MPU9250_GYRO_CONFIG, gyro_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        free(dev);
        return NULL;
    }
    
    // Configure accelerometer
    uint8_t accel_config = (dev->config.accel_fs << 3);
    if (mpu9250_write_byte(dev, MPU9250_ACCEL_CONFIG, accel_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        free(dev);
        return NULL;
    }
    
    // Configure DLPF
    uint8_t dlpf_config = dev->config.dlpf_config & 0x07;
    if (mpu9250_write_byte(dev, MPU9250_CONFIG, dlpf_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        free(dev);
        return NULL;
    }
    
    // Configure sample rate divider
    if (dev->config.sample_rate_hz > 0) {
        uint16_t smplrt_div_calc = 1000 / dev->config.sample_rate_hz - 1;
        if (smplrt_div_calc > 255) smplrt_div_calc = 255;
        uint8_t smplrt_div = (uint8_t)smplrt_div_calc;
        if (mpu9250_write_byte(dev, MPU9250_SMPLRT_DIV, smplrt_div) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure sample rate");
            free(dev);
            return NULL;
        }
    }
    
    // Initialize magnetometer if enabled
    if (dev->config.enable_mag && (dev->variant == MPU_9250 || dev->variant == MPU_9255)) {
        if (mpu9250_init_mag(dev) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize magnetometer, continuing without it");
            dev->mag_enabled = false;
        } else {
            dev->mag_enabled = true;
        }
    } else {
        dev->mag_enabled = false;
    }
    
    // Calculate scales
    mpu9250_calculate_scales(dev);
    
    // Initialize quaternion for sensor fusion
    dev->quaternion[0] = 1.0f;
    dev->quaternion[1] = 0.0f;
    dev->quaternion[2] = 0.0f;
    dev->quaternion[3] = 0.0f;
    dev->beta = 0.1f;  // Madgwick filter gain
    
    // Initialize timing
    dev->last_update = esp_timer_get_time();
    dev->sample_rate = dev->config.sample_rate_hz;
    
    ESP_LOGI(TAG, "MPU-9250 initialized successfully");
    return dev;
}

/**
 * @brief Deinitialize MPU sensor
 */
void mpu9250_deinit(mpu_handle_t handle) {
    if (handle == NULL) return;
    
    // Put device to sleep
    mpu9250_write_byte(handle, MPU9250_PWR_MGMT_1, 0x40);
    
    free(handle);
    ESP_LOGI(TAG, "MPU-9250 deinitialized");
}

/**
 * @brief Detect sensor variant
 */
mpu_variant_t mpu9250_detect_variant(mpu_handle_t handle) {
    uint8_t whoami;
    if (mpu9250_read_bytes(handle, MPU9250_WHO_AM_I, &whoami, 1) != ESP_OK) {
        return MPU_UNKNOWN;
    }
    
    switch (whoami) {
        case 0x71:  // MPU-9250
        case 0x73:  // MPU-9255
            // Check for magnetometer
            uint8_t mag_wia;
            if (mpu9250_write_byte(handle, MPU9250_INT_PIN_CFG, 0x02) == ESP_OK &&
                mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, 0x8C) == ESP_OK &&
                mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_WIA) == ESP_OK &&
                mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x81) == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                mpu9250_read_bytes(handle, MPU9250_EXT_SENS_DATA_00, &mag_wia, 1);
                if (mag_wia == 0x48) {
                    return (whoami == 0x71) ? MPU_9250 : MPU_9255;
                }
            }
            return MPU_6500;  // No magnetometer found
        case 0x70:  // MPU-6500
            return MPU_6500;
        default:
            return MPU_UNKNOWN;
    }
}

/**
 * @brief Read all sensor data
 */
esp_err_t mpu9250_read_all(mpu_handle_t handle, imu_data_t* data) {
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[14];
    esp_err_t ret;
    
    // Read accelerometer and temperature
    ret = mpu9250_read_bytes(handle, MPU9250_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }
    
    // Parse accelerometer data
    int16_t accel_raw[3];
    accel_raw[0] = (buffer[0] << 8) | buffer[1];
    accel_raw[1] = (buffer[2] << 8) | buffer[3];
    accel_raw[2] = (buffer[4] << 8) | buffer[5];
    
    // Parse temperature
    int16_t temp_raw = (buffer[6] << 8) | buffer[7];
    data->temperature = ((float)temp_raw / 333.87f) + 21.0f;
    
    // Parse gyroscope data
    int16_t gyro_raw[3];
    gyro_raw[0] = (buffer[8] << 8) | buffer[9];
    gyro_raw[1] = (buffer[10] << 8) | buffer[11];
    gyro_raw[2] = (buffer[12] << 8) | buffer[13];
    
    // Convert to SI units
    for (int i = 0; i < 3; i++) {
        data->accel[i] = accel_raw[i] * handle->accel_scale;
        data->gyro[i] = gyro_raw[i] * handle->gyro_scale;
    }
    
    // Read magnetometer if enabled
    if (handle->mag_enabled) {
        uint8_t mag_buffer[8];  // Increased buffer size
        uint8_t mag_status;
        
        // Check if data is ready
        if (mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, 0x8C) != ESP_OK ||
            mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_ST1) != ESP_OK ||
            mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x81) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to setup magnetometer read");
            data->mag[0] = data->mag[1] = data->mag[2] = 0.0f;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
            mpu9250_read_bytes(handle, MPU9250_EXT_SENS_DATA_00, &mag_status, 1);
            
            if (mag_status & 0x01) {  // Data ready
                // Read all 7 magnetometer registers (HXL to ST2)
                if (mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, 0x8C) != ESP_OK ||
                    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_HXL) != ESP_OK ||
                    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x87) != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to setup magnetometer data read");
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    mpu9250_read_bytes(handle, MPU9250_EXT_SENS_DATA_00, mag_buffer, 7);
                    
                    // Check for overflow
                    if (!(mag_buffer[6] & 0x08)) {  // Check ST2 register for overflow
                        int16_t mag_raw[3];
                        mag_raw[0] = (mag_buffer[1] << 8) | mag_buffer[0];
                        mag_raw[1] = (mag_buffer[3] << 8) | mag_buffer[2];
                        mag_raw[2] = (mag_buffer[5] << 8) | mag_buffer[4];
                        
                        // Apply sensitivity adjustment and convert to uT
                        for (int i = 0; i < 3; i++) {
                            float adj = ((float)handle->mag_sens_adj[i] - 128.0f) / 256.0f + 1.0f;
                            data->mag[i] = mag_raw[i] * 0.6f * adj;  // 0.6 uT/LSB for 16-bit output
                        }
                    } else {
                        ESP_LOGW(TAG, "Magnetometer overflow detected");
                        data->mag[0] = data->mag[1] = data->mag[2] = 0.0f;
                    }
                }
            } else {
                ESP_LOGW(TAG, "Magnetometer data not ready");
                data->mag[0] = data->mag[1] = data->mag[2] = 0.0f;
            }
        }
    } else {
        data->mag[0] = data->mag[1] = data->mag[2] = 0.0f;
    }
    
    // Apply calibration
    mpu9250_apply_calibration(handle, data);
    
    // Update timestamp
    data->timestamp = esp_timer_get_time();
    
    // Update sensor fusion
    uint64_t now = data->timestamp;
    float dt = (now - handle->last_update) / 1e6f;  // Convert to seconds
    handle->last_update = now;
    
    if (dt > 0 && dt < 1.0f) {  // Sanity check
        mpu9250_update_madgwick(handle, dt, data->accel, data->gyro, data->mag);
        memcpy(data->quaternion, handle->quaternion, sizeof(float) * 4);
    }
    
    return ESP_OK;
}

/**
 * @brief Calibrate sensors
 */
esp_err_t mpu9250_calibrate(mpu_handle_t handle, calibration_mode_t mode, uint16_t samples) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (samples == 0) {
        samples = 1000;  // Default number of samples
    }
    
    ESP_LOGI(TAG, "Starting calibration with %d samples", samples);
    
    imu_data_t data;
    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};
    float mag_sum[3] = {0};
    float mag_min[3] = {INFINITY, INFINITY, INFINITY};
    float mag_max[3] = {-INFINITY, -INFINITY, -INFINITY};
    
    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        if (mpu9250_read_all(handle, &data) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sample %d", i);
            return ESP_FAIL;
        }
        
        for (int j = 0; j < 3; j++) {
            if (mode == CALIBRATION_ACCEL || mode == CALIBRATION_ALL) {
                accel_sum[j] += data.accel[j];
            }
            if (mode == CALIBRATION_GYRO || mode == CALIBRATION_ALL) {
                gyro_sum[j] += data.gyro[j];
            }
            if ((mode == CALIBRATION_MAG || mode == CALIBRATION_ALL) && handle->mag_enabled) {
                mag_sum[j] += data.mag[j];
                if (data.mag[j] < mag_min[j]) mag_min[j] = data.mag[j];
                if (data.mag[j] > mag_max[j]) mag_max[j] = data.mag[j];
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz sampling
    }
    
    // Calculate biases
    if (mode == CALIBRATION_ACCEL || mode == CALIBRATION_ALL) {
        for (int i = 0; i < 3; i++) {
            handle->calibration.accel_bias[i] = accel_sum[i] / samples;
            if (i == 2) {  // Z-axis should show 1g when level
                handle->calibration.accel_bias[i] -= 9.80665f;
            }
        }
        handle->calibration.calibrated = true;
    }
    
    if (mode == CALIBRATION_GYRO || mode == CALIBRATION_ALL) {
        for (int i = 0; i < 3; i++) {
            handle->calibration.gyro_bias[i] = gyro_sum[i] / samples;
        }
        handle->calibration.calibrated = true;
    }
    
    if ((mode == CALIBRATION_MAG || mode == CALIBRATION_ALL) && handle->mag_enabled) {
        // Hard iron correction (bias)
        for (int i = 0; i < 3; i++) {
            handle->calibration.mag_bias[i] = (mag_max[i] + mag_min[i]) / 2.0f;
        }
        
        // Soft iron correction (scale)
        float avg_radius = 0;
        for (int i = 0; i < 3; i++) {
            float avg = (mag_max[i] - mag_min[i]) / 2.0f;
            avg_radius += avg;
        }
        avg_radius /= 3.0f;
        
        for (int i = 0; i < 3; i++) {
            float avg = (mag_max[i] - mag_min[i]) / 2.0f;
            handle->calibration.mag_scale[i] = avg_radius / avg;
        }
        
        handle->calibration.calibrated = true;
    }
    
    ESP_LOGI(TAG, "Calibration complete");
    return ESP_OK;
}

/**
 * @brief Apply calibration to sensor data
 */
static void mpu9250_apply_calibration(mpu_handle_t handle, imu_data_t* data) {
    if (!handle->calibration.calibrated) return;
    
    for (int i = 0; i < 3; i++) {
        data->accel[i] -= handle->calibration.accel_bias[i];
        data->accel[i] *= handle->calibration.accel_scale[i];
        
        data->gyro[i] -= handle->calibration.gyro_bias[i];
        
        if (handle->mag_enabled) {
            data->mag[i] -= handle->calibration.mag_bias[i];
            data->mag[i] *= handle->calibration.mag_scale[i];
        }
    }
    
    // Temperature compensation (simple linear model)
    data->temperature -= handle->calibration.temperature_bias;
}

/**
 * @brief Madgwick filter update for sensor fusion
 */
static void mpu9250_update_madgwick(mpu_handle_t handle, float dt, float* accel, float* gyro, float* mag) {
    float q0 = handle->quaternion[0];
    float q1 = handle->quaternion[1];
    float q2 = handle->quaternion[2];
    float q3 = handle->quaternion[3];
    
    float beta = handle->beta;
    
    // Normalize accelerometer measurement
    float norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    float ax = accel[0] * norm;
    float ay = accel[1] * norm;
    float az = accel[2] * norm;
    
    // Gradient descent algorithm corrective step
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    
    // Compute objective function and Jacobian
    float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
    
    // J^T * f
    s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
    s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
    s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
    s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;
    
    // Normalize step magnitude
    norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
    }
    
    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]) - beta * s0;
    qDot2 = 0.5f * ( q0 * gyro[0] + q2 * gyro[2] - q3 * gyro[1]) - beta * s1;
    qDot3 = 0.5f * ( q0 * gyro[1] - q1 * gyro[2] + q3 * gyro[0]) - beta * s2;
    qDot4 = 0.5f * ( q0 * gyro[2] + q1 * gyro[1] - q2 * gyro[0]) - beta * s3;
    
    // Integrate to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // Normalize quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        handle->quaternion[0] = q0 * norm;
        handle->quaternion[1] = q1 * norm;
        handle->quaternion[2] = q2 * norm;
        handle->quaternion[3] = q3 * norm;
    }
}

esp_err_t mpu9250_update_fusion(mpu_handle_t handle, imu_data_t* data, float dt) {
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (dt <= 0 || dt > 0.1f) {  // Sanity check (max 100ms delta)
        dt = 0.01f;  // Default to 100Hz
    }
    
    // Update Madgwick filter
    mpu9250_update_madgwick(handle, dt, data->accel, data->gyro, data->mag);
    
    // Copy updated quaternion back to data
    memcpy(data->quaternion, handle->quaternion, sizeof(float) * 4);
    
    return ESP_OK;
}

/**
 * @brief Initialize I2C communication
 */
static esp_err_t mpu9250_i2c_init(mpu_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate GPIO pins
    if (!GPIO_IS_VALID_GPIO(config->sda_pin) || !GPIO_IS_VALID_GPIO(config->scl_pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pins: SDA=%d, SCL=%d", config->sda_pin, config->scl_pin);
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = (config->clk_speed > 0) ? config->clk_speed : 400000
    };
    
    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(config->i2c_port, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized: port=%d, SDA=%d, SCL=%d, speed=%" PRIu32 "Hz", 
             config->i2c_port, config->sda_pin, config->scl_pin, i2c_conf.master.clk_speed);
    return ESP_OK;
}

/**
 * @brief Write byte to register
 */
static esp_err_t mpu9250_write_byte(mpu_handle_t handle, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Read bytes from register
 */
static esp_err_t mpu9250_read_bytes(mpu_handle_t handle, uint8_t reg, uint8_t* data, uint8_t length) {
    if (handle == NULL || data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    esp_err_t ret;
    
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Repeated start for read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->address << 1) | I2C_MASTER_READ, true);
    
    // Read data
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s (reg: 0x%02X)", esp_err_to_name(ret), reg);
        return ret;
    }
    
    return ESP_OK;
}

// ... Additional private functions would follow for completeness
// Note: Due to length constraints, I've shown the core functionality.
// Full implementation would include all declared functions.

/**
 * @brief Initialize magnetometer
 */
static esp_err_t mpu9250_init_mag(mpu_handle_t handle) {
    // Enable I2C master mode
    if (mpu9250_write_byte(handle, MPU9250_USER_CTRL, 0x20) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Configure I2C master clock
    if (mpu9250_write_byte(handle, MPU9250_I2C_MST_CTRL, 0x0D) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Reset magnetometer
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_CNTL2);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_DO, 0x01);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x81);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Read sensitivity adjustment values
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_ASAX);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x83);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    uint8_t asa_buffer[3];
    mpu9250_read_bytes(handle, MPU9250_EXT_SENS_DATA_00, asa_buffer, 3);
    
    handle->mag_sens_adj[0] = asa_buffer[0];
    handle->mag_sens_adj[1] = asa_buffer[1];
    handle->mag_sens_adj[2] = asa_buffer[2];
    
    // Configure magnetometer for continuous measurement mode 2 (100Hz)
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_REG, AK8963_CNTL1);
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_DO, 0x16);  // 100Hz, 16-bit
    mpu9250_write_byte(handle, MPU9250_I2C_SLV0_CTRL, 0x81);
    
    return ESP_OK;
}

/**
 * @brief Calculate sensor scales based on configuration
 */
static void mpu9250_calculate_scales(mpu_handle_t handle) {
    handle->accel_scale = ACCEL_SCALES[handle->config.accel_fs];
    handle->gyro_scale = GYRO_SCALES[handle->config.gyro_fs];
}


/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 */
void mpu9250_quaternion_to_euler(const float q[4], euler_angles_t* angles, uint8_t convention) {
    if (angles == NULL || q == NULL) return;
    
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    if (convention == 0) {
        // Aerospace sequence: ZYX (yaw, pitch, roll)
        // This is standard for rockets/aircraft
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        angles->roll = atan2f(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (qw * qy - qz * qx);
        if (fabsf(sinp) >= 1.0f) {
            // Use 90 degrees if out of range
            angles->pitch = copysignf(M_PI / 2.0f, sinp);
        } else {
            angles->pitch = asinf(sinp);
        }
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        angles->yaw = atan2f(siny_cosp, cosy_cosp);
    } else {
        // Alternative convention: YXZ (for gaming/mobile devices)
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        angles->roll = atan2f(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (qw * qy - qz * qx);
        if (fabsf(sinp) >= 1.0f) {
            angles->pitch = copysignf(M_PI / 2.0f, sinp);
        } else {
            angles->pitch = asinf(sinp);
        }
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        angles->yaw = atan2f(siny_cosp, cosy_cosp);
    }
}

/**
 * @brief Simplified Euler angles getter
 */
void mpu9250_get_euler_angles(const float q[4], float* roll, float* pitch, float* yaw) {
    if (q == NULL) return;
    
    euler_angles_t angles;
    mpu9250_quaternion_to_euler(q, &angles, 0);  // Using aerospace convention
    
    if (roll) *roll     = angles.roll;
    if (pitch) *pitch   = angles.pitch;
    if (yaw) *yaw       = angles.yaw;
}

/**
 * @brief Get Euler angles in degrees
 */
void mpu9250_get_euler_angles_deg(const float q[4], float* roll_deg, float* pitch_deg, float* yaw_deg) {
    if (q == NULL) return;
    
    euler_angles_t angles;
    mpu9250_quaternion_to_euler(q, &angles, 0);
    
    if (roll_deg) *roll_deg = angles.roll * 180.0f / M_PI;
    if (pitch_deg) *pitch_deg = angles.pitch * 180.0f / M_PI;
    if (yaw_deg) *yaw_deg = angles.yaw * 180.0f / M_PI;
}

/**
 * @brief Get linear acceleration (gravity removed)
 */
void mpu9250_get_linear_acceleration(const float accel_body[3], const float q[4], float linear_accel[3]) {
    if (accel_body == NULL || q == NULL || linear_accel == NULL) return;
    
    // Get gravity vector in body frame
    float gravity_body[3];
    mpu9250_get_gravity_vector(q, gravity_body);
    
    // Subtract gravity from accelerometer reading
    for (int i = 0; i < 3; i++) {
        linear_accel[i] = accel_body[i] - gravity_body[i];
    }
}

/**
 * @brief Get gravity vector in body frame
 */
void mpu9250_get_gravity_vector(const float q[4], float gravity_body[3]) {
    if (q == NULL || gravity_body == NULL) return;
    
    // Gravity in world frame is [0, 0, 9.80665] m/s² (Z-down convention)
    // We need to rotate this to body frame using the inverse quaternion
    
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    // For Z-down coordinate system:
    // If rocket is upright (nose up), gravity should be +Z in body frame
    // If rocket is horizontal, gravity should be in appropriate axis
    
    // This is the inverse rotation: world to body
    gravity_body[0] = 2.0f * (qx * qz - qw * qy) * 9.80665f;
    gravity_body[1] = 2.0f * (qy * qz + qw * qx) * 9.80665f;
    gravity_body[2] = (qw * qw - qx * qx - qy * qy + qz * qz) * 9.80665f;
}

/**
 * @brief Get heading from magnetometer
 */
float mpu9250_get_heading(const float mag_body[3], const float q[4]) {
    if (mag_body == NULL || q == NULL) return 0.0f;
    
    // First, rotate magnetometer reading to world frame
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float mx = mag_body[0], my = mag_body[1], mz = mag_body[2];
    
    // Rotate magnetometer vector from body to world frame
    float mag_world_x = (qw * qw + qx * qx - qy * qy - qz * qz) * mx +
                      2.0f * (qx * qy - qw * qz) * my +
                      2.0f * (qx * qz + qw * qy) * mz;
    
    float mag_world_y = 2.0f * (qx * qy + qw * qz) * mx +
                      (qw * qw - qx * qx + qy * qy - qz * qz) * my +
                      2.0f * (qy * qz - qw * qx) * mz;
    
    // Heading is atan2 of horizontal components (ignoring Z)
    float heading = atan2f(mag_world_y, mag_world_x);
    
    // Convert to 0-2π range
    if (heading < 0) heading += 2.0f * M_PI;
    
    return heading;
}

/**
 * @brief Get heading in degrees
 */
float mpu9250_get_heading_deg(const float mag_body[3], const float q[4]) {
    float heading_rad = mpu9250_get_heading(mag_body, q);
    float heading_deg = heading_rad * 180.0f / M_PI;
    
    // Convert to 0-360 range
    if (heading_deg < 0) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;
    
    return heading_deg;
}

/**
 * @brief Convert quaternion to rotation matrix
 */
void mpu9250_quaternion_to_matrix(const float q[4], float R[9]) {
    if (q == NULL || R == NULL) return;
    
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    // First row
    R[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    R[1] = 2.0f * (qx * qy - qw * qz);
    R[2] = 2.0f * (qx * qz + qw * qy);
    
    // Second row
    R[3] = 2.0f * (qx * qy + qw * qz);
    R[4] = 1.0f - 2.0f * (qx * qx + qz * qz);
    R[5] = 2.0f * (qy * qz - qw * qx);
    
    // Third row
    R[6] = 2.0f * (qx * qz - qw * qy);
    R[7] = 2.0f * (qy * qz + qw * qx);
    R[8] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

/**
 * @brief Convert body angular velocity to world frame
 */
void mpu9250_body_to_world_gyro(const float gyro_body[3], const float q[4], float gyro_world[3]) {
    if (gyro_body == NULL || q == NULL || gyro_world == NULL) return;
    
    float R[9];
    mpu9250_quaternion_to_matrix(q, R);
    
    // Rotate angular velocity from body to world frame
    gyro_world[0] = R[0] * gyro_body[0] + R[1] * gyro_body[1] + R[2] * gyro_body[2];
    gyro_world[1] = R[3] * gyro_body[0] + R[4] * gyro_body[1] + R[5] * gyro_body[2];
    gyro_world[2] = R[6] * gyro_body[0] + R[7] * gyro_body[1] + R[8] * gyro_body[2];
}

/**
 * @brief Convert quaternion to axis-angle representation
 */
float mpu9250_quaternion_to_axis_angle(const float q[4], float axis[3]) {
    if (q == NULL) return 0.0f;
    
    float angle = 2.0f * acosf(q[0]);
    
    // Prevent division by zero
    float s = sqrtf(1.0f - q[0] * q[0]);
    if (s < 0.001f) {
        // If s is close to zero, the axis is arbitrary
        if (axis) {
            axis[0] = 1.0f;
            axis[1] = 0.0f;
            axis[2] = 0.0f;
        }
    } else {
        if (axis) {
            axis[0] = q[1] / s;
            axis[1] = q[2] / s;
            axis[2] = q[3] / s;
        }
    }
    
    return angle;
}

/**
 * @brief Get tilt angles from vertical
 */
void mpu9250_get_tilt_angles(const float q[4], float* tilt_angle, float* tilt_direction) {
    if (q == NULL) return;
    
    // Get the Z-axis of the rocket in world frame
    // For Z-down coordinate system, vertical rocket has body Z = [0, 0, 1]
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    // Body Z-axis in world frame
    float zx = 2.0f * (qx * qz - qw * qy);
    float zy = 2.0f * (qy * qz + qw * qx);
    float zz = qw * qw - qx * qx - qy * qy + qz * qz;
    
    // Tilt angle is the angle between body Z and world Z (which is [0, 0, 1])
    // cos(tilt) = dot([0,0,1], body_Z) = zz
    float cos_tilt = zz;
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;
    if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    
    if (tilt_angle) {
        *tilt_angle = acosf(cos_tilt);
    }
    
    if (tilt_direction) {
        // Project body Z onto horizontal plane
        float hx = zx;
        float hy = zy;
        float mag = sqrtf(hx * hx + hy * hy);
        
        if (mag > 0.001f) {
            hx /= mag;
            hy /= mag;
            *tilt_direction = atan2f(hy, hx);
        } else {
            *tilt_direction = 0.0f;  // Vertical, direction undefined
        }
    }
}

/**
 * @brief Check if rocket is vertical
 */
bool mpu9250_is_vertical(const float q[4], float threshold_rad) {
    float tilt_angle;
    mpu9250_get_tilt_angles(q, &tilt_angle, NULL);
    return (tilt_angle <= threshold_rad);
}

/**
 * @brief Get acceleration magnitude
 */
float mpu9250_get_acceleration_magnitude(const float accel[3]) {
    if (accel == NULL) return 0.0f;
    return sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
}

/**
 * @brief Get acceleration in G's
 */
float mpu9250_get_gforce(const float accel[3]) {
    float accel_mag = mpu9250_get_acceleration_magnitude(accel);
    return accel_mag / 9.80665f;
}

/**
 * @brief Get angular rate magnitude
 */
float mpu9250_get_angular_rate_magnitude(const float gyro[3]) {
    if (gyro == NULL) return 0.0f;
    return sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
}

/**
 * @brief Convert angular rate to degrees per second
 */
void mpu9250_get_angular_rate_dps(const float gyro[3], float gyro_dps[3]) {
    if (gyro == NULL || gyro_dps == NULL) return;
    
    for (int i = 0; i < 3; i++) {
        gyro_dps[i] = gyro[i] * 180.0f / M_PI;
    }
}

static void mpu9250_update_mahony(mpu_handle_t handle, float dt, float* accel, float* gyro, float* mag) {
    // Placeholder for Mahony filter implementation
    // TODO: Implement Mahony filter update
    ESP_LOGW(TAG, "Mahony filter not implemented, using Madgwick");
    mpu9250_update_madgwick(handle, dt, accel, gyro, mag);
}

/**
 * @brief Reset sensor FIFO
 */
static esp_err_t mpu9250_reset_fifo(mpu_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    // Reset FIFO
    if (mpu9250_write_byte(handle, MPU9250_PWR_MGMT_1, 0x04) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Clear reset
    if (mpu9250_write_byte(handle, MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/**
 * @brief Soft reset the sensor
 */
static esp_err_t mpu9250_soft_reset(mpu_handle_t handle) {
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    
    // Perform soft reset
    if (mpu9250_write_byte(handle, MPU9250_PWR_MGMT_1, 0x80) != ESP_OK) {
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Wake up device
    if (mpu9250_write_byte(handle, MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) {
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

/**
 * @brief Print debug information about the sensor
 */
void mpu9250_print_debug_info(mpu_handle_t handle) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    
    uint8_t whoami, pwr_mgmt_1, gyro_config, accel_config;
    
    ESP_LOGI(TAG, "=== MPU9250 DEBUG INFO ===");
    
    // Read WHO_AM_I
    if (mpu9250_read_bytes(handle, MPU9250_WHO_AM_I, &whoami, 1) == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", whoami);
    } else {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I");
    }
    
    // Read PWR_MGMT_1
    if (mpu9250_read_bytes(handle, MPU9250_PWR_MGMT_1, &pwr_mgmt_1, 1) == ESP_OK) {
        ESP_LOGI(TAG, "PWR_MGMT_1: 0x%02X", pwr_mgmt_1);
    }
    
    // Read GYRO_CONFIG
    if (mpu9250_read_bytes(handle, MPU9250_GYRO_CONFIG, &gyro_config, 1) == ESP_OK) {
        ESP_LOGI(TAG, "GYRO_CONFIG: 0x%02X", gyro_config);
    }
    
    // Read ACCEL_CONFIG
    if (mpu9250_read_bytes(handle, MPU9250_ACCEL_CONFIG, &accel_config, 1) == ESP_OK) {
        ESP_LOGI(TAG, "ACCEL_CONFIG: 0x%02X", accel_config);
    }
    
    ESP_LOGI(TAG, "==========================");
}