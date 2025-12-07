/**
 * @file mpu9250.h
 * @brief MPU-9250/6500/9255 IMU Driver for ESP-IDF
 * @author Mathias Kulola
 * @date 2024
 * 
 * @details This driver provides a modular interface for MPU-9250/6500/9255
 *          IMU sensors with focus on rocket control system applications.
 *          Features include sensor fusion, calibration, and temperature compensation.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MPU sensor variants
 */
typedef enum {
    MPU_UNKNOWN = 0,
    MPU_6500,       /**< MPU-6500: 6-axis (gyro + accel) */
    MPU_9250,       /**< MPU-9250: 9-axis + AK8963 magnetometer */
    MPU_9255        /**< MPU-9255: 9-axis, improved version */
} mpu_variant_t;

/**
 * @brief Full scale ranges for accelerometer
 */
typedef enum {
    ACCEL_FS_2G = 0,    /**< ±2g */
    ACCEL_FS_4G,        /**< ±4g */
    ACCEL_FS_8G,        /**< ±8g */
    ACCEL_FS_16G        /**< ±16g */
} accel_fs_t;

/**
 * @brief Full scale ranges for gyroscope
 */
typedef enum {
    GYRO_FS_250DPS = 0, /**< ±250°/s */
    GYRO_FS_500DPS,     /**< ±500°/s */
    GYRO_FS_1000DPS,    /**< ±1000°/s */
    GYRO_FS_2000DPS     /**< ±2000°/s */
} gyro_fs_t;

/**
 * @brief Digital low-pass filter settings
 */
typedef enum {
    DLPF_OFF = 0,       /**< No filter, 8kHz sampling */
    DLPF_184HZ,         /**< 184Hz */
    DLPF_92HZ,          /**< 92Hz */
    DLPF_41HZ,          /**< 41Hz */
    DLPF_20HZ,          /**< 20Hz */
    DLPF_10HZ,          /**< 10Hz */
    DLPF_5HZ            /**< 5Hz */
} dlpf_config_t;

/**
 * @brief Calibration modes
 */
typedef enum {
    CALIBRATION_NONE = 0,
    CALIBRATION_ACCEL,
    CALIBRATION_GYRO,
    CALIBRATION_MAG,
    CALIBRATION_ALL
} calibration_mode_t;

/**
 * @brief Sensor data structure for rocket control
 */
typedef struct {
    float accel[3];     /**< Accelerometer data in m/s² [x, y, z] */
    float gyro[3];      /**< Gyroscope data in rad/s [x, y, z] */
    float mag[3];       /**< Magnetometer data in uT [x, y, z] (if available) */
    float temperature;  /**< Temperature in °C */
    uint64_t timestamp; /**< ESP timer timestamp */
    float quaternion[4]; /**< Estimated orientation quaternion [w, x, y, z] */
} imu_data_t;

/**
 * @brief Calibration data structure
 */
typedef struct {
    float accel_bias[3];
    float accel_scale[3];
    float gyro_bias[3];
    float mag_bias[3];
    float mag_scale[3];
    float temperature_bias;
    bool calibrated;
} calibration_data_t;

/**
 * @brief IMU configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_address;
    accel_fs_t accel_fs;
    gyro_fs_t gyro_fs;
    dlpf_config_t dlpf_config;
    uint16_t sample_rate_hz;
    bool enable_mag;
    bool enable_temp;
    uint8_t interrupt_pin;
} mpu_config_t;

/**
 * @brief Euler angles structure
 */
typedef struct {
    float roll;     /**< Roll angle in radians */
    float pitch;    /**< Pitch angle in radians */
    float yaw;      /**< Yaw angle in radians */
} euler_angles_t;

/**
 * @brief IMU handle structure (opaque)
 */
typedef struct mpu9250_dev_t* mpu_handle_t;

/**
 * @brief Initialize MPU sensor
 * 
 * @param config Pointer to configuration structure
 * @return mpu_handle_t Handle to initialized sensor, NULL on failure
 */
mpu_handle_t mpu9250_init(const mpu_config_t* config);

/**
 * @brief Deinitialize MPU sensor and free resources
 * 
 * @param handle Sensor handle
 */
void mpu9250_deinit(mpu_handle_t handle);

/**
 * @brief Detect and identify MPU sensor variant
 * 
 * @param handle Sensor handle
 * @return mpu_variant_t Detected sensor variant
 */
mpu_variant_t mpu9250_detect_variant(mpu_handle_t handle);

/**
 * @brief Perform sensor self-test
 * 
 * @param handle Sensor handle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_self_test(mpu_handle_t handle);

/**
 * @brief Calibrate sensors
 * 
 * @param handle Sensor handle
 * @param mode Calibration mode
 * @param samples Number of samples for calibration (0 = default)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_calibrate(mpu_handle_t handle, calibration_mode_t mode, uint16_t samples);

/**
 * @brief Read all sensor data
 * 
 * @param handle Sensor handle
 * @param data Pointer to data structure to fill
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_read_all(mpu_handle_t handle, imu_data_t* data);

/**
 * @brief Read raw sensor data (for low-level access)
 * 
 * @param handle Sensor handle
 * @param accel_raw Raw accelerometer data (optional)
 * @param gyro_raw Raw gyroscope data (optional)
 * @param mag_raw Raw magnetometer data (optional)
 * @param temp_raw Raw temperature data (optional)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_read_raw(mpu_handle_t handle, int16_t* accel_raw, int16_t* gyro_raw, 
                          int16_t* mag_raw, int16_t* temp_raw);

/**
 * @brief Update sensor fusion algorithm (Mahony/Madgwick)
 * 
 * @param handle Sensor handle
 * @param dt Time delta since last update in seconds
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_update_fusion(mpu_handle_t handle, float dt);

/**
 * @brief Reset sensor fusion to default orientation
 * 
 * @param handle Sensor handle
 */
void mpu9250_reset_fusion(mpu_handle_t handle);

/**
 * @brief Get calibration data
 * 
 * @param handle Sensor handle
 * @param cal_data Pointer to calibration structure to fill
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_get_calibration(mpu_handle_t handle, calibration_data_t* cal_data);

/**
 * @brief Save calibration to NVS (Non-Volatile Storage)
 * 
 * @param handle Sensor handle
 * @param key Storage key name
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_save_calibration(mpu_handle_t handle, const char* key);

/**
 * @brief Load calibration from NVS
 * 
 * @param handle Sensor handle
 * @param key Storage key name
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_load_calibration(mpu_handle_t handle, const char* key);

/**
 * @brief Set sensor wake-on-motion threshold
 * 
 * @param handle Sensor handle
 * @param threshold_mg Threshold in mg (milli-g)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_set_wom_threshold(mpu_handle_t handle, uint8_t threshold_mg);

/**
 * @brief Enable/disable FIFO buffer for burst reads
 * 
 * @param handle Sensor handle
 * @param enable True to enable FIFO
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t mpu9250_set_fifo(mpu_handle_t handle, bool enable);

/**
 * @brief Read multiple samples from FIFO
 * 
 * @param handle Sensor handle
 * @param buffer Buffer to store samples
 * @param max_samples Maximum samples to read
 * @return int Number of samples actually read
 */
int mpu9250_read_fifo(mpu_handle_t handle, imu_data_t* buffer, int max_samples);


/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * 
 * @param q Quaternion [w, x, y, z]
 * @param angles Pointer to Euler angles structure to fill
 * @param convention Rotation convention (0: aerospace ZYX, 1: YXZ for gaming)
 */
void mpu9250_quaternion_to_euler(const float q[4], euler_angles_t* angles, uint8_t convention);

/**
 * @brief Convert quaternion to Euler angles (simplified API, uses aerospace convention)
 * 
 * @param q Quaternion [w, x, y, z]
 * @param roll Pointer to store roll angle in radians
 * @param pitch Pointer to store pitch angle in radians
 * @param yaw Pointer to store yaw angle in radians
 */
void mpu9250_get_euler_angles(const float q[4], float* roll, float* pitch, float* yaw);

/**
 * @brief Convert quaternion to Euler angles in degrees
 * 
 * @param q Quaternion [w, x, y, z]
 * @param roll_deg Pointer to store roll angle in degrees
 * @param pitch_deg Pointer to store pitch angle in degrees
 * @param yaw_deg Pointer to store yaw angle in degrees
 */
void mpu9250_get_euler_angles_deg(const float q[4], float* roll_deg, float* pitch_deg, float* yaw_deg);

/**
 * @brief Get linear acceleration (gravity-removed) in world frame
 * 
 * @param accel_body Accelerometer reading in body frame [x, y, z] m/s²
 * @param q Quaternion [w, x, y, z]
 * @param linear_accel Pointer to store linear acceleration [x, y, z] m/s²
 */
void mpu9250_get_linear_acceleration(const float accel_body[3], const float q[4], float linear_accel[3]);

/**
 * @brief Get gravity vector in body frame
 * 
 * @param q Quaternion [w, x, y, z]
 * @param gravity_body Pointer to store gravity vector in body frame [x, y, z] m/s²
 */
void mpu9250_get_gravity_vector(const float q[4], float gravity_body[3]);

/**
 * @brief Get heading from magnetometer (requires calibrated magnetometer)
 * 
 * @param mag_body Magnetometer reading in body frame [x, y, z] uT
 * @param q Quaternion [w, x, y, z]
 * @return float Heading in radians (0 = North, π/2 = East, etc.)
 */
float mpu9250_get_heading(const float mag_body[3], const float q[4]);

/**
 * @brief Get heading in degrees
 * 
 * @param mag_body Magnetometer reading in body frame [x, y, z] uT
 * @param q Quaternion [w, x, y, z]
 * @return float Heading in degrees (0-360)
 */
float mpu9250_get_heading_deg(const float mag_body[3], const float q[4]);

/**
 * @brief Get rotation matrix from quaternion
 * 
 * @param q Quaternion [w, x, y, z]
 * @param R Pointer to 3x3 rotation matrix (row-major)
 */
void mpu9250_quaternion_to_matrix(const float q[4], float R[9]);

/**
 * @brief Get angular velocity in world frame
 * 
 * @param gyro_body Gyroscope reading in body frame [x, y, z] rad/s
 * @param q Quaternion [w, x, y, z]
 * @param gyro_world Pointer to store angular velocity in world frame [x, y, z] rad/s
 */
void mpu9250_body_to_world_gyro(const float gyro_body[3], const float q[4], float gyro_world[3]);

/**
 * @brief Get orientation as axis-angle representation
 * 
 * @param q Quaternion [w, x, y, z]
 * @param axis Pointer to store rotation axis [x, y, z] (unit vector)
 * @return float Rotation angle in radians
 */
float mpu9250_quaternion_to_axis_angle(const float q[4], float axis[3]);

/**
 * @brief Get tilt angles (useful for rocket orientation)
 * 
 * @param q Quaternion [w, x, y, z]
 * @param tilt_angle Pointer to store total tilt angle from vertical (radians)
 * @param tilt_direction Pointer to store tilt direction (0 = North, π/2 = East, etc.)
 */
void mpu9250_get_tilt_angles(const float q[4], float* tilt_angle, float* tilt_direction);

/**
 * @brief Check if rocket is vertical (for launch detection)
 * 
 * @param q Quaternion [w, x, y, z]
 * @param threshold_rad Maximum deviation from vertical in radians
 * @return true Rocket is within threshold of vertical
 * @return false Rocket is not vertical
 */
bool mpu9250_is_vertical(const float q[4], float threshold_rad);

/**
 * @brief Get acceleration magnitude (total G-force)
 * 
 * @param accel Accelerometer reading [x, y, z] m/s²
 * @return float Acceleration magnitude in m/s²
 */
float mpu9250_get_acceleration_magnitude(const float accel[3]);

/**
 * @brief Get acceleration magnitude in G's
 * 
 * @param accel Accelerometer reading [x, y, z] m/s²
 * @return float Acceleration magnitude in G's
 */
float mpu9250_get_gforce(const float accel[3]);

/**
 * @brief Get angular rate magnitude
 * 
 * @param gyro Gyroscope reading [x, y, z] rad/s
 * @return float Angular rate magnitude in rad/s
 */
float mpu9250_get_angular_rate_magnitude(const float gyro[3]);

/**
 * @brief Get angular rate in degrees per second
 * 
 * @param gyro Gyroscope reading [x, y, z] rad/s
 * @param gyro_dps Pointer to store angular rate [x, y, z] in degrees/s
 */
void mpu9250_get_angular_rate_dps(const float gyro[3], float gyro_dps[3]);

#ifdef __cplusplus
}
#endif


#endif /* MPU9250_H */