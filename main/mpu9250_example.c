/**
 * @file mpu9250_example.c
 * @brief Example usage of MPU-9250 driver for rocket control system
 * @author Mathias Kulola
 * @date 2024
 */

#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MPU9250_EXAMPLE";

void mpu9250_example_task(void* pvParameters) {
    ESP_LOGI(TAG, "Starting MPU-9250 example for rocket control");
    
    // Configure MPU-9250
    mpu_config_t config = {
        .i2c_port = I2C_NUM_0,
        .i2c_address = MPU9250_I2C_ADDR,
        .accel_fs = ACCEL_FS_16G,       // Rocket high-G environment
        .gyro_fs = GYRO_FS_2000DPS,     // High rotation rates expected
        .dlpf_config = DLPF_184HZ,      // Balance noise and bandwidth
        .sample_rate_hz = 200,          // 200Hz update rate
        .enable_mag = true,             // Enable magnetometer for heading
        .enable_temp = true,            // Enable temperature for compensation
        .interrupt_pin = GPIO_NUM_4     // Optional interrupt pin
    };
    
    // Initialize sensor
    mpu_handle_t imu = mpu9250_init(&config);
    if (imu == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MPU-9250");
        vTaskDelete(NULL);
        return;
    }
    
    // Perform self-test
    if (mpu9250_self_test(imu) != ESP_OK) {
        ESP_LOGW(TAG, "Self-test failed, but continuing...");
    }
    
    // Calibrate sensors (should be done with rocket stationary)
    ESP_LOGI(TAG, "Calibrating sensors (keep device stationary)...");
    if (mpu9250_calibrate(imu, CALIBRATION_ALL, 1000) != ESP_OK) {
        ESP_LOGW(TAG, "Calibration failed, using factory defaults");
    }
    
    // Save calibration to NVS for future use
    mpu9250_save_calibration(imu, "rocket_calibration");
    
    // Main control loop
    imu_data_t sensor_data;
    float dt = 1.0f / config.sample_rate_hz;
    
    while (1) {
        // Read sensor data
        if (mpu9250_read_all(imu, &sensor_data) == ESP_OK) {
            // Update sensor fusion with fixed time step
            mpu9250_update_fusion(imu, dt);
            
            // For rocket control, you would typically:
            // 1. Use quaternion for attitude control
            // 2. Use gyro for rate control
            // 3. Use accel for vibration analysis and G-force monitoring
            // 4. Use mag for heading (pre-launch)
            
            ESP_LOGI(TAG, "Accel: %.2f, %.2f, %.2f m/s²", 
                     sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2]);
            ESP_LOGI(TAG, "Gyro: %.2f, %.2f, %.2f rad/s", 
                     sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2]);
            ESP_LOGI(TAG, "Quaternion: %.3f, %.3f, %.3f, %.3f", 
                     sensor_data.quaternion[0], sensor_data.quaternion[1],
                     sensor_data.quaternion[2], sensor_data.quaternion[3]);
            
            // Calculate Euler angles for telemetry
            float roll = atan2f(2.0f * (sensor_data.quaternion[0] * sensor_data.quaternion[1] + 
                                       sensor_data.quaternion[2] * sensor_data.quaternion[3]),
                               1.0f - 2.0f * (sensor_data.quaternion[1] * sensor_data.quaternion[1] + 
                                            sensor_data.quaternion[2] * sensor_data.quaternion[2]));
            
            float pitch = asinf(2.0f * (sensor_data.quaternion[0] * sensor_data.quaternion[2] - 
                                       sensor_data.quaternion[3] * sensor_data.quaternion[1]));
            
            float yaw = atan2f(2.0f * (sensor_data.quaternion[0] * sensor_data.quaternion[3] + 
                                      sensor_data.quaternion[1] * sensor_data.quaternion[2]),
                              1.0f - 2.0f * (sensor_data.quaternion[2] * sensor_data.quaternion[2] + 
                                            sensor_data.quaternion[3] * sensor_data.quaternion[3]));
            
            ESP_LOGI(TAG, "Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°", 
                     roll * 180.0f / M_PI, pitch * 180.0f / M_PI, yaw * 180.0f / M_PI);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000 / config.sample_rate_hz));
    }
    
    // Cleanup
    mpu9250_deinit(imu);
    vTaskDelete(NULL);
}

