/**
 * @file mpu9250_example.c
 * @brief Enhanced example with orientation metrics for rocket control
 * @author Mathias Kulola
 * @date 2024
 */

#include <math.h>

#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char* TAG = "MPU9250_EXAMPLE";

void print_orientation_metrics(const imu_data_t* data) {
    // Get Euler angles in radians
    float roll, pitch, yaw;
    mpu9250_get_euler_angles(data->quaternion, &roll, &pitch, &yaw);
    
    // Get Euler angles in degrees
    float roll_deg, pitch_deg, yaw_deg;
    mpu9250_get_euler_angles_deg(data->quaternion, &roll_deg, &pitch_deg, &yaw_deg);
    
    // Get linear acceleration (gravity removed)
    float linear_accel[3];
    mpu9250_get_linear_acceleration(data->accel, data->quaternion, linear_accel);
    
    // Get gravity vector in body frame
    float gravity_body[3];
    mpu9250_get_gravity_vector(data->quaternion, gravity_body);
    
    // Get heading (if magnetometer available)
    float heading_rad = 0, heading_deg = 0;
    if (fabs(data->mag[0]) > 0.1f || fabs(data->mag[1]) > 0.1f || fabs(data->mag[2]) > 0.1f) {
        heading_rad = mpu9250_get_heading(data->mag, data->quaternion);
        heading_deg = mpu9250_get_heading_deg(data->mag, data->quaternion);
    }
    
    // Get tilt from vertical (useful for rocket)
    float tilt_angle, tilt_direction;
    mpu9250_get_tilt_angles(data->quaternion, &tilt_angle, &tilt_direction);
    
    // Check if vertical (for launch detection)
    bool is_vertical = mpu9250_is_vertical(data->quaternion, 0.087f); // 5 degrees threshold
    
    // Get acceleration metrics
    float accel_mag = mpu9250_get_acceleration_magnitude(data->accel);
    float gforce = mpu9250_get_gforce(data->accel);
    
    // Get angular rate metrics
    float gyro_mag = mpu9250_get_angular_rate_magnitude(data->gyro);
    float gyro_dps[3];
    mpu9250_get_angular_rate_dps(data->gyro, gyro_dps);
    
    // Convert to axis-angle representation
    float axis[3];
    float rotation_angle = mpu9250_quaternion_to_axis_angle(data->quaternion, axis);
    
    // Print all metrics
    ESP_LOGI(TAG, "\n\n=== ROCKET ORIENTATION METRICS ===");
    ESP_LOGI(TAG, "Euler Angles (deg): Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°", 
             roll_deg, pitch_deg, yaw_deg);
    ESP_LOGI(TAG, "Euler Angles (rad): Roll: %.3f, Pitch: %.3f, Yaw: %.3f", 
             roll, pitch, yaw);
    
    if (heading_deg > 0) {
        ESP_LOGI(TAG, "Heading: %.1f° (%.3f rad)", heading_deg, heading_rad);
    }
    
    ESP_LOGI(TAG, "Tilt from vertical: %.1f° (Direction: %.1f°)", 
             tilt_angle * 180.0f / M_PI, tilt_direction * 180.0f / M_PI);
    ESP_LOGI(TAG, "Vertical status: %s", is_vertical ? "VERTICAL" : "NOT VERTICAL");
    
    ESP_LOGI(TAG, "=== ACCELERATION METRICS ===");
    ESP_LOGI(TAG, "Accel (body): X: %.2f, Y: %.2f, Z: %.2f m/s²", 
             data->accel[0], data->accel[1], data->accel[2]);
    ESP_LOGI(TAG, "Linear Accel (world): X: %.2f, Y: %.2f, Z: %.2f m/s²", 
             linear_accel[0], linear_accel[1], linear_accel[2]);
    ESP_LOGI(TAG, "Gravity (body): X: %.2f, Y: %.2f, Z: %.2f m/s²", 
             gravity_body[0], gravity_body[1], gravity_body[2]);
    ESP_LOGI(TAG, "Total Acceleration: %.2f m/s² (%.2f G)", accel_mag, gforce);
    
    ESP_LOGI(TAG, "=== ROTATION METRICS ===");
    ESP_LOGI(TAG, "Gyro (body): X: %.1f, Y: %.1f, Z: %.1f rad/s", 
             data->gyro[0], data->gyro[1], data->gyro[2]);
    ESP_LOGI(TAG, "Gyro (dps): X: %.1f, Y: %.1f, Z: %.1f °/s", 
             gyro_dps[0], gyro_dps[1], gyro_dps[2]);
    ESP_LOGI(TAG, "Angular Rate Magnitude: %.1f rad/s (%.1f °/s)", 
             gyro_mag, gyro_mag * 180.0f / M_PI);
    
    ESP_LOGI(TAG, "Axis-Angle: Angle: %.1f°, Axis: [%.3f, %.3f, %.3f]", 
             rotation_angle * 180.0f / M_PI, axis[0], axis[1], axis[2]);
    
    ESP_LOGI(TAG, "Quaternion: w: %.3f, x: %.3f, y: %.3f, z: %.3f", 
             data->quaternion[0], data->quaternion[1], 
             data->quaternion[2], data->quaternion[3]);
    
    ESP_LOGI(TAG, "Temperature: %.1f°C", data->temperature);
    ESP_LOGI(TAG, "=================================");

    /**
     * @attention TO BE REMOVED... just added small delay to avoid too quick logs..
     */
    vTaskDelay(pdMS_TO_TICKS(200));
}

void rocket_control_logic(const imu_data_t* data) {
    // Example rocket control logic using orientation metrics
    
    // 1. Launch detection - check if rocket is vertical and accelerating
    static bool launch_detected = false;
    if (!launch_detected) {
        bool is_vertical = mpu9250_is_vertical(data->quaternion, 0.087f); // 5° threshold
        float accel_mag = mpu9250_get_acceleration_magnitude(data->accel);
        
        if (is_vertical && accel_mag > 15.0f) { // > 1.5G
            ESP_LOGI(TAG, "LAUNCH DETECTED!");
            launch_detected = true;
            // Trigger launch sequence
        }
    }
    
    // 2. Apogee detection - check when vertical velocity changes sign
    static float prev_vertical_vel = 0.0f;
    static float vertical_vel = 0.0f;
    static uint32_t sample_count = 0;
    
    if (launch_detected) {
        // Integrate linear acceleration in Z direction to get velocity
        float linear_accel[3];
        mpu9250_get_linear_acceleration(data->accel, data->quaternion, linear_accel);
        
        // Simple integration (in real system, use proper integration with time)
        vertical_vel += linear_accel[2] * 0.01f; // Assuming 100Hz sampling
        
        // Detect when velocity changes from positive to negative
        if (prev_vertical_vel > 0.1f && vertical_vel < -0.1f) {
            ESP_LOGI(TAG, "APOGEE DETECTED!");
            // Trigger recovery system
        }
        
        prev_vertical_vel = vertical_vel;
        sample_count++;
    }
    
    // 3. Attitude control - use roll, pitch, yaw for stabilization
    float roll, pitch, yaw;
    mpu9250_get_euler_angles(data->quaternion, &roll, &pitch, &yaw);
    
    // Simple PD controller for roll stabilization
    float roll_error = -roll;  // Target roll = 0 (upright)
    float roll_rate = data->gyro[0];  // Roll rate from gyro
    
    // Example control output (would go to actuator manager)
    float control_output = 0.1f * roll_error + 0.05f * roll_rate;
    
    // 4. Telemetry - package important metrics
    float tilt_angle, tilt_direction;
    mpu9250_get_tilt_angles(data->quaternion, &tilt_angle, &tilt_direction);
    
    // These metrics are useful for ground station telemetry
    // telemetry_send(roll, pitch, yaw, tilt_angle, data->accel, data->gyro);
}

void mpu9250_example_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Starting enhanced MPU-9250 example for rocket control");
    
    mpu_config_t config = {
        .i2c_port = I2C_NUM_0,
        .i2c_address = 0x68,
        .sda_pin = 21,
        .scl_pin = 22,
        .clk_speed = 400000,
        .accel_fs = ACCEL_FS_16G,
        .gyro_fs = GYRO_FS_2000DPS,
        .dlpf_config = DLPF_184HZ,
        .sample_rate_hz = 200,
        .enable_mag = true,
        .enable_temp = true,
        .interrupt_pin = GPIO_NUM_NC
    };
    
    mpu_handle_t imu = mpu9250_init(&config);
    if (imu == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MPU-9250");
        vTaskDelete(NULL);
        return;
    }
    
    // Print debug info
    mpu9250_print_debug_info(imu);
    
    // Calibrate
    ESP_LOGI(TAG, "Calibrating IMU...");
    if (mpu9250_calibrate(imu, CALIBRATION_ALL, 1000) != ESP_OK) {
        ESP_LOGW(TAG, "Calibration failed or incomplete");
    }
    
    ESP_LOGI(TAG, "Starting main loop...");
    
    imu_data_t sensor_data;
    uint32_t iteration = 0;
    
    while (1)
    {
        if (mpu9250_read_all(imu, &sensor_data) == ESP_OK) {
            // Update sensor fusion with fixed time step
            float dt = 1.0f / config.sample_rate_hz;
            mpu9250_update_fusion(imu, &sensor_data, dt);
            
            // Print orientation metrics every 50 iterations
            if (iteration % 50 == 0) {
                print_orientation_metrics(&sensor_data);
            }
            
            iteration++;
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
            // Try to reinitialize on failure
            static int fail_count = 0;
            fail_count++;
            if (fail_count > 10) {
                ESP_LOGE(TAG, "Too many failures, restarting task...");
                vTaskDelete(NULL);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz sampling
    }
    
    mpu9250_deinit(imu);
    vTaskDelete(NULL);
}

void example_main()
{
    xTaskCreate(mpu9250_example_task, "mpu9250_example", 4096, NULL, 5, NULL);
}