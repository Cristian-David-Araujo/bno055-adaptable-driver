#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bno055.h"
#include "platform_esp32s3.h"

#define RAD_TO_DEG 57.2957795 // Conversion factor from radians to degrees

BNO055_t bno055;

void init_sensor() {
    int8_t success = 0;
    success = BNO055_Init(&bno055, 17, 18);
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to initialize BNO055 sensor\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_Init(&bno055, 17, 18);
    }
    
}

void print_sensor_data(void *pvParameters) {
    uint32_t timestamp = 1000000; // Start at 1 second
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    
    while (true) {
        // Read Euler angles
        BNO055_ReadAll(&bno055);

        // Data from the BNO055 sensor
        ax = bno055.ax;
        ay = bno055.ay;
        az = bno055.az;

        gx = bno055.gx;
        gy = bno055.gy;
        gz = bno055.gz;

        mx = bno055.mx;
        my = bno055.my;
        mz = bno055.mz;

        // Get Euler angles
        roll = bno055.roll;
        pitch = bno055.pitch;
        yaw = bno055.yaw;

        // Convert to degrees
        roll *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;

        // Invert yaw direction
        yaw = 360.0 - yaw;
        
        // Normalize yaw to [0, 360] degrees
        if (yaw >= 360.0) {
            yaw -= 360.0;
        } else if (yaw < 0.0) {
            yaw += 360.0;
        }

        // Convert magnetic field au to uT (1au = 50uT)
        mx *= 50.0;
        my *= 50.0;
        mz *= 50.0;

        // accel in m/s^2 to g
        ax /= 9.81;
        ay /= 9.81;
        az /= 9.81;

        // gyro in rad/s to deg/s
        gx *= RAD_TO_DEG;
        gy *= RAD_TO_DEG;
        gz *= RAD_TO_DEG;

        


        // Orientation message
        printf("A,%" PRIu32 ",%.4f,%.4f,%.4f\r\n", timestamp, roll, pitch, yaw);

        // Inertial message (gyroscope and accelerometer)
        printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, gx, gy, gz, ax, ay, az);

        // Magnetometer message
        printf("M,%" PRIu32 ",%.4f,%.4f,%.4f\r\n", timestamp, mx, my, mz);
        
        // Increment timestamp by 1 second (1,000,000 microseconds)
        timestamp += 50000;
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait 1 second
    }
}

void app_main() {
    init_sensor(); // Initialize BNO055 sensor
    xTaskCreate(print_sensor_data, "SensorTask", 4096, NULL, 1, NULL);
}
