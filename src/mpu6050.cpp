#include "mpu6050.h"
#include <Arduino.h>

MPU6050_Imu::MPU6050_Imu(){

}

void MPU6050_Imu::setup(){
    // Wake up imu
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); 
    Wire.write(0x03);                   // Set PLL clock to use Z Axis Gyro as reference. also resets imu
    Wire.endTransmission();
    delay(100);

    // Configure Gyroscope
    Wire.beginTransmission(MPU6050_ADDR);    // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x1A);                        // Register 1A for Low Pass Filter (both gyro and accelerometer)
    Wire.write(0x06);                        // Set to 5Hz bandwidth
    Wire.endTransmission();
    delay(100);
 
    Wire.beginTransmission(MPU6050_ADDR);       
    Wire.write(0x1B);                        // Set sensitivity at Register 0x1B
    Wire.write(0x00);                        // Sensitivity at +- 250 degrees/s. 
    Wire.endTransmission();                  // IMPORTANT: This sets 131 bits/degree/s.
    delay(100);

    // Configure Accelerometer
    Wire.beginTransmission(MPU6050_ADDR); 
    Wire.write(0x1C);                       // Set sensitivity at Register 0x1C
    Wire.write(0x00);                       // Set sensitivity at +-2g --> 16384 LSB/g
    Wire.endTransmission();                
    calibrate();
}

void MPU6050_Imu::calibrate(){


    #if IMU_DEBUG
    Serial.println("Calibrating Gyro Offset");
    #endif

    MSVector3 sum_gyro = MSVector3(0,0,0);

    int samples = 2000;
    for (int i=0; i<samples; i++){
        ImuData data = imu_read();
        sum_gyro = sum_gyro + data.gyro;
        delay(1);
    }
    gyro_offset = sum_gyro*(1.0f/samples);

    #if IMU_DEBUG
    Serial.printf("gyro offset: %f  %f  %f", gyro_offset.x, gyro_offset.y, gyro_offset.z);
    delay(2000);
    #endif

}

ImuData MPU6050_Imu::imu_read(){
    last_read = micros();
    ImuData data;

    //unsigned long before_read = micros();
    // Request accelerometer data
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDR, 14);

    //Read accelerometer data as bits
    int16_t accelX = Wire.read()<<8 | Wire.read();
    int16_t accelY = Wire.read()<<8 | Wire.read();
    int16_t accelZ = Wire.read()<<8 | Wire.read();

    Wire.read(); Wire.read();      // Temperature data, not needed

    // Read gyro data
    int16_t gyroX = Wire.read()<<8 | Wire.read();
    int16_t gyroY = Wire.read()<<8 | Wire.read();
    int16_t gyroZ = Wire.read()<<8 | Wire.read();
    //Serial.println(micros() - start);

    // Calculate angular velocities  IMPORTANT: Roll and Pitch are assigned to Y and X respectively,
    // Because of how I mount my gyro 
    data.gyro.x = (float)gyroX/131.0 * gyro_remap.x;             // convention: right roll = positive
    data.gyro.y = (float)gyroY/131.0 * gyro_remap.y;         // convention: up pitch = positive
    data.gyro.z = (float)gyroZ/131.0 * gyro_remap.z;      // convention: right yaw = positive

    // Calculate accelerometer data
    data.accel.x = ((float)accelX/16384.0 - accel_offset.x)*accel_remap.x;
    data.accel.y = ((float)accelY/16384.0 - accel_offset.y)*accel_remap.y;
    data.accel.z = ((float)accelZ/16384.0 - accel_offset.z)*accel_remap.z;
    
    data.gyro = data.gyro * (M_PI/180.0f) - gyro_offset;
    //data.accel = data.accel * 9.81;
    #if IMU_DEBUG
    if (millis() - last_debug > 40){
        last_debug = millis();
        Serial.printf("gyro: %f  %f  %f   accel: %f  %f  %f \n", 
                        data.gyro.x, data.gyro.y, data.gyro.z, 
                        data.accel.x, data.accel.y, data.accel.z);
    }
    #endif
    //Serial.printf("Read time: %i \n", micros() - before_read);
   
    return data;
}