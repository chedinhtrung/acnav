#include "icm20948.h"

#include <Arduino.h>

ICM20948::ICM20948(){

}

void ICM20948::setup(){
    // Wake up imu
    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(0x06); 
    Wire.write(0x02);                   // Set PLL clock, exits sleep mode
    Wire.endTransmission();
    delay(100);

    // Configure Gyroscope
    Wire.beginTransmission(ICM20948_ADDR);   
    Wire.write(0x7F);                       // Set register bank to 1
    Wire.write(0x10);                        
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR);   
    Wire.write(0x01);                       // gyro scale 250dps, enable DLPF at 23.9Hz
    Wire.write(0x21);                        
    Wire.endTransmission();
    delay(100);

    // Configure Accelerometer
    Wire.beginTransmission(ICM20948_ADDR);   
    Wire.write(0x7F);                       // Set register bank to 2
    Wire.write(0x20);                        
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR); 
    Wire.write(0x14);                       
    Wire.write(0x21);                       // Set accel range to 2g, DLPF to 23.9Hz
    Wire.endTransmission();   
    delay(100);

    Wire.beginTransmission(ICM20948_ADDR);   
    Wire.write(0x7F);                       // reset register bank to 0
    Wire.write(0x00);                        
    Wire.endTransmission();
    delay(100);

    calibrate();
}

void ICM20948::calibrate(){


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

ImuData ICM20948::imu_read(){
    last_read = micros();
    ImuData data;

    //unsigned long before_read = micros();
    // Request accelerometer data
    Wire.beginTransmission(ICM20948_ADDR);
    Wire.write(0x2D);
    Wire.endTransmission();
    Wire.requestFrom(ICM20948_ADDR, 12);

    //Read accelerometer data as bits
    int16_t accelX = Wire.read()<<8 | Wire.read();
    int16_t accelY = Wire.read()<<8 | Wire.read();
    int16_t accelZ = Wire.read()<<8 | Wire.read();

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