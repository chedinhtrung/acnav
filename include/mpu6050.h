#include <Wire.h>
#include "datastructs.h"
#include <msvector.h>

#define MPU6050_ADDR 0x68
#ifndef MPU6050 
#define MPU6050 

#define IMU_DEBUG 0

struct RawImuData {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct ImuData { 
    MSVector3 gyro;
    MSVector3 accel;
};

class MPU6050_Imu {
    private: 
        MSVector3 gyro_offset;
        MSVector3 accel_offset = MSVector3(0.016f,-0.0015f,0.124f);
        MSVector3 gyro_remap = MSVector3(1,-1,-1);
        MSVector3 accel_remap = MSVector3(1,-1,-1);
        unsigned long last_debug = millis(); 
    public: 
        unsigned long last_read = micros();
        MPU6050_Imu();
        void setup();
        ImuData imu_read();
        void calibrate();
};



#endif
