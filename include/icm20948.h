#ifndef ICM20948_IMU
#define ICM20948_IMU
#include "imu.h"
#include <Wire.h>

#define ICM20948_ADDR 0x68

class ICM20948 {
    private: 
        MSVector3 gyro_offset;
        MSVector3 accel_offset = MSVector3(0.05f,0.0f,0.13f);
        MSVector3 gyro_remap = MSVector3(1,-1,-1);
        MSVector3 accel_remap = MSVector3(1,-1,-1);
        unsigned long last_debug = millis(); 
    public: 
        unsigned long last_read = micros();
        ICM20948();
        void setup();
        ImuData imu_read();
        void calibrate();
};

#endif 