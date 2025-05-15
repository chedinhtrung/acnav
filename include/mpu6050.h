#include <Wire.h>
#include "datastructs.h"

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
    Vector3D gyro;
    Vector3D accel;
};

class MPU6050_Imu {
    private: 
        Vector3D gyro_offset = { 
            .x = 0.0, .y = 0.0, .z = 0.0
        };
        Vector3D accel_offset = { 
            .x = 0.0, .y = 0.0, .z = 0.0
        };
        Vector3D gyro_remap = { 
            .x = 1.0, .y = 1.0, .z = 1.0
        };
        Vector3D accel_remap = { 
            .x = 1.0, .y = 1.0, .z = 1.0
        };
        unsigned long last_debug = millis(); 
    public: 
        unsigned long last_read = micros();
        MPU6050_Imu();
        void setup();
        ImuData imu_read();
        void calibrate();
};



#endif
