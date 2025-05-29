#include <Wire.h>
#include "datastructs.h"
#include <msvector.h>
#include "imu.h"
#define MPU6050_ADDR 0x68
#ifndef MPU6050 
#define MPU6050 

class MPU6050_Imu {
    private: 
        MSVector3 gyro_offset;
        
       
        # ifdef REFERENCE
        MSVector3 accel_offset = MSVector3(0.016f,-0.0015f,0.124f);
        #endif
        #ifdef MEASURE
        MSVector3 accel_offset = MSVector3(0.05f,0.0f,0.13f);
        #endif
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
