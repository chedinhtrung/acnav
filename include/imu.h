#include "datastructs.h"
#include <msvector.h>

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