#ifndef MAV_ESKF
#define MAV_EKSF

#include "quaternion.h" 

union ErrorState;

union State {
    struct StateStruct {
        MSVector3 p;     // Position in NED
        MSVector3 v;     // velocity in NED
        Quaternion q;    // Drone orientation
        MSVector3 ab;    // Accelerometer bias
        MSVector3 gb;    // Gyroscope bias
        MSVector3 g;     // gravity vector
    } state; 
    float statevec[19];

    BLA::Matrix<3,1>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<3,1,float>*>(statevec);
    }

    const BLA::Matrix<3,1>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<3,1,float>*>(statevec);
    }

    void propagate(MSVector3 accel, MSVector3 gyro, float dt);
    void update(const ErrorState &es);
    
};

union ErrorState {
    struct StateStruct {
        MSVector3 p;     // Position in NED
        MSVector3 v;     // velocity in NED
        MSVector3 theta;    // Infinitesimal angle rotation
        MSVector3 ab;    // Accelerometer bias
        MSVector3 gb;    // Gyroscope bias
        MSVector3 g;     // gravity vector
    } d; // d for delta
    float dvec[18];

    BLA::Matrix<18,1, float>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<18,1,float>*>(dvec);
    }

    const BLA::Matrix<18,1, float>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<18,1,float>*>(dvec);
    }

    void propagate(const State &n, MSVector3 accel, MSVector3 gyro, float dt);
    void update_accel(MSVector3 accel, float dt);
    void update_pos(MSVector3 pos, float dt);
    void update_vel(MSVector3 vel, float dt);
};


#endif
