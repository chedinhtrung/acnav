#ifndef QUAT_ESKF
#define QUAT_ESKF

#include "quaternion.h"
#include "msvector.h"

#ifdef ARDUINO
#include <Arduino.h>
#undef F
#endif

union QErrorState;

union QState {
    Quaternion q;
    float qvec[3];

    public:
    QState();

    void propagate(MSVector3 gyro, float dt);
    void update(QErrorState &es);
};

union QErrorState {
    MSVector3 dtheta;
    float dthetavec[3];

    public:
    QErrorState();

    BLA::Matrix<3,1, float>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<3,1,float>*>(dthetavec);
    }

    const BLA::Matrix<3,1, float>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<3,1,float>*>(dthetavec);
    }


    public:
    void propagate(const QState &n, MSVector3 gyro, float dt);
    void reset();
};

struct QEskf {
    /*
        State vector is organized in order: 
        orientation quaternion, accel bias, gyro bias
        Error state vector is organized: 
        Error angle, accel bias, gyro bias
    */
    QState state; 
    QErrorState error_state;
    
    float accel_var = (0.01f)*(0.01f);        // acceleration variance (for measurement only) 
    float gyro_var = (1.0f/180*M_PI)*(1.0f/180*M_PI);         // Theta_i
    float flat_var = (6.0f/180.0f*M_PI)*(6.0f/180.0f*M_PI); // initial flatness covariance of 2 degrees

    const float KF_DT = 2.0f*1e-3;

    public:
    QEskf();

    // State transition matrix F
    
    // State uncertainty Covariance matrix P 
    BLA::Matrix<3,3, float> P = BLA::Eye<3,3,float>() * flat_var;

    BLA::Matrix<3,3,float> Q = BLA::Eye<3,3,float>() * KF_DT * KF_DT * gyro_var;

    // Accelerometer noise covariance matrix Ra 
     BLA::Matrix<3,3,float> Ra = BLA::Eye<3,3,float>()*accel_var;

    void propagate(MSVector3 gyro, float dt);
    void update(MSVector3 accel);

};

#endif