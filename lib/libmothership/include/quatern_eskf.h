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
    struct StateStruct {
        Quaternion q;
        MSVector3 ab;
        MSVector3 gb;
    } state;
    float statevec[10];

    public:
    QState();

    BLA::Matrix<10,1>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<10,1,float>*>(statevec);
    }

    const BLA::Matrix<10,1>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<10,1,float>*>(statevec);
    }

    void propagate(MSVector3 gyro, float dt);
    void update(QErrorState &es);
};

union QErrorState {
    struct ErrorStateStruct {
        MSVector3 theta;
        MSVector3 ab;
        MSVector3 gb;
    } d;
    float dvec[9];

    public:
    QErrorState();

    BLA::Matrix<9,1, float>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<9,1,float>*>(dvec);
    }

    const BLA::Matrix<9,1, float>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<9,1,float>*>(dvec);
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
    
    float accel_var = (0.02*9.81)*(0.02*9.81);        // acceleration variance (for measurement only) 
    float accel_drift_var = 1e-6;  // A_i
    float gyro_var = (0.1/180*M_PI)*(0.1/180/M_PI);         // Theta_i
    float gyro_drift_var = 1e-8;   // Omega_i

    float flat_var = (6.0f/180.0f*M_PI)*(6.0f/180.0f*M_PI); // initial flatness covariance of 2 degrees

    float KF_DT;

    public:
    QEskf();

    // State transition matrix F
    BLA::Matrix<9,9,float> F = BLA::Eye<9,9,float>();
    
    // State uncertainty Covariance matrix P 
    BLA::Matrix<9,9, float> P = BLA::Eye<9,9,float>();

    BLA::Matrix<9,9,float> Q = BLA::Eye<9,9,float>();

    // Accelerometer noise covariance matrix Ra 
     BLA::Matrix<3,3,float> Ra = BLA::Eye<3,3,float>()*accel_var;

    // System covariance matrix Q

    void build_matrix_F(MSVector3 gyro, float dt);
    void build_matrix_Q();
    void init_matrix_P();

    void propagate(MSVector3 gyro, float dt);
    void update(MSVector3 accel);

};

#endif