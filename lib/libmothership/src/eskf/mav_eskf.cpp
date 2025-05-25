#include "mav_eskf.h"
#include "BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "msvector.h"
/*

Based on Joan Sola Quaternion Kinematics for the error-state Kalman filter
P. 51

*/ 

void State::propagate(MSVector3 accel, MSVector3 gyro, float dt){
    /*
        accel, gyro: new measurements from IMU
    */
    state.p = state.p + state.v * dt + (state.q.rotate(accel - state.ab) + state.g)*dt*dt;

    state.v = state.v + state.q.rotate(accel - state.ab)*dt;

    state.q = state.q * Quaternion((gyro - state.gb)*0.5*dt);
}

void State::update(const ErrorState &es){

}

void ErrorState::propagate(const State &n, MSVector3 accel, MSVector3 gyro, float dt) {
    /*
        n: current (nominal) State vector
        accel, gyro: new measurements from IMU
    */
    // state propagation

    d.p = d.p + d.v * dt;
    
    d.v = d.v - (n.state.q.rotate((accel - n.state.ab).cross(d.theta)) + n.state.q.rotate(d.ab) - d.g)*dt;

    d.theta = ((gyro - n.state.gb)*dt).to_dq().T().rotate(d.theta) - d.gb*dt;

    // Covariance propagation
    

}

void ErrorState::update_accel(MSVector3 accel, float dt){

}