#include "quatern_eskf.h"

QState::QState(){
    q = Quaternion(1.0f,0.0f,0.0f,0.0f);
}

void QState::propagate(MSVector3 gyro, float dt){
    q = q * (gyro*dt).to_dq();
}

void QState::update(QErrorState &es){
    q = q * es.dtheta.to_dq();
}

QErrorState::QErrorState(){
    dtheta = MSVector3(0.0f, 0.0f, 0.0f);
}

void QErrorState::propagate(const QState &n, MSVector3 gyro, float dt){
    dtheta = (gyro*dt).to_dq().T().rotate(dtheta);
}

void QErrorState::reset(){
    dthetavec[0] = 0.0f;
    dthetavec[1] = 0.0f;
    dthetavec[2] = 0.0f;
}

QEskf::QEskf() {
    P(2,2) = 5e-7;
}

void QEskf::propagate(MSVector3 gyro, float dt){
    float gyro_norm = gyro.norm();

    // nominal and error state propagate
    state.propagate(gyro, dt);
    error_state.propagate(state, gyro, dt);
    
    BLA::Matrix<3,3,float> F = (gyro * dt).to_dq().T().to_R();
    
    // Covar matrix P propagate 
    P = (~F) * P * F + Q;

    if (P(2,2) > 1e-6){
        P(2,2) = 1e-6;
    }
}

void QEskf::update(MSVector3 accel){
    MSVector3 g(0.0f,0.0f,-1.0f);

    float lin_accel_mag = accel.norm();
    float lin_accel_var = (lin_accel_mag - 1.0) * (lin_accel_mag - 1.0);

    BLA::Matrix<3,3,float> V = Ra + lin_accel_var*BLA::Eye<3,3,float>();

    MSVector3 RT_v = state.q.T().rotate(g); 
    BLA::Matrix<3,3,float> H =  RT_v.to_skewsymL();
    
    BLA::Matrix<3,3,float> K = P * (~H) * BLA::Inverse((H*P*(~H) + V));
    
    BLA::Matrix<3,1,float>& current_error_state = error_state.as_BlaVec();
    current_error_state += K * (accel - RT_v).as_BlaVec();

    BLA::Eye<3,3,float> I;
    P = (I - K*H) * P * (~(I - K*H)) + K * V *(~K);

    state.update(error_state);

    error_state.reset();
}