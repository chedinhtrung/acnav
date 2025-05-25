#include "quatern_eskf.h"

QState::QState(){
    state.q = Quaternion(1.0f,0.0f,0.0f,0.0f);
    state.ab = MSVector3(0,0,0);
    state.gb = MSVector3(0,0,0);
}

void QState::propagate(MSVector3 gyro, float dt){
    state.q = state.q * ((gyro - state.gb)*dt).to_dq();
}

void QState::update(QErrorState &es){
    state.q = state.q * es.d.theta.to_dq();
    state.ab = state.ab + es.d.ab; 
    state.gb = state.gb + es.d.gb;
}

QErrorState::QErrorState(){
    d.theta = MSVector3(0.0f, 0.0f, 0.0f);
    d.ab = MSVector3(0.0f, 0.0f, 0.0f);
    d.gb = MSVector3(0.0f, 0.0f, 0.0f);
}

void QErrorState::propagate(const QState &n, MSVector3 gyro, float dt){
    d.theta = ((gyro - n.state.gb)*dt).to_dq().T().rotate(d.theta) - d.gb*dt;
}

void QErrorState::reset(){
    memset(this, 0, sizeof(QErrorState));
}

void QEskf::build_matrix_Q(){
    // inserts the covariance values into the intial Q matrix
    // Reference: Joan Sola P.61 Eq. 271
    for (int i=0; i<3; i++){
        Q(i,i) = KF_DT * KF_DT * gyro_var;
        Q(i+3, i+3) = KF_DT * accel_drift_var;
        Q(i+6, i+6) = KF_DT * gyro_drift_var;
    }
}

void QEskf::init_matrix_P(){
    for (int i=0; i<3; i++){
        P(i,i) = flat_var;
        P(i+3, i+3) = accel_drift_var;
        P(i+6, i+6) = gyro_drift_var;
    }
}

QEskf::QEskf() {
    build_matrix_Q();
    init_matrix_P();
}

void QEskf::propagate(MSVector3 gyro, float dt){
    
    // nominal and error state propagate
    state.propagate(gyro, dt);
    error_state.propagate(state, gyro, dt);
    
    build_matrix_F(gyro, dt);
    // Covar matrix P propagate 
    P = ~F * P * F + Q;
}

void QEskf::build_matrix_F(MSVector3 gyro, float dt){
    BLA::Matrix<3,3,float> R = (gyro * dt).to_dq().T().to_R();         // Reference: Joan Sola p.61 Eq. 270

    // memcpy R_Mat into F and Inserts -DT into F()
    for (int j=0; j<3; j++){
        float* col_ptr_F = &F(0, j);
        float* col_ptr_R = &R(0, j);
        memcpy(col_ptr_F, col_ptr_R, sizeof(float)*3);
        F(j, 6+j) = -dt;
    }
}

void QEskf::update(MSVector3 accel){
    MSVector3 g(0,0,9.81);
    MSVector3 RT_v = state.state.q.T().rotate(g*-1.0f); 
    BLA::Matrix<3,3,float> partial_theta =  RT_v.to_skewsymL();


    BLA::Matrix<3,9,float> H = BLA::Zeros<3,9,float>();

    // Fill H with partials relative to d\theta and da_b
    for (int j=0; j<3; j++){
        float* col_ptr_H = &H(0,j);
        float* col_ptr_partial_theta = &partial_theta(0,j);
        memcpy(col_ptr_H, col_ptr_partial_theta, sizeof(float)*3);
        H(j, 3+j) = 1.0f;
    }

    MSVector3 h_x_hat = state.state.q.T().rotate(g*-1.0f) + state.state.ab + error_state.d.ab;
    

    BLA::Matrix<9,3,float> K = P * (~H) * BLA::Inverse((H*P*(~H) + Ra));
    
    BLA::Matrix<9,1,float> current_error_state = error_state.as_BlaVec();

    current_error_state = current_error_state + K * (accel - h_x_hat).as_BlaVec();

    state.update(error_state);

    error_state.reset();

}