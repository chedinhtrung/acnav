#include <iostream>
#include <cstring> 
#include <math.h>

#include "quaternion.h"
#include "msvector.h"

int main(){
    char buf[100];

    MSVector3 dtheta(0.002, 0.002, 0.002);
    Quaternion rq(0.92388f, 0.10227f, 0.30683f, 0.204553f);      // rotate 120 deg around 1,1,1

    MSVector3 v(1.0f,2.0f,3.0f);
    // rotate with rq^T then dtheta^T, calculate difference to only rotate with rq
    MSVector3 diff_1 = dtheta.to_dq().T().rotate(rq.T().rotate(v)) - rq.T().rotate(v);

    // difference calculated with jacobian
    BLA::Matrix<3,1> diff_2 = MSVector3(rq.T().to_R() * v.as_BlaVec()).to_skewsymL() * dtheta.as_BlaVec();

    diff_1.print(buf);
    std::cout << "Testing Jacobian \n";
    std::cout << "Expecting: " << buf << "\n";
    sprintf(buf, "%f %f %f", diff_2(0,0), diff_2(1,0), diff_2(2,0));
    std::cout << "Got:      " << buf << "\n\n";


    dtheta = MSVector3(0.01, 0.015, 0.01);
    rq = Quaternion(0.5f, 0.5f, 0.5f, 0.5f);      // rotate 120 deg around 1,1,1

    // rotate with rq^T then dtheta^T, calculate difference to only rotate with rq
    diff_1 = dtheta.to_dq().T().rotate(rq.T().rotate(v)) - rq.T().rotate(v);

    // difference calculated with jacobian
    diff_2 = MSVector3(rq.T().to_R() * v.as_BlaVec()).to_skewsymL() * dtheta.as_BlaVec();

    diff_1.print(buf);
    std::cout << "Testing Jacobian \n";
    std::cout << "Expecting: " << buf << "\n";
    sprintf(buf, "%f %f %f", diff_2(0,0), diff_2(1,0), diff_2(2,0));
    std::cout << "Got:      " << buf << "\n\n";
}