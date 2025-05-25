#include <iostream>
#include <cstring> 
#include <math.h>

#include "quaternion.h"
#include "msvector.h"

int main(){

    MSVector3 v(0.0f,1.0f,0.0f);

    Quaternion q(0.0f,1.0f,0.0f,0.0f);
    
    MSVector3 rv = q.rotate(v);
    
    char buf[100];

    rv.print(buf);
    std::cout << "Testing rotate with quaternion \n";
    std::cout << "Expecting: " << "(0,-1,0)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    Quaternion q1(sqrt(2.0f)/2.0f, 0.0f,0.0f,sqrt(2.0f)/2.0f);
    v = MSVector3(1.0f, 0.0f, 0.0f); 
    rv = q1.rotate(v);

    rv.print(buf);
    std::cout << "Testing rotate with quaternion 2 \n";
    std::cout << "Expecting: " << "(0,1,0)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    Quaternion q2(0.5f, 0.5f, 0.5f, 0.5f);      
    v = MSVector3(1,2,3);
    rv = q2.rotate(v);

    rv.print(buf);
    std::cout << "Testing rotate with quaternion 3 \n";
    std::cout << "Expecting: " << "(3, 1, 2)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    Quaternion q3(0.92388f, 0.10227f, 0.30683f, 0.204553f);      // rotate 120 deg around 1,1,1
    v = MSVector3(1,2,3);
    rv = q3.rotate(v);

    rv.print(buf);
    std::cout << "Testing rotate with quaternion 3 \n";
    std::cout << "Expecting: " << "(1.92399, 2.041, 2.47628)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    Quaternion q4(0.92388f, 0.10227f, 0.30683f, 0.204553f);      // rotate 120 deg around 1,1,1
    v = MSVector3(1,2,3);
    rv = q4.rotate(v);

    rv.print(buf);
    std::cout << "Testing rotate with quaternion and rotmat \n";
    std::cout << "Expecting: " << buf << "\n";
    BLA::Matrix<3,3,float> R = q4.to_R();
    BLA::Matrix<3,1,float> R_times_v = R * v.as_BlaVec();
    sprintf(buf, "(%f, %f, %f)", R_times_v(0,0), R_times_v(1,0), R_times_v(2,0));
    std::cout << "Got:      " << buf << "\n\n";

    MSVector3 v_back = q4.T().rotate(rv);
    v_back.print(buf);
    std::cout << "Testing inverse rotation with quaternion and rotmat \n";
    std::cout << "Expecting: " << buf << "\n";
    BLA::Matrix<3,1,float> R_T_times_v = ~R * R_times_v;
    sprintf(buf, "(%f, %f, %f)", R_T_times_v(0,0), R_T_times_v(1,0), R_T_times_v(2,0));
    std::cout << "Got:      " << buf << "\n\n";

    return 0;
}