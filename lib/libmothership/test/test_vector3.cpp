#include "msvector.h"
#include <iostream>
#include <cstring> 
#include <math.h>

int main(){

    MSVector3 v(1, 2, 3);

    char buf[100];
    v.print(buf);
    std::cout << "Testing vec \n";
    std::cout << "Expecting: " << "(1, 2, 3)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    sprintf(buf, "(%f, %f, %f)", v.data[0], v.data[1], v.data[2]);
    std::cout << "Testing union \n";
    std::cout << "Expecting: " << "(1, 2, 3)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    BLA::Matrix<3,1, float> &vec = v.as_BlaVec();           // vec has to be of type reference if we want modding vec changes v as well
    BLA::Matrix<3,3,float> testmat = BLA::Eye<3,3,float>();
    testmat(0,0) = 2;
    testmat(1,1) = 3; 
    testmat(2,2) = 4;
    
    BLA::Matrix<3,1, float> res = testmat * vec;
    vec = res;
    res(0,0) = 100;        // Test if = operator copies

    v.print(buf);
    std::cout << "Testing BLA functions \n";
    std::cout << "Expecting: " << "(2, 6, 12)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";
    
    MSVector3 u(1,2,3);
    u.as_BlaVec() = testmat * u.as_BlaVec();
    std::cout << "Testing BLA functions 2\n";
    std::cout << "Expecting: " << "(2, 6, 12)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";
    return 0;
}
