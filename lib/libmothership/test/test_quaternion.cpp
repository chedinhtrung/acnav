#include <iostream>
#include <cstring> 
#include <math.h>

#include "quaternion.h"

int main(){

    Quaternion q1, q2;
    
    q1 = Quaternion(1, 2, 3, 4);
    q2 = Quaternion(5, 6, 7, 8); 

    // test addition 
    Quaternion q3 = q1 + q2; 

    Quaternion q4 = q1 * q2;

    Quaternion q5 = q1.T();
    
    char buf[100];
    q3.print(buf);
    std::cout << "Testing sum \n";
    std::cout << "Expecting: " << "(6, 8, 10, 12)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    q4.print(buf);
    std::cout << "Testing product \n";
    std::cout << "Expecting: " << "(-60, 12, 30, 24)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    bool eq = q3 == q1 + q2;
    std::cout << "Testing equality \n";
    std::cout << "Expecting: " << "1" << "\n";
    std::cout << "Got:      " << eq << "\n\n";

    q5.print(buf);
    std::cout << "Testing conjugate \n";
    std::cout << "Expecting: " << "(1, -2, -3, -4)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    std::cout << "Testing norm \n";
    std::cout << "Expecting: " << sqrt(30) << "\n";
    std::cout << "Got:      " << q1.norm() << "\n\n";

    Quaternion q6(0,3,0,4);
    q6.normize();
    q6.print(buf);
    std::cout << "Testing normize \n";
    std::cout << "Expecting: " << "(0, 0.6, 0, 0.8)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    sprintf(buf, "(%f, %f, %f, %f)", q6.data[0], q6.data[1], q6.data[2], q6.data[3]);
    std::cout << "Testing union \n";
    std::cout << "Expecting: " << "(0, 0.6, 0, 0.8)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    q6.data[3] += 1;
    q6.print(buf);
    std::cout << "Testing union 2 \n";
    std::cout << "Expecting: " << "(0, 0.6, 0, 1.8)" << "\n";
    std::cout << "Got:      " << buf << "\n\n";

    Quaternion q8(sqrt(2.0f)/2.0f, 0, 0, sqrt(2.0f)/2.0f);
    Quaternion q8inv = q8.inv();
    Quaternion q8T = q8.T();

    q8inv.print(buf);
    std::cout << "Testing conjugate = inverse for unit q \n";
    std::cout << "Expecting: " << buf << "\n";
    q8T.print(buf);
    std::cout << "Got:      " << buf << "\n\n";

    return 0;
}