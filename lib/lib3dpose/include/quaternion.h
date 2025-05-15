#ifndef QUATERNION_LIB
#define QUARTERNIONS_LIB
#include "datastructs.h"
#define EQ_EPSILON 0.000001

struct Quaternion { 
    float w, i, j, k;
    public:
    Quaternion(float wq, float iq, float jq, float kq);
    Quaternion() = default;
    float norm();
    void normize();
    Quaternion T();        //  returns conjugate. 
    Quaternion operator +(Quaternion q);
    Quaternion operator *(Quaternion q);
    Quaternion operator *(float s);
    bool operator ==(Quaternion q);
    void print(char* buf);
    Vector3D vec();

};


#endif