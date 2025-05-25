#ifndef QUATERNION
#define QUATERNION

#define EQ_EPSILON 1e-6

#include "BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "msvector.h"


/*
    WARNING: 
    This struct uses Union. It relies on w,i,j,k being all the same type and the same 
    type as data[4]. Only then is the memory layout contiguous. 
    Breaking this will lead to RANDOM BEHAVIOR
*/

struct Quaternion { 
    public:
    union {
        struct {
            float w, i, j, k;
        }; 
        float data[4];
    };

    Quaternion(float wq, float iq, float jq, float kq); 
    Quaternion(MSVector3 vec);   // create pure quaternion from vec
    Quaternion(BLA::Matrix<3,3,float> rotmat); // convert from rotation matrix
    Quaternion();
    float norm() const;
    void normize();
    Quaternion T() const;        //  returns conjugate. 
    Quaternion operator +(Quaternion q) const;
    Quaternion operator *(Quaternion q) const;
    Quaternion operator *(float s) const;
    Quaternion operator /(float s) const;
    Quaternion operator /(Quaternion q) const;
    Quaternion inv();
    bool operator ==(Quaternion q) const;
    void print(char* buf) const;
    void operator =(Quaternion q);
    MSVector3 vec() const;
    BLA::Matrix<3,3,float> to_R() const;      // converts to rotation matrix
    MSVector3 rotate(MSVector3 v) const;
};


#endif 