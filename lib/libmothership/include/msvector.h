#ifndef MSVECTOR
#define MSVECTOR

// Just some reimplemented Vector3 for convenience
#include "BasicLinearAlgebra/BasicLinearAlgebra.h"

struct Quaternion;

#define EQ_EPSILON 1e-6

/*
    WARNING: 
    This struct uses Union. It relies on x,y,z being all the same type and the same 
    type as data. Only then is the memory layout contiguous. 
    Breaking this will lead to RANDOM BEHAVIOR
*/

struct MSVector3 {
    public:
    union {
        struct {
            float x, y, z = 0.0f;
        };
        float data[3];
    };

    MSVector3(float vx, float vy, float vz);
    MSVector3(BLA::Matrix<3,1,float> bla_vec);
    MSVector3();

    BLA::Matrix<3,1>& as_BlaVec(){
        return *reinterpret_cast<BLA::Matrix<3,1,float>*>(data);
    }

    const BLA::Matrix<3,1>& as_BlaVec() const{
        return *reinterpret_cast<const BLA::Matrix<3,1,float>*>(data);
    }

    void print(char* buf);

    float operator*(MSVector3 v);
    MSVector3 operator*(float s);
    bool operator ==(MSVector3 v);
    void operator =(MSVector3 v);
    MSVector3 operator+(MSVector3 v);
    MSVector3 operator-(MSVector3 v);
    void normize();
    float norm();
    MSVector3 cross(MSVector3 v);
    Quaternion to_dq();         // Stable conversion to quaternion for small angles
    BLA::Matrix<3,3,float> to_skewsymL();
};


#endif