#include "msvector.h"
#include <cstdio>
#include <math.h>
#include "quaternion.h"

void MSVector3::print(char* buf){
    sprintf(buf, "(%f, %f, %f)", x, y, z);
}

MSVector3::MSVector3(float vx, float vy, float vz){
    x = vx; 
    y = vy; 
    z = vz;
}

MSVector3::MSVector3(){

}

MSVector3::MSVector3(BLA::Matrix<3,1,float> bla_vec){
    x = bla_vec(0,0);
    y = bla_vec(1,0);
    z = bla_vec(2,0);
}

float MSVector3::operator*(MSVector3 v){
    return x * v.x + y * v.y + z * v.z;
}

MSVector3 MSVector3::operator*(float s){
    return MSVector3(x * s, y * s, z * s);
}

bool MSVector3::operator==(MSVector3 v){
    return abs(x - v.x) < EQ_EPSILON && abs(y - v.y) < EQ_EPSILON && abs(z - v.z) < EQ_EPSILON;
}

void MSVector3::operator =(MSVector3 v){
    x = v.x;
    y = v.y;
    z = v.z;
}

MSVector3 MSVector3::operator+(MSVector3 v){
    return MSVector3(x + v.x, y + v.y, z + v.z);
}

MSVector3 MSVector3::operator-(MSVector3 v){
    return MSVector3(x - v.x, y - v.y, z - v.z);
}

float MSVector3::norm(){
    return sqrt(x*x + y*y + z*z);
}

void MSVector3::normize(){
    float n = norm();
    x /= n;
    y /= n;
    z /= n;
}

MSVector3 MSVector3::cross(MSVector3 v){
    return MSVector3(
        y * v.z - z * v.y, 
        v.x * z - x * v.z, 
        x * v.y - v.x * y
    );
}

Quaternion MSVector3::to_dq(){
    float phi = norm();
    if (phi < 1e-7f){
        return Quaternion(1, x/2.0f, y/2.0f, z/2.0f);   // Stable conversion to quaternion for small angles
    }
    
    MSVector3 u_sinphi = *this * (1.0f/phi) * sinf(phi/2.0f);
    return Quaternion(cosf(phi/2.0f), u_sinphi.x, u_sinphi.y, u_sinphi.z);
}

BLA::Matrix<3,3,float> MSVector3::to_skewsymL(){
    return BLA::Matrix<3,3,float> (
        0 , -z, y,
        z , 0 , -x,
        -y, x , 0
    );
}