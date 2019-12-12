#include "vector3.h"

Vector3 add(Vector3 a, Vector3 b)
{
    // Vector3 result;
    // result.x = a.x + b.x;
    // result.y = a.y + b.y;
    // result.z = a.z + b.z;
    // return result;
    return (Vector3){
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

// The resulting Vector3 points from 'from' to 'to'.
Vector3 sub(Vector3 from, Vector3 to)
{
    // Vector3 result;
    // result.x = to.x - from.x;
    // result.y = to.y - from.y;
    // result.z = to.z - from.z;
    // return result;
    return (Vector3){
        to.x - from.x,
        to.y - from.y,
        to.z - from.z
    };
}

Vector3 mul(Vector3 vec3, float f)
{
    // Vector3 result;
    // result.x = vec3.x * f;
    // result.y = vec3.y * f;
    // result.z = vec3.z * f;
    // return result;
    return (Vector3){
        vec3.x * f,
        vec3.y * f,
        vec3.z * f
    };
}

float magnitude(Vector3 vec3)
{
    return sqrtf(powf(vec3.x, 2) + powf(vec3.y, 2) + powf(vec3.z, 2));
}

Vector3 norm(Vector3 vec3)
{
    float magInv = 1 / magnitude(vec3);
    // Vector3 result;
    // result.x = vec3.x * magInv; 
    // result.y = vec3.y * magInv;
    // result.z = vec3.z * magInv;
    // return result;
    return mul(vec3, magInv);
}
