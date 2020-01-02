#include "vector3.h"

Vector3 add(Vector3 a, Vector3 b)
{
    return (Vector3){
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

// The resulting Vector3 points from 'from' to 'to'.
Vector3 sub(Vector3 from, Vector3 to)
{
    return (Vector3){
        to.x - from.x,
        to.y - from.y,
        to.z - from.z
    };
}

Vector3 mul(Vector3 vec3, float f)
{
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
    return mul(vec3, magInv);
}
