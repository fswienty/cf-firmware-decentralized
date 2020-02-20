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
    return sqrtf(vec3.x * vec3.x + vec3.y * vec3.y + vec3.z * vec3.z);
}

Vector3 norm(Vector3 vec3)
{
    return mul(vec3, 1 / magnitude(vec3));
}

Vector3 clamp(Vector3 vec3, float maxLength)
{
    float length = magnitude(vec3);
    if (length > maxLength)
    {
        return mul(vec3, maxLength / length);
    }
    return vec3;
}
