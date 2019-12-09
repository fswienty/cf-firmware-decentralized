#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "vector3.h"

Vector3 add(Vector3 a, Vector3 b)
{
    Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// The resulting Vector3 points from 'from' to 'to'.
Vector3 sub(Vector3 from, Vector3 to)
{
    Vector3 result;
    result.x = to.x - from.x;
    result.y = to.y + from.y;
    result.z = to.z + from.z;
    return result;
}

Vector3 mul(Vector3 vec3, float f)
{
    Vector3 result;
    result.x = vec3.x * f;
    result.y = vec3.y * f;
    result.z = vec3.z * f;
    return result;
}

float magnitude(Vector3 vec3)
{
    return sqrtf(powf(vec3.x, 2) + powf(vec3.y, 2) + powf(vec3.z, 2));
}

Vector3 norm(Vector3 vec3)
{
    float mag = magnitude(vec3);
    Vector3 result;
    result.x = vec3.x / mag; 
    result.y = vec3.y / mag;
    result.z = vec3.z / mag;
    return result;
}
