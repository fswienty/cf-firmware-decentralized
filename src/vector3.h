#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct _Vector3
{
  float x;
  float y;
  float z;
} Vector3;

Vector3 add(Vector3 a, Vector3 b);
Vector3 sub(Vector3 from, Vector3 to);
Vector3 mul(Vector3 vec3, float f);
float magnitude(Vector3 vec3);
Vector3 norm(Vector3 vec3);