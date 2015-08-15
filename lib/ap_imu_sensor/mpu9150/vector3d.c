
#include "vector3d.h"

void vector3DotProduct(vector3d_t a, vector3d_t b, float *d)
{
	*d = a[VEC3_X] * b[VEC3_X] + a[VEC3_Y] * b[VEC3_Y] + a[VEC3_Z] * b[VEC3_Z];
}

void vector3CrossProduct(vector3d_t a, vector3d_t b, vector3d_t d)
{
	d[VEC3_X] = a[VEC3_Y] * b[VEC3_Z] - a[VEC3_Z] * b[VEC3_Y];
	d[VEC3_Y] = a[VEC3_Z] * b[VEC3_X] - a[VEC3_X] * b[VEC3_Z];
	d[VEC3_Z] = a[VEC3_X] * b[VEC3_Y] - a[VEC3_Y] * b[VEC3_X];
}

