
#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <math.h>

#define	DEGREE_TO_RAD		((float)M_PI / 180.0f)
#define	RAD_TO_DEGREE		(180.0f / (float)M_PI)

#define TWO_PI				(2.0f * (float)M_PI)

#define VEC3_X		0
#define VEC3_Y		1
#define VEC3_Z		2

typedef float vector3d_t[3];

void vector3DotProduct(vector3d_t a, vector3d_t b, float *d);
void vector3CrossProduct(vector3d_t a, vector3d_t b, vector3d_t d);

#endif /* VECTOR3D_H */

