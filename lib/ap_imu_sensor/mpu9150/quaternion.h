
#ifndef MPUQUATERNION_H
#define MPUQUATERNION_H

#include "vector3d.h"

#define QUAT_W		0
#define QUAT_X		1
#define QUAT_Y		2
#define QUAT_Z		3

typedef float quaternion_t[4];

void quaternionNormalize(quaternion_t q);
void quaternionToEuler(quaternion_t q, vector3d_t v);
void eulerToQuaternion(vector3d_t v, quaternion_t q);
void quaternionConjugate(quaternion_t s, quaternion_t d);
void quaternionMultiply(quaternion_t qa, quaternion_t qb, quaternion_t qd);


#endif /* MPUQUATERNION_H */

