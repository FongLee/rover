#ifndef AP_MATH_H
#define AP_MATH_H

#include "matrix.h"

#define PI 3.141592653589793f

//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define VEC3_X	0
#define VEC3_Y	1
#define VEC3_Z	2

#define IS_SET(field, mask) (((field)&(mask)) == (mask))
#define SET_MASK(field, mask) (field) |= (mask)
#define UNSET_MASK(field, mask) (field) &= ~(mask)
/**
 * a varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is
 * returned.
 * @param  v [description]
 * @return   [description]
 */
float safe_asin(float v);

/**
 * a varient of sqrt() that checks the input ranges and ensures a
 * valid value as output. If a negative number is given then 0 is
 * returned. The reasoning is that a negative number for sqrt() in our
 * code is usually caused by small numerical rounding errors, so the
 * real input should have been zero
 * @param  v [description]
 * @return   [description]
 */
float safe_sqrt(float v);

/**
 * constrain a value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
float constrain_float(float amt, float low, float high);

/**
 * constrain a int16_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
int constrain_int(int amt, int low, int high);

/**
 * constrain a int32_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
long constrain_long(long amt, long low, long high);

/**
 * degrees -> radians
 * @param  deg [description]
 * @return     [description]
 */
float radians(float deg);

/**
 * radians -> degrees
 * @param  rad [description]
 * @return     [description]
 */
float degrees(float rad);

/**
 * square
 * @param  v [description]
 * @return   [description]
 */
float sq(float v);

/**
 * 2D vector length
 * @param  a [description]
 * @param  b [description]
 * @return   [description]
 */
float pythagorous2(float a, float b);

/**
 * 3D vector length
 * @param  a [description]
 * @param  b [description]
 * @param  c [description]
 * @return   [description]
 */
float pythagorous3(float a, float b, float c);

float inv_sqrt(float x);

int sor_iteration(MAT * matrix_a, VEC *vec_b, VEC *out);

#endif