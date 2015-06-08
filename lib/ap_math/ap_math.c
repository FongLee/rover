#include <math.h>
#include "ap_math.h"


/**
 * a varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is
 * returned.
 * @param  v [description]
 * @return   [description]
 */
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0;
	}
	if (v >= 1.0f)
	{
		return PI / 2;
	}
	if (v <= -1.0f)
	{
		return -PI / 2;
	}
	return asin(v);
}

/**
 * a varient of sqrt() that checks the input ranges and ensures a
 * valid value as output. If a negative number is given then 0 is
 * returned. The reasoning is that a negative number for sqrt() in our
 * code is usually caused by small numerical rounding errors, so the
 * real input should have been zero
 * @param  v [description]
 * @return   [description]
 */
float safe_sqrt(float v)
{
	float ret = sqrt(v);
	if (isnan(ret))
	{
		return 0;
	}
	return ret;
}

/**
 * constrain a value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
float constrain_float(float amt, float low, float high)
{
	if (isnan(amt))
	{
		return (low + high) * 0.5f;
	}
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * constrain a int16_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
int constrain_int(int amt, int low, int high)
{
	return ((amt) < (low) ? (low) :((amt) > (high) ? (high) : (amt)));
}

/**
 * constrain a int32_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
long constrain_long(long amt, long low, long high)
{
	return ((amt) < (low) ? (low) :((amt) > (high) ? (high) : (amt)));
}

/**
 * degrees -> radians
 * @param  deg [description]
 * @return     [description]
 */
float radians(float deg)
{
	return deg * DEG_TO_RAD;
}

/**
 * radians -> degrees
 * @param  rad [description]
 * @return     [description]
 */
float degrees(float rad)
{
	return rad * RAD_TO_DEG;
}

/**
 * square
 * @param  v [description]
 * @return   [description]
 */
float sq(float v)
{
	return v * v;
}

/**
 * 2D vector length
 * @param  a [description]
 * @param  b [description]
 * @return   [description]
 */
float pythagorous2(float a, float b)
{
	return sqrt(sq(a) + sq(b));
}

/**
 * 3D vector length
 * @param  a [description]
 * @param  b [description]
 * @param  c [description]
 * @return   [description]
 */
float pythagorous3(float a, float b, float c)
{
	return sqrt(sq(a) + sq(b) + sq(c));
}

/**
 * the fast square root algorithm
 * @param  x [description]
 * @return   [description]
 */
float inv_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * SOR iteration method
 * A * b = c
 * @param  matrix_a [description]
 * @param  vec_b    [description]
 * @param  out      [description]
 * @return          [description]
 */
int sor_iteration(MAT * matrix_a, VEC *vec_b, VEC *out)
{
	if (matrix_a == NULL || vec_b == NULL || out == NULL)
	{
		return -1;
	}
	if (matrix_a->m != matrix_a->n
		|| vec_b->dim != matrix_a->m
		|| out->dim != vec_b->dim)
	{
		return -1;
	}

	unsigned int para_num= matrix_a->m;
	double w = 0.9;
	double precision = 1.0e-10;
	double tmp;
	//double x[PARA_NUM] = {0};
	double *x;
	//double x[matrix_a->m] = {0};
	x = malloc(sizeof(double) * para_num);

	int i = 0, j = 0;
	for (i = 0; i < para_num; i++)
	{
		x[i] =0;
	}
	for (int k = 0; k < 1000; k++)
	{
		for (i = 0; i < para_num; i++)
		{
			tmp = 0;
			for (j = 0; j < para_num; j++)
			{
				if (i != j)
				{
					tmp +=matrix_a->me[i][j] * out->ve[j];
				}
				out->ve[i] = (1 - w) * x[i] + w * (vec_b->ve[i] - tmp) / matrix_a->me[i][i];
			}
		}

		for (i = 0; i < para_num; i++)
		{
			if (fabs(out->ve[i] - x[i]) < precision)
			{
				if (i == (para_num - 1))
				{
         			// fprintf(stdout, "your relaxaton factor is %f\n", w);
            //         fprintf(stdout, "your precision is %f\n", precision);
                    fprintf(stdout, "iteration is %d\n", k);
            //         fprintf(stdout, "iteration result is :\n");
            //         for (int i = 0; i < para_num; i++)
            //         {
            //             fprintf(stdout, "x%d is %g\n", i ,out->ve[i]);
            //         }

                    free(x);
                    return 0;
				}
			}
			else
				break;
		}

		for ( i = 0; i < para_num; i++)
		{
			x[i] = out->ve[i];
		}
	}
	 free(x);
	return -1;
}

