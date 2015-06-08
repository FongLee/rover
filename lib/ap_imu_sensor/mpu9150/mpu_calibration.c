#include <math.h>
#include <stdio.h>

#include "matrix_kalman.h"
#include "mpu_calibration.h"
#include "ap_math.h"

#ifdef MEMWATCH
#include "memwatch.h"
#endif


//#include "matrix.h"
#define PARA_NUM 6
#define MEA_NUM 6


/**
 * imu_calibration
 * @param  mea        collected n group data,n*3
 * @param  cal_bias   [description]
 * @param  cal_factor [description]
 * @return            [description]
 */
int imu_calibration(MAT *mea, VEC *cal_bias, VEC *cal_factor)
{
	if (mea == NULL || cal_bias == NULL || cal_factor == NULL)
	{
		return -1;
	}

	if (cal_bias->dim != 3 || cal_factor->dim != 3 || mea->n != 3)
	{
		return -1;
	}

	unsigned int measure_num = mea->m;
	//int i = 0 , j = 0;
	MAT * matrix_v;
	matrix_v = m_get(PARA_NUM, measure_num);
	// 	x0 * x0, 	x1 * x1, 	x2 * x2, 	x3 * x3, 	x4 * x4,	x5 * x5,
	// 	y0 * y0, 	y1 * y1, 	y2 * y2, 	y3 * y3, 	y4 * y4,	y5 * y5,
	// 	z0 * z0, 	z1 * z1, 	z2 * z2, 	z3 * z3, 	z4 * z4, 	z5 * z5,
	// 	x0, 	 	x1, 	   	x2, 		x3, 		x4, 		x5,
	// 	y0, 		y1, 		y2, 		y3, 		y4, 		y5,
	// 	z0, 		z1, 		z2, 		z3, 		z4, 		z5
	for (int i = 0; i < measure_num; i++)
	{
		matrix_v->me[0][i] =mea->me[i][0] * mea->me[i][0];
		matrix_v->me[1][i] =mea->me[i][1] * mea->me[i][1];
		matrix_v->me[2][i] =mea->me[i][2] * mea->me[i][2];
		matrix_v->me[3][i] =mea->me[i][0];
		matrix_v->me[4][i] =mea->me[i][1];
		matrix_v->me[5][i] =mea->me[i][2];

	}

	//m_output(matrix_v);
	MAT * matrix_a;
	matrix_a = m_get(PARA_NUM, PARA_NUM);

	MAT * matrix_vt;
	matrix_vt = m_get(measure_num, PARA_NUM);
	m_transp(matrix_v, matrix_vt);

	m_mlt(matrix_v, matrix_vt, matrix_a);

	VEC *vec_b;
	vec_b = v_get(PARA_NUM);

	for (int i = 0; i < PARA_NUM; i++)
		for (int j = 0; j < PARA_NUM; j++)
		{
			vec_b->ve[i] += -matrix_v->me[i][j];
		}

	VEC *para;
	para = v_get(PARA_NUM);

	sor_iteration(matrix_a, vec_b, para);

	cal_bias->ve[0] = para->ve[3] / 2 / para->ve[0];
	cal_bias->ve[1] = para->ve[4] / 2 / para->ve[1];
	cal_bias->ve[2] = para->ve[5] / 2 / para->ve[2];

	double tmp_c;
	tmp_c = 9.80 * 9.80 /(para->ve[3] * para->ve[3] / para->ve[0] / 4
				+para->ve[4] * para->ve[4] / para->ve[1] /4
				+para->ve[5] * para->ve[5] / para->ve[2] /4 -1.0);
	cal_factor->ve[0] = sqrt(fabs(tmp_c * para->ve[0]));
	cal_factor->ve[1] = sqrt(fabs(tmp_c * para->ve[1]));
	cal_factor->ve[2] = sqrt(fabs(tmp_c * para->ve[2]));

	m_free(matrix_v);
	m_free(matrix_vt);
	m_free(matrix_a);

	v_free(vec_b);
	v_free(para);
	return 0;
}

// int sor_iteration(MAT * matrix_a, VEC *vec_b, VEC *out)
// {
// 	double w = 0.9;
// 	double precision = 0.000001;
// 	double tmp;
// 	double x[PARA_NUM] = {0};

// 	int i = 0, j = 0;
// 	for (int k = 0; k < 1000; k++)
// 	{
// 		for (i = 0; i < PARA_NUM; i++)
// 		{
// 			tmp = 0;
// 			for (j = 0; j < PARA_NUM; j++)
// 			{
// 				if (i != j)
// 				{
// 					tmp +=matrix_a->me[i][j] * out->ve[j];
// 				}
// 				out->ve[i] = (1 - w) * x[i] + w * (vec_b->ve[i] - tmp) / matrix_a->me[i][i];
// 			}
// 		}

// 		for (i = 0; i < PARA_NUM; i++)
// 		{
// 			if (fabs(out->ve[i] - x[i]) < precision)
// 			{
// 				if (i == 3)
// 				{
//          			fprintf(stdout, "your relaxaton factor is %f\n", w);
//                     fprintf(stdout, "your precision is %f\n", precision);
//                     fprintf(stdout, "iteration is %d\n", k);
//                     fprintf(stdout, "iteration result is :\n");
//                     for (int i = 0; i < PARA_NUM; i++)
//                     {
//                        //cout << "x" << i << ":" << matrix_x[i] << endl;
//                         fprintf(stdout, "x%d is %g\n", i ,out->ve[i]);
//                     }
//                     // cout <<"x1 = " << matrix_x[0] << "    x2 = " << matrix_x[1];
//                     // cout <<       "x3 = " << matrix_x[2] << "    x4 = " << matrix_x[3];
//                     // cout <<       "x5 = " << matrix_x[4] << "    x6 = " << matrix_x[5] << endl;
//                     return 0;
// 				}
// 			}
// 			else
// 				break;
// 		}

// 		for ( i = 0; i < PARA_NUM; i++)
// 		{
// 			x[i] = out->ve[i];
// 		}
// 	}

// 	return -1;
// }

