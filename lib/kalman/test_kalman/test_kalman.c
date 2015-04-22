#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <math.h>

#include "Console.h"
#include "kalman.h"
#include "matrix.h"
#include "matrix_kalman.h"

#define PRECISION 1e-3

bool is_equal(float a, float b);
bool test_matrix_equal(MAT *m1, MAT * m2);
int init_suite();
int end_suite();
int test_predict_model();
void test1_predict_update();
void test2_predict();
int test_update_model();
void test1_update();

/**
 * verify whether two data is equal
 * @param  a [description]
 * @param  b [description]
 * @return   [description]
 */
bool is_equal(float a, float b)
{
	return (fabs(a-b) <= PRECISION) ? true:false;
}

/**
 * verify whether two matrix is equal
 * @param  m1 [description]
 * @param  m2 [description]
 * @return    [description]
 */
bool test_matrix_equal(MAT *m1, MAT *m2)
{
	if (m1 == NULL || m2 == NULL)
		return false;
	if (m1->m != m2->m || m1->n != m2->n)
		return false;

	for (int i = 0; i < m1->m; i++)
	{
		for (int j = 0; j < m1->n; j++)
		{
			if (!is_equal(m1->me[i][j], m2->me[i][j]))
				return false;
		}
	}
	return true;
}

int init_suite()
{
	return 0;
}

int end_suite()
{
	return 0;
}

/**
 * test predict() function and update() function
 */
void test1_predict_update()
{
	kalman_t *f_test;
	alloc_kalman_filter(&f_test, 3, 2, 4);
	//x(k) = (1.1, 2.2, 3.3)^T
	set_matrix(f_test->state_estimate, 1.1, 2.2, 3.3);
	//P(k) = (5.5, 0, 0,
	//			0, 6.6, 0,
	//			0, 0, 7.7)
	set_matrix(f_test->covariance_estimate, 5.5, 0.0, 0.0, 
											0.0, 6.6, 0.0,
											0.0, 0.0, 7.7);
	//Q = (1.2, 0, 0,
	//		0, 2.3, 0,
	//		0, 0, 3.4)
	set_matrix(f_test->process_noise_covariance, 1.2, 0.0, 0.0,
													0.0, 2.3, 0.0, 
													0.0, 0.0, 3.4);
	//R= (5.6, 0, 
	//		0, 7.8)
	set_matrix(f_test->measure_noise_covariance, 5.6, 0.0, 
													0.0, 7.8); 
								
	//z(k) = (12, 13)^T
	set_matrix(f_test->measure, 12.0, 13.0);
	//u(k-1) = (1, 2, 3, 4)^T
	set_matrix(f_test->control_input, 1.0, 2.0, 3.0, 4.0);
	//H = (10.0, 11.0, 12.0,
	//		13.0, 14.0, 15.0)
	set_matrix(f_test->measure_model, 10.0, 11.0, 12.0,
										13.0, 14.0, 15.0);
	//B = 23, 24, 25, 26,
	//		33, 34, 35, 36,
	//		45, 46, 47, 48)
	set_matrix(f_test->control_input_model, 23.0, 24.0, 25.0, 26.0,
											33.0, 34.0, 35.0, 36.0,
											45.0, 46.0, 47.0, 48.0);
	//A =  1 2 3 
	//		4 5 6
	//		7 8 9 
	set_matrix(f_test->state_transition, 1.0, 2.0, 3.0,
											4.0, 5.0, 6.0,
											7.0, 8.0, 9.0);

	predict(f_test);
	MAT * state_pre;
	state_pre = m_get(3, 1);
	MAT * cov_pre;
	cov_pre = m_get(3, 3);
	set_matrix(state_pre, 265.4, 385.2, 525.0);
	set_matrix(cov_pre, 102.4, 226.6, 352.0,
						226.6, 532.5, 833.8,
						352.0, 833.8, 1319.0);
	CU_ASSERT(test_matrix_equal(f_test->state_predict, state_pre));
	CU_ASSERT(test_matrix_equal(f_test->covariance_predict, cov_pre));

	update(f_test);
	MAT * state_esti;
	MAT * cov_esti;
	state_esti = m_get(3, 1);
	cov_esti = m_get(3, 3);
	set_matrix(state_esti, 92.3664, -2.3668, -75.4838);
	set_matrix(cov_esti, 5.1610,  -0.1683,  -4.2337,
							-0.1683, 1.5819, -1.314,
							-4.2337, -1.3144, 4.8296);
	CU_ASSERT(test_matrix_equal(f_test->state_estimate, state_esti));
	CU_ASSERT(test_matrix_equal(f_test->covariance_estimate, cov_esti));
	//m_output(f_test->state_estimate);
	//m_output(f_test->covariance_estimate);

}

/**
 * add suite and test
 * @return [description]
 */
int test_predict_model()
{
	CU_pSuite p_suite = NULL;
	p_suite = CU_add_suite("test_predict", init_suite, end_suite);
	if (p_suite == NULL)
	{
		return -1;
	}
	if (CU_add_test(p_suite, "test1_predict", test1_predict_update) == NULL)
	{
		return -1;
	}
	return 0;
}