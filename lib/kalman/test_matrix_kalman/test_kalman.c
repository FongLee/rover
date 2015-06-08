#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <math.h>

#include "Console.h"
//#include "CUnit.h"
//#include "TestDB.h"
//#include "Automated.h"
#include "kalman.h"
#include "matrix_kalman.h"

#define PRECISION 1e-6

int init_suite();
int end_suite();
bool is_equal(float a, float b);
bool test_matrix_equal(MAT *m1, MAT *m2);
void test1_m_by_trans();
void test2_m_by_trans();

// CU_TestInfo test_cases[] =
// {
// 	{"texting example1: ", test1_m_by_trans},
// 	{"texting example2: ", test2_m_by_trans},
// 	CU_TEST_INFO_NULL
// };


// CU_SuiteInfo suites[] =
// {
// 	{"test function m_mlt_by_trans()", init_suite, end_suite, test_cases},
// 	CU_SUITE_INFO_NULL
// };

int init_suite()
{
	return 0;
}

int end_suite()
{
	return 0;
}

/**
 * verify whether two data is equal
 * @param  a [description]
 * @param  b [description]
 * @return   [description]
 */
bool is_equal(float a, float b)
{
	return (fabs(a-b) <= PRECISION) ? true : false;
}

/**
 * verify whether the matrix is equal
 * @param  m1 [description]
 * @param  m2 [description]
 * @return    [description]
 */
bool test_matrix_equal(MAT *m1, MAT *m2)
{
	int i = 0;
	int j = 0;
	//verify the dimension of matrix
	if (m1->m != m2->m || m1->n != m2->n)
	{
		return false;
	}

	for (i = 0; i < m1->m; i++)
	{
		for (j = 0; j < m1->n; j++)
		{
			if (!is_equal(m1->me[i][j], m2->me[i][j]))
			{
				return false;
			}
		}
	}
	return true;
}

/**
 * test a example1 using m_mlt_by_trans() function
 */
void test1_m_by_trans()
{
	MAT *a = m_get(3, 3);
	MAT *b = m_get(3,3);
	MAT *c = m_get(3, 3);
	MAT *d = m_get(3, 3);
	set_matrix(a, 1.0, 2.0, 3.0,
					4.0, 5.0, 6.0,
					7.0, 8.0, 9.0);
	set_matrix(b, 4.0, 4.0, 4.0,
					4.0, 5.0, 6.0,
					7.0, 8.0, 9.0);
	set_matrix(c, 24.0, 32.0, 50.0,
				 	60.0, 77.0, 122.0,
					96.0, 122.0, 194.0);
	m_output(a);
	m_output(b);
	m_output(c);
	m_mlt_by_trans(a, b, d);
	m_output(d);
	CU_ASSERT(test_matrix_equal(c, d));
	m_free(a);
	m_free(b);
	m_free(c);
	m_free(d);
}

/**
 * test a example2 using m_mlt_by_trans() function
 */
void test2_m_by_trans()
{
	MAT *a = m_get(3, 3);
	MAT *b = m_get(3,3);
	MAT *c = m_get(3, 3);
	MAT *d = m_get(3, 3);
	set_matrix(a, 11.0, 12.0, 14.0,
					15.0, 16.0, 17.0,
					18.0, 19.0, 20.0);
	set_matrix(b, 1.1, 2.2, 3.3,
					4.4, 5.5, 6.6,
					7.7, 8.8, 9.9);
	set_matrix(c, 84.7, 206.8, 328.9,
				 	107.8, 266.2, 424.6,
					127.6, 315.7, 503.8);
	m_output(a);
	m_output(b);
	m_output(c);
	m_mlt_by_trans(a, b, d);
	m_output(d);
	CU_ASSERT(test_matrix_equal(c, d));
	m_free(a);
	m_free(b);
	m_free(c);
	m_free(d);

}

int test_m_trans_module()
{
	CU_pSuite p_suite = NULL;
	p_suite = CU_add_suite("test_matrix_kalman", init_suite, end_suite);
	if (p_suite == NULL)
	{
		return -1;
	}
	if (CU_add_test(p_suite, "test1_m_by_trans", test1_m_by_trans) == NULL
		|| CU_add_test(p_suite, "test2_m_by_trans", test2_m_by_trans) == NULL)
	{
		return -1;
	}
	// if ( CU_register_suites(suites) != CUE_SUCCESS)
	// {
	// 	return -1;
	// }

	return 0;
}
