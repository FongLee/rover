#include <stdio.h>
#include <assert.h>

#include "Console.h"

int test_predict_model();

int main()
{
	if (CU_initialize_registry() != CUE_SUCCESS)
	{
		return CU_get_error();
	}
	assert(CU_get_registry() != NULL);
	assert(!CU_is_test_running());
	if (test_predict_model() != 0)
	{
		CU_cleanup_registry();
		return CU_get_error();
	}
	CU_console_run_tests();
	CU_cleanup_registry();

	return 0;
}