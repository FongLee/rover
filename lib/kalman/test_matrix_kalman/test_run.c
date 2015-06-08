#include <stdio.h>
#include <assert.h>

#include "Console.h"

extern int test_m_trans_module();

int main(int argc, char const *argv[])
{
	if (CU_initialize_registry() != CUE_SUCCESS)
	{
		return CU_get_error();
	}

	assert(CU_get_registry() != NULL);
	assert(!CU_is_test_running());
	if (test_m_trans_module() != 0)
	{
		CU_cleanup_registry();
		return CU_get_error();
	}
	CU_console_run_tests();
	CU_cleanup_registry();

	return 0;
}
