#include <stdarg.h>
#include "matrix.h"

#ifndef MEMWATCH
#include "memwatch.h"
#endif
MAT *m_mlt_by_trans(MAT *a, MAT *b, MAT *out)
{
	static MAT *c;
	c = m_get(b->n, b->m);
	return m_mlt(a, m_transp(b, c), out);
}

MAT *set_matrix(MAT *m, ...)
{
	va_list ap;
	va_start(ap, m);
	for (int i = 0; i < m->m; i++)
		for (int j = 0; j < m->n; j++)
		{
			m->me[i][j] = va_arg(ap, double);
		}

	return m;
}