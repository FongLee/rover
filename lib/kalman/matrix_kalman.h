#ifndef MATRIX_KALMAN_H
#define MATRIX_KALMAN_H

#include "matrix.h"

MAT *m_mlt_by_trans(MAT *a, MAT *b, MAT *out);

MAT *set_matrix(MAT *m, ...);

#endif