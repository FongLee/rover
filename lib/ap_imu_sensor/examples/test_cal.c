#include <stdio.h>

#include "matrix.h"
#include "mpu_calibration.h"

#include "matrix_kalman.h"

int main()
{

	MAT * mea_imu;
	VEC * compass_bias;
	VEC * compass_factor;

	mea_imu = m_get(6, 3);
	set_matrix(mea_imu,
					-6.0, 28.0, -104.0,
					71.0, 111.0, 104.0,
					-10.0, 175.0, 105.0,
					-79.0, 91.0, 106.0,
					-144.0,92.0,-94.0,
					118.0,108.0,-108.0);
	m_output(mea_imu);
	compass_bias = v_get(3);
	compass_factor = v_get(3);

	imu_calibration(mea_imu, compass_bias, compass_factor);
	//v_output(compass_bias);
	//v_output(compass_factor);
	//fprintf(stdout, "%s\n", );

	return 0;
}
