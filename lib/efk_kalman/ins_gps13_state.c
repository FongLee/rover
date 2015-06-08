
#define NUMX 13
#define NUMW 9
#define NUMV 10
#define NUMU 6

static const int8_t frow_min[NUMX] = { 3, 4, 5, 6, 6, 6, 7, 6, 6, 6, 13, 13, 13 };
static const int8_t frow_max[NUMX] = { 3, 4, 5, 9, 9, 9, 12, 12, 12, 12, -1, -1, -1 };
static const int8_t grow_min[NUMX] = { 9, 9, 9, 3, 3, 3, 0, 0, 0, 0, 6, 7, 8 };
static const int8_t grow_max[NUMX] = { -1, -1, -1, 5, 5, 5, 2, 2, 2, 2, 6, 7, 8 };
static const int8_t hrow_min[NUMV] = { 0, 1, 2, 3, 4, 5, 6, 6, 6, 2 };
static const int8_t hrow_max[NUMV] = { 0, 1, 2, 3, 4, 5, 9, 9, 9, 2 };

static struct
{
	float f[NUMX][NUMX];
	float g[NUMX][NUMW];
	float h[NUMV][NUMX];

	float be[3];

	float p[NUMX][NUMX];
	float x[NUMX];

	float q[NUMW];
	float r[NUMV];
} ekf;

struct nav_t nav;

uint16_t ins_get_num_states()
{
	return NUMX;
}

void ins_gps_init()
{
	ekf.be[0] = 1.0f;
	ekf.be[1] = 0.0f;
	ekf.be[2] = 0.0f;

	for (int i = 0; i < NUMX; i++)
	{
		for (int j = 0;j < NUMX; j++)
		{
			ekf.p[i][j] = 0.0f;
			ekf.f[i][j] = 0.0f;
		}
		for (int j = 0; j < NUMW; j++)
		{
			ekf.g[i][j] = 0.0f;
		}

		for (int j = 0; j < NUMV; j++)
		{
			ekf.h[j][i] = 0.0f;
		}

		ekf.x[i] = 0.0f;
	}

	for (int i = 0; i < NUMW; i++)
	{
		ekf.q[i] = 0.0f;
	}

	for (int i = 0; i < NUMV; i++)
	{
		ekf.r[i] = 0.0f;
	}

	ekf.p[0][0] = ekf.p[1][1] = ekf.p[2][2] = 25.0f;
	ekf.p[3][3] = ekf.p[4][4] = ekf.p[5][5] = 5.0f;
	ekf.p[6][6] = ekf.p[7][7] = ekf.p[8][8] = 1e-5f;
	ekf.p[10[10] = ekf.p[11][11] = ekf.p[12][12] = 1e-9f;

	ekf.x[0] = ekf.x[1] = ekf.x[2] = ekf.x[3] = ekf.x[4] = ekf.x[5] = 0.0f;
	ekf.x[6] = 1.0f;
	ekf.x[7] = ekf.x[8] = ekf.x[9] = 0.0f;
	ekf.x[10] = ekf.x[11] = ekf.x[12] = 0.0f;

	ekf.q[0] = ekf.q[1] = ekf.q[2] = 50e-4f;
	ekf.q[3] = ekf.q[4] = ekf.q[5] = 0.00001f;
	ekf.q[6] = ekf.q[7] = ekf.q[8] = 2e-8f;

	ekf.r[0] = ekf.r[1] = 0.004f;
	ekf.r[2] = 0.036f;
	ekf.r[3] = ekf.r[4] = 0.004f;
	ekf.r[5] = 100.0f;
	ekf.r[6] = ekf.r[7] = ekf.r[8] = 0.005f;
	ekf.r[9] = .25f;


}

void ins_set_state(float pos[3], float vel[3], float q[4], float gyro_bias[3])
{
	ekf.x[0] = pos[0];
	ekf.x[1] = pos[1];
	ekf.x[2] = pos[2];
	ekf.x[3] = vel[0];
	ekf.x[4] = vel[1];
	ekf.x[5] = vel[2];
	ekf.x[6] = q[0];
	ekf.x[7] = q[1];
	ekf.x[8] = q[2];
	ekf.x[9] = q[3];
	ekf.x[10] = gyro_bias[0];
	ekf.x[11] = gyro_bias[1];
	ekf.x[12] = gyro_bias[2];

}

void ins_pos_vel_reset(float pos[3], float vel[3])
{
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < NUMX; j++)
		{
			ekf.p[i][j] = 0;
			ekf.p[j][i] = 0;
		}
	}
	ekf.p[0][0] = ekf.p[1][1] = ekf.p[2][2] = 25;
	ekf.p[3][3] = ekf.p[4][4] = ekf.p[5][5] = 5;

	ekf.x[0] = pos[0];
	ekf.x[1] = pos[1];
	ekf.x[2] = pos[2];
	ekf.x[3] = vel[0];
	ekf.x[4] = vel[1];
	ekf.x[5] = vel[2];
}

void ins_set_pos_vel_var(float posvar[3], float velvar[3])
{
	ekf.r[0] = posvar[0];
	ekf.r[1] = posvar[1];
	ekf.r[2] = posvar[2];
	ekf.r[3] = velvar[0];
	ekf.r[4] = velvar[1];
	ekf.r[5] = velvar[2];
}

void ins_state_prediction(float gyro_data[3], float accel_data[3], float dt)
{
	float u[6];
	float qmag;
	u[0] = gyro_data[0];
	u[1] = gyro_data[1];
	u[2] = gyro_data[2];

	u[3] = accel_data[0];
	u[4] = accel_data[1];
	u[5] = accel_data[2];
	linearize_fg(ekf.x, u, ekf.f, ekf.g);
	runge_kutta(ekf.x, u, dt);

	qmag = inv_sqrt(ekf.x[6] * ekf.x[6] + ekf.x[7] * ekf.x[7] + ekf.x[8] * ekf.x[8] + ekf.x[9] * ekf.x[9]);
	ekf.x[6] /= qmag;
	ekf.x[7] /= qmag;
	ekf.x[8] /= qmag;
	ekf.x[9] /= qmag;

	nav.pos[0] = ekf.x[0];
	nav.pos[1] = ekf.x[1];
	nav.pos[2] = ekf.x[2];
	nav.vel[0] = ekf.x[3];
	nav.vel[1] = ekf.x[4];
	nav.vel[2] = ekf.x[5];
	nav.q[0] = ekf.x[6];
	nav.q[1] = ekf.x[7];
	nav.q[2] = ekf.x[8];
	nav.q[3] = ekf.x[9];
	nav.gyro_bias[0] = ekf.x[10];
	nav.gyro_bias[1] = ekf.x[11];
	nav.gyro_bias[2] = ekf.x[12];
}

void ins_covariance_prediction(float dt)
{
	covariance_prediction(ekf.f, ekf.g, ekf.q, dt, ekf.p);
}

float zeros[3] = {0, 0, 0};

void mag_correction(float mag_data[3])
{
	ins_correction(mag_data, zeros, zeros, zeros[0], MAG_SENSORS);
}

void ins_correction(float mag_data[3], float pos[3], float vel[3],
						float baro_alt, uint16_t sensors_used)
{
	float z[10], y[10];
	float bmag, qmag;
	z[0] = pos[0];
	z[1] = pos[1];
	z[2] = pos[2];
	z[3] = vel[0];
	z[4] = vel[1];
	z[5] = vel[2];
	bmag = inv_sqrt(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
	z[6] = mag_data[0] * bmag;
	z[7] = mag_data[1] * bmag;
	z[8] = mag_data[2] * bmag;
	z[9] = baro_alt;
	linearize_h(ekf.x, ekf.be, ekf.h);
	measurement_eq(ekf.x, ekf.be, y);
	serial_update(ekf.h, ekf.r, z, y, ekf.p, ekf.x, sensors_used);
	qmag = inv_sqrt(ekf.x[6] * ekf.x[6] +ekf.x[7] * ekf.x[7] + ekf.x[8] * ekf.x[8]);
	ekf.x[6] *= qmag;
	ekf.x[7] *= qmag;
	ekf.x[8] *= qmag;
	ekf.x[9] *= qmag;

	nav.pos[0] = ekf.x[0];
	nav.pos[1] = ekf.x[1];
	nav.pos[2] = ekf.x[2];
	nav.vel[0] = ekf.x[3];
	nav.vel[1] = ekf.x[4];
	nav.vel[2] = ekf.x[5];
	nav.q[0] = ekf.x[6];
	nav.q[1] = ekf.x[7];
	nav.q[2] = ekf.x[8];
	nav.q[3] = ekf.x[9];
	nav.gyro_bias[0] = ekf.x[10];
	nav.gyro_bias[1] = ekf.x[11];
	nav.gyro_bias[2] = ekf.x[12];

}

void covariance_prediction(float f[NUMX][NUMX],float G[NUMX][NUMW],
							float q[NUMW], float dt, float p[NUMX][NUMX])
{


}

void serial_update(float h[NUMV][NUMX], float r[NUMV], float z[NUMV],
				float y[NUMV], float p[NUMX][NUMX], float x[NUMX],
				uint16_t sensors_used)
{
	float hp[NUMX], hphr,error;
	uint8_t i, j, k, m;
	float km[NUMX];
	for (m = 0; m < NUMV; m++)
	{
		if (sensors_used & (0x01 << m))
		{
			for (j = 0; j < NUMX; j++)
			{
				hp[j] = 0;
				for (k = hrow_min[m]; k <= hrow_max[m]; k++)
				{
					hp[j] +=
				}
			}
		}
	}
}
