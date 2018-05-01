#include <math.h>

//#include <stdio.h>

#include "ap_math.h"
#include "ap_navigation.h"
#include "matrix_kalman.h"

#define W_IE 			(7.29212e-5)
#define R_EARTH 		6371000
#define G_ACC   		9.78
#define DELT_T 			1e-3
#define Q_POSITION 		0.3
#define Q_VEL 			0.1
#define R_POSITION 		1e-8
#define R_VEL 			3e-6

MAT * update_c_bn(MAT *c_bn, VEC *qua);
VEC *calculate_f_n(VEC *f_b, MAT *c_bn, VEC *f_n);
VEC *update_acceleration(nav_data_t *nav,  double t);
VEC *update_location(nav_data_t *nav, double t);
void copy_mpu_data(nav_data_t *nav, mpudata_t *mpu);

nav_data_t nav_data;

/**
 * navigation init
 * @param  nav data structure
 * @return     0: success;
 */
int navigation_init(nav_data_t * nav)
{
	nav->velocity_nav = v_get(3);
	nav->last_velocity = v_get(3);
	nav->last_acce = v_get(3);
	nav->w_nie = v_get(3);
	nav->w_nen = v_get(3);
	nav->w_nin = v_get(3);

	nav->c_nb = m_get(3, 3);
	nav->last_c_bn = m_get(3, 3);
	nav->w_n_skew = m_get(3, 3);

	nav->last_quaternion  = v_get(4);

	nav->last_f_b = v_get(3);
	nav->last_f_n = v_get(3);

	nav->flag_nav_failure = false;


	nav->location = v_get(3);
	nav->origin = v_get(3);

	alloc_kalman_filter(&(nav->kalman_navigation), 4, 4, 4);
	//initialize A matrix
	set_matrix(nav->kalman_navigation->state_transition,
				1.0, 	0.0, 	DELT_T/R_EARTH, 	0.0,
				0.0, 	1.0, 	0.0, 		DELT_T/R_EARTH,
				0.0, 	0.0,	1.0,		0.0,
				0.0,	0.0, 	0.0, 		1.0);
	//initialize B matrix
	set_matrix(nav->kalman_navigation->control_input_model,
				0, 0, 		0, 		0,
				0, DELT_T, 	0, 		0,
				0, 0,		DELT_T, 0,
				0, 0, 		0, 		DELT_T);
	//initialize H matrix
	set_matrix(nav->kalman_navigation->measure_model,
				1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0);
	//initialize Q matrix
	set_matrix(nav->kalman_navigation->process_noise_covariance,
				Q_POSITION, 0.0, 		0.0, 	0.0,
				0.0, 		Q_POSITION, 0.0, 	0.0,
				0.0, 		0.0, 		Q_VEL, 	0.0,
				0.0, 		0.0, 		0.0, 	Q_VEL);
	sm_mlt(DELT_T, nav->kalman_navigation->process_noise_covariance, nav->kalman_navigation->process_noise_covariance);

	//initialize R matrix
	set_matrix(nav->kalman_navigation->measure_noise_covariance,
				R_POSITION, 	0.0, 		0.0, 	0.0,
 				0.0, 			R_POSITION, 0.0, 	0.0,
 				0.0, 			0.0, 		R_VEL, 	0.0,
 				0.0, 			0.0, 		0.0, 	R_VEL);

	//initialize X
	set_matrix(nav->kalman_navigation->state_estimate,
				0.0, 0.0, 0.0, 0.0);

	float deviation = 1000.0;
	m_ident(nav->kalman_navigation->covariance_estimate);
	sm_mlt(deviation * deviation, nav->kalman_navigation->covariance_estimate, nav->kalman_navigation->covariance_estimate);

	nav->flag_get_home = false;
	nav->home_position.lat = 0;
	nav->home_position.lon = 0;
	nav->flag_use_mpu_init = false;
	nav->last_use_mpu_nav_time = 0;

	//pthead_mutex_init(&(nav->mutex));
	return 0;

}

/**
 * navitation close
 * @param  nav structure nav
 * @return     0: success
 */
int navigation_close(nav_data_t * nav)
{

	v_free(nav->velocity_nav);
	v_free(nav->last_velocity);
	v_free(nav->last_acce);
	v_free(nav->w_nie);
	v_free(nav->w_nen);
	v_free(nav->w_nin);

	m_free(nav->c_nb);
	m_free(nav->last_c_bn);
	m_free(nav->w_n_skew);

	v_free(nav->last_quaternion);
	v_free(nav->last_f_b);
	v_free(nav->last_f_n);
	v_free(nav->location);
	v_free(nav->origin);

	free_kalman_filter(&(nav->kalman_navigation));
	nav->flag_get_home = false;

	return 0;

}

/**
 * copy mpu structure to nav structure
 * @param nav structure nav
 * @param mpu structure mpu
 */
void copy_mpu_data(nav_data_t *nav, mpudata_t *mpu)
{

	nav->last_use_mpu_nav_time = mpu->magTimestamp;
	nav->last_quaternion->ve[0] = mpu->fusedQuat[0];
	nav->last_quaternion->ve[1] = mpu->fusedQuat[1];
	nav->last_quaternion->ve[2] = mpu->fusedQuat[2];
	nav->last_quaternion->ve[3] = mpu->fusedQuat[3];

	nav->last_f_b->ve[0] = mpu->calibratedAccel[0];
	nav->last_f_b->ve[1] = mpu->calibratedAccel[1];
	nav->last_f_b->ve[2] = mpu->calibratedAccel[2];

}

/**
 * navigation handler
 * @param  nav  structure nav
 * @param  mpu  structure mpu
 * @param  gps  structure gps
 * @param  flag USE_MPU or USE_GPS
 * @return      0: success
 */
int inertial_navigation(nav_data_t * nav, mpudata_t * mpu, gps_data_t  *gps, int flag)
{

	double delt_t = 0;
	double px = 0;
	double py = 0;

	//v_output(nav->location);

	if(flag == USE_MPU)
	{
		pthread_mutex_lock(&(nav->mutex));

		if(nav->flag_use_mpu_init == false)
		{
			nav->flag_use_mpu_init = true;

			copy_mpu_data(nav, mpu);
		//	phread_mutex_unlock(&(nav->mutex));

			return 0;
		}

		update_c_bn(nav->last_c_bn, nav->last_quaternion);
		if(nav->flag_get_home == false)
		{
			copy_mpu_data(nav, mpu);
		//	phread_mutex_unlock(&(nav->mutex));
			return 0;
		}
		delt_t = mpu->magTimestamp - nav->last_use_mpu_nav_time;
		delt_t *= 1e-3;

		//v_output(nav->quaternion);
		//m_output(nav->c_bn);

		calculate_f_n(nav->last_f_b, nav->last_c_bn, nav->last_f_n);
		update_acceleration(nav, delt_t);

		copy_mpu_data(nav, mpu);
		px = nav->kalman_navigation->state_estimate->me[0][0];
		py = nav->kalman_navigation->state_estimate->me[1][0];

		
		//initialize Q matrix
		set_matrix(nav->kalman_navigation->process_noise_covariance,
					Q_POSITION, 0.0,		0.0,	0.0,
					0.0,		Q_POSITION, 0.0,	0.0,
					0.0,		0.0,		Q_VEL,	0.0,
					0.0,		0.0,		0.0,	Q_VEL);
		sm_mlt(delt_t, nav->kalman_navigation->process_noise_covariance, nav->kalman_navigation->process_noise_covariance);
		set_matrix(nav->kalman_navigation->state_transition,
					1.0, 	0.0, 	delt_t/R_EARTH, 	0.0,
					0.0, 	1.0, 	0.0, 		delt_t/R_EARTH / cos(px),
					0.0, 	0.0,	1.0,		0.0,
					0.0,	0.0, 	0.0, 		1.0);
		set_matrix(nav->kalman_navigation->control_input_model,
					0, 0, 		0, 		0,
					0, delt_t, 	0, 		0,
					0, 0,		delt_t, 0,
					0, 0, 		0, 		delt_t);
		set_matrix(nav->kalman_navigation->control_input,
					0, -nav->last_velocity->ve[1] * px / cos(px) * tan(px) / R_EARTH, nav->last_acce->ve[0], nav->last_acce->ve[1]);
		

		predict(nav->kalman_navigation);
		nav->last_velocity->ve[0] = nav->kalman_navigation->state_predict->me[2][0];
		nav->last_velocity->ve[1] = nav->kalman_navigation->state_predict->me[3][0];
		nav->kalman_navigation->state_estimate->me[0][0] = nav->kalman_navigation->state_predict->me[0][0];
		nav->kalman_navigation->state_estimate->me[1][0] = nav->kalman_navigation->state_predict->me[1][0];
		nav->kalman_navigation->state_estimate->me[2][0] = nav->kalman_navigation->state_predict->me[2][0];
		nav->kalman_navigation->state_estimate->me[3][0] = nav->kalman_navigation->state_predict->me[3][0];

		//phread_mutex_unlock(&(nav->mutex));
		//m_output(nav->kalman_navigation->state_estimate);
		
	}
	else if(flag == USE_GPS)
	{
		pthread_mutex_lock(&(nav->mutex));
		mv_mlt(nav->last_c_bn, gps->velocity_body, nav->velocity_nav);


		if(nav->flag_get_home == false)
		{
			memcpy(&(nav->home_position), &(gps_data.position), sizeof(nmeaPOS));
			nav->kalman_navigation->state_estimate->me[0][0] = nav->home_position.lat;
			nav->kalman_navigation->state_estimate->me[1][0] = nav->home_position.lon;
			nav->kalman_navigation->state_estimate->me[2][0] = nav->velocity_nav->ve[0];
			nav->kalman_navigation->state_estimate->me[3][0] = nav->velocity_nav->ve[1];
			memcpy(nav->last_velocity, nav->velocity_nav, sizeof(VEC));
			nav->flag_get_home = true; 
		}
		else
		{
			nav->kalman_navigation->measure->me[0][0] = gps->position.lat;
			nav->kalman_navigation->measure->me[1][0] = gps->position.lon;
			nav->kalman_navigation->measure->me[2][0] = nav->velocity_nav->ve[0];
			nav->kalman_navigation->measure->me[3][0] = nav->velocity_nav->ve[1];
			update(nav->kalman_navigation);
			nav->last_velocity->ve[0] = nav->kalman_navigation->state_estimate->me[2][0];
			nav->last_velocity->ve[1] = nav->kalman_navigation->state_estimate->me[3][0];
			nav->speed = sqrt(nav->last_velocity->ve[0] * nav->last_velocity->ve[0]
								+ nav->last_velocity->ve[1] * nav->last_velocity->ve[1]);
			fprintf(stdout, "speed is %f\n", nav->speed);
			m_output(nav->kalman_navigation->state_estimate);
			
		}
		//phread_mutex_unlock(&(nav->mutex));

	}

	return 0;

}


/**
 * calculate the angular velocity of earth
 * @param  w_nie    w_nie
 * @param  latitude latitude in RAD
 * @return          [description]
 */
VEC * calculate_w_nie(VEC *w_nie, float latitude)
{
	w_nie->ve[0] = 0;
	//latitude_gps is the latitude of position
	w_nie->ve[1] = W_IE * cos(latitude);
	w_nie->ve[2] = W_IE * sin(latitude);
	return w_nie;
}

/**
 * calculate angular velocity of location
 * @param  w_nen    w_nen
 * @param  v_n      v_n
 * @param  latitude latitude in RAD
 * @return          w_nen
 */
VEC * calculate_w_nen(VEC * w_nen, VEC *v_n, float latitude)
{
	if (w_nen == VNULL || v_n == VNULL)
	{
		return NULL;
	}
	if (w_nen->dim != 3 || v_n->dim != 3)
	{
		return NULL;
	}

	w_nen->ve[0] = -v_n->ve[1] / R_EARTH;
	w_nen->ve[1] = v_n->ve[0] / R_EARTH;
	w_nen->ve[2] = -v_n->ve[0] * tan(latitude) / R_EARTH;
	return w_nen;
}


/**
 * calculate the w_nin = w_nie + w_nen
 * @param  w_nie w_nie
 * @param  w_nen w_nen
 * @param  w_nin w_nin
 * @return       ret(VEC)
 */
VEC *calculate_w_nin(VEC *w_nie, VEC *w_nen, VEC *w_nin)
{
	return v_add(w_nie, w_nen, w_nin);
}


/**
 * calculate the attitude matrix
 * @param  c_nb c_nb
 * @param  c_bn c_bn
 * @return      ret(MAT)
 */
MAT * calculate_c_nb(MAT * c_nb, MAT *c_bn)
{
	if (c_nb == MNULL || c_bn == MNULL)
	{
		return NULL;
	}
	return m_transp(c_nb, c_bn);
}


/**
 * caculate angular velocity of attitude, w_bnb = w_bib - c_nb * w_nin
 * @param  w_bib w_bib
 * @param  w_nin w_nin
 * @param  c_nb  c_nb
 * @param  w_bnb w_bnb
 * @return       ret(VEC)
 */
VEC * calculate_w_bnb(VEC *w_bib, VEC * w_nin, MAT *c_nb, VEC *w_bnb)
{
	return mv_mltadd(w_bib, w_nin, c_nb, -1, w_bnb);
}


/**
 * calculate skew-symmetric matrix of w_bnb
 * @param  w_bnb      w_bnb
 * @param  w_bnb_skew w_bnb_skew
 * @return            ret(MAT)
 */
MAT * calculate_w_bnb_skew(VEC *w_bnb, MAT *w_bnb_skew)
{
	w_bnb_skew->me[1][0] = w_bnb->ve[0];
	w_bnb_skew->me[2][0] = w_bnb->ve[1];
	w_bnb_skew->me[3][0] = w_bnb->ve[2];
	w_bnb_skew->me[2][1] = -w_bnb->ve[2];
	w_bnb_skew->me[3][1] = w_bnb->ve[1];
	w_bnb_skew->me[3][2] = -w_bnb->ve[0];

	w_bnb_skew->me[0][1] = -w_bnb->ve[0];
	w_bnb_skew->me[0][2] = -w_bnb->ve[1];
	w_bnb_skew->me[0][3] = -w_bnb->ve[2];
	w_bnb_skew->me[1][2] = w_bnb->ve[2];
	w_bnb_skew->me[1][3] = -w_bnb->ve[1];
	w_bnb_skew->me[2][3] = w_bnb->ve[0];

	return w_bnb_skew;
}


/**
 * 4th order Runge--Kutta method
 */
double rk4(VEC *f(double t, VEC *x, MAT *m, VEC *out), double t, VEC *x, MAT *m, double h)
{
	static VEC *v1 = VNULL, *v2 = VNULL, *v3 = VNULL, *v4 = VNULL;
	static VEC *temp = VNULL;

	if (x == VNULL)
	{
		error(E_NULL, "rk4");
	}

	v1 = v_resize(v1, x->dim);
	v2 = v_resize(v2, x->dim);
	v3 = v_resize(v3, x->dim);
	v4 = v_resize(v4, x->dim);
	temp = v_resize(temp, x->dim);

	MEM_STAT_REG(v1, TYPE_VEC);
	MEM_STAT_REG(v2, TYPE_VEC);
	MEM_STAT_REG(v3, TYPE_VEC);
	MEM_STAT_REG(v4, TYPE_VEC);
	MEM_STAT_REG(temp, TYPE_VEC);

	(*f)(t, x, m, v1);
	v_mltadd(x, v1, 0.5*h, temp);
	(*f)(t+0.5 * h, temp, m, v2);
	v_mltadd(x, v2, 0.5*h, temp);
	(*f)(t+0.5 * h, temp, m, v3);
	v_mltadd(x, v3, h, temp);
	(*f)(t + h, temp, m, v4);

	v_copy(v1, temp);
	v_mltadd(temp, v2, 2.0, temp);
	v_mltadd(temp, v3, 2.0, temp);
	v_add(temp, v4, temp);

	v_mltadd(x, temp, h / 6.0, x);
	return t + h;
}

/**
 *
 * @param
 * @param
 * @param
 * @param
 * @return
 */
/**
 * differential equation
 * @param  t   delt_t
 * @param  x   x
 * @param  m   m
 * @param  out out
 * @return     ret(VEC)
 */
VEC *f(double t, VEC *x, MAT *m, VEC *out)
{
	if (x == VNULL || out == VNULL)
	{
		error(E_NULL, "f");
	}
	if (x->dim != 4 || out->dim != 4 )
	{
		error(E_SIZES, "f");
	}
	//static MAT * temp_w_bnb;
	//0.5 * w_bnb
	//sm_mlt(0.5, w_bnb_skew, temp_w_bnb);


	return mv_mlt(m, x, out);//m * x
}


/**
 * update quaternion
 * @param  qua        qua
 * @param  w_bnb_skew w_bnb_skew
 * @param  t_imu      t_imu
 * @param  dt_imu     dt_imu
 * @return            ret(double)
 */
double update_quaternion(VEC *qua, MAT * w_bnb_skew, double t_imu, double dt_imu)
{
	if (qua == VNULL || w_bnb_skew == MNULL )
	{
		error(E_NULL, "update_q");
	}
	static MAT * m_que;
	//w_bnb_skew * 0.5
	sm_mlt(0.5, w_bnb_skew, m_que);
	return rk4(f, t_imu, qua, m_que, dt_imu);
}


/**
 * update c_bn according to quaternion
 * @param  c_bn c_bn
 * @param  qua  qua
 * @return      ret(MAT)
 */
MAT * update_c_bn(MAT *c_bn, VEC *qua)
{

	if (c_bn == MNULL || qua == VNULL)
	{
		return NULL;
	}
	if (c_bn->m != 3 || c_bn->n != 3 || qua->dim != 4)
	{
		return NULL;
	}

	static float q0, q1, q2, q3;
	q0 = qua->ve[0];
	q1 = qua->ve[1];
	q2 = qua->ve[2];
	q3 = qua->ve[3];
	c_bn->me[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	c_bn->me[0][1] = 2 * (q1 * q2 - q0 * q3);
	c_bn->me[0][2] = 2 * (q1 * q3 + q0 * q2);
	c_bn->me[1][0] = 2 * (q1 * q2 + q0 * q3);
	c_bn->me[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 *q3;
	c_bn->me[1][2] = 2 * (q2 * q3 - q0 * q1);
	c_bn->me[2][0] = 2 * (q1 * q3 - q0 * q2);
	c_bn->me[2][1] = 2 * (q2 * q3 + q1 * q0);
	c_bn->me[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	return c_bn;
}


/**
 * calculate projection specific force int navigation system
 * f_n = f_b * c_bn
 * @param  f_b  f_b
 * @param  c_bn c_bn
 * @param  f_n  f_n
 * @return      ret(VEC)
 */
VEC *calculate_f_n(VEC *f_b, MAT *c_bn, VEC *f_n)
{
	if (f_b == VNULL || c_bn == MNULL)
	{
		return NULL;
	}

	//f_n = c_bn * f_b
	return mv_mlt(c_bn, f_b, f_n);
}


/**
 * calculate skew-symmetric matrix
 * @param  w_nie    w_nie
 * @param  w_nen    w_nen
 * @param  w_n_skew w_n_skew
 * @return          w_n_skew
 */
MAT * calculate_w_ie_skew(VEC *w_nie, VEC *w_nen, MAT *w_n_skew)
{
	if (w_nie == VNULL || w_n_skew == MNULL)
	{
		return NULL;
	}
	if (w_n_skew->m != 3 || w_n_skew->n != 3)
	{
		return NULL;
	}

	static float w_ex, w_ey, w_ez;
	static float w_nx, w_ny, w_nz;
	w_ex = w_nie->ve[0];
	w_ey = w_nie->ve[1];
	w_ez = w_nie->ve[2];
	w_nx = w_nen->ve[0];
	w_ny = w_nen->ve[1];
	w_nz = w_nen->ve[2];

	w_n_skew->me[1][0] = 2*w_ez + w_nz;
	w_n_skew->me[2][0] = -(2*w_ey + w_ny);
	w_n_skew->me[2][1] = 2*w_ex + w_nx;
	w_n_skew->me[0][1] = -(2*w_ez + w_nz);
	w_n_skew->me[0][2] = 2*w_ey + w_ny;
	w_n_skew->me[1][2] = -(2*w_ex + w_nx);
	return w_n_skew;
}


/**
 * update velocity
 * @param  nav structure nav
 * @param  t   delt_t
 * @return     nav->last_acce
 */
VEC *update_acceleration(nav_data_t *nav,  double t)
{

	nav->last_f_n->ve[2] = nav->last_f_n->ve[2] - G_ACC;

	/**
	 * first order Euler mathod
	 */
	//last_acce = last_f_n - w_n_skew * vel
	mv_mltadd(nav->last_f_n, nav->last_velocity, nav->w_n_skew, -1.0, nav->last_acce);

	return nav->last_acce;
}

/**
 * update_location
 * @param  nav structure nav
 * @param  t   delt_t
 * @return     nav->location(VEC)
 */
VEC *update_location(nav_data_t *nav, double t)
{
	if(nav == NULL || t < 0)
	{
		return NULL;
	}
	//location = location + t * velocity
	v_mltadd(nav->location, nav->last_velocity, t, nav->location);

	return nav->location;

}


