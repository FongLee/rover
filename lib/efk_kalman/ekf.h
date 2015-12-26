#ifndef EKF_H
#define EKF_H
//#include <stdio.h>

#include "matrix.h"

typedef struct ekfilter
{
	int state_dimension;
	int measure_dimension;
	int input_dimension;
	//X(k)
	MAT * state_estimate;
	//F(k)
	MAT * state_transition_f;
	//A
	MAT * state_transition;
	//B
	MAT * control_input_model;
	//u(k-1)
	MAT * control_input;
	//Q
	MAT * process_noise_covariance;
	//z(k)
	MAT * measure;
	//H(k)
	MAT * measure_model;
	//R
	MAT * measure_noise_covariance;
	//P(k)
	MAT * covariance_estimate;
	//x(k|k-1)
	MAT * state_predict;
	//P(k|k-1)
	MAT * covariance_predict;
	//y(k)
	MAT * innovation;
	//S(K)
	MAT * innovation_covariance;
	//S(k)^-1
	MAT * inverse_innovation_covariance;
	//K(k)
	MAT * k_gain;
	//state_dimension * 1
	MAT * tmp1_state_dimension;
	MAT * tmp2_state_dimension;
	//measure_dimension * 1
	MAT * tmp_measure_dimension;
	//measure_dimension * state_dimension
	MAT * tmp_measure_state;
	//state_dimension * state_dimension
	MAT * tmp1_state_state;
	MAT * tmp2_state_state;
} ekf_t;

/**
 * alloc a kalman_filter
 * @param  f                 [description]
 * @param  state_dimension   [description]
 * @param  measure_dimension [description]
 * @param  input_dimension   [description]
 * @return                   [description]
 */
ekf_t *alloc_ekf_filter(ekf_t **f, int state_dimension, int measure_dimension, int input_dimension);

/**
 * prediction in the kalman filter
 * @param  f [description]
 * @return   [description]
 */
int predict_ekf(ekf_t * f);

/**
 * prediction in the kalman filter, no update covariance
 * @param  f kalman handler
 * @return   0: success
 */
int predict_only_ekf(ekf_t *f);

/**
 * update in kalman filter
 * @param  f [description]
 * @return   [description]
 */
int update_ekf(ekf_t *f);

/**
 * free a kalman_filter
 * @param f [description]
 */
void free_ekf_filter(ekf_t **f);
#endif
