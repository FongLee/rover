#ifndef KALMAN_H
#define KALMAN_H
//#include <stdio.h>

#include "matrix.h"

typedef struct kalman
{
	int state_dimension;
	int measure_dimension;
	int input_dimension;
	//X(k)
	MAT * state_estimate;
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
} kalman_t;

/**
 * alloc a kalman_filter
 * @param  f                 [description]
 * @param  state_dimension   [description]
 * @param  measure_dimension [description]
 * @param  input_dimension   [description]
 * @return                   [description]
 */
kalman_t *alloc_kalman_filter(kalman_t **f, int state_dimension, int measure_dimension, int input_dimension);

/**
 * prediction in the kalman filter
 * @param  f [description]
 * @return   [description]
 */
int predict(kalman_t * f);

/**
 * prediction in the kalman filter, no update covariance
 * @param  f kalman handler
 * @return   0: success
 */
int predict_only(kalman_t *f);

/**
 * update in kalman filter
 * @param  f [description]
 * @return   [description]
 */
int update(kalman_t *f);

/**
 * free a kalman_filter
 * @param f [description]
 */
void free_kalman_filter(kalman_t **f);
#endif