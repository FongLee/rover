#include <malloc.h>

#include "kalman.h"
#include "matrix2.h" //m_inverse
#include "matrix_kalman.h"

#ifdef MEMWATCH
#include "memwatch.h"
#endif

/**
 * alloc a kalman_filter
 * @param  f                  kalmanm handle
 * @param  state_dimension   n dimension state
 * @param  measure_dimension m dimension measure data
 * @param  input_dimension   
 * @return                   >0: a pointer to kalman handle; NULL: err
 */
kalman_t * alloc_kalman_filter(kalman_t **f, int state_dimension, int measure_dimension, int input_dimension)
{
	*f = (kalman_t *)malloc(sizeof(kalman_t));
	if (*f == NULL)
	{
		return NULL;
	}

	(*f)->state_dimension = state_dimension;
	(*f)->measure_dimension = measure_dimension;
	(*f)->input_dimension = input_dimension;
	//x(k)
	(*f)->state_estimate = m_get(state_dimension, 1);
	//A
	(*f)->state_transition = m_get(state_dimension, state_dimension);
	//B
	(*f)->control_input_model = m_get(state_dimension, input_dimension);
	//u(k-1)
	(*f)->control_input = m_get(input_dimension, 1);
	//Q
	(*f)->process_noise_covariance = m_get(state_dimension, state_dimension);
	//H(k)
	(*f)->measure_model = m_get(measure_dimension, state_dimension);
	//R
	(*f)->measure_noise_covariance = m_get(measure_dimension, measure_dimension);
	//P(k)
	(*f)->covariance_estimate = m_get(state_dimension, state_dimension);
	//x(k|k-1)
	(*f)->state_predict = m_get(state_dimension, 1);
	//P(k|k-1)
	(*f)->covariance_predict = m_get(state_dimension, state_dimension);
	//z(k)
	(*f)->measure = m_get(measure_dimension, 1);
	//y(k)
	(*f)->innovation = m_get(measure_dimension, 1);
	//S(k)
	(*f)->innovation_covariance = m_get(measure_dimension, measure_dimension);
	//S(k)^-1
	(*f)->inverse_innovation_covariance = m_get(measure_dimension, measure_dimension);
	//K(k)
	(*f)->k_gain = m_get(state_dimension, measure_dimension);
	//state_dimension * 1
	(*f)->tmp1_state_dimension = m_get(state_dimension, 1);
	(*f)->tmp2_state_dimension = m_get(state_dimension, 1);
	//measure_dimension * 1
	(*f)->tmp_measure_dimension = m_get(measure_dimension, 1);
	//measure_dimension * state_dimension
	(*f)->tmp_measure_state = m_get(measure_dimension, state_dimension);
	//state_dimension * state_dimension
	(*f)->tmp1_state_state = m_get(state_dimension, state_dimension);
	(*f)->tmp2_state_state = m_get(state_dimension, state_dimension);
	
	return *f;

}

/**
 * free a kalman_filter
 * @param f [description]
 */
void free_kalman_filter(kalman_t **f)
{
	m_free((*f)->state_estimate);
	m_free((*f)->state_transition);
	m_free((*f)->control_input_model);
	m_free((*f)->control_input);
	m_free((*f)->process_noise_covariance);
	m_free((*f)->measure);
	m_free((*f)->measure_model);
	m_free((*f)->measure_noise_covariance);
	m_free((*f)->covariance_estimate);
	m_free((*f)->state_predict);
	m_free((*f)->covariance_predict);
	m_free((*f)->innovation);
	m_free((*f)->innovation_covariance);
	m_free((*f)->inverse_innovation_covariance);
	m_free((*f)->k_gain);
	m_free((*f)->tmp1_state_dimension);
	m_free((*f)->tmp2_state_dimension);
	m_free((*f)->tmp_measure_dimension);
	m_free((*f)->tmp_measure_state);
	m_free((*f)->tmp1_state_state);
	m_free((*f)->tmp2_state_state);
	
	free(*f);
}

/**
 * prediction in the kalman filter
 * @param  f kalman handler
 * @return   0: success
 */
int predict(kalman_t *f)
{
	//x(k|k-1) = A * x(k-1) + B * u(k-1)
	
	//tmp1 = A * x(k-1)
	m_mlt(f->state_transition, f->state_estimate, f->tmp1_state_dimension);
	//tmp2 = B * u(k-1)
	m_mlt(f->control_input_model, f->control_input, f->tmp2_state_dimension);
	//x(k|k-1) = tmp1 + tmp2
	m_add(f->tmp1_state_dimension, f->tmp2_state_dimension, f->state_predict);
	
	//P(k|k-1) = A * P(k-1) * A^T + Q
	//tmp1 = A * P(k-1)
	m_mlt(f->state_transition, f->covariance_estimate, f->tmp1_state_state);
	//tmp2 = tmp1 * A^T
	mmtr_mlt(f->tmp1_state_state, f->state_transition, f->tmp2_state_state);
	//P(k|k-1) = tmp2 + Q
	m_add(f->tmp2_state_state, f->process_noise_covariance, f->covariance_predict); 
	return 0;
}



/**
 * prediction in the kalman filter, no update covariance
 * @param  f kalman handler
 * @return   0: success
 */
int predict_only(kalman_t *f)
{
	//x(k|k-1) = A * x(k-1) + B * u(k-1)
	
	//tmp1 = A * x(k-1)
	m_mlt(f->state_transition, f->state_estimate, f->tmp1_state_dimension);
	//tmp2 = B * u(k-1)
	m_mlt(f->control_input_model, f->control_input, f->tmp2_state_dimension);
	//x(k|k-1) = tmp1 + tmp2
	m_add(f->tmp1_state_dimension, f->tmp2_state_dimension, f->state_predict);
	
	return 0;
}

/**
 * update in kalman filter
 * @param  f [description]
 * @return   [description]
 */
int update(kalman_t *f)
{
	//y(k) = z(k) - H * x(k|k-1)
	
	//tmp = H * x(k|k-1)
	m_mlt(f->measure_model, f->state_predict, f->tmp_measure_dimension);
	//y(k) = z(k) - tmp
	m_sub(f->measure, f->tmp_measure_dimension, f->innovation);

	//S(k) = H * P(k|k-1) * H^T+ R
	//tmp = H * P(k|k-1)
	m_mlt(f->measure_model, f->covariance_predict, f->tmp_measure_state);
	//S(k) = tmp * H^T 
	mmtr_mlt(f->tmp_measure_state, f->measure_model, f->innovation_covariance);
	//S(k) = S(k) + R
	m_add(f->innovation_covariance, f->measure_noise_covariance, f->innovation_covariance);
	//S(k)^-1
	m_inverse(f->innovation_covariance, f->inverse_innovation_covariance);

	//K(k) = P(k|k-1) * H^T * S(k)^-1
	//tmp = P(k|k-1) * H^T
	mmtr_mlt(f->covariance_predict, f->measure_model, f->tmp1_state_state);
	//K(k) = tmp * S(k)^-1
	m_mlt(f->tmp1_state_state, f->inverse_innovation_covariance,f->k_gain);

	//x(k) = x(k|k-1) + K(k) * y(k)
	//tmp = k(k) * y(k)
	m_mlt(f->k_gain, f->innovation, f->tmp1_state_dimension);
	//x(k) = x(k|k-1) + tmp
	m_add(f->state_predict, f->tmp1_state_dimension, f->state_estimate);

	//P(k) = P(k|k-1) - K(k) * H * P(k|k-1)
	//tmp = K(k) * H
	m_mlt(f->k_gain, f->measure_model, f->tmp1_state_state);
	//P(k) = tmp * P(k|k-1)
	m_mlt(f->tmp1_state_state, f->covariance_predict, f->covariance_estimate);
	//P(k) = P(k|k-1) - P(k)
	m_sub(f->covariance_predict, f->covariance_estimate, f->covariance_estimate);

	return 0;
}
