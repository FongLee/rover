#include <stdio.h>

#include "kalman.h"
#include "matrix_kalman.h"
#include "matrix.h"

int main()
{
	kalman_t *f;
	alloc_kalman_filter(&f, 2, 1, 1);
	set_matrix(f->state_transition, 
				1.0, 1.0,
				0.0, 1.0);
	// set_matrix(f.state_transition, 
	// 		1.0, -1.0,
	// 		0.0, 1.0);
	set_matrix(f->control_input_model, 0.0, 0.0 );
	set_matrix(f->measure_model, 1.0, 0.0);
	m_ident(f->process_noise_covariance);
	// set_matrix(f.process_noise_covariance, 
	// 			0.001, 0, 
	// 			0, 0.003);
	m_ident(f->measure_noise_covariance);
	//set_matrix(f.measure_noise_covariance, 0.003);

	float deviation = 1000.0;
	set_matrix(f->state_estimate, 10 * deviation);
	//set_matrix(f.state_estimate, 0, 0);
	//p(k|k)
	m_ident(f->covariance_estimate);
	sm_mlt(deviation * deviation, f->covariance_estimate, f->covariance_estimate);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "state_transition is :\n");
	m_output(f->state_transition);
	fprintf(stdout, "measure_model is :\n");
	m_output(f->measure_model);
	fprintf(stdout, "process_noise_covariance is :\n");
	m_output(f->process_noise_covariance);
	fprintf(stdout, "measure_noise_covariance is :\n");
	m_output(f->measure_noise_covariance);
	fprintf(stdout, "measure is :\n");
	m_output(f->measure);
	fprintf(stdout, "state_predict is :\n");
	m_output(f->state_predict);
	fprintf(stdout, "covariance_predict is :\n");
	m_output(f->covariance_predict);
	fprintf(stdout, "innovation is :\n");
	m_output(f->innovation);
	fprintf(stdout, "innovation_covariance is :\n");
	m_output(f->innovation_covariance);
	fprintf(stdout, "inverse_innovation_covariance is :\n");
	m_output(f->inverse_innovation_covariance);
	fprintf(stdout, "k_gain is :\n");
	m_output(f->k_gain);
	fprintf(stdout, "state_estimate is :\n");
	m_output(f->state_estimate);
	fprintf(stdout, "covariance_estimate is :\n");
	m_output(f->covariance_estimate);
	//fprintf(stdout, "alloc_filter succeed!\n");
#endif 
	
	for (int i =0; i < 20; i++)
	{
		set_matrix(f->measure, (double) i);
		set_matrix(f->control_input, 0);
		fprintf(stdout, "\n");
		fprintf(stdout, "measure is :\n");
		m_output(f->measure);	
		//m_output(f->measure);
		predict(f);
		update(f);
		fprintf(stdout, "state_estimate is :\n");
		m_output(f->state_estimate);
	}

	// fprintf(stdout, "position is %f\n", f->state_estimate->me[0][0]);
	// fprintf(stdout, "velocity is %f\n", f->state_estimate->me[1][0]);
	fprintf(stdout, "position is %f\n", m_get_val(f->state_estimate, 0, 0));
	fprintf(stdout, "velocity is %f\n", m_get_val(f->state_estimate, 1, 0));
	free_kalman_filter(&f);
}
