#include <stdio.h>
#include "kalman.h"
#include "matrix_kalman.h"
#include "matrix.h"

int main()
{
	kalman_filter f = alloc_filter(2, 1);
	// set_matrix(f.state_transition, 
	// 			1.0, 1.0,
	// 			0.0, 1.0);
	set_matrix(f.state_transition, 
			1.0, -1.0,
			0.0, 1.0);
	set_matrix(f.control_input_model, )
	set_matrix(f.observation_model, 1.0, 0.0);
	//m_ident(f.process_noise_covariance);
	set_matrix(f.process_noise_covariance, 
				0.001, 0, 
				0, 0.003);
	//m_ident(f.observation_noise_covariance);
	set_matrix(f.observation_noise_covariance, 0.003);

	float deviation = 1000.0;
	//set_matrix(f.state_estimate, 10 * deviation);
	set_matrix(f.state_estimate, 0, 0);
	//p(k|k)
	m_ident(f.estimate_covariance);
	sm_mlt(deviation * deviation, f.estimate_covariance, f.estimate_covariance);
#ifdef KALMAN_DEBUG
	fprintf(stdout, "state_transition is :\n");
	m_output(f.state_transition);
	fprintf(stdout, "observation_model is :\n");
	m_output(f.observation_model);
	fprintf(stdout, "process_noise_covariance is :\n");
	m_output(f.process_noise_covariance);
	fprintf(stdout, "observation_noise_covariance is :\n");
	m_output(f.observation_noise_covariance);
	fprintf(stdout, "observation is :\n");
	m_output(f.observation);
	fprintf(stdout, "predicted_state is :\n");
	m_output(f.predicted_state);
	fprintf(stdout, "predicted_estimate_covariance is :\n");
	m_output(f.predicted_estimate_covariance);
	fprintf(stdout, "innovation is :\n");
	m_output(f.innovation);
	fprintf(stdout, "innovation_covariance is :\n");
	m_output(f.innovation_covariance);
	fprintf(stdout, "inverse_innovation_covariance is :\n");
	m_output(f.inverse_innovation_covariance);
	fprintf(stdout, "optimal_gain is :\n");
	m_output(f.optimal_gain);
	fprintf(stdout, "state_estimate is :\n");
	m_output(f.state_estimate);
	fprintf(stdout, "estimate_covariance is :\n");
	m_output(f.estimate_covariance);
	//fprintf(stdout, "alloc_filter succeed!\n");
#endif 
	
	for (int i =0; i < 10; i++)
	{
		set_matrix(f.observation, (double) i);
		fprintf(stdout, "\n");
		fprintf(stdout, "observation is :\n");
		m_output(f.observation);	
		//m_output(f.observation);
		update(f);
		fprintf(stdout, "state_estimate is :\n");
		m_output(f.state_estimate);
	}

	// fprintf(stdout, "position is %f\n", f.state_estimate->me[0][0]);
	// fprintf(stdout, "velocity is %f\n", f.state_estimate->me[1][0]);
	fprintf(stdout, "position is %f\n", m_get_val(f.state_estimate, 0, 0));
	fprintf(stdout, "velocity is %f\n", m_get_val(f.state_estimate, 1, 0));
	free_filter(f);
}
