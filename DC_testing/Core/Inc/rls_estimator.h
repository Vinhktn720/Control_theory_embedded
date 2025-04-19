#ifndef RLS_ESTIMATOR_H
#define RLS_ESTIMATOR_H
#include "arm_math.h"
#define NUM_PARAMS 4

typedef struct{
	arm_matrix_instance_f32 Theta_;
	arm_matrix_instance_f32 P_;
	float lamba;
	float u_prev[3];
	float y_prev[3];
} RLS_Estimator;

void RLS_Init(RLS_Estimator* estimator, float lamba, float* initial_Theta, float initial_P);
void RLS_Update(RLS_Estimator* estimator, float u, float y);

#endif
