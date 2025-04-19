#include "rls_estimator.h"
#include <math.h>

// Static buffers for matrix operations
static float P_data[16];
static float Theta_data[4];
static float phi_data[4];
static float temp_4[4];
static float L_data[4];
static float temp_4x4[16];

void RLS_Init(RLS_Estimator* estimator, float lamba, float* initial_Theta, float initial_P) {
    // Initialize parameter vector
	Theta_data[0] = initial_Theta[0];
	Theta_data[1] = initial_Theta[1];
	Theta_data[2] = initial_Theta[2];
	Theta_data[3] = initial_Theta[3];
    arm_mat_init_f32(&estimator->Theta_, NUM_PARAMS, 1, Theta_data);

    for(int i=0; i<16; i++) P_data[i] = 0.0f;
    for(int i=0; i<4; i++) P_data[i*5] = initial_P;
    arm_mat_init_f32(&estimator->P_, NUM_PARAMS, NUM_PARAMS, P_data);

    estimator->lamba = lamba;
    estimator->u_prev[0] = estimator->u_prev[1] = 0.0f;
    estimator->y_prev[0] = estimator->y_prev[1] = 0.0f;
}

void RLS_Update(RLS_Estimator* estimator, float u, float y) {
	//Update the perivous values of input and output
	estimator->u_prev[2] = estimator->u_prev[1];
	estimator->u_prev[1] = estimator->u_prev[0];
	estimator->u_prev[0] = u;
	estimator->y_prev[2] = estimator->y_prev[1];
	estimator->y_prev[1] = estimator->y_prev[0];
	estimator->y_prev[0] = y;
	//Create regression matrix 4x1
	phi_data[0] = -estimator->y_prev[1];
	phi_data[1] = -estimator->y_prev[2];
	phi_data[2] = estimator->u_prev[1];
	phi_data[3] = estimator->u_prev[2];
	arm_matrix_instance_f32 phi = {NUM_PARAMS, 1, phi_data};

	//1. Calculate innovation error

    float y_hat;
    arm_dot_prod_f32(estimator->Theta_.pData, phi_data, NUM_PARAMS, &y_hat);
    float e = y - y_hat;

    arm_matrix_instance_f32 P_phi = {NUM_PARAMS, 1, temp_4};
    arm_mat_mult_f32(&estimator->P_, &phi, &P_phi);

    float denominator;
    arm_dot_prod_f32(phi_data, P_phi.pData, NUM_PARAMS, &denominator);
    denominator += estimator->lamba;

    arm_scale_f32(P_phi.pData, 1.0f/denominator, L_data, NUM_PARAMS);

    // 3. Update parameter estimates
    arm_matrix_instance_f32 L = {NUM_PARAMS, 1, L_data};
    arm_matrix_instance_f32 Theta_predict = {NUM_PARAMS, 1, temp_4};
    arm_mat_mult_f32(&L, &(arm_matrix_instance_f32){1, 1, &e}, &Theta_predict);
    arm_mat_add_f32(&estimator->Theta_, &Theta_predict, &estimator->Theta_);

    // 4. Update matrix
    arm_matrix_instance_f32 phi_T = {1, NUM_PARAMS, temp_4};
    arm_mat_trans_f32(&phi, &phi_T);
    float temp[4];
    arm_matrix_instance_f32 phi_T_P = {1, NUM_PARAMS, temp};
    arm_mat_mult_f32(&phi_T, &estimator->P_, &phi_T_P);
    arm_matrix_instance_f32 L_phiT = {NUM_PARAMS, NUM_PARAMS, temp_4x4};
    arm_mat_mult_f32(&L, &phi_T_P, &L_phiT);
    arm_mat_sub_f32(&estimator->P_, &L_phiT, &L_phiT);
    arm_mat_scale_f32(&L_phiT, 1.0f/estimator->lamba, &estimator->P_);


}
