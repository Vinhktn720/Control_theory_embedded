#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float last_error;
	float intergal;
	float output;
	float output_max;
	float Ts;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float integral_max, float output_max);
float PID_Compute(PID_Controller *pid, float setpoint, float feedback);

#endif //PID_CONTROLLER_H
