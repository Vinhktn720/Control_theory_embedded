#include "pid_controller.h"

void PID_Init(PID_Controller  *pid, float kp, float ki, float kd,float Ts, float output_max){
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->Ts = Ts;
	pid->output_max = output_max;
	pid->last_error = 0.0f;
	pid->intergal = 0.0f;
}

float PID_Compute(PID_Controller *pid, float setpoint, float feedback){
	float error = setpoint - feedback;
	pid->intergal += (error)*pid->Ts;
	if(pid->intergal > pid->output_max/pid->Ki){
		pid->intergal = pid->output_max/pid->Ki;
	}
	else if(pid->intergal < -pid->output_max/pid->Ki){
		pid->intergal = -pid->output_max/pid->Ki;
	}
	float derivative = (error - pid->last_error)/(pid->Ts);

	pid->output = (pid->Kp * error) + (pid->Ki * pid->intergal) + (pid->Kd * derivative);

	if (pid->output > pid->output_max) {
		pid->output = pid->output_max;
	} else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
	}
	pid->last_error = error;
	return pid->output;
}
