#include "PID.h"

void PIDController_Init(PIDController *pid) {

	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	float error = setpoint - measurement;
	float proportional = pid->Kp * error;

	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
	if (pid->integrator > pid->limMaxInt) {
		pid->integrator = pid->limMaxInt;
	} else if (pid->integrator < pid->limMinInt) {
		pid->integrator = pid->limMinInt;
	}

	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / (2.0f * pid->tau + pid->T);

	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax) {
		pid->out = pid->limMax;
	} else if (pid->out < pid->limMin) {
		pid->out = pid->limMin;
	}

	pid->prevError       = error;
	pid->prevMeasurement = measurement;

	/* Return controller output */
	return pid->out;

}
