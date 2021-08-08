#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	float Kp;
	float Ki;
	float Kd;
	float tau;
	float limMin;
	float limMax;
	float limMinInt;
	float limMaxInt;
	float T;

	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;

	float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
