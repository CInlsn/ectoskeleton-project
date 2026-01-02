#include "pid.h"
#include <math.h>

static inline void abs_limit(float *a, float max) {
  if (*a > max)
    *a = max;
  else if (*a < -max)
    *a = -max;
}

float PID_calc(PID* pid, float measure, float target) {
  float err = target - measure;
	float delta_err = err - pid->last_err;
	
	if (err < pid->integral_threshold && err > -pid->integral_threshold) {
    pid->integral += pid->ki * err;
    abs_limit(&(pid->integral), pid->integral_limit);
		} 
	else {
    pid->integral = 0;
		}
	
  float out = pid->kp * err \
            + pid->integral \
            + pid->kd * (pid->filter_k*delta_err+(1-pid->filter_k)*pid->last_delta_err);
  abs_limit(&out, pid->max_out);
	
  pid->last_err = err;
	pid->last_delta_err = delta_err;

	 if ((pid->output_deadband != 0) && (fabs(err) < pid->output_deadband)){
		pid->out = 0;
		return 0;
	}
  else{
		pid->out  = out;
		return out;
	}
}


float PID_dry_calc(PID* pid, float measure, float target) {
  float err = target - measure;
	float integral = pid->integral;
  if (err < pid->integral_threshold && err > -pid->integral_threshold) {
   integral += pid->ki * err;
    abs_limit(&(integral), pid->integral_limit);
  } else {
    integral = 0;
  }
  float out = pid->kp * err \
            + integral \
            + pid->kd * (err - pid->last_err);
  abs_limit(&out, pid->max_out);
  
  if ((pid->output_deadband != 0) && (fabs(out) < pid->output_deadband))
    return 0;
  else
    return out;
}
