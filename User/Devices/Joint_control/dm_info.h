#ifndef DM_INFO_H
#define DM_INFO_H

typedef struct{
	float Kp;
	float Kd;
	float Tq;
}con_parameter_t;

typedef struct {
	float pos;
	float vel;
}motor_info_t;

typedef struct {
	con_parameter_t con_parameter;
	motor_info_t motor_info;
}dm_motor_info_t;

#endif

