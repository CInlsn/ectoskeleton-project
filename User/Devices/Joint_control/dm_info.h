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

#define ROD_LINIT 0
#define ROD_RINIT	0
#define ROD_LMAX 0.87f
#define ROD_RMAX 0.5f
#define ROD_LMIN -0.5f
#define ROD_RMIN -0.87f
#define DF_LIMIT 0.15f

#define KNEE_INIT 0
#define KNEE_MAX 0.5f
#define KNEE_MIN -0.5f
#endif

