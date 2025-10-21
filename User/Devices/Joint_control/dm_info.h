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

#define ROD_LINIT -0.4f
#define ROD_RINIT	4.64f
#define ROD_LMAX 1.5f
#define ROD_RMAX 5.05f
#define ROD_LMIN -0.8f
#define ROD_RMIN 2.72f
#define DF_LIMIT 0.346f
#endif

