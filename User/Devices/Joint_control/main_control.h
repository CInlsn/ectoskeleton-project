#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

#include "dm_drv.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "dm_info.h"
#include "controller.h"

typedef enum {
	STOP,
	UP_PART,
	LOW_PART,
}control_mode_e;

typedef enum{
	WAIST,
	HIP,
	KNEE,
	CALF,
	EMPTY,
}body_mode_e;

typedef struct{
	float postemp_set;
	float last_pos;
}pos_set_t;

typedef struct{
	control_mode_e control_mode;
	body_mode_e body_mode;
	pos_set_t pos_set[10];
}main_control_t;

extern dm_motor_info_t dm_motor_info[16];
extern main_control_t main_control;

void mainTask(void *argument);
#endif
