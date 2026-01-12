#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

#include "dm_drv.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "dm_info.h"
#include "controller.h"
#include "unitree_motor.h"

#define PITCH_RATIO 0.001f
#define ROLL_RATIO 0.0005f
#define UP_LIM1 0.5f
#define LOW_LIM1 -0.1f
#define UP_LIM2 0.1f
#define LOW_LIM2 -0.5f
typedef enum {
    EMPTY = 0,
    TOE_MODE,
    CALF_MODE,
} control_mode_e;

typedef struct{
	float kp;
	float kd;
}motor_pd_t;

typedef struct{
	float postemp_set;
	float last_pos;
}pos_set_t;

extern control_mode_e control_mode;
void mainTask(void *argument);
#endif
