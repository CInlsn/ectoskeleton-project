#ifndef GIMBLE_H
#define GIMBLE_H

#include "dm_drv.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "dm_info.h"

extern dm_motor_info_t dm_motor_info[16];
void gimbleTask(void *argument);
#endif
