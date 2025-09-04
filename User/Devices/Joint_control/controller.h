#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

void controller_Reveive(uint8_t data);
void controller_Handle(void *argument);

#endif
