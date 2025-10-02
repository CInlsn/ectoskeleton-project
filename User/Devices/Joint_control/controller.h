#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define CHANNEL_MAX 1792
#define CHANNEL_MEDIUM 992
#define CHANNEL_MIN 192
#define UP 3
#define MIED 2
#define DOWN 1

typedef struct{
	float channel[4];
	int SW[5];
}controller_t;

extern controller_t controller;

void controller_Reveive(uint8_t data);
void controller_Handle(void *argument);

#endif
