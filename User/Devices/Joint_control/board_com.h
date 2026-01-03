#ifndef BOARD_COM_H
#define BOARD_COM_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "controller.h"
#include "main.h"

#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0x55

#pragma pack(push, 1)
typedef struct{
    uint8_t  header1;
    uint8_t  header2;
    float    channel[4];
    uint8_t  sw[5];
    uint8_t  checksum;
} controller_mess_t;
#pragma pack(pop)

extern void comTask(void *argument);
void send_message(UART_HandleTypeDef *huart,controller_mess_t data);
#endif
