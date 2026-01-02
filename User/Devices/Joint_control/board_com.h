#ifndef BOARD_COM_H
#define BOARD_COM_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "cmsis_os2.h"

#define FRAME_HEAD1  0xAA
#define FRAME_HEAD2  0x55

#pragma pack(push,1)
typedef struct{
    uint8_t  header1;
    uint8_t  header2;
    float    channel[4];
    uint8_t  sw[5];
    uint8_t  checksum;
} controller_mess_t;
#pragma pack(pop)

//void board_com_rx_init_dma(void);
void board_com_init_it(void);
void board_com_poll(void);   // 可选：在任务里调用
void comTask(void *argument);
#endif
