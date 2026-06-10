#ifndef GONGWANG_H
#define GONGWANG_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GONGWANG_CMD_ABS             0x02
#define GONGWANG_ABS_FRAME_LEN       6U
#define GONGWANG_RX_DMA_BUF_SIZE     8U
#define GONGWANG_CACHE_LINE_SIZE     32U
#define GONGWANG_UART_TIMEOUT_MS     500U
#define GONGWANG_ABS_BITS_UART2      14U
#define GONGWANG_ABS_BITS_UART3      17U
#define GONGWANG_PI                  3.14159265358979323846f
#define GONGWANG_TWO_PI              (2.0f * GONGWANG_PI)


typedef enum {
    GONGWANG_BUS_UART2 = 0,
    GONGWANG_BUS_UART3 = 1,
    GONGWANG_BUS_COUNT
} GongWangBusId_t;

typedef struct {
    GongWangBusId_t bus_id;
    uint8_t sf;
    uint8_t ea0;
    uint8_t ea1;
    uint8_t ca0;
    uint8_t ca1;
    uint8_t crc_ok;
    uint8_t online;
    uint32_t abs_raw;
    float abs_rad;
    float total_rad;
    int32_t turn_count;
    uint32_t tick_ms;
    uint32_t update_count;
} GongWangAbsState_t;

typedef struct {
    float pos;
    float totalpos;
    int32_t turn_count;
    uint8_t online;
    uint32_t abs_raw;
} GongWangHipYaw_t;

extern volatile GongWangAbsState_t g_gongwang_abs_state[GONGWANG_BUS_COUNT];
extern volatile GongWangHipYaw_t hip_yaw[GONGWANG_BUS_COUNT];

void GongWang_Init(void);
void GongWang_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
