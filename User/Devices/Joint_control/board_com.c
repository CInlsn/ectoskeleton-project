#include "board_com.h"
#include "stdlib.h"
#include "func_lib.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern controller_t controller; 

controller_mess_t tx_msg;  // 上一次成功发送的消息
controller_mess_t new_msg;//新消息

static uint8_t calc_checksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}


void pack_controller_message(controller_mess_t *msg)
{
    msg->header1 = 0xAA;
    msg->header2 = 0x55;

    for (int i = 0; i < 4; i++) {
        msg->channel[i] = controller.channel[i];
    }

    for (int i = 0; i < 5; i++) {
        msg->sw[i] = (uint8_t)controller.SW[i];
    }

    msg->checksum = calc_checksum((uint8_t *)msg, sizeof(controller_mess_t) - 1);
}


void send_message(UART_HandleTypeDef *huart, controller_mess_t data)
{
    static uint8_t first_send = 1;

    // 不是第一次发送 → 比较是否变化
    if (!first_send) {
        if (memcmp(&data, &tx_msg, sizeof(controller_mess_t)) == 0) {
            return;  // 数据没变化，不发送
        }
    }

    // UART DMA 空闲才允许发送
    if (huart->gState != HAL_UART_STATE_READY) {
        return;
    }

    // 记录为最新数据
    memcpy(&tx_msg, &data, sizeof(controller_mess_t));
    first_send = 0;

    // DMA 发送
    HAL_UART_Transmit_DMA(huart, (uint8_t *)&tx_msg, sizeof(controller_mess_t));
}

void comTask(void *argument)
{
    while (1) {
        pack_controller_message(&new_msg);        // 打包遥控器数据
        send_message(&huart1, new_msg);           // 发送（若变化）
        osDelay(1);                                // 1ms 检查一次
    }
}
