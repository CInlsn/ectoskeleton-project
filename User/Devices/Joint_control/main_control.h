#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

#include "dm_drv.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "dm_info.h"
#include "controller.h"
#include "usbd_cdc_if.h"

extern dm_motor_info_t dm_motor_info[16];

void mainTask(void *argument);
void XoutTask_CAN1(void *argument);
void XoutTask_CAN2(void *argument);

/*----------------Bilateral control begin-----------------*/
#define USB_BILATERAL_KP_MOTOR1     10.0f
#define USB_BILATERAL_KP_MOTOR3     10.0f
#define USB_BILATERAL_KP_MOTOR5     30.0f
#define USB_BILATERAL_KP_MOTOR7     30.0f
#define USB_BILATERAL_KP_MOTOR9     0.0f
#define USB_BILATERAL_KV            0.1f
#define USB_BILATERAL_KD            0.0125f
#define USB_BILATERAL_PE            0.0000125f
#define USB_BILATERAL_TORQUE_LIMIT  3.5f
#define USB_BILATERAL_CONTROL_PERIOD_MS 2U
#define USB_XOUT_POLL_PERIOD_MS     2U
#define USB_STATE_TX_PERIOD_MS      2U
#define USB_BILATERAL_RX_PERIOD_MS  USB_STATE_TX_PERIOD_MS
#define USB_BILATERAL_RX_TIMEOUT_MS 50U
#define USB_BILATERAL_SMOOTH_ALPHA  0.35f
/*----------------Bilateral control end-----------------*/

/*----------------USB com begin-----------------*/
#define USB_SIDE_ID_LEFT        0x01
#define USB_SIDE_ID_RIGHT       0x02
#define USB_CURRENT_SIDE_ID     USB_SIDE_ID_LEFT

#define USB_XOUT_HEAD1          0xAA
#define USB_XOUT_HEAD2          0x55
#define USB_XOUT_CMD            0x81
#define USB_XOUT_SIDE_ID        USB_CURRENT_SIDE_ID
// current project = left leg

#define USB_XOUT_MOTOR_NUM      5
#define USB_XOUT_PAYLOAD_LEN    (1 + 2 + USB_XOUT_MOTOR_NUM * 5)
// payload = 1 byte side + 2 byte seq + 5 * (1 byte id + 4 byte float)

#define USB_XOUT_FRAME_LEN      (2 + 1 + 1 + USB_XOUT_PAYLOAD_LEN + 1)
uint8_t USB_Send_All_Xout(void);
void USB_Task(void *argument);

#define USB_STATE_RX_HEAD1          0xAA
#define USB_STATE_RX_HEAD2          0x55
#define USB_STATE_RX_CMD            0x01
#define USB_STATE_RX_MOTOR_NUM      5
#define USB_STATE_RX_DATA_LEN       (1 + USB_STATE_RX_MOTOR_NUM * 9)
// data = 1 byte side_id + 5 * (1 byte motor id + 4-byte float position + 4-byte float velocity), little-endian

#define USB_STATE_RX_FRAME_LEN      (2 + 1 + USB_STATE_RX_DATA_LEN + 1)

void USB_StateRx_ProcessBytes(const uint8_t *buf, uint32_t len);
/*----------------USB com end-----------------*/
#endif
