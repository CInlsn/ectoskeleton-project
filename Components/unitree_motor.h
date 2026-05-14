#ifndef _UNITREE_MOTOR_H_
#define _UNITREE_MOTOR_H_

#include "main.h"
#include "cmsis_os2.h"
#include "controller.h"
#include <stdint.h>
#include <stdlib.h>

#define UNITREE_HEADER_IDENTIFIER 0xEEFE

#define UNITREE_TX_LEN              34U
#define UNITREE_RX_LEN              78U
#define UNITREE_RX_DONE_FLAG        (1U << 0)
#define UNITREE_UART_TIMEOUT_MS     20U
#define UNITREE_COMM_PERIOD_MS      1U

/*
 * Manual RS485 direction control.
 *
 * CtrBoard-H7_V1.0 pin diagram:
 *   USART2-RS485 DE = PD4
 *   USART3-RS485 DE = PB14
 *
 * If CubeMX has already configured USART2/USART3 in hardware RS485-DE mode,
 * change this to 0 to avoid manually toggling DE as GPIO.
 */
#ifndef UNITREE_RS485_MANUAL_DE
#define UNITREE_RS485_MANUAL_DE     0
#endif

#if UNITREE_RS485_MANUAL_DE
#define UNITREE_UART2_DE_GPIO_Port  GPIOD
#define UNITREE_UART2_DE_Pin        GPIO_PIN_4
#define UNITREE_UART3_DE_GPIO_Port  GPIOB
#define UNITREE_UART3_DE_Pin        GPIO_PIN_14
#endif

#define HOMING_VEL0                 1.0f
#define HOMING_VEL1                -1.0f
#define HOMING_VEL2                 1.0f
#define HOMING_KD                   4000.0f

#define HOMING_STABLE_CNT           5U
#define HOMING_POS_DELTA_RAD_TH     0.001f
#define HOMING_VEL_RAD_TH           0.01f
#define HOMING_TIMEOUT_MS           8000U

#define MOTOR_NUM                   3

/* A1 reducer ratio. */
#define GEAR_RATIO                  9.1f

/* Protocol scaling. */
#define TORQUE_SCALE                256.0f
#define VELOCITY_SCALE              128.0f
#define POSITION_SCALE              (16384.0f / (2.0f * 3.1415926f))

/* Output-side safety limits. Tune according to your mechanism. */
#define TAU_OUT_MAX_NM              15.0f
#define VEL_OUT_MAX_RAD             20.0f
#define POS_OUT_MAX_RAD             6.28f

/* Rotor-side protocol limits from A1 protocol/manual. */
#define TORQUE_CMD_MAX              128.0f
#define VELOCITY_CMD_MAX            256.0f

typedef __packed struct {
    __packed struct {
        uint16_t start;
        uint8_t  motorID;
        uint8_t  reserved;
    } COMHead;

    __packed struct {
        uint8_t  mode;
        uint8_t  modifyBit_reserved;
        uint8_t  readBit_reserved;
        uint8_t  reserved;
        uint32_t modify_reserved;
        int16_t  t_torque;
        int16_t  w_speed;
        int32_t  pos;
        uint16_t kP;
        uint16_t kW;
        uint8_t  lowHzMotorCmdIndex_reserved;
        uint8_t  lowHzMotorCmdByte_reserved;
        int32_t  reserved2;
    } MasterComdV3;

    uint32_t CRCdata;
} UnitreeMotor_A1_Command_t;

typedef __packed struct {
    __packed struct {
        uint16_t start;
        uint8_t  motorID;
        uint8_t  reserved;
    } COMHead;

    __packed struct {
        uint8_t  mode;
        uint8_t  ReadBit_reserved;
        uint8_t  Temp;
        uint8_t  MError;
        uint32_t Read;
        int16_t  T_torque;
        int16_t  W_speed;
        int32_t  LW;
        uint16_t W2_reserved;
        uint32_t LW2_reserved;
        int16_t  MotorAcc;
        int16_t  OutAcc_reserved;
        int32_t  Motor_Pos;
        int32_t  Motor2_Pos_reserved;
    } ServoComdV3;

    __packed struct {
        int16_t AngleSpeed_x;
        int16_t AngleSpeed_y;
        int16_t AngleSpeed_z;
        int16_t ACC_x;
        int16_t ACC_y;
        int16_t ACC_z;
    } IMU_Info;

    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
    uint32_t reserved5;
    uint32_t reserved6;
    uint32_t CRCdata;
} UnitreeMotor_A1_Status_t;

typedef __packed struct {
    UART_HandleTypeDef *control_uart;
    uint32_t Motor_ID;
    int16_t torque;
    uint8_t *rxBuffer;
} UnitreeMotor_Info_t;

typedef enum {
    UNITREE_MOTOR_ON = 0,
    UNITREE_MOTOR_OFF
} Unitree_Motor_Output_Mode_e;

typedef struct {
    float pos_out_rad;
    float vel_out_rad_s;
    float tau_out_nm;
} UnitreeMotor_State_SI_t;

extern UnitreeMotor_A1_Command_t UnitreeMotors_Command[4];
extern UnitreeMotor_A1_Status_t  UnitreeMotors_Status[4];
extern UnitreeMotor_Info_t       UnitreeMotors_Info[4];
extern UnitreeMotor_State_SI_t   UnitreeMotors_State_SI[4];

extern int16_t UnitreeMotors_Tor[4];
extern int32_t UnitreeMotors_Pos[4];

extern float   UnitreeMotors_TotalPos[3];
extern int32_t motor_zero_offset_rad[3];
extern int32_t motor_accumulated_counts[3];
extern uint8_t homing_done[3];

uint32_t crc32_core(uint32_t *ptr, uint32_t len);

void UnitreeMotor_Info_initialize(void);
void UnitreeMotor_SendCommand(uint8_t motor_index, UART_HandleTypeDef *control_huart);
void UnitreeMotor_ReceiveCommand(uint8_t *rxBuffer, uint8_t motor_index);

void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode);
void UnitreeMotor_SetCurrentRaw(uint8_t motor, int16_t T_raw);
void UnitreeMotor_SetTorque(uint8_t motor, float tau_out_nm);
void UnitreeMotor_SetPosition(uint8_t motor, float pos_out_rad, float kp, float kd);
void UnitreeMotor_SetVelocity(uint8_t motor, float vel_out_rad_s, float kd);
void UnitreeMotor_SetMixed(uint8_t motor, float tau_out_nm, float vel_out_rad_s, float pos_out_rad, float kp, float kd);

void UnitreeMotor_Homing_All(void);

/* New separated communication threads. */
void unitree_motor_uart2Task(void *argument);
void unitree_motor_uart3Task(void *argument);


#endif
