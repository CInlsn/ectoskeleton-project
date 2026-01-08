#ifndef _UNITREE_MOTOR_H_
#define _UNITREE_MOTOR_H_

#include "main.h"
#include "cmsis_os2.h"
#include "controller.h"
#include <stdlib.h>

#define UNITREE_HEADER_IDENTIFIER 0xEEFE
#define HOMING_VEL0           1.5   
#define HOMING_VEL1						-1.5
#define HOMING_VEL2						1.5
#define HOMING_KD             4

#define HOMING_STABLE_CNT   5

#define HOMING_POS_DELTA_RAD_TH  0.001f   // ~0.06 deg
#define HOMING_VEL_RAD_TH        0.01f

/* Homing 最大等待时间（ms，防死循环） */
#define HOMING_TIMEOUT_MS        8000

#define MOTOR_NUM 3
/* ================== 电机/减速器参数 ================== */
#define GEAR_RATIO        9.1f

/* ================== 协议缩放因子 ================== */
#define TORQUE_SCALE      256.0f
#define VELOCITY_SCALE    128.0f
#define POSITION_SCALE    (16384.0f / (2.0f * 3.1415926f))

/* ================== SI 域安全上限（输出轴） ================== */
/* ?? 这些值你可以根据机械和电源再调 */
#define TAU_OUT_MAX_NM    15.0f      // 输出轴最大力矩（Nm）
#define VEL_OUT_MAX_RAD  20.0f       // 输出轴最大角速度（rad/s）
#define POS_OUT_MAX_RAD  6.28f       // 输出轴最大位置（rad），示例

/* ================== 协议域硬上限（转子侧，来自手册） ================== */
#define TORQUE_CMD_MAX    (128.0f)   // Nm（转子侧）
#define VELOCITY_CMD_MAX (256.0f)    // rad/s（转子侧）
// declarations of the motor command will be sent
typedef __packed struct{
	__packed struct{
		uint16_t start;
		uint8_t motorID;
		uint8_t reserved;
	} COMHead;
	__packed struct{
		uint8_t 	mode;
		uint8_t 	modifyBit_reserved;
		uint8_t 	readBit_reserved;
		uint8_t 	reserved;
		uint32_t 	modify_reserved;
		int16_t 	t_torque;
		int16_t 	w_speed;
		int32_t 	pos;
		uint16_t 	kP;
		uint16_t 	kW;
		uint8_t 	lowHzMotorCmdIndex_reserved;
		uint8_t 	lowHzMotorCmdByte_reserved;
		int32_t 	reserved2;
	}MasterComdV3;
	uint32_t CRCdata;
} UnitreeMotor_A1_Command_t;

extern int16_t UnitreeMotors_Tor[];
extern int32_t UnitreeMotors_Pos[];
// declaration of the motor status(receive from the motor signal)
typedef __packed struct{
	__packed struct{
		uint16_t 	start;
		uint8_t 	motorID;
		uint8_t 	reserved;
	} COMHead;
	__packed struct{
		uint8_t 	mode;
		uint8_t 	ReadBit_reserved;
		uint8_t 	Temp; //7
		uint8_t 	MError;
		uint32_t 	Read;
		int16_t 	T_torque;
		int16_t		W_speed; //15-16
		int32_t 	LW;
		uint16_t 	W2_reserved;
		uint32_t 	LW2_reserved; //23-26
		int16_t 	MotorAcc;
		int16_t 	OutAcc_reserved;
		int32_t 	Motor_Pos;
		int32_t 	Motor2_Pos_reserved;  //35-38
	}ServoComdV3;
	__packed struct{
		int16_t 	AngleSpeed_x;
		int16_t 	AngleSpeed_y;
		int16_t 	AngleSpeed_z;
		int16_t 	ACC_x;
		int16_t 	ACC_y;
		int16_t 	ACC_z;
	} IMU_Info;
	
	uint32_t reserved1;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
	uint32_t reserved5;
	uint32_t reserved6;
	uint32_t CRCdata;
} UnitreeMotor_A1_Status_t;


typedef __packed struct{
	UART_HandleTypeDef* control_uart;
	uint32_t Motor_ID;
	int16_t torque;
	uint8_t* rxBuffer;
} UnitreeMotor_Info_t;

typedef enum{
	UNITREE_MOTOR_ON = 0,
	UNITREE_MOTOR_OFF
} Unitree_Motor_Output_Mode_e;

typedef struct {
    float pos_out_rad;      // 输出轴角度 [rad]
    float vel_out_rad_s;    // 输出轴角速度 [rad/s]
    float tau_out_nm;       // 输出轴力矩 [Nm]
} UnitreeMotor_State_SI_t;

extern UnitreeMotor_State_SI_t UnitreeMotors_State_SI[4];
extern uint8_t motor_comm_index;
extern float UnitreeMotors_TotalPos[3];
extern UnitreeMotor_Info_t UnitreeMotors_Info[4];

void unitree_motorTask(void *argument);

void UnitreeMotor_Info_initialize(void);

void UnitreeMotor_SendCommand(uint8_t Motor_Index, UART_HandleTypeDef* control_huart);

int16_t UnitreeMotor_set(int32_t Motor_Index, int16_t Torque);

void UnitreeMotor_ReceiveCommand(uint8_t* rxBuffer);

void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode);

void UnitreeMotor_SetTorque(uint8_t motor, float tau_out_nm);

void UnitreeMotor_SetPosition(uint8_t motor,float pos_out_rad,float kp,float kd);

void UnitreeMotor_SetVelocity(uint8_t motor, float vel_out_rad_s, float kd);

void UnitreeMotor_SetMixed(uint8_t motor,float tau_out_nm,float vel_out_rad_s,float pos_out_rad,float kp,float kd);

void UnitreeMotor_Homing_All(void);
#endif
