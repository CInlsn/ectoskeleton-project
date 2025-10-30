#ifndef _UNITREE_MOTOR_H_
#define _UNITREE_MOTOR_H_

#include "main.h"
#include "cmsis_os2.h"

#define UNITREE_HEADER_IDENTIFIER 0xEEFE

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

void unitree_motorTask(void *argument);

void UnitreeMotor_Info_initialize(void);

int16_t UnitreeMotor_set(int32_t Motor_Index, int16_t Torque);

void UnitreeMotor_Changer(void*_);

void UnitreeMotor_ReceiveCommand(uint8_t* rxBuffer);

void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode);
#endif
