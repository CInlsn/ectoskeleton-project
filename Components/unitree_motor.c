#include "unitree_motor.h"

UnitreeMotor_A1_Command_t UnitreeMotors_Command[4];
UnitreeMotor_A1_Status_t UnitreeMotors_Status[4];
UnitreeMotor_Info_t UnitreeMotors_Info[4]; 
int32_t UnitreeMotors_Pos[4];
int16_t UnitreeMotors_Tor[4];

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern osThreadId_t unitree_motorHandle;

int mistaken = 0, correct = 0;
int32_t saved_Pos;

uint8_t rxBuffer0[120];
uint8_t rxBuffer1[120];
uint8_t rxBuffer2[120];
uint8_t rxBuffer3[120];
//check for recieve
uint32_t recv_crc ;

volatile uint8_t currentSendingMotorIndex = 0;
UART_HandleTypeDef* currentSendingUart = NULL;

void UnitreeMotor_Info_initialize(){
	UnitreeMotors_Info[0].Motor_ID 		 = 0x00;
	UnitreeMotors_Info[0].rxBuffer		 = rxBuffer0;
	
	UnitreeMotors_Info[1].Motor_ID 		 = 0x01;
	UnitreeMotors_Info[1].rxBuffer		 = rxBuffer1;

	UnitreeMotors_Info[2].Motor_ID 		 = 0x02;
	UnitreeMotors_Info[2].rxBuffer		 = rxBuffer2;
	
	UnitreeMotors_Info[3].Motor_ID 		 = 0x03;
	UnitreeMotors_Info[3].rxBuffer		 = rxBuffer3;

	// initialization for some other same thing
	for(int Motor_Index = 0; Motor_Index <= 3; Motor_Index++){
		UnitreeMotors_Info[Motor_Index].torque			 						= 0.0;
		UnitreeMotors_Command[Motor_Index].MasterComdV3.mode 		= 0;
		UnitreeMotors_Command[Motor_Index].MasterComdV3.modifyBit_reserved	= 0xFF;
		UnitreeMotors_Command[Motor_Index].COMHead.start 				= UNITREE_HEADER_IDENTIFIER;
		UnitreeMotors_Command[Motor_Index].COMHead.motorID 			= UnitreeMotors_Info[Motor_Index].Motor_ID;
	}
}
uint32_t crc32_core(uint32_t* ptr, uint32_t len){
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	for (uint32_t i = 0; i < len; i++){
		xbit = 1UL << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++){
			if (CRC32 & 0x80000000U){
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			}
			else
			CRC32 <<= 1;
			if (data & xbit)
			CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

void UnitreeMotor_SendCommand(uint8_t Motor_Index, UART_HandleTypeDef* control_huart){
	uint8_t garbage;
	garbage = garbage;
	currentSendingMotorIndex = Motor_Index;
	currentSendingUart = control_huart;
	// only need to update the CRCdata because the torch might been changed through set()
	UnitreeMotors_Command[Motor_Index].CRCdata = crc32_core((uint32_t*)(&UnitreeMotors_Command[Motor_Index]),7);
	// do some checks
	while (__HAL_UART_GET_FLAG(control_huart, UART_FLAG_RXNE)){
    volatile uint8_t garbage = (uint8_t)(control_huart->Instance->RDR);
}
	// it depends on which huart is using
//	if (control_huart == &huart2){HAL_GPIO_WritePin(RS485_DIR2_GPIO_Port, RS485_DIR2_Pin, GPIO_PIN_SET);}
//	if (control_huart == &huart3){HAL_GPIO_WritePin(RS485_DIR3_GPIO_Port, RS485_DIR3_Pin, GPIO_PIN_SET);}
	HAL_UART_Transmit_DMA(control_huart, (uint8_t* )&UnitreeMotors_Command[Motor_Index], 34);
}

void UnitreeMotor_ReceiveCommand(uint8_t* rxBuffer, uint8_t Motor_Index){
	UnitreeMotor_A1_Status_t* temp_signal_ptr =((UnitreeMotor_A1_Status_t*)rxBuffer);
	recv_crc = (uint32_t)rxBuffer[74] |
                        ((uint32_t)rxBuffer[75] << 8) |
                        ((uint32_t)rxBuffer[76] << 16) |
                        ((uint32_t)rxBuffer[77] << 24);
	if(crc32_core((uint32_t*)rxBuffer,18) == temp_signal_ptr->CRCdata){//19
		correct++;
		UnitreeMotors_Status[Motor_Index] = *temp_signal_ptr;
		UnitreeMotors_Pos[Motor_Index] = UnitreeMotors_Status[Motor_Index].ServoComdV3.Motor_Pos;
		UnitreeMotors_Tor[Motor_Index] = UnitreeMotors_Status[Motor_Index].ServoComdV3.T_torque;
	}else
	mistaken++;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    // 切换 RS485 方向
//    if (huart == &huart2)
//        HAL_GPIO_WritePin(RS485_DIR2_GPIO_Port, RS485_DIR2_Pin, GPIO_PIN_RESET);
//    else if (huart == &huart3)
//        HAL_GPIO_WritePin(RS485_DIR3_GPIO_Port, RS485_DIR3_Pin, GPIO_PIN_RESET);
		
    HAL_UART_Receive_DMA(huart,UnitreeMotors_Info[currentSendingMotorIndex].rxBuffer, 78);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if (huart == &huart2){
		UnitreeMotor_ReceiveCommand(UnitreeMotors_Info[currentSendingMotorIndex].rxBuffer,currentSendingMotorIndex);
		}
}

void UnitreeMotor_SetPos(int32_t Motor_index, int pos){
	UnitreeMotors_Command[Motor_index].MasterComdV3.pos = pos;
}

void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode){
	if(UNITREE_MOTOR_ON == mode){
		UnitreeMotors_Command[motor_index].MasterComdV3.mode = 10;
	}else{
		UnitreeMotors_Command[motor_index].MasterComdV3.mode = 0;
	}
}

int16_t UnitreeMotor_SetCurrent(int32_t Motor_Index, int16_t Torque){
	int error_flag = 0;
	// the input check
	if (Torque == 0)
		UnitreeMotors_Command[Motor_Index].MasterComdV3.mode = 0;
	else if (UnitreeMotors_Command[Motor_Index].MasterComdV3.mode == 0)
		UnitreeMotors_Command[Motor_Index].MasterComdV3.mode = 10;
	// the check finished
	if(error_flag != 0){return error_flag;}
	UnitreeMotors_Command[Motor_Index].MasterComdV3.t_torque = Torque;
	return error_flag;
}

void unitree_motorTask(void *argument){
	UnitreeMotor_Info_initialize();
	osDelay(1000);
	for (int i = 0; i < 3; i++){
     UnitreeMotor_SetOutputMode(i, UNITREE_MOTOR_ON);
     UnitreeMotor_SetCurrent(i, 5); // 小电流旋转，合法命令
  }
	while(1){
		for (int i = 2; i >= 0; i--){
			UnitreeMotor_SendCommand(i,&huart2);
			osDelay(10);
		}
	}
}

