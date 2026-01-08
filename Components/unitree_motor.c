#include "unitree_motor.h"

UnitreeMotor_A1_Command_t UnitreeMotors_Command[4];
UnitreeMotor_A1_Status_t UnitreeMotors_Status[4];
UnitreeMotor_Info_t UnitreeMotors_Info[4]; 
UnitreeMotor_State_SI_t UnitreeMotors_State_SI[4];

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2,huart5;
extern UART_HandleTypeDef huart3;
extern osThreadId_t unitree_motorHandle;

int mistaken = 0, correct = 0;
int32_t saved_Pos;
int aaa1 = 0;
int aaa2 = 0;

uint8_t rxBuffer0[120];
uint8_t rxBuffer1[120];
uint8_t rxBuffer2[120];
uint8_t rxBuffer3[120];
//check for recieve
uint32_t recv_crc ;

int16_t homing_vel[3]={HOMING_VEL0,HOMING_VEL1,HOMING_VEL2};
float motor_zero_offset_rad[3] = {0};
float UnitreeMotors_TotalPos[3] = {0};
uint8_t homing_done[3] = {0};
volatile uint8_t currentSendingMotorIndex = 0;
UART_HandleTypeDef* currentSendingUart = NULL;
static inline float clampf(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

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
	HAL_UART_Transmit_DMA(control_huart, (uint8_t* )&UnitreeMotors_Command[Motor_Index], 34);
}

void UnitreeMotor_ReceiveCommand(uint8_t* rxBuffer){
	UnitreeMotor_A1_Status_t* temp_signal_ptr =((UnitreeMotor_A1_Status_t*)rxBuffer);
	uint8_t motor_id = temp_signal_ptr->COMHead.motorID;

	if(crc32_core((uint32_t*)rxBuffer,18) == temp_signal_ptr->CRCdata ){//19
		correct++;
		UnitreeMotors_Status[motor_id] = *temp_signal_ptr;
		
		int16_t T_raw   = temp_signal_ptr->ServoComdV3.T_torque;
    int16_t W_raw   = temp_signal_ptr->ServoComdV3.W_speed;
    int32_t P_raw   = temp_signal_ptr->ServoComdV3.Motor_Pos;
		
		float tau_motor = ((float)T_raw) / TORQUE_SCALE;        // Nm
    float vel_motor = ((float)W_raw) / VELOCITY_SCALE;      // rad/s
    float pos_motor = ((float)P_raw) / POSITION_SCALE;      // rad

    UnitreeMotors_State_SI[motor_id].tau_out_nm =tau_motor * GEAR_RATIO;

    UnitreeMotors_State_SI[motor_id].vel_out_rad_s =vel_motor / GEAR_RATIO;

    UnitreeMotors_State_SI[motor_id].pos_out_rad =pos_motor / GEAR_RATIO;
		
		if (homing_done[motor_id]){
			UnitreeMotors_TotalPos[motor_id] =UnitreeMotors_State_SI[motor_id].pos_out_rad - motor_zero_offset_rad[motor_id];
		}
		else{
			UnitreeMotors_TotalPos[motor_id] = 0.0f;
		}
	}
	else
	mistaken++;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    HAL_UART_Receive_DMA(huart,UnitreeMotors_Info[currentSendingMotorIndex].rxBuffer, 78);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if (huart == &huart2){
		  UnitreeMotor_ReceiveCommand(UnitreeMotors_Info[currentSendingMotorIndex].rxBuffer);
      aaa1++;
		}
    if (huart == &huart5)
    {		controller_Reveive(RxTemp);
        HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxTemp, 1);
				aaa2++;
    }
}


void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode){
	if(UNITREE_MOTOR_ON == mode){
		UnitreeMotors_Command[motor_index].MasterComdV3.mode = 10;
	}else{
		UnitreeMotors_Command[motor_index].MasterComdV3.mode = 0;
	}
}

void UnitreeMotor_SetTorque(uint8_t motor, float tau_out_nm)
{
    /* 1. SI 域限幅（输出轴） */
    tau_out_nm = clampf(tau_out_nm, -TAU_OUT_MAX_NM, TAU_OUT_MAX_NM);

    /* 2. 输出轴 → 转子侧 */
    float tau_motor = tau_out_nm / GEAR_RATIO;

    /* 3. 协议缩放 */
    float T_cmd_f = tau_motor * TORQUE_SCALE;

    /* 4. 协议域限幅（双保险） */
    T_cmd_f = clampf(T_cmd_f,
                     -TORQUE_CMD_MAX * TORQUE_SCALE,
                      TORQUE_CMD_MAX * TORQUE_SCALE);

    int16_t T_cmd = (int16_t)T_cmd_f;

    /* 5. 写命令 */
    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = T_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = 0;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kP = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kW = 0;
}

void UnitreeMotor_SetPosition(uint8_t motor,float pos_out_rad,float kp,float kd){
    /* 1. SI 域限幅 */
    pos_out_rad = clampf(pos_out_rad,-POS_OUT_MAX_RAD,POS_OUT_MAX_RAD);

    /* 2. 输出轴 → 转子侧 */
    float pos_motor = pos_out_rad * GEAR_RATIO;

    /* 3. 协议缩放 */
    int32_t Pos_cmd = (int32_t)(pos_motor * POSITION_SCALE);

    /* 4. 写命令 */
    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = 0;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = 0;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = Pos_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.kP = (uint16_t)(kp * 2048.0f);
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd * 1024.0f);
}

void UnitreeMotor_SetVelocity(uint8_t motor, float vel_out_rad_s, float kd)
{
    /* 1. SI 域限幅 */
    vel_out_rad_s = clampf(vel_out_rad_s,-VEL_OUT_MAX_RAD,VEL_OUT_MAX_RAD);

    /* 2. 输出轴 → 转子侧 */
    float vel_motor = vel_out_rad_s * GEAR_RATIO;

    /* 3. 协议缩放 */
    float W_cmd_f = vel_motor * VELOCITY_SCALE;

    /* 4. 协议域限幅 */
    W_cmd_f = clampf(W_cmd_f,-VELOCITY_CMD_MAX * VELOCITY_SCALE, VELOCITY_CMD_MAX * VELOCITY_SCALE);

    int16_t W_cmd = (int16_t)W_cmd_f;

    /* 5. 写命令 */
    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = 0;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = W_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kP = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd * 1024.0f);
}

void UnitreeMotor_SetMixed(uint8_t motor,float tau_out_nm,float vel_out_rad_s,float pos_out_rad,float kp,float kd){
    /* ===== SI 域限幅 ===== */
    tau_out_nm     = clampf(tau_out_nm,     -TAU_OUT_MAX_NM, TAU_OUT_MAX_NM);
    vel_out_rad_s  = clampf(vel_out_rad_s,  -VEL_OUT_MAX_RAD, VEL_OUT_MAX_RAD);
    pos_out_rad    = clampf(pos_out_rad,    -POS_OUT_MAX_RAD, POS_OUT_MAX_RAD);

    /* ===== 转子侧 ===== */
    float tau_motor = tau_out_nm / GEAR_RATIO;
    float vel_motor = vel_out_rad_s * GEAR_RATIO;
    float pos_motor = pos_out_rad * GEAR_RATIO;

    /* ===== 协议缩放 ===== */
    int16_t T_cmd = (int16_t)(tau_motor * TORQUE_SCALE);
    int16_t W_cmd = (int16_t)(vel_motor * VELOCITY_SCALE);
    int32_t P_cmd = (int32_t)(pos_motor * POSITION_SCALE);

    /* ===== 写命令 ===== */
    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = T_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = W_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = P_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.kP = (uint16_t)(kp * 2048.0f);
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd * 1024.0f);
}

void UnitreeMotor_Homing_All(void)
{
    uint32_t start_tick = osKernelGetTickCount();

    /* 稳定计数 */
    uint8_t stable_cnt0 = 0;
    uint8_t stable_cnt1 = 0;
    uint8_t stable_cnt2 = 0;

    /* 上一时刻位置 */
    float last_pos0 = UnitreeMotors_State_SI[0].pos_out_rad;
    float last_pos1 = UnitreeMotors_State_SI[1].pos_out_rad;
    float last_pos2 = UnitreeMotors_State_SI[2].pos_out_rad;

    /* ===== 1. 进入低速 homing 速度模式并发送 ===== */
    UnitreeMotor_SetVelocity(0, homing_vel[0], HOMING_KD);
    UnitreeMotor_SetVelocity(1, homing_vel[1], HOMING_KD);
    UnitreeMotor_SetVelocity(2, homing_vel[2], HOMING_KD);

    UnitreeMotor_SendCommand(0, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(1, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(2, &huart2);
    osDelay(2);

    /* ===== 2. 轮询等待稳定 ===== */
    while (1)
    {
        /* --- motor 0 --- */
        float curr_pos0 = UnitreeMotors_State_SI[0].pos_out_rad;
        float curr_vel0 = UnitreeMotors_State_SI[0].vel_out_rad_s;
        float delta0    = curr_pos0 - last_pos0;
        last_pos0 = curr_pos0;

        if (fabsf(delta0) < HOMING_POS_DELTA_RAD_TH &&
            fabsf(curr_vel0) < HOMING_VEL_RAD_TH)
            stable_cnt0++;
        else
            stable_cnt0 = 0;

        /* --- motor 1 --- */
        float curr_pos1 = UnitreeMotors_State_SI[1].pos_out_rad;
        float curr_vel1 = UnitreeMotors_State_SI[1].vel_out_rad_s;
        float delta1    = curr_pos1 - last_pos1;
        last_pos1 = curr_pos1;

        if (fabsf(delta1) < HOMING_POS_DELTA_RAD_TH &&
            fabsf(curr_vel1) < HOMING_VEL_RAD_TH)
            stable_cnt1++;
        else
            stable_cnt1 = 0;

        /* --- motor 2 --- */
        float curr_pos2 = UnitreeMotors_State_SI[2].pos_out_rad;
        float curr_vel2 = UnitreeMotors_State_SI[2].vel_out_rad_s;
        float delta2    = curr_pos2 - last_pos2;
        last_pos2 = curr_pos2;

        if (fabsf(delta2) < HOMING_POS_DELTA_RAD_TH &&
            fabsf(curr_vel2) < HOMING_VEL_RAD_TH)
            stable_cnt2++;
        else
            stable_cnt2 = 0;

        /* --- 是否全部稳定 --- */
        if (stable_cnt0 >= HOMING_STABLE_CNT &&
            stable_cnt1 >= HOMING_STABLE_CNT &&
            stable_cnt2 >= HOMING_STABLE_CNT)
        {
            break;
        }

        /* --- 超时保护 --- */
        if ((osKernelGetTickCount() - start_tick) > HOMING_TIMEOUT_MS)
        {
            break;
        }

        /* 周期性 resend，防止电机 watchdog 停止 */
        UnitreeMotor_SendCommand(0, &huart2);
        osDelay(2);
        UnitreeMotor_SendCommand(1, &huart2);
        osDelay(2);
        UnitreeMotor_SendCommand(2, &huart2);

        osDelay(10);
    }

    /* ===== 3. 停止电机 + 记录零点 ===== */
    UnitreeMotor_SetVelocity(0, 0.0f, 2.0f);
    UnitreeMotor_SetVelocity(1, 0.0f, 2.0f);
    UnitreeMotor_SetVelocity(2, 0.0f, 2.0f);

    UnitreeMotor_SendCommand(0, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(1, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(2, &huart2);

    motor_zero_offset_rad[0] = UnitreeMotors_State_SI[0].pos_out_rad;
    motor_zero_offset_rad[1] = UnitreeMotors_State_SI[1].pos_out_rad;
    motor_zero_offset_rad[2] = UnitreeMotors_State_SI[2].pos_out_rad;

    UnitreeMotors_TotalPos[0] = 0.0f;
    UnitreeMotors_TotalPos[1] = 0.0f;
    UnitreeMotors_TotalPos[2] = 0.0f;

    homing_done[0] = 1;
    homing_done[1] = 1;
    homing_done[2] = 1;
}
