#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"

int motor_flag = 0;
static uint32_t last_tick = 0;
int calf_flag;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

control_mode_e control_mode;
pos_set_t pos_set[3];

motor_pd_t motor_pd[3];

void val_limit(float *val,float min,float max){
	if(*val <= min){
		*val = min;
		calf_flag = -1;
	}
	else if(*val >= max){
		*val = max;
		calf_flag = 1;
	};
	calf_flag = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == GPIO_PIN_15 ) {
      uint32_t now = HAL_GetTick();            
      if (now - last_tick < 200)                  
        return;        
      last_tick = now;                      
      if(motor_flag==0) motor_flag = 1;
			else motor_flag=0;
    }
}

void mode_swtich(){
		if (controller.SW[4] == UP){
			control_mode = CALF_MODE;
		}
		else if (controller.SW[4] == DOWN){
			control_mode = TOE_MODE;
		}
		else
			control_mode = EMPTY;
}

void mode_control(){
	if (control_mode == EMPTY){
		UnitreeMotor_SetVelocity(0, 0,1);
		UnitreeMotor_SendCommand(0, &huart2);
		osDelay(1);
		UnitreeMotor_SetVelocity(1, 0,1);
		UnitreeMotor_SendCommand(1, &huart2);
		osDelay(1);
		UnitreeMotor_SetVelocity(2, 0,1);
		UnitreeMotor_SendCommand(2, &huart2);			
		osDelay(1);
	}
	else if (control_mode == TOE_MODE){
		pos_set[0].postemp_set =  pos_set[0].last_pos+ PITCH_RATIO * controller.channel[1];		
		val_limit(&pos_set[0].postemp_set,UnitreeMotors_TotalPos[2]-0.2f,UnitreeMotors_TotalPos[2]);
		pos_set[0].last_pos = pos_set[0].postemp_set;
		UnitreeMotor_SetPosition(0,pos_set[0].postemp_set + motor_zero_offset_rad[0],motor_pd[0].kp,motor_pd[0].kd);
//		UnitreeMotor_SetMixed(0,0,0,pos_set[0].postemp_set + motor_zero_offset_rad[0],motor_pd[0].kp,motor_pd[0].kd);
		UnitreeMotor_SendCommand(0, &huart2);
    osDelay(1);
		UnitreeMotor_SetVelocity(1, 0,1);
		UnitreeMotor_SendCommand(1, &huart2);
		osDelay(1);
		UnitreeMotor_SetVelocity(2, 0,1);
		UnitreeMotor_SendCommand(2, &huart2);			
		osDelay(1);
	}
	else if (control_mode == CALF_MODE){
		pos_set[0].postemp_set =  pos_set[0].last_pos + 0.4f*PITCH_RATIO * controller.channel[1];		
		val_limit(&pos_set[0].postemp_set,pos_set[2].postemp_set-0.2f,pos_set[2].postemp_set);
		pos_set[0].last_pos = pos_set[0].postemp_set;
		pos_set[1].postemp_set =  pos_set[1].last_pos - PITCH_RATIO * controller.channel[1] + ROLL_RATIO * controller.channel[3];
		val_limit(&pos_set[1].postemp_set,LOW_LIM1,UP_LIM1);
		pos_set[1].last_pos = pos_set[1].postemp_set;
		pos_set[2].postemp_set =  pos_set[2].last_pos + PITCH_RATIO * controller.channel[1] + ROLL_RATIO * controller.channel[3];	
		val_limit(&pos_set[2].postemp_set,LOW_LIM2,UP_LIM2);
		pos_set[2].last_pos = pos_set[2].postemp_set;
		UnitreeMotor_SetPosition(0,pos_set[0].postemp_set + motor_zero_offset_rad[0],motor_pd[0].kp,motor_pd[0].kd);
		UnitreeMotor_SendCommand(0, &huart2);
    osDelay(1);
		UnitreeMotor_SetPosition(1,pos_set[1].postemp_set + motor_zero_offset_rad[1],motor_pd[1].kp,motor_pd[1].kd);
		UnitreeMotor_SendCommand(1, &huart2);
    osDelay(1);
		UnitreeMotor_SetPosition(2,pos_set[2].postemp_set + motor_zero_offset_rad[2],motor_pd[2].kp,motor_pd[2].kd);
		UnitreeMotor_SendCommand(2, &huart2);
    osDelay(1);
	}
	else{
		UnitreeMotor_SetVelocity(0, 0,1);
		UnitreeMotor_SendCommand(0, &huart2);
		osDelay(1);
		UnitreeMotor_SetVelocity(1, 0,1);
		UnitreeMotor_SendCommand(1, &huart2);
		osDelay(1);
		UnitreeMotor_SetVelocity(2, 0,1);
		UnitreeMotor_SendCommand(2, &huart2);			
		osDelay(1);
	}
}
void motor_init(){
	UnitreeMotor_SetOutputMode(0, UNITREE_MOTOR_ON);
	UnitreeMotor_SetOutputMode(1, UNITREE_MOTOR_ON);
	UnitreeMotor_SetOutputMode(2, UNITREE_MOTOR_ON);
	pos_set[0].last_pos = 0;
	pos_set[1].last_pos = 0;
	pos_set[2].last_pos = 0;
	pos_set[0].postemp_set = 0;
	pos_set[1].postemp_set = 0;
	pos_set[2].postemp_set = 0;
	motor_pd[0].kp = 1000.0f;
	motor_pd[0].kd = 2000.0f;
	motor_pd[1].kp = 1000.0f;
	motor_pd[1].kd = 2000.0f;
	motor_pd[2].kp = 1000.0f;
	motor_pd[2].kd = 2000.0f;
}
void mainTask(void *argument){
	UnitreeMotor_Info_initialize();
	UnitreeMotor_Homing_All();
	motor_init();
	
	while(1){
		if (motor_flag == 1){		
			mode_swtich();
			mode_control();
		}
		else{
			UnitreeMotor_SetVelocity(0, 0,1);
			UnitreeMotor_SendCommand(0, &huart2);
			osDelay(1);
			UnitreeMotor_SetVelocity(1, 0,1);
			UnitreeMotor_SendCommand(1, &huart2);
			osDelay(1);
			UnitreeMotor_SetVelocity(2, 0,1);
			UnitreeMotor_SendCommand(2, &huart2);			
			osDelay(3);
		
		}

	}
}
