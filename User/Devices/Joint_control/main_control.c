#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"

int motor_flag = 0;
static uint32_t last_tick = 0;
int calf_flag;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

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

void mainTask(void *argument){
	UnitreeMotor_Info_initialize();
	UnitreeMotor_SetOutputMode(0, UNITREE_MOTOR_ON);
	UnitreeMotor_SetOutputMode(1, UNITREE_MOTOR_ON);
	UnitreeMotor_SetOutputMode(2, UNITREE_MOTOR_ON);
	UnitreeMotor_Homing_All();
	while(1){
		if (motor_flag == 1){		
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
