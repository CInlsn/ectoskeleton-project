#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"

int motor_flag = 0;
static uint32_t last_tick = 0;
int calf_flag;

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

	while(1){
		osDelay(10);
	}
}
