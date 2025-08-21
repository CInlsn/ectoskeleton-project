#include "dm_drv.h"
#include "gimble.h"
#include "stdlib.h"
extern Joint_Motor_t motor[15];
int xxx = 0;
int flag = 0;
static uint32_t last_tick = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15 ) {
        
        uint32_t now = HAL_GetTick();                // 毫秒级时基
        
        if (now - last_tick < 200)                   // 如果距离上次小于 50?ms，则认为是抖动，忽略
            return;
        
        last_tick = now;                            // 更新上次有效按下时间
        
        // —— 真正的按键按下处理 ——  
        if(xxx==0) xxx = 1;
				else xxx=0;
    }
}
void gimbleTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	while(1){
		if(motor[1].para.state ==0){
		enable_motor_mode(&hfdcan1,motor[1].para.id,MIT_MODE);
		}
		else{
			if(xxx == 0)
				mit_ctrl(&hfdcan1,motor[1].para.id,0,0,0,0,0);
			else
				mit_ctrl(&hfdcan1,motor[1].para.id,0,0,0,0,5);
		}
		osDelay(10);
	}
}
