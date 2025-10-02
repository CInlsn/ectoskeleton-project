#include "main_control.h"
#include "stdlib.h"

extern Joint_Motor_t motor[15];
int motor_flag = 0;
static uint32_t last_tick = 0;
dm_motor_info_t dm_motor_info[16];
main_control_t main_control;


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
void mode_switch(){
	switch (controller.SW[4]){
		case (UP):
			main_control.control_mode = UP_PART;
			break;
		case (MIED):
			main_control.control_mode = STOP;
			break;
		case (DOWN):
			main_control.control_mode = LOW_PART;
			break;
		default:
			break;
	}
	
	if(main_control.control_mode == UP_PART){
		switch (controller.SW[3]){
			case (UP):
				switch (controller.SW[2]){
					case (UP):
						main_control.body_mode = WAIST ;
						break;
					case (DOWN):
						main_control.body_mode = EMPTY;
						break;
					default:
						main_control.body_mode = EMPTY;
						break;
				}
			break;
			case (DOWN):
				switch (controller.SW[2]){
					case (UP):
						main_control.body_mode = EMPTY ;
						break;
					case (DOWN):
						main_control.body_mode = HIP;
						break;
					default:
						main_control.body_mode = EMPTY;
						break;
				}
			break;
			default:
				main_control.body_mode = EMPTY;
				break;
		}
	}
	else if(main_control.control_mode == LOW_PART){
		switch (controller.SW[3]){
			case (UP):
				switch (controller.SW[2]){
					case (UP):
						main_control.body_mode = KNEE ;
						break;
					case (DOWN):
						main_control.body_mode = EMPTY;
						break;
					default:
						main_control.body_mode = EMPTY;
						break;
				}
			break;
			case (DOWN):
				switch (controller.SW[2]){
					case (UP):
						main_control.body_mode = EMPTY ;
						break;
					case (DOWN):
						main_control.body_mode = CALF;
						break;
					default:
						main_control.body_mode = EMPTY;
						break;
				}
			break;
			default:
				main_control.body_mode = EMPTY;
				break;
		}
	}
}

void mode_active(){
		if (main_control.control_mode == STOP || main_control.body_mode == EMPTY){
			for (int i = 0;i < 5;i++){
				mit_ctrl(&hfdcan1,motor[2*i+1].para.id,&dm_motor_info[0]);
			}
		}
		else if (main_control.control_mode == UP_PART){
			if (main_control.body_mode == WAIST ){
			}
			else if (main_control.body_mode == HIP){
			}
		}
		else if (main_control.control_mode == LOW_PART){
			if (main_control.body_mode == KNEE){
			}
			else if (main_control.body_mode == CALF){
				
			}
		}
		else{
			for (int i = 0;i < 5;i++){
				mit_ctrl(&hfdcan1,motor[2*i+1].para.id,&dm_motor_info[0]);
			}
		}
}

void motor_info_init(){
	for (int i =0;i<16;i++){
		dm_motor_info[i].con_parameter.Kd =0;
		dm_motor_info[i].con_parameter.Kp =0;
		dm_motor_info[i].con_parameter.Tq =0;
		dm_motor_info[i].motor_info.pos = 0;
		dm_motor_info[i].motor_info.vel = 0;
	}
}

void motor_enable(){
	for (int i = 0;i<5;i++){
		if(motor[2*i+1].para.state ==0){
			enable_motor_mode(&hfdcan1,motor[2*i+1].para.id,MIT_MODE);
		}
	}
}
void mainTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	motor_info_init();
	dm_motor_info[1].con_parameter.Kd =2;
	dm_motor_info[1].con_parameter.Kp =6;
	dm_motor_info[1].con_parameter.Tq =0;
	dm_motor_info[1].motor_info.pos = -0.4;//-0.4
	dm_motor_info[1].motor_info.vel = 0;
	dm_motor_info[3].con_parameter.Kd =2;
	dm_motor_info[3].con_parameter.Kp =6;
	dm_motor_info[3].con_parameter.Tq =0;
	dm_motor_info[3].motor_info.pos = 4.64;//4.64
	dm_motor_info[3].motor_info.vel = 0;
	while(1){
			motor_enable();
			if(motor_flag == 0){
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[5].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[7].para.id,&dm_motor_info[0]);
				
			}
			else{
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[1]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[3]);
			}		
		osDelay(10);
	}
}
