#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"

extern Joint_Motor_t motor[15];
int motor_flag = 0;
static uint32_t last_tick = 0;
dm_motor_info_t dm_motor_info[16] = {0};
main_control_t main_control;
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
void mode_switch(){
	switch (controller.SW[4]){
		case (UP):
			main_control.control_mode = UP_PART;
			break;
		case (MID):
			main_control.control_mode = STOP;
			main_control.body_mode = EMPTY;
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
			mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
			mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
		}
		else if (main_control.control_mode == UP_PART){
			if (main_control.body_mode == WAIST ){
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
			}
			else if (main_control.body_mode == HIP){
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
			}
		}
		else if (main_control.control_mode == LOW_PART){
			if (main_control.body_mode == KNEE){
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
			}
			else if (main_control.body_mode == CALF){
				main_control.pos_set[1].postemp_set = main_control.pos_set[1].last_pos + 0.1f * controller.channel[1] - 0.05f * controller.channel[0];
				main_control.pos_set[3].postemp_set = main_control.pos_set[3].last_pos - 0.1f * controller.channel[1] - 0.05f * controller.channel[0];
				
				if ((ROD_LMAX-main_control.pos_set[1].postemp_set)-(main_control.pos_set[3].postemp_set-ROD_RMIN)>DF_LIMIT && \
					(ROD_LMAX-main_control.pos_set[1].postemp_set)-(main_control.pos_set[3].postemp_set-ROD_RMIN) < -DF_LIMIT){
					main_control.pos_set[1].postemp_set = main_control.pos_set[1].last_pos + 0.1f * controller.channel[1];
					main_control.pos_set[3].postemp_set = main_control.pos_set[3].last_pos - 0.1f * controller.channel[1];
				}
					
				val_limit(&main_control.pos_set[1].postemp_set,ROD_LMIN ,ROD_LMAX);
				val_limit(&main_control.pos_set[3].postemp_set,ROD_RMIN ,ROD_RMAX);
				
				dm_motor_info[1].motor_info.pos = main_control.pos_set[1].postemp_set;
				dm_motor_info[3].motor_info.pos = main_control.pos_set[3].postemp_set;
				
				main_control.pos_set[1].last_pos = main_control.pos_set[1].postemp_set;
				main_control.pos_set[3].last_pos = main_control.pos_set[3].postemp_set;
				
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[1]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[3]);
			}
		}
		else{
			mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
			mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
		}
}

void motor_info_init(){		
		dm_motor_info[1].con_parameter.Kd =2;
		dm_motor_info[1].con_parameter.Kp =6;
		dm_motor_info[1].con_parameter.Tq =0;
		dm_motor_info[1].motor_info.pos = ROD_LINIT;//-0.4
		dm_motor_info[1].motor_info.vel = 0;
	
		dm_motor_info[3].con_parameter.Kd =2;
		dm_motor_info[3].con_parameter.Kp =6;
		dm_motor_info[3].con_parameter.Tq =0;
		dm_motor_info[3].motor_info.pos = ROD_RINIT;//4.64
		dm_motor_info[3].motor_info.vel = 0;
		
		main_control.pos_set[1].last_pos = ROD_LINIT;
		main_control.pos_set[3].last_pos = ROD_RINIT;
}

void motor_enable(){
	if(motor[1].para.state ==0){
			enable_motor_mode(&hfdcan1,motor[1].para.id,MIT_MODE);
	}
	if(motor[3].para.state ==0){
			enable_motor_mode(&hfdcan1,motor[3].para.id,MIT_MODE);
	}
}
void mainTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	motor_info_init();
	while(1){
		motor_enable();
		if(motor_flag == 0){
			mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
			mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
		}
		else{
			mode_switch();
			mode_active();
		}		
		osDelay(10);
	}
}
