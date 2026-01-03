#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"

extern Joint_Motor_t motor[15];
int motor_flag = 0;
static uint32_t last_tick = 0;
dm_motor_info_t dm_motor_info[16] = {0};
main_control_t main_control;
int calf_flag;

extern osThreadId_t xout_can1_Handle;
extern osThreadId_t xout_can2_Handle;
extern osThreadAttr_t xout_can1_attributes;
extern osThreadAttr_t xout_can2_attributes;

int xout_task_started = 0;

void val_limit(float *val,float min,float max){
	if(*val <= min){
		*val = min;
		calf_flag = -1;
		return;
	}
	else if(*val >= max){
		*val = max;
		calf_flag = 1;
		return;
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
			mit_free(&hfdcan1, &motor[1]);
      mit_free(&hfdcan1, &motor[3]);
			mit_free(&hfdcan2, &motor[5]);
			mit_free(&hfdcan2, &motor[7]);
			mit_free(&hfdcan1, &motor[9]);
		}
		else if (main_control.control_mode == UP_PART){

			if (main_control.body_mode == WAIST ){
				main_control.pos_set[9].xouttemp_set = main_control.pos_set[9].last_xout + 0.02f * controller.channel[1];
				val_limit(&main_control.pos_set[9].xouttemp_set,KNEE_MIN ,KNEE_MAX);
				main_control.pos_set[9].last_xout = main_control.pos_set[9].xouttemp_set;
				mit_ctrl_abs(&hfdcan1, &motor[9], &dm_motor_info[9],main_control.pos_set[9].xouttemp_set);
				
				mit_free(&hfdcan1, &motor[1]);
				mit_free(&hfdcan1, &motor[3]);
				mit_free(&hfdcan2, &motor[5]);
				mit_free(&hfdcan2, &motor[7]);
			}
			else if (main_control.body_mode == HIP){
				main_control.pos_set[7].xouttemp_set = main_control.pos_set[7].last_xout + 0.02f * controller.channel[0];
				
				val_limit(&main_control.pos_set[7].xouttemp_set,KNEE_MIN ,KNEE_MAX);
				
				main_control.pos_set[7].last_xout = main_control.pos_set[7].xouttemp_set;
				
				mit_ctrl_abs(&hfdcan2, &motor[7], &dm_motor_info[7],main_control.pos_set[7].xouttemp_set);
				mit_free(&hfdcan1, &motor[1]);
				mit_free(&hfdcan1, &motor[3]);
				mit_free(&hfdcan2, &motor[5]);
				mit_free(&hfdcan1, &motor[9]);
			}
		}
		else if (main_control.control_mode == LOW_PART){
			if (main_control.body_mode == KNEE){
				main_control.pos_set[5].xouttemp_set = main_control.pos_set[5].last_xout + 0.02f * controller.channel[1];
				
				val_limit(&main_control.pos_set[5].xouttemp_set,KNEE_MIN ,KNEE_MAX);
				
				main_control.pos_set[5].last_xout = main_control.pos_set[5].xouttemp_set;
				
				mit_ctrl_abs(&hfdcan2, &motor[5], &dm_motor_info[5],main_control.pos_set[5].xouttemp_set);
				mit_free(&hfdcan1, &motor[1]);
				mit_free(&hfdcan1, &motor[3]);
				mit_free(&hfdcan2, &motor[7]);
				mit_free(&hfdcan1, &motor[9]);
			}
			else if (main_control.body_mode == CALF){
				int tmp_flag = 1;
				if ((ROD_LMAX-main_control.pos_set[1].xouttemp_set)-(main_control.pos_set[3].xouttemp_set-ROD_RMIN)>DF_LIMIT || \
					(ROD_LMAX-main_control.pos_set[1].xouttemp_set)-(main_control.pos_set[3].xouttemp_set-ROD_RMIN) < -DF_LIMIT){
					tmp_flag = 0;
				}
				
				main_control.pos_set[1].xouttemp_set = main_control.pos_set[1].last_xout + 0.02f * controller.channel[1] - 0.005f * controller.channel[0]*tmp_flag;
				main_control.pos_set[3].xouttemp_set = main_control.pos_set[3].last_xout - 0.02f * controller.channel[1] - 0.005f * controller.channel[0]*tmp_flag;
					
				val_limit(&main_control.pos_set[1].xouttemp_set,ROD_LMIN ,ROD_LMAX);
				val_limit(&main_control.pos_set[3].xouttemp_set,ROD_RMIN ,ROD_RMAX);
				
				main_control.pos_set[1].last_xout = main_control.pos_set[1].xouttemp_set;
				main_control.pos_set[3].last_xout = main_control.pos_set[3].xouttemp_set;
				
				mit_ctrl_abs(&hfdcan1, &motor[1], &dm_motor_info[1],main_control.pos_set[1].xouttemp_set);
				mit_ctrl_abs(&hfdcan1, &motor[3], &dm_motor_info[3],main_control.pos_set[3].xouttemp_set);
				mit_free(&hfdcan2, &motor[5]);
				mit_free(&hfdcan1, &motor[7]);
				mit_free(&hfdcan1, &motor[9]);
			}
		}
		else{
			mit_free(&hfdcan1, &motor[1]);
      mit_free(&hfdcan1, &motor[3]);
			mit_free(&hfdcan2, &motor[5]);
			mit_free(&hfdcan1, &motor[9]);
			mit_free(&hfdcan2, &motor[7]);
		}
}

void motor_info_init(){		
		dm_motor_info[1].con_parameter.Kd =16;
		dm_motor_info[1].con_parameter.Kp =28;
		dm_motor_info[1].con_parameter.Tq =0;
		dm_motor_info[1].motor_info.pos = 0;
		dm_motor_info[1].motor_info.vel = 0;
	
		dm_motor_info[3].con_parameter.Kd =16;
		dm_motor_info[3].con_parameter.Kp =28;
		dm_motor_info[3].con_parameter.Tq =0;
		dm_motor_info[3].motor_info.pos = 0;
		dm_motor_info[3].motor_info.vel = 0;
	
		dm_motor_info[5].con_parameter.Kd =16;
		dm_motor_info[5].con_parameter.Kp =30;
		dm_motor_info[5].con_parameter.Tq =0;
		dm_motor_info[5].motor_info.pos = 0;
		dm_motor_info[5].motor_info.vel = 0;
	
		dm_motor_info[7].con_parameter.Kd =16;
		dm_motor_info[7].con_parameter.Kp =30;
		dm_motor_info[7].con_parameter.Tq =0;
		dm_motor_info[7].motor_info.pos = 0;
		dm_motor_info[7].motor_info.vel = 0;
		
		dm_motor_info[9].con_parameter.Kd =12;
		dm_motor_info[9].con_parameter.Kp =25;
		dm_motor_info[9].con_parameter.Tq =0;
		dm_motor_info[9].motor_info.pos = 0;
		dm_motor_info[9].motor_info.vel = 0;
		
		main_control.pos_set[1].last_xout = ROD_LINIT;
		main_control.pos_set[3].last_xout = ROD_RINIT;
		main_control.pos_set[5].last_xout = KNEE_INIT;
		main_control.pos_set[7].last_xout = HIP_INIT;
		main_control.pos_set[9].last_xout = WAIST_INIT;
}

void motor_enable(){
//    static int enabled = 0;
//    if(enabled) return;

    if(motor[1].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[1].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[3].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[3].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[5].para.state ==0){
        enable_motor_mode(&hfdcan2,motor[5].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[7].para.state ==0){
        enable_motor_mode(&hfdcan2,motor[7].para.id,MIT_MODE);
		osDelay (5);
		}
		if(motor[9].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[9].para.id,MIT_MODE);
		osDelay (5);
		}
//    enabled = 1;
}
void mainTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	motor_info_init();
	motor_enable();
  if(!xout_task_started){
    xout_can1_Handle = osThreadNew(XoutTask_CAN1, NULL, &xout_can1_attributes);
    xout_can2_Handle = osThreadNew(XoutTask_CAN2, NULL, &xout_can2_attributes);
    xout_task_started = 1;
  }
	while(1){
///*    当你认为零点位置不对时，请用以下代码重设零点*/
//		dm_save_zero(&hfdcan1, 1);
//		dm_save_zero(&hfdcan1, 3);
//		dm_save_zero(&hfdcan2, 5);
//		dm_save_zero(&hfdcan2, 7);
//		dm_save_zero(&hfdcan1, 9);
		motor_enable();
//		mode_switch();
//		if(motor_flag == 0){
//			main_control.control_mode = STOP;
//		}
//		
//		mode_active();
		if(motor_flag ==1){
			mit_ctrl_abs(&hfdcan1, &motor[1], &dm_motor_info[1],ROD_LINIT);
			mit_ctrl_abs(&hfdcan1, &motor[3], &dm_motor_info[3],ROD_RINIT);
			mit_ctrl_abs(&hfdcan2, &motor[5], &dm_motor_info[5],KNEE_INIT);
			mit_ctrl_abs(&hfdcan2, &motor[7], &dm_motor_info[7],HIP_INIT);
			mit_ctrl_abs(&hfdcan1, &motor[9], &dm_motor_info[9],HIP_INIT);
		}
		else{
				mit_free(&hfdcan1, &motor[1]);
				mit_free(&hfdcan1, &motor[3]);
				mit_free(&hfdcan2, &motor[5]);
				mit_free(&hfdcan1, &motor[7]);
				mit_free(&hfdcan1, &motor[9]);
		}
		osDelay(2);
	}
}

void XoutTask_CAN1(void *argument)
{
    const uint16_t ids[] = {1, 3 , 9};
    const uint8_t num_ids = sizeof(ids)/sizeof(ids[0]);
    uint8_t idx = 0;

    for(;;){
        dm_read_xout(&hfdcan1, ids[idx]);  // 发给 CAN1
        idx++;
        if (idx >= num_ids) idx = 0;

        osDelay(5);  
    }
}

void XoutTask_CAN2(void *argument)
{
    const uint16_t ids[] = {5, 7};
    const uint8_t num_ids = sizeof(ids)/sizeof(ids[0]);
    uint8_t idx = 0;

    for(;;){
        dm_read_xout(&hfdcan2, ids[idx]);  // 发给 CAN2
        idx++;
        if (idx >= num_ids) idx = 0;

        osDelay(5); 
    }
}
