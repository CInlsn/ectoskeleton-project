#include "gimble.h"
#include "stdlib.h"
extern Joint_Motor_t motor[15];
int motor_flag = 0;
int flag = 0;
static uint32_t last_tick = 0;
dm_motor_info_t dm_motor_info[16];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15 ) {
        
        uint32_t now = HAL_GetTick();                // ���뼶ʱ��
        
        if (now - last_tick < 200)                   // ��������ϴ�С�� 50?ms������Ϊ�Ƕ���������
            return;
        
        last_tick = now;                            // �����ϴ���Ч����ʱ��
        
        // ���� �����İ������´��� ����  
        if(motor_flag==0) motor_flag = 1;
				else motor_flag=0;
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
void gimbleTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	motor_info_init();
	dm_motor_info[1].con_parameter.Kd =1;
	dm_motor_info[1].con_parameter.Kp =10;
	dm_motor_info[1].con_parameter.Tq =0;
	dm_motor_info[1].motor_info.pos = 0.3;
	dm_motor_info[1].motor_info.vel = 0;
	dm_motor_info[3].con_parameter.Kd =1;
	dm_motor_info[3].con_parameter.Kp =10;
	dm_motor_info[3].con_parameter.Tq =0;
	dm_motor_info[3].motor_info.pos = 0.7;
	dm_motor_info[3].motor_info.vel = 0;
	while(1){
		if(motor[1].para.state ==0){
				enable_motor_mode(&hfdcan1,motor[1].para.id,MIT_MODE);
		}
		if(motor[3].para.state ==0){
				enable_motor_mode(&hfdcan1,motor[3].para.id,MIT_MODE);
		}
		else{
			if(motor_flag == 0){
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[0]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[0]);
			}
			else
				mit_ctrl(&hfdcan1,motor[1].para.id,&dm_motor_info[1]);
				mit_ctrl(&hfdcan1,motor[3].para.id,&dm_motor_info[3]);
				
		}
		osDelay(10);
	}
}
