#include "dm_drv.h"

#include "fdcan.h"
#include "arm_math.h"

extern Joint_Motor_t motor[15];
float Hex_To_Float(uint32_t *Byte,int num)//十六进制到浮点数
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)//浮点数到十六进制转换
{
	return *( uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}


/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1]<<8) | rx_data[2];
        motor->para.v_int = (rx_data[3]<<4)|(rx_data[4]>>4);
        motor->para.t_int = ((rx_data[4]&0xF)<<8)|rx_data[5];
        // ① MIT 内部坐标：p_m（映射位置）
        motor->para.pos_m = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);

        motor->para.Tmos  = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);

        // ② 如果已经完成 offset 标定，则对外位置 = 绝对位置 xout
        if (motor->para.offset_inited){
            motor->para.pos = motor->para.xout;
        }
        else{
            // 启动早期还没有 xout / offset，就先用 p_m 顶一下
            motor->para.pos = motor->para.pos_m;
        }

        motor->para.totalpos += motor->para.pos - motor->para.lastpos;
        motor->para.lastpos   = motor->para.pos;
    }
}

void dm_read_xout(FDCAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    uint8_t data[4];

    data[0] = motor_id & 0xFF;       // CANID_L
    data[1] = (motor_id >> 8) & 0xFF; // CANID_H
    data[2] = 0x33;                  // read command
    data[3] = 0x51;                  // RID = xout

    canx_send_data(hcan, 0x7FF, data, 4);
}

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
static float dm_abs_to_mit_pos(Joint_Motor_t *m, float x_abs)
{
    float p_des = x_abs + m->para.pos_offset;  // 带上 offset，转到 p_m 坐标系

    // 选做：如果超过 PMAX 映射范围，可以做一下 wrap
    const float span = (P_MAX - P_MIN);  // 一般 P_MIN = -PMAX, P_MAX = +PMAX
    while (p_des > P_MAX) p_des -= span;
    while (p_des < P_MIN) p_des += span;

    return p_des;
}

void mit_ctrl(hcan_t* hcan, uint16_t motor_id, dm_motor_info_t *dm_info)//float pos, float vel,float kp, float kd, float torq
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;
	pos_tmp = float_to_uint(dm_info->motor_info.pos, P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(dm_info->motor_info.vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(dm_info->con_parameter.Kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(dm_info->con_parameter.Kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(dm_info->con_parameter.Tq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}

void mit_ctrl_abs(FDCAN_HandleTypeDef *hcan,Joint_Motor_t *m,dm_motor_info_t *info, float x_abs){
    if (m->para.offset_inited)
    {
        float p_des = dm_abs_to_mit_pos(m, x_abs);
        info->motor_info.pos = p_des;
    }
    else
    {
        info->motor_info.pos = x_abs;
    }
    mit_ctrl(hcan, m->para.id, info);
}
//无电流输出
void mit_free(hcan_t* hcan, Joint_Motor_t *m)
{		
    dm_motor_info_t free_cmd;

    free_cmd.con_parameter.Kp = 0.0f;   // 不锁位置
    free_cmd.con_parameter.Kd = 0.0f;   // 不阻尼
    free_cmd.con_parameter.Tq = 0.0f;   // 不输出扭矩

    free_cmd.motor_info.pos = 0.0f;     // 这里必须写，不表示位置控制，只是MIT框架要求
    free_cmd.motor_info.vel = 0.0f;
		
    mit_ctrl(hcan, m->para.id, &free_cmd);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}

void dm_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    canx_send_data(hcan, id + MIT_MODE, data, 8);
    HAL_Delay(50);
}


