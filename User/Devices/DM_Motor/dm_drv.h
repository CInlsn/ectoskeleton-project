#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"
#include "dm_info.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

//以下是DM4310的参数，用其他电机需要更改下面参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
#define POS_TO_ANGLE 180 / 3.1415926f

#define RDC 0x33
#define RID 0x51


typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	float pos_m;
	float lastpos;
	float totalpos;
	float pos;
	float vel;
	float tor;
	float angle;
	float lastangle;
	float totalangle;
	float Tmos;
	float Tcoil;
	int convert;
	
	float    xout;          // 输出轴绝对位置（0x51读到的值）
  float    pos_offset;    // = pos_m - xout
  uint8_t  offset_inited; // =1 表示已经算过 offset，可以做绝对位置控制
}motor_fbpara_t;


typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;



extern void dm_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm_read_xout(FDCAN_HandleTypeDef *hcan, uint16_t motor_id);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//关节电机
extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, dm_motor_info_t *dm_info);
extern void mit_ctrl_abs(FDCAN_HandleTypeDef *hcan,Joint_Motor_t *m,dm_motor_info_t *info, float x_abs);
extern void mit_free(hcan_t* hcan, Joint_Motor_t *m);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);


extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void dm_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);

#endif /* __DM4310_DRV_H__ */

