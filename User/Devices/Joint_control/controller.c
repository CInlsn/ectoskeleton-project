#include "controller.h"
#include "string.h"
#include "usart.h"
#include "cmsis_os2.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

extern UART_HandleTypeDef huart5 , huart2;
extern osThreadId_t unitree_motorHandle;
// Parameters related to receiving data  接收数据相关参数
uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;

// data-caching mechanism  数据缓存
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t sbus_data[SBUS_RECV_MAX] = {0};
int16_t g_sbus_channels[18] = {0}; 

uint8_t RxTemp = 0;

int aaa = 0 ;

controller_t controller;
// Parses SBUS data into channel values  解析SBUS的数据，转化成通道数值。
static int controller_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // 安全检测，检测是否失联或者数据错误
    // Security detection to check for lost connections or data errors
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
    }
    return failsafe_status;
}

// Receives SBUS cache data  接收SBUS的缓存数据
void controller_Reveive(uint8_t data)
{
    // If the protocol start flag is met, data is received  如果符合协议开始标志，则开始接收数据
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    // Finish receiving a frame of data  完成接收一帧数据
    if (sbus_start && (sbus_buf_index >= (SBUS_RECV_MAX - 1)))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
        }
    }
}

void controller_solve(int16_t *data){
	for (int i = 0;i < 4; i++){
		if (data[i]<=CHANNEL_MAX && data[i]>=CHANNEL_MIN){
			controller.channel[i] = 2.0f * (data[i] - CHANNEL_MEDIUM) /(CHANNEL_MAX - CHANNEL_MIN);
		}
	}
	controller.SW[0] = MID;
	for (int i = 4 ; i<8 ;i++){
		switch (data[i]){
			case CHANNEL_MAX:
				controller.SW[i-3] = DOWN;
				break;
			case CHANNEL_MEDIUM:
				controller.SW[i-3] = MID;
				break;
			case CHANNEL_MIN:
				controller.SW[i-3] = UP;
				break;
			default:
				break;
			
		}
	}
}

void USART_Init(void)
{
    HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxTemp, 1);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart5)
//    {		controller_Reveive(RxTemp);
//        HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxTemp, 1);
//    }
//		// need to modify, seems it only wakeup the Thread  --  unitree_motor
//		if (huart == &huart2){osThreadFlagsSet(unitree_motorHandle, 1);}
//		//	if (huart == &huart3){osThreadFlagsSet(unitree_thread_bus2, 1);}
//}

// SBUS receives and processes data handle  SBUS接收处理数据句柄
void controller_Handle(void *argument)
{	 	
	USART_Init();
	while(1){
		    if (sbus_new_cmd)
    {
        int res = controller_Parse_Data();
				controller_solve(g_sbus_channels);
        sbus_new_cmd = 0;
        if (res) return;
    }
		osDelay(1);
}
}

