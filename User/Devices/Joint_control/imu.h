#ifndef _IMU_H
#define _IMU_H
#include "stdint.h"

/* interval for every IMU data frame */
#define INTERVAL_MS 2000

#define SPI_DMA_GYRO_LENGTH       8
#define SPI_DMA_ACCEL_LENGTH      9
#define SPI_DMA_ACCEL_TEMP_LENGTH 4
#define IMU_DR_SHIFTS        0
#define IMU_SPI_SHIFTS       1
#define IMU_UPDATE_SHIFTS        2
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define CBOARD_R_TOWARDS_LEFT 0
#define CBOARD_R_TOWARDS_FRONT 90
#define CBOARD_R_TOWARDS_RIGHT 180
#define CBOARD_R_TOWARDS_BACK 270

#define IMU_BIAS_ADDR 0x080E0000U

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2
#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

/* Typedefs for DJI offered driver */
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

typedef struct
{
    float x, y, z;
} _3AxiesData;

typedef struct
{
		_3AxiesData attitude;
} IMU;

typedef struct
{
	float pitch, yaw, roll;
	float totalYaw;
	int refYawCircle;
	float gyroYaw, gyroPitch, gyroRoll;
	
} Attitude;

typedef enum{
	IMU_CALI_OK,
	IMU_CALI_NO_STORED_DATA,
	IMU_CALI_STORE_DATA_INTEGRITY_FAIL,
	IMU_CALI_PWR_DOWN,
	IMU_CALI_DATA_OVER_RANGE,
	IMU_CALI_HEAT_CONTROL_FAIL,
	IMU_CALI_FAIL
} IMU_Calibrate_StatusTypeDef;

extern Attitude attitude;
extern volatile uint8_t gyro_update_flag;
extern volatile uint8_t accel_update_flag;
extern volatile uint8_t accel_temp_update_flag;
extern volatile uint8_t mag_update_flag;
extern volatile uint8_t imu_start_dma_flag;

void IMU_Init(void);
void IMU_DataFrame_Handler(IMU const *imu);
void imu_Task (void *argument);

void SPI2_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void SPI2_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);
void DMA2_Stream2_IRQHandler_IMU(void);

void imu_cmd_spi_dma(void);

#endif
