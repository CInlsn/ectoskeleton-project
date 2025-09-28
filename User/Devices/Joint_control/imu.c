#include "imu.h"
#include "pid.h"
#include "spi.h"
#include "BMI088_driver.h"
#include "BMI088Middleware.h"

#define IMU_INSTALLATION_ANGLE CBOARD_R_TOWARDS_BACK

#define MPU_TEMP_CONTROL
#define PI 3.14159265f
#define FORCE_CALIBRATE 0
Attitude attitude;

fp32 temp;
fp32 gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 accel[3] = {0.0f, 0.0f, 0.0f};
fp32 mag[3] = {0.0f, 0.0f, 0.0f};
fp32 quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

typedef enum{
	CALI_FSM_WAIT_START,
	CALI_FSM_WAIT_TEMPERATURE_CONTROL,
	CALI_FSM_CALIBRATING,
	CALI_FSM_CALCULATE_RESULT,
	CALI_FSM_FINISH
} Calibration_StateTypeDef;

typedef struct{
	float gyro[3];
} calibration_offset_t;

typedef struct{
	uint32_t header;
	calibration_offset_t offset_payload;
	uint32_t tail;
} calibration_store_t;

calibration_offset_t imu_debias_offset;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGTH];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGTH] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGTH];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGTH] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGTH];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGTH] = {0xA2,0xFF,0xFF,0xFF};
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern uint8_t LED_Override;
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

extern SPI_HandleTypeDef hspi2;

float timing_time = 0.001f;   //loop run time , unit s.
// Acceleror meter filter
fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_filter_3[3] = {0.0f, 0.0f, 0.0f};
fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

bmi088_real_data_t bmi088_real_data;

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};   

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
#ifdef APPLY_BIAS
fp32 gyro_cali_offset[3];
#endif

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
#ifdef APPLY_BIAS
fp32 accel_cali_offset[3];
#endif

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
#ifdef APPLY_BIAS
fp32 mag_cali_offset[3];
#endif

void AHRS_init(fp32 quat[4], fp32 accel[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
#ifdef MPU_TEMP_CONTROL
  #include "pid.h"
  extern TIM_HandleTypeDef htim3;
  #define HEAT_HAL_TIM htim3
  #define HEAT_LL_TIM TIM3
  #define HEAT_CH TIM_CHANNEL_4
	#define TEMP_FACTOR 1
  #define TARGET_TEMP 47 
	PID temp_pid = {
      .kp = 600.f,
      .ki = 5.f,
      .kd = 20000.f,
      .max_out = 10000.0f,
      .integral_limit = 2000.0f,
			.integral_threshold = 10.f,
    };
	
  void mpu_temp_control(float raw_temp) {

    static int32_t pulse = 0;

    /* apply Low-pass Filter for temperature sensor */
    static float temp=0;
    #define TS 0.001f /* sample interval in ms */
    #define FC 100.0f /* cut-off frequency */
    /* filter update ratio in % */
    #define LPF_ALPHA ((int32_t) ((TS / (TS + 1 / (2 * PI * FC))) * 100.0f))
    temp = ((100-LPF_ALPHA) * temp + LPF_ALPHA * raw_temp) / 100;
    #undef TS
    #undef FC
    #undef LPF_ALPHA

    if (temp < 0.9 * TARGET_TEMP) {
      pulse = 60000;
    } else {
      pulse = PID_calc(&temp_pid, temp, TARGET_TEMP) + 17000;
      if (pulse < 0) pulse = 0;
    }

    /* not really switching, compiler will optimize this */
    switch (HEAT_CH) {
      case TIM_CHANNEL_4:
        HEAT_LL_TIM->CCR4 = pulse;
        break;
    }
  }

#endif
	
__weak void IMU_DataFrame_Handler(IMU const *imu) {
	
//	float lastYaw = attitude.yaw;
	attitude.yaw = (float)imu->attitude.x/PI*180;
	attitude.gyroYaw = bmi088_real_data.gyro[2];
	
	attitude.roll = (float)imu->attitude.z/PI*180;
	attitude.pitch = -(float)imu->attitude.y/PI*180;
	
	attitude.gyroPitch = -bmi088_real_data.gyro[1];
	attitude.gyroRoll = bmi088_real_data.gyro[0];
	
//	if(attitude.yaw >90 && lastYaw<-90)
//		attitude.refYawCircle--;
//	if(attitude.yaw<-90&& lastYaw >90)
//		attitude.refYawCircle++;
//	attitude.totalYaw = attitude.refYawCircle *360.0f +attitude.yaw;
}


void imu_Task (void *argument){
		while(1){
				
		}
}
