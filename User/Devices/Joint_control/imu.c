#include "imu.h"
#include "pid.h"

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

void imu_Task (void *argument){
		while(1){
				
		}
}
