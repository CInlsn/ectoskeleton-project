#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        osDelay(1);
    }
}

void BMI088_delay_us(uint16_t us)
{
	osDelay((us<1000)?1:us/1000);
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

