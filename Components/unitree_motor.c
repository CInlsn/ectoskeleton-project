#include "unitree_motor.h"
#include "main.h"
#include <math.h>
#include <string.h>

/*
 * STM32H7 + A1 motor dual UART/RS485 version
 *
 * Wiring target:
 *   motor1(array index 1) -> USART2-RS485
 *   motor2(array index 2) -> USART3-RS485
 *
 * A1 protocol:
 *   TX command: 34 bytes
 *   RX status : 78 bytes
 *
 * Important CubeMX settings:
 *   USART2 / USART3: 4800000 baud, 8 data bits, no parity, 1 stop bit
 *   DMA TX and DMA RX enabled for both USART2 and USART3
 *
 * RS485 DE:
 *   According to CtrBoard-H7_V1.0 pin diagram:
 *     USART2 DE = PD4
 *     USART3 DE = PB14
 *   If you already enabled USART Hardware RS485-DE in CubeMX, set
 *     UNITREE_RS485_MANUAL_DE to 0 in unitree_motor.h
 */

UnitreeMotor_A1_Command_t UnitreeMotors_Command[4];
UnitreeMotor_A1_Status_t  UnitreeMotors_Status[4];
UnitreeMotor_Info_t       UnitreeMotors_Info[4];
UnitreeMotor_State_SI_t   UnitreeMotors_State_SI[4];

int32_t UnitreeMotors_Pos[4] = {0};
int16_t UnitreeMotors_Tor[4] = {0};

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

/* If your remote-controller code uses these symbols, keep them external here. */
extern uint8_t RxTemp;
void controller_Reveive(uint8_t data);

int mistaken = 0;
int correct  = 0;

uint8_t rxBuffer0[UNITREE_RX_LEN];
uint8_t rxBuffer1[UNITREE_RX_LEN];
uint8_t rxBuffer2[UNITREE_RX_LEN];
uint8_t rxBuffer3[UNITREE_RX_LEN];

float   homing_vel[3] = {HOMING_VEL0, HOMING_VEL1, HOMING_VEL2};
int32_t motor_zero_offset_rad[3] = {0};
float   UnitreeMotors_TotalPos[3] = {0};
uint8_t homing_done[3] = {0};

int32_t motor_accumulated_counts[3] = {0};
int32_t last_raw_pos[3]             = {0};
int32_t homing_offset_counts[3]     = {0};
uint8_t motor_first_sync[3]         = {0};

typedef struct {
    UART_HandleTypeDef *huart;
    volatile uint8_t active_motor_index;
    volatile uint32_t tx_done_cnt;
    volatile uint32_t rx_done_cnt;
    volatile uint32_t timeout_cnt;
    osThreadId_t owner_thread;
} UnitreeMotor_BusCtx_t;

UnitreeMotor_BusCtx_t s_bus2 = {
    .huart = &huart2,
    .active_motor_index = 1,
    .tx_done_cnt = 0,
    .rx_done_cnt = 0,
    .timeout_cnt = 0,
    .owner_thread = NULL
};

UnitreeMotor_BusCtx_t s_bus3 = {
    .huart = &huart3,
    .active_motor_index = 2,
    .tx_done_cnt = 0,
    .rx_done_cnt = 0,
    .timeout_cnt = 0,
    .owner_thread = NULL
};

static inline float clampf_local(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static UnitreeMotor_BusCtx_t *UnitreeMotor_GetBus(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) return &s_bus2;
    if (huart == &huart3) return &s_bus3;
    return NULL;
}

#if UNITREE_RS485_MANUAL_DE

static void UnitreeMotor_RS485_SetTx(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        HAL_GPIO_WritePin(UNITREE_UART2_DE_GPIO_Port, UNITREE_UART2_DE_Pin, GPIO_PIN_SET);
    } else if (huart == &huart3) {
        HAL_GPIO_WritePin(UNITREE_UART3_DE_GPIO_Port, UNITREE_UART3_DE_Pin, GPIO_PIN_SET);
    }
}

static void UnitreeMotor_RS485_SetRx(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        HAL_GPIO_WritePin(UNITREE_UART2_DE_GPIO_Port, UNITREE_UART2_DE_Pin, GPIO_PIN_RESET);
    } else if (huart == &huart3) {
        HAL_GPIO_WritePin(UNITREE_UART3_DE_GPIO_Port, UNITREE_UART3_DE_Pin, GPIO_PIN_RESET);
    }
}

#else

static void UnitreeMotor_RS485_SetTx(UART_HandleTypeDef *huart)
{
    (void)huart;
}

static void UnitreeMotor_RS485_SetRx(UART_HandleTypeDef *huart)
{
    (void)huart;
}

#endif

static void UnitreeMotor_EnablePowerIfNeeded(void)
{
#if defined(PWR_OUT1_GPIO_Port) && defined(PWR_OUT1_Pin)
    HAL_GPIO_WritePin(PWR_OUT1_GPIO_Port, PWR_OUT1_Pin, GPIO_PIN_SET);
#endif

#if defined(PWR_OUT2_GPIO_Port) && defined(PWR_OUT2_Pin)
    HAL_GPIO_WritePin(PWR_OUT2_GPIO_Port, PWR_OUT2_Pin, GPIO_PIN_SET);
#endif
}

/* Software CRC32, same polynomial as Unitree manual. */
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++) {
        xbit = 1UL << 31;
        data = ptr[i];

        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000U) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }

            if (data & xbit) {
                CRC32 ^= dwPolynomial;
            }

            xbit >>= 1;
        }
    }

    return CRC32;
}

void UnitreeMotor_Info_initialize(void)
{
    static uint8_t initialized = 0;
    if (initialized) {
        return;
    }
    initialized = 1;

    UnitreeMotor_EnablePowerIfNeeded();

    /*
     * Keep array index as logical motor number.
     * If your physical A1 IDs are different, modify only these two lines:
     *   UnitreeMotors_Info[1].Motor_ID
     *   UnitreeMotors_Info[2].Motor_ID
     */
    UnitreeMotors_Info[0].Motor_ID = 0x00;
    UnitreeMotors_Info[0].rxBuffer = rxBuffer0;
    UnitreeMotors_Info[0].control_uart = NULL;

    UnitreeMotors_Info[1].Motor_ID = 0x01;   /* motor1 on UART2 */
    UnitreeMotors_Info[1].rxBuffer = rxBuffer1;
    UnitreeMotors_Info[1].control_uart = &huart2;

    UnitreeMotors_Info[2].Motor_ID = 0x02;   /* motor2 on UART3 */
    UnitreeMotors_Info[2].rxBuffer = rxBuffer2;
    UnitreeMotors_Info[2].control_uart = &huart3;

    UnitreeMotors_Info[3].Motor_ID = 0x03;
    UnitreeMotors_Info[3].rxBuffer = rxBuffer3;
    UnitreeMotors_Info[3].control_uart = NULL;

    for (int motor_index = 0; motor_index < 4; motor_index++) {
        UnitreeMotors_Info[motor_index].torque = 0.0f;

        memset(&UnitreeMotors_Command[motor_index], 0, sizeof(UnitreeMotors_Command[motor_index]));
        UnitreeMotors_Command[motor_index].COMHead.start = UNITREE_HEADER_IDENTIFIER;
        UnitreeMotors_Command[motor_index].COMHead.motorID = (uint8_t)UnitreeMotors_Info[motor_index].Motor_ID;

        UnitreeMotors_Command[motor_index].MasterComdV3.mode = 0;
        UnitreeMotors_Command[motor_index].MasterComdV3.modifyBit_reserved = 0xFF;
    }

    UnitreeMotor_RS485_SetRx(&huart2);
    UnitreeMotor_RS485_SetRx(&huart3);
}

/*
 * Blocking send-receive transaction for one motor on one UART bus.
 * It is intended to be called by only one thread per UART.
 */
void UnitreeMotor_SendCommand(uint8_t motor_index, UART_HandleTypeDef *control_huart)
{
    if (motor_index >= 4 || control_huart == NULL) {
        return;
    }

    UnitreeMotor_BusCtx_t *bus = UnitreeMotor_GetBus(control_huart);
    if (bus == NULL) {
        return;
    }

    bus->active_motor_index = motor_index;
    /*
     * The thread that calls SendCommand is the one waiting for this RX.
     * During homing this may be mainTask; during normal communication it is
     * unitree_motor_uart2Task or unitree_motor_uart3Task.
     */
    bus->owner_thread = osThreadGetId();

    /* Clear old DMA RX state before a new transaction. */
    HAL_UART_AbortReceive(control_huart);

#if defined(__HAL_UART_CLEAR_OREFLAG)
    __HAL_UART_CLEAR_OREFLAG(control_huart);
#endif
#if defined(__HAL_UART_CLEAR_FEFLAG)
    __HAL_UART_CLEAR_FEFLAG(control_huart);
#endif
#if defined(__HAL_UART_CLEAR_NEFLAG)
    __HAL_UART_CLEAR_NEFLAG(control_huart);
#endif

    while (__HAL_UART_GET_FLAG(control_huart, UART_FLAG_RXNE)) {
        volatile uint8_t garbage = (uint8_t)(control_huart->Instance->RDR);
        (void)garbage;
    }

    UnitreeMotors_Command[motor_index].COMHead.motorID =
        (uint8_t)UnitreeMotors_Info[motor_index].Motor_ID;

    /* Send packet is 34 bytes. CRC length is 7 uint32_t words. */
    UnitreeMotors_Command[motor_index].CRCdata =
        crc32_core((uint32_t *)(&UnitreeMotors_Command[motor_index]), 7);

    UnitreeMotor_RS485_SetTx(control_huart);

    if (HAL_UART_Transmit_DMA(control_huart,
                              (uint8_t *)&UnitreeMotors_Command[motor_index],
                              UNITREE_TX_LEN) != HAL_OK) {
        UnitreeMotor_RS485_SetRx(control_huart);
        return;
    }

    /*
     * RX DMA is started in HAL_UART_TxCpltCallback after DE is switched back
     * to receive mode. Then HAL_UART_RxCpltCallback wakes up this thread.
     */
    uint32_t flags = osThreadFlagsWait(UNITREE_RX_DONE_FLAG,
                                       osFlagsWaitAny,
                                       UNITREE_UART_TIMEOUT_MS);

    if ((flags & UNITREE_RX_DONE_FLAG) == 0U) {
        bus->timeout_cnt++;
        HAL_UART_AbortReceive(control_huart);
        UnitreeMotor_RS485_SetRx(control_huart);
        return;
    }

    UnitreeMotor_ReceiveCommand(UnitreeMotors_Info[motor_index].rxBuffer, motor_index);
}

void UnitreeMotor_ReceiveCommand(uint8_t *rxBuffer, uint8_t motor_index)
{
    if (rxBuffer == NULL || motor_index >= 4) {
        return;
    }

    UnitreeMotor_A1_Status_t *temp_signal_ptr = (UnitreeMotor_A1_Status_t *)rxBuffer;

    /* RX packet is 78 bytes. CRC length is 18 uint32_t words. */
    if (crc32_core((uint32_t *)rxBuffer, 18) == temp_signal_ptr->CRCdata) {
        correct++;

        UnitreeMotors_Status[motor_index] = *temp_signal_ptr;

        int32_t current_raw = temp_signal_ptr->ServoComdV3.Motor_Pos;

        if (!motor_first_sync[motor_index]) {
            last_raw_pos[motor_index] = current_raw;
            motor_accumulated_counts[motor_index] = current_raw;
            motor_first_sync[motor_index] = 1;
            return;
        }

        int32_t diff = current_raw - last_raw_pos[motor_index];

        if (diff > 8192) {
            diff -= 16384;
        } else if (diff < -8192) {
            diff += 16384;
        }

        motor_accumulated_counts[motor_index] += diff;
        last_raw_pos[motor_index] = current_raw;

        int16_t T_raw = temp_signal_ptr->ServoComdV3.T_torque;
        int16_t W_raw = temp_signal_ptr->ServoComdV3.W_speed;
        int32_t P_raw = temp_signal_ptr->ServoComdV3.Motor_Pos;

        UnitreeMotors_Tor[motor_index] = T_raw;

        if (homing_done[motor_index]) {
            UnitreeMotors_Pos[motor_index] =
                motor_accumulated_counts[motor_index] - homing_offset_counts[motor_index];
        } else {
            UnitreeMotors_Pos[motor_index] = P_raw;
        }

        float tau_motor = ((float)T_raw) / TORQUE_SCALE;
        float vel_motor = ((float)W_raw) / VELOCITY_SCALE;
        float pos_motor = ((float)P_raw) / POSITION_SCALE;

        UnitreeMotors_State_SI[motor_index].tau_out_nm    = tau_motor * GEAR_RATIO;
        UnitreeMotors_State_SI[motor_index].vel_out_rad_s = vel_motor / GEAR_RATIO;
        UnitreeMotors_State_SI[motor_index].pos_out_rad   = pos_motor / GEAR_RATIO;

        if (homing_done[motor_index]) {
            int32_t relative_counts =
                motor_accumulated_counts[motor_index] - homing_offset_counts[motor_index];

            UnitreeMotors_TotalPos[motor_index] =
                ((float)relative_counts / 16384.0f * 2.0f * 3.1415926f) / GEAR_RATIO;
        } else {
            UnitreeMotors_TotalPos[motor_index] = 0.0f;
        }
    } else {
        mistaken++;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UnitreeMotor_BusCtx_t *bus = UnitreeMotor_GetBus(huart);

    if (bus != NULL) {
        bus->tx_done_cnt++;

        UnitreeMotor_RS485_SetRx(huart);

        uint8_t motor_index = bus->active_motor_index;
        if (motor_index < 4) {
            HAL_UART_Receive_DMA(huart, UnitreeMotors_Info[motor_index].rxBuffer, UNITREE_RX_LEN);
        }
        return;
    }

    /*
     * If other modules also use UART TX callbacks, put their callback here.
     * Example:
     *   Comm_UART_Tx_Finish_Callback(huart);
     */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UnitreeMotor_BusCtx_t *bus = UnitreeMotor_GetBus(huart);

    if (bus != NULL) {
        bus->rx_done_cnt++;
        if (bus->owner_thread != NULL) {
            osThreadFlagsSet(bus->owner_thread, UNITREE_RX_DONE_FLAG);
        }
        return;
    }

    if (huart == &huart5) {
        controller_Reveive(RxTemp);
        HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxTemp, 1);
    }
}

void UnitreeMotor_SetOutputMode(int32_t motor_index, Unitree_Motor_Output_Mode_e mode)
{
    if (motor_index < 0 || motor_index >= 4) {
        return;
    }

    if (UNITREE_MOTOR_ON == mode) {
        UnitreeMotors_Command[motor_index].MasterComdV3.mode = 10;
    } else {
        UnitreeMotors_Command[motor_index].MasterComdV3.mode = 0;
    }
}

void UnitreeMotor_SetCurrentRaw(uint8_t motor, int16_t T_raw)
{
    if (motor >= 4) {
        return;
    }

    if (T_raw == 0) {
        UnitreeMotors_Command[motor].MasterComdV3.mode = 0;
    } else {
        UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    }

    UnitreeMotors_Command[motor].MasterComdV3.t_torque = T_raw;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = 0;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kP       = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kW       = 0;
}

void UnitreeMotor_SetTorque(uint8_t motor, float tau_out_nm)
{
    if (motor >= 4) {
        return;
    }

    tau_out_nm = clampf_local(tau_out_nm, -TAU_OUT_MAX_NM, TAU_OUT_MAX_NM);

    float tau_motor = tau_out_nm / GEAR_RATIO;
    float T_cmd_f = tau_motor * TORQUE_SCALE;

    T_cmd_f = clampf_local(T_cmd_f,
                           -TORQUE_CMD_MAX * TORQUE_SCALE,
                            TORQUE_CMD_MAX * TORQUE_SCALE);

    int16_t T_cmd = (int16_t)T_cmd_f;

    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = T_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = 0;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kP = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kW = 0;
}

void UnitreeMotor_SetPosition(uint8_t motor, float pos_out_rad, float kp, float kd)
{
    if (motor >= 4) {
        return;
    }

    pos_out_rad = clampf_local(pos_out_rad, -POS_OUT_MAX_RAD, POS_OUT_MAX_RAD);

    float pos_motor = pos_out_rad * GEAR_RATIO;
    int32_t Pos_cmd = (int32_t)(pos_motor * POSITION_SCALE);

    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = 0;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = 0;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = Pos_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.kP = (uint16_t)(kp);
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd);
}

void UnitreeMotor_SetVelocity(uint8_t motor, float vel_out_rad_s, float kd)
{
    if (motor >= 4) {
        return;
    }

    vel_out_rad_s = clampf_local(vel_out_rad_s, -VEL_OUT_MAX_RAD, VEL_OUT_MAX_RAD);

    float vel_motor = vel_out_rad_s * GEAR_RATIO;
    float W_cmd_f = vel_motor * VELOCITY_SCALE;

    W_cmd_f = clampf_local(W_cmd_f,
                           -VELOCITY_CMD_MAX * VELOCITY_SCALE,
                            VELOCITY_CMD_MAX * VELOCITY_SCALE);

    int16_t W_cmd = (int16_t)W_cmd_f;

    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = 0;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = W_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kP = 0;
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd);
}

void UnitreeMotor_SetMixed(uint8_t motor, float tau_out_nm, float vel_out_rad_s, float pos_out_rad, float kp, float kd)
{
    if (motor >= 4) {
        return;
    }

    tau_out_nm    = clampf_local(tau_out_nm,    -TAU_OUT_MAX_NM, TAU_OUT_MAX_NM);
    vel_out_rad_s = clampf_local(vel_out_rad_s, -VEL_OUT_MAX_RAD, VEL_OUT_MAX_RAD);
    pos_out_rad   = clampf_local(pos_out_rad,   -POS_OUT_MAX_RAD, POS_OUT_MAX_RAD);

    float tau_motor = tau_out_nm / GEAR_RATIO;
    float vel_motor = vel_out_rad_s * GEAR_RATIO;
    float pos_motor = pos_out_rad * GEAR_RATIO;

    int16_t T_cmd = (int16_t)(tau_motor * TORQUE_SCALE);
    int16_t W_cmd = (int16_t)(vel_motor * VELOCITY_SCALE);
    int32_t P_cmd = (int32_t)(pos_motor * POSITION_SCALE);

    UnitreeMotors_Command[motor].MasterComdV3.mode = 10;
    UnitreeMotors_Command[motor].MasterComdV3.t_torque = T_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.w_speed  = W_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.pos      = P_cmd;
    UnitreeMotors_Command[motor].MasterComdV3.kP = (uint16_t)(kp);
    UnitreeMotors_Command[motor].MasterComdV3.kW = (uint16_t)(kd);
}

void UnitreeMotor_Homing_All(void)
{
    uint32_t start_tick = osKernelGetTickCount();

    uint8_t stable_cnt1 = 0;
    uint8_t stable_cnt2 = 0;

    float last_pos1 = UnitreeMotors_State_SI[1].pos_out_rad;
    float last_pos2 = UnitreeMotors_State_SI[2].pos_out_rad;

    UnitreeMotor_SetVelocity(1, homing_vel[1], HOMING_KD);
    UnitreeMotor_SetVelocity(2, homing_vel[2], HOMING_KD);

    UnitreeMotor_SendCommand(1, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(2, &huart3);
    osDelay(2);

    while (1) {
        float curr_pos1 = UnitreeMotors_State_SI[1].pos_out_rad;
        float curr_vel1 = UnitreeMotors_State_SI[1].vel_out_rad_s;
        float delta1 = curr_pos1 - last_pos1;
        last_pos1 = curr_pos1;

        if (fabsf(delta1) < HOMING_POS_DELTA_RAD_TH &&
            fabsf(curr_vel1) < HOMING_VEL_RAD_TH) {
            stable_cnt1++;
        } else {
            stable_cnt1 = 0;
        }

        float curr_pos2 = UnitreeMotors_State_SI[2].pos_out_rad;
        float curr_vel2 = UnitreeMotors_State_SI[2].vel_out_rad_s;
        float delta2 = curr_pos2 - last_pos2;
        last_pos2 = curr_pos2;

        if (fabsf(delta2) < HOMING_POS_DELTA_RAD_TH &&
            fabsf(curr_vel2) < HOMING_VEL_RAD_TH) {
            stable_cnt2++;
        } else {
            stable_cnt2 = 0;
        }

        if (stable_cnt1 >= HOMING_STABLE_CNT &&
            stable_cnt2 >= HOMING_STABLE_CNT) {
            break;
        }

        if ((osKernelGetTickCount() - start_tick) > HOMING_TIMEOUT_MS) {
            break;
        }

        UnitreeMotor_SendCommand(1, &huart2);
        osDelay(2);
        UnitreeMotor_SendCommand(2, &huart3);

        osDelay(10);
    }

    UnitreeMotor_SetVelocity(1, 0.0f, 2.0f);
    UnitreeMotor_SetVelocity(2, 0.0f, 2.0f);

    UnitreeMotor_SendCommand(1, &huart2);
    osDelay(2);
    UnitreeMotor_SendCommand(2, &huart3);
    osDelay(2);

    homing_offset_counts[1] = motor_accumulated_counts[1];
    homing_offset_counts[2] = motor_accumulated_counts[2];

    UnitreeMotors_TotalPos[1] = 0.0f;
    UnitreeMotors_TotalPos[2] = 0.0f;

    homing_done[1] = 1;
    homing_done[2] = 1;
}

/* Thread 1: motor1 on UART2 */
void unitree_motor_uart2Task(void *argument)
{
    (void)argument;

    UnitreeMotor_Info_initialize();
    s_bus2.owner_thread = osThreadGetId();
    UnitreeMotor_RS485_SetRx(&huart2);

    while (1) {
        if (homing_done[1]) {
            UnitreeMotor_SendCommand(1, &huart2);
        }
        osDelay(UNITREE_COMM_PERIOD_MS);
    }
}

/* Thread 2: motor2 on UART3 */
void unitree_motor_uart3Task(void *argument)
{
    (void)argument;

    UnitreeMotor_Info_initialize();
    s_bus3.owner_thread = osThreadGetId();
    UnitreeMotor_RS485_SetRx(&huart3);

    while (1) {
        if (homing_done[2]) {
            UnitreeMotor_SendCommand(2, &huart3);
        }
        osDelay(UNITREE_COMM_PERIOD_MS);
    }
}

