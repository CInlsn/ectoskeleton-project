#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"
#include "pid.h"
#include <math.h>

/*
 * main_control.c for dual UART Unitree communication:
 *
 *   motor1(array index 1) -> USART2-RS485 communication task
 *   motor2(array index 2) -> USART3-RS485 communication task
 *
 * This file only updates command variables:
 *   UnitreeMotor_SetCurrentRaw(1, cmd1);
 *   UnitreeMotor_SetCurrentRaw(2, cmd2);
 *
 * Do NOT call UnitreeMotor_SendCommand() periodically here.
 * Actual sending is done by:
 *   unitree_motor_uart2Task()
 *   unitree_motor_uart3Task()
 */

volatile int motor_flag = 0;
static uint32_t last_tick = 0;
int calf_flag;

control_mode_e control_mode = EMPTY;
pos_set_t pos_set[3];
float vel_set[3];
float toq_set[3];
motor_pd_t motor_pd[3];
PID pid_pos[3];
//PID pid_vel[3];
//PID pid_toq[3];

#define HOLD_POS_DEADBAND_RAD   0.01f    // 小于这个误差认为已经到位，避免来回抖
#define HOLD_RAW_LIMIT_1        1500.0f   // motor1 最大 raw 力矩
#define HOLD_RAW_LIMIT_2        1500.0f   // motor2 最大 raw 力矩
#define HOLD_RAW_MIN_1          40.0f     // motor1 最小回正 raw 力矩
#define HOLD_RAW_MIN_2          40.0f     // motor2 最小回正 raw 力矩

static float get_raw_limit(uint8_t motor)
{
    if (motor == 1) return HOLD_RAW_LIMIT_1;
    if (motor == 2) return HOLD_RAW_LIMIT_2;
    return 0.0f;
}

static float get_raw_min(uint8_t motor)
{
    if (motor == 1) return HOLD_RAW_MIN_1;
    if (motor == 2) return HOLD_RAW_MIN_2;
    return 0.0f;
}

/*
 * pid_raw: PID_calc 输出的 raw 力矩
 * pos_err: 目标位置 - 当前实际位置
 */
static int16_t process_hold_torque(uint8_t motor, float pid_raw, float pos_err)
{
    float limit = get_raw_limit(motor);
    float min_torque = get_raw_min(motor);

    /* 1. 误差很小，认为已经回到位，直接不给力，防止目标点附近抖动 */
    if (fabsf(pos_err) < HOLD_POS_DEADBAND_RAD) {
        return 0;
    }

    /* 2. 先限幅，防止腿接地时力矩过大震动 */
    if (pid_raw > limit) {
        pid_raw = limit;
    }
    else if (pid_raw < -limit) {
        pid_raw = -limit;
    }

    /* 3. 加最小回正力矩：有误差但 PID 输出太小，就补到最小值 */
    if (pid_raw > 0.0f && pid_raw < min_torque) {
        pid_raw = min_torque;
    }
    else if (pid_raw < 0.0f && pid_raw > -min_torque) {
        pid_raw = -min_torque;
    }
    else if (pid_raw == 0.0f) {
        /*
         * 理论上有误差但 PID 输出为 0 时，用误差方向给一个最小回正力矩。
         * 如果发现方向反了，就在 UnitreeMotor_SetCurrentRaw() 处给对应电机加负号。
         */
        pid_raw = (pos_err > 0.0f) ? min_torque : -min_torque;
    }

    return (int16_t)pid_raw;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15) {
        uint32_t now = HAL_GetTick();

        if (now - last_tick < 200) {
            return;
        }

        last_tick = now;
        motor_flag = !motor_flag;

        /*
         * 按键切换状态后，下一次进入控制时让目标位置从当前位置开始。
         * 注意：不要在中断里调用复杂函数，这里只改 flag。
         */
    }
}

void sync_pos_set_to_actual_all(void)
{
    for (int i = 1; i < 3; i++) {
        pos_set[i].last_pos = UnitreeMotors_TotalPos[i];
        pos_set[i].postemp_set = UnitreeMotors_TotalPos[i];
        vel_set[i] = 0.0f;
        toq_set[i] = 0.0f;
    }
}

static control_mode_e last_mode = EMPTY;

void mode_swtich(void)
{
    if (controller.SW[4] == UP) {
        control_mode = CALF_MODE;
    }
    else if (controller.SW[4] == DOWN) {
        control_mode = TOE_MODE;
    }
    else {
        control_mode = EMPTY;
    }

    if (control_mode != last_mode) {
        sync_pos_set_to_actual_all();
        last_mode = control_mode;
    }
}

void mode_control(void)
{
    if (control_mode == EMPTY) {
        UnitreeMotor_SetCurrentRaw(1, 0);
        UnitreeMotor_SetCurrentRaw(2, 0);
    }

    else if (control_mode == TOE_MODE) {
        /*
         * 当前双电机通信版本只控制 motor1 和 motor2。
         * TOE_MODE 原本预留给 motor0，这里先让 motor1/2 零力矩。
         */
        UnitreeMotor_SetCurrentRaw(1, 0);
        UnitreeMotor_SetCurrentRaw(2, 0);
    }

    else if (control_mode == CALF_MODE) {
        /*
         * CALF_MODE：motor1 和 motor2 控制小腿。
         * UnitreeMotors_TotalPos[1/2] 由两个 UART 通信线程持续刷新。
         */

        pos_set[1].postemp_set =
            pos_set[1].last_pos
            - PITCH_RATIO * controller.channel[1]
            + ROLL_RATIO  * controller.channel[3];

        pos_set[2].postemp_set =
            pos_set[2].last_pos
            + PITCH_RATIO * controller.channel[1]
            + ROLL_RATIO  * controller.channel[3];

        // 如有机械限位，建议在这里打开
        // val_limit(&pos_set[1].postemp_set, LOW_LIM1, UP_LIM1);
        // val_limit(&pos_set[2].postemp_set, LOW_LIM2, UP_LIM2);

        float err1 = pos_set[1].postemp_set - UnitreeMotors_TotalPos[1];
        float err2 = pos_set[2].postemp_set - UnitreeMotors_TotalPos[2];

        toq_set[1] = PID_calc(&pid_pos[1],
                              UnitreeMotors_TotalPos[1],
                              pos_set[1].postemp_set);

        toq_set[2] = PID_calc(&pid_pos[2],
                              UnitreeMotors_TotalPos[2],
                              pos_set[2].postemp_set);

        pos_set[1].last_pos = pos_set[1].postemp_set;
        pos_set[2].last_pos = pos_set[2].postemp_set;

        /* 加最小回正力矩 + 输出限幅 */
        int16_t cmd1 = process_hold_torque(1, toq_set[1], err1);
        int16_t cmd2 = process_hold_torque(2, toq_set[2], err2);

        /*
         * 如果实测发现某个电机方向反了，就只在这里改符号：
         *   UnitreeMotor_SetCurrentRaw(1, -cmd1);
         *   UnitreeMotor_SetCurrentRaw(2, -cmd2);
         */
        UnitreeMotor_SetCurrentRaw(1, cmd1);
        UnitreeMotor_SetCurrentRaw(2, cmd2);
    }

    else {
        UnitreeMotor_SetCurrentRaw(1, 0);
        UnitreeMotor_SetCurrentRaw(2, 0);
    }
}

void motor_init(void)
{
    UnitreeMotor_SetOutputMode(1, UNITREE_MOTOR_ON);
    UnitreeMotor_SetOutputMode(2, UNITREE_MOTOR_ON);

    pos_set[1].last_pos = 0.0f;
    pos_set[2].last_pos = 0.0f;
    pos_set[1].postemp_set = 0.0f;
    pos_set[2].postemp_set = 0.0f;

    vel_set[1] = 0.0f;
    vel_set[2] = 0.0f;
    toq_set[1] = 0.0f;
    toq_set[2] = 0.0f;

    motor_pd[1].kp = 2500.0f;
    motor_pd[1].kd = 3000.0f;
    motor_pd[2].kp = 2500.0f;
    motor_pd[2].kd = 3000.0f;

    PID_init(&pid_pos[1], 1600, 5, 25, 10, 0.01, 200, 0, 1);
    PID_init(&pid_pos[2], 1600, 5, 25, 10, 0.01, 200, 0, 1);

    UnitreeMotor_SetCurrentRaw(1, 0);
    UnitreeMotor_SetCurrentRaw(2, 0);
}

void mainTask(void *argument)
{
    (void)argument;
    UnitreeMotor_Info_initialize();
    motor_init();

    UnitreeMotor_Homing_All();
    sync_pos_set_to_actual_all();
    last_mode = EMPTY;

    while (1) {
        if (motor_flag) {
            mode_swtich();
            mode_control();
        }
        else {
            /*
             * 这里只是写命令缓存。
             * 实际发给电机由 unitree_motor_uart2Task 和 unitree_motor_uart3Task 完成。
             */
            UnitreeMotor_SetCurrentRaw(1, 0);
            UnitreeMotor_SetCurrentRaw(2, 0);
            sync_pos_set_to_actual_all();
            last_mode = EMPTY;
        }

        osDelay(3);
    }
}
