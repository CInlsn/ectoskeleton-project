#include "main_control.h"
#include "stdlib.h"
#include "func_lib.h"
#include <string.h>

extern Joint_Motor_t motor[15];
dm_motor_info_t dm_motor_info[16] = {0};

extern osThreadId_t xout_can1_Handle;
extern osThreadId_t xout_can2_Handle;
extern osThreadAttr_t xout_can1_attributes;
extern osThreadAttr_t xout_can2_attributes;

int xout_task_started = 0;

static const uint8_t usb_state_expected_ids[USB_STATE_RX_MOTOR_NUM] = {1, 3, 5, 7, 9};
float usb_remote_pos[16] = {0.0f};
float usb_remote_vel[16] = {0.0f};
static volatile uint8_t usb_state_rx_enabled = 0U;
static volatile uint32_t usb_state_last_rx_tick = 0U;
static uint8_t usb_state_rx_stream[USB_STATE_RX_FRAME_LEN * 2] = {0};
static uint16_t usb_state_rx_stream_len = 0U;

#define USB_STATE_RX_MOTOR_OFFSET 4U
#define USB_STATE_RX_MOTOR_STRIDE 9U

typedef struct
{
    uint8_t side_id;
    uint8_t motor_id[USB_STATE_RX_MOTOR_NUM];
    float position[USB_STATE_RX_MOTOR_NUM];
    float velocity[USB_STATE_RX_MOTOR_NUM];
} usb_state_rx_frame_t;

typedef struct
{
    float pos;
    float vel;
    uint8_t inited;
} usb_smoothed_state_t;

static uint8_t usb_calc_checksum(const uint8_t *data, uint16_t len);
static uint8_t USB_State_FindMotorSlot(uint8_t id, uint8_t *slot);
static void USB_State_StreamDrop(uint16_t count);
static void USB_State_ProcessStream(void);
static uint8_t USB_State_ParseFrame(const uint8_t *frame_bytes, usb_state_rx_frame_t *frame);
static void USB_State_HandleFrame(const usb_state_rx_frame_t *frame);
static float USB_Bilateral_Clamp(float value, float min_value, float max_value);
static float USB_Bilateral_ComputeTorque(float local_pos,
                                         float local_vel,
                                         float remote_pos,
                                         float remote_vel,
                                         float kp);
static void USB_Bilateral_ApplyMotor1(float torque);
static void USB_Bilateral_ApplyMotor3(float torque);
static void USB_Bilateral_ApplyMotor5(float torque);
static void USB_Bilateral_ApplyMotor7(float torque);
static void USB_Bilateral_FreeAll(void);
static uint8_t USB_BilateralControl_Task(void);
static void USB_Bilateral_UpdateSmoothedState(usb_smoothed_state_t *state,
                                              float raw_pos,
                                              float raw_vel,
                                              uint32_t age_ms);

static usb_smoothed_state_t usb_remote_state1 = {0};
static usb_smoothed_state_t usb_remote_state3 = {0};
static usb_smoothed_state_t usb_remote_state5 = {0};
static usb_smoothed_state_t usb_remote_state7 = {0};

void motor_info_init(){		
		dm_motor_info[1].con_parameter.Kd =16;
		dm_motor_info[1].con_parameter.Kp =28;
		dm_motor_info[1].con_parameter.Tq =0;
		dm_motor_info[1].motor_info.pos = 0;
		dm_motor_info[1].motor_info.vel = 0;
	
		dm_motor_info[3].con_parameter.Kd =16;
		dm_motor_info[3].con_parameter.Kp =28;
		dm_motor_info[3].con_parameter.Tq =0;
		dm_motor_info[3].motor_info.pos = 0;
		dm_motor_info[3].motor_info.vel = 0;
	
		dm_motor_info[5].con_parameter.Kd =16;
		dm_motor_info[5].con_parameter.Kp =30;
		dm_motor_info[5].con_parameter.Tq =0;
		dm_motor_info[5].motor_info.pos = 0;
		dm_motor_info[5].motor_info.vel = 0;
	
		dm_motor_info[7].con_parameter.Kd =16;
		dm_motor_info[7].con_parameter.Kp =30;
		dm_motor_info[7].con_parameter.Tq =0;
		dm_motor_info[7].motor_info.pos = 0;
		dm_motor_info[7].motor_info.vel = 0;
		
		dm_motor_info[9].con_parameter.Kd =12;
		dm_motor_info[9].con_parameter.Kp =25;
		dm_motor_info[9].con_parameter.Tq =0;
		dm_motor_info[9].motor_info.pos = 0;
		dm_motor_info[9].motor_info.vel = 0;
}

void motor_enable(){
//    static int enabled = 0;
//    if(enabled) return;

    if(motor[1].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[1].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[3].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[3].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[5].para.state ==0){
        enable_motor_mode(&hfdcan2,motor[5].para.id,MIT_MODE);
		osDelay (5);
		}
    if(motor[7].para.state ==0){
        enable_motor_mode(&hfdcan2,motor[7].para.id,MIT_MODE);
		osDelay (5);
		}
		if(motor[9].para.state ==0){
        enable_motor_mode(&hfdcan1,motor[9].para.id,MIT_MODE);
		osDelay (5);
		}
//    enabled = 1;
}

static uint8_t USB_State_FindMotorSlot(uint8_t id, uint8_t *slot)
{
    uint8_t i;

    for (i = 0; i < USB_STATE_RX_MOTOR_NUM; i++)
    {
        if (usb_state_expected_ids[i] == id)
        {
            if (slot != NULL)
            {
                *slot = i;
            }
            return 1U;
        }
    }

    return 0U;
}

static void USB_State_StreamDrop(uint16_t count)
{
    if (count >= usb_state_rx_stream_len)
    {
        usb_state_rx_stream_len = 0U;
        return;
    }

    memmove(usb_state_rx_stream,
            &usb_state_rx_stream[count],
            usb_state_rx_stream_len - count);
    usb_state_rx_stream_len -= count;
}

static void USB_State_ProcessStream(void)
{
    usb_state_rx_frame_t frame;

    while (usb_state_rx_stream_len >= 2U)
    {
        uint16_t head_index = 0U;

        while ((uint16_t)(head_index + 1U) < usb_state_rx_stream_len)
        {
            if ((usb_state_rx_stream[head_index] == USB_STATE_RX_HEAD1) &&
                (usb_state_rx_stream[head_index + 1U] == USB_STATE_RX_HEAD2))
            {
                break;
            }

            head_index++;
        }

        if (head_index > 0U)
        {
            USB_State_StreamDrop(head_index);
            continue;
        }

        if (usb_state_rx_stream_len < USB_STATE_RX_FRAME_LEN)
        {
            return;
        }

        if (USB_State_ParseFrame(usb_state_rx_stream, &frame))
        {
            USB_State_HandleFrame(&frame);
            USB_State_StreamDrop(USB_STATE_RX_FRAME_LEN);
            continue;
        }

        USB_State_StreamDrop(1U);
    }
}

static uint8_t USB_State_ParseFrame(const uint8_t *frame_bytes, usb_state_rx_frame_t *frame)
{
    uint8_t i;
    uint8_t checksum;
    uint8_t seen_mask = 0U;

    if ((frame_bytes == NULL) || (frame == NULL))
    {
        return 0U;
    }

    if ((frame_bytes[0] != USB_STATE_RX_HEAD1) ||
        (frame_bytes[1] != USB_STATE_RX_HEAD2) ||
        (frame_bytes[2] != USB_STATE_RX_CMD))
    {
        return 0U;
    }

    frame->side_id = frame_bytes[3];
    if ((frame->side_id != USB_SIDE_ID_LEFT) &&
        (frame->side_id != USB_SIDE_ID_RIGHT))
    {
        return 0U;
    }

    checksum = usb_calc_checksum(&frame_bytes[2], 1U + USB_STATE_RX_DATA_LEN);
    if (checksum != frame_bytes[USB_STATE_RX_FRAME_LEN - 1U])
    {
        return 0U;
    }

    for (i = 0; i < USB_STATE_RX_MOTOR_NUM; i++)
    {
        uint16_t motor_offset = USB_STATE_RX_MOTOR_OFFSET +
                                ((uint16_t)i * USB_STATE_RX_MOTOR_STRIDE);
        uint8_t slot;
        uint8_t bit;

        frame->motor_id[i] = frame_bytes[motor_offset];
        if (!USB_State_FindMotorSlot(frame->motor_id[i], &slot))
        {
            return 0U;
        }

        bit = (uint8_t)(1U << slot);
        if ((seen_mask & bit) != 0U)
        {
            return 0U;
        }

        seen_mask |= bit;
        memcpy(&frame->position[i], &frame_bytes[motor_offset + 1U], sizeof(frame->position[i]));
        memcpy(&frame->velocity[i], &frame_bytes[motor_offset + 5U], sizeof(frame->velocity[i]));
    }

    return 1U;
}

static void USB_State_HandleFrame(const usb_state_rx_frame_t *frame)
{
    uint8_t i;

    if (frame->side_id != USB_CURRENT_SIDE_ID)
    {
        return;
    }

    __disable_irq();
    for (i = 0; i < USB_STATE_RX_MOTOR_NUM; i++)
    {
        usb_remote_pos[frame->motor_id[i]] = frame->position[i];
        usb_remote_vel[frame->motor_id[i]] = frame->velocity[i];
    }
    usb_state_last_rx_tick = HAL_GetTick();
    usb_state_rx_enabled = 1U;
    __enable_irq();
}

static float USB_Bilateral_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static float USB_Bilateral_ComputeTorque(float local_pos,
                                         float local_vel,
                                         float remote_pos,
                                         float remote_vel,
                                         float kp)
{
    float torque;

    torque = -USB_BILATERAL_KV * (local_vel - remote_vel)
             - (USB_BILATERAL_KD + USB_BILATERAL_PE) * local_vel
             - kp * (local_pos - remote_pos);

    return USB_Bilateral_Clamp(torque,
                               -USB_BILATERAL_TORQUE_LIMIT,
                               USB_BILATERAL_TORQUE_LIMIT);
}

static void USB_Bilateral_ApplyMotor1(float torque)
{
    dm_motor_info_t cmd;

    cmd = dm_motor_info[1];
    cmd.con_parameter.Kp = 0.0f;
    cmd.con_parameter.Kd = 0.0f;
    cmd.con_parameter.Tq = USB_Bilateral_Clamp(torque, T_MIN, T_MAX);
    cmd.motor_info.pos = 0.0f;
    cmd.motor_info.vel = 0.0f;

    mit_ctrl(&hfdcan1, 1, &cmd);
}

static void USB_Bilateral_ApplyMotor3(float torque)
{
    dm_motor_info_t cmd;

    cmd = dm_motor_info[3];
    cmd.con_parameter.Kp = 0.0f;
    cmd.con_parameter.Kd = 0.0f;
    cmd.con_parameter.Tq = USB_Bilateral_Clamp(torque, T_MIN, T_MAX);
    cmd.motor_info.pos = 0.0f;
    cmd.motor_info.vel = 0.0f;

    mit_ctrl(&hfdcan1, 3, &cmd);
}

static void USB_Bilateral_ApplyMotor5(float torque)
{
    dm_motor_info_t cmd;

    cmd = dm_motor_info[5];
    cmd.con_parameter.Kp = 0.0f;
    cmd.con_parameter.Kd = 0.0f;
    cmd.con_parameter.Tq = USB_Bilateral_Clamp(torque, T_MIN, T_MAX);
    cmd.motor_info.pos = 0.0f;
    cmd.motor_info.vel = 0.0f;

    mit_ctrl(&hfdcan2, 5, &cmd);
}

static void USB_Bilateral_ApplyMotor7(float torque)
{
    dm_motor_info_t cmd;

    cmd = dm_motor_info[7];
    cmd.con_parameter.Kp = 0.0f;
    cmd.con_parameter.Kd = 0.0f;
    cmd.con_parameter.Tq = USB_Bilateral_Clamp(torque, T_MIN, T_MAX);
    cmd.motor_info.pos = 0.0f;
    cmd.motor_info.vel = 0.0f;

    mit_ctrl(&hfdcan2, 7, &cmd);
}

static void USB_Bilateral_FreeAll(void)
{
    USB_Bilateral_ApplyMotor1(0.0f);
    usb_remote_state1.inited = 0U;
    usb_remote_state3.inited = 0U;
    usb_remote_state5.inited = 0U;
    usb_remote_state7.inited = 0U;
    mit_free(&hfdcan1, &motor[3]);
    mit_free(&hfdcan2, &motor[5]);
    mit_free(&hfdcan2, &motor[7]);
    mit_free(&hfdcan1, &motor[9]);
}

static void USB_Bilateral_UpdateSmoothedState(usb_smoothed_state_t *state,
                                              float raw_pos,
                                              float raw_vel,
                                              uint32_t age_ms)
{
    float predict_age_s;
    float target_pos;
    float target_vel;

    if (age_ms > USB_BILATERAL_RX_PERIOD_MS)
    {
        age_ms = USB_BILATERAL_RX_PERIOD_MS;
    }

    predict_age_s = 0.001f * (float)age_ms;
    target_pos = raw_pos + raw_vel * predict_age_s;
    target_vel = raw_vel;

    if ((state == NULL) || (state->inited == 0U))
    {
        if (state != NULL)
        {
            state->pos = target_pos;
            state->vel = target_vel;
            state->inited = 1U;
        }
        return;
    }

    state->pos += USB_BILATERAL_SMOOTH_ALPHA * (target_pos - state->pos);
    state->vel += USB_BILATERAL_SMOOTH_ALPHA * (target_vel - state->vel);
}
    float torque1;
    float torque3;
    float torque5;
    float torque7;
static uint8_t USB_BilateralControl_Task(void)
{
    uint8_t enabled;
    uint32_t last_rx_tick;
    uint32_t age_ms;
    float remote_pos1_snapshot;
    float remote_vel1_snapshot;
    float remote_pos3_snapshot;
    float remote_vel3_snapshot;
    float remote_pos5_snapshot;
    float remote_vel5_snapshot;
    float remote_pos7_snapshot;
    float remote_vel7_snapshot;
    float local_pos1;
    float local_vel1;
    float local_pos3;
    float local_vel3;
    float local_pos5;
    float local_vel5;
    float local_pos7;
    float local_vel7;
    float local_pitch13;
    float local_pitch_vel13;
    float remote_pitch13;
    float remote_pitch_vel13;
    float pitch_torque13;


    while (1)
    {
        __disable_irq();
        enabled = usb_state_rx_enabled;
        last_rx_tick = usb_state_last_rx_tick;
        remote_pos1_snapshot = usb_remote_pos[1];
        remote_vel1_snapshot = usb_remote_vel[1];
        remote_pos3_snapshot = usb_remote_pos[3];
        remote_vel3_snapshot = usb_remote_vel[3];
        remote_pos5_snapshot = usb_remote_pos[5];
        remote_vel5_snapshot = usb_remote_vel[5];
        remote_pos7_snapshot = usb_remote_pos[7];
        remote_vel7_snapshot = usb_remote_vel[7];
        __enable_irq();

        if (!enabled)
        {
            return 0U;
        }

        age_ms = HAL_GetTick() - last_rx_tick;
        if (age_ms <= USB_BILATERAL_RX_TIMEOUT_MS)
        {
            break;
        }

        __disable_irq();
        if ((usb_state_rx_enabled != 0U) &&
            (usb_state_last_rx_tick == last_rx_tick))
        {
            usb_state_rx_enabled = 0U;
            __enable_irq();
            return 0U;
        }
        __enable_irq();
    }

    USB_Bilateral_UpdateSmoothedState(&usb_remote_state1,
                                      remote_pos1_snapshot,
                                      remote_vel1_snapshot,
                                      age_ms);
    USB_Bilateral_UpdateSmoothedState(&usb_remote_state3,
                                      remote_pos3_snapshot,
                                      remote_vel3_snapshot,
                                      age_ms);
    USB_Bilateral_UpdateSmoothedState(&usb_remote_state5,
                                      remote_pos5_snapshot,
                                      remote_vel5_snapshot,
                                      age_ms);
    USB_Bilateral_UpdateSmoothedState(&usb_remote_state7,
                                      remote_pos7_snapshot,
                                      remote_vel7_snapshot,
                                      age_ms);

    local_pos1 = motor[1].para.xout;
    local_vel1 = motor[1].para.vel;
    local_pos3 = motor[3].para.xout;
    local_vel3 = motor[3].para.vel;
    local_pos5 = motor[5].para.xout;
    local_vel5 = motor[5].para.vel;
    local_pos7 = motor[7].para.xout;
    local_vel7 = motor[7].para.vel;

    local_pitch13 = (-local_pos1 + local_pos3) * 0.5f;
    local_pitch_vel13 = (-local_vel1 + local_vel3) * 0.5f;
    remote_pitch13 = (-usb_remote_state1.pos + usb_remote_state3.pos) * 0.5f;
    remote_pitch_vel13 = (-usb_remote_state1.vel + usb_remote_state3.vel) * 0.5f;

    pitch_torque13 = USB_Bilateral_ComputeTorque(local_pitch13,
                                                 local_pitch_vel13,
                                                 remote_pitch13,
                                                 remote_pitch_vel13,
                                                 USB_BILATERAL_KP_MOTOR1);
    torque1 = -0.5f * pitch_torque13;
    torque3 = 0.5f * pitch_torque13;
    torque5 = USB_Bilateral_ComputeTorque(local_pos5,
                                          local_vel5,
                                          usb_remote_state5.pos,
                                          usb_remote_state5.vel,
                                          USB_BILATERAL_KP_MOTOR5);
    torque7 = USB_Bilateral_ComputeTorque(local_pos7,
                                          local_vel7,
                                          usb_remote_state7.pos,
                                          usb_remote_state7.vel,
                                          USB_BILATERAL_KP_MOTOR7);

    USB_Bilateral_ApplyMotor1(torque1);
    USB_Bilateral_ApplyMotor3(torque3);
    USB_Bilateral_ApplyMotor5(torque5);
    USB_Bilateral_ApplyMotor7(torque7);
    mit_free(&hfdcan1, &motor[9]);

    return 1U;
}

void USB_StateRx_ProcessBytes(const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        if (usb_state_rx_stream_len >= sizeof(usb_state_rx_stream))
        {
            USB_State_StreamDrop(1U);
        }

        usb_state_rx_stream[usb_state_rx_stream_len++] = buf[i];
        USB_State_ProcessStream();
    }
}

void mainTask(void *argument){
	for(int i = 0; i< 15;i+=1){
		joint_motor_init(&motor[i],i,MIT_MODE);
	}
	motor_info_init();
	motor_enable();
  if(!xout_task_started){
    xout_can1_Handle = osThreadNew(XoutTask_CAN1, NULL, &xout_can1_attributes);
    xout_can2_Handle = osThreadNew(XoutTask_CAN2, NULL, &xout_can2_attributes);
    xout_task_started = 1;
  }
	while(1){
///*    当你认为零点位置不对时，请用以下代码重设零点*/
//		dm_save_zero(&hfdcan1, 1);
//		dm_save_zero(&hfdcan1, 3);
//		dm_save_zero(&hfdcan2, 5);
//		dm_save_zero(&hfdcan2, 7);
//		dm_save_zero(&hfdcan1, 9);
		motor_enable();
		if (USB_BilateralControl_Task())
		{
			osDelay(2);
			continue;
		}
		USB_Bilateral_FreeAll();
		osDelay(2);
	}
}

void XoutTask_CAN1(void *argument)
{
    const uint16_t ids[] = {1, 3 , 9};
    const uint8_t num_ids = sizeof(ids)/sizeof(ids[0]);
    uint8_t idx = 0;

    for(;;){
        dm_read_xout(&hfdcan1, ids[idx]);  // 发给 CAN1
        idx++;
        if (idx >= num_ids) idx = 0;

        osDelay(2);  
    }
}

void XoutTask_CAN2(void *argument)
{
    const uint16_t ids[] = {5, 7};
    const uint8_t num_ids = sizeof(ids)/sizeof(ids[0]);
    uint8_t idx = 0;

    for(;;){
        dm_read_xout(&hfdcan2, ids[idx]);  // 发给 CAN2
        idx++;
        if (idx >= num_ids) idx = 0;

        osDelay(3); 
    }
}

/*----------------USB com begin-----------------*/

uint8_t usb_xout_tx_buf[USB_XOUT_FRAME_LEN];
static uint16_t usb_xout_seq = 0;

static uint8_t usb_calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum;
}

static void usb_put_u16_le(uint8_t *buf, uint16_t value)
{
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
}

static void usb_put_float_le(uint8_t *buf, float value)
{
    union
    {
        float f;
        uint8_t b[4];
    } u;

    u.f = value;

    buf[0] = u.b[0];
    buf[1] = u.b[1];
    buf[2] = u.b[2];
    buf[3] = u.b[3];
}

uint8_t USB_Send_All_Xout(void)
{
    const uint8_t ids[USB_XOUT_MOTOR_NUM] = {1, 3, 5, 7, 9};

    uint16_t index = 0;
    uint8_t i;
    uint8_t id;

    usb_xout_tx_buf[index++] = USB_XOUT_HEAD1;
    usb_xout_tx_buf[index++] = USB_XOUT_HEAD2;
    usb_xout_tx_buf[index++] = USB_XOUT_CMD;
    usb_xout_tx_buf[index++] = USB_XOUT_PAYLOAD_LEN;

    usb_xout_tx_buf[index++] = USB_XOUT_SIDE_ID;
    usb_put_u16_le(&usb_xout_tx_buf[index], usb_xout_seq);
    usb_xout_seq++;
    index += 2;

    for (i = 0; i < USB_XOUT_MOTOR_NUM; i++)
    {
        id = ids[i];

        usb_xout_tx_buf[index++] = id;
        usb_put_float_le(&usb_xout_tx_buf[index], motor[id].para.xout);
        index += 4;
    }

    usb_xout_tx_buf[index++] = usb_calc_checksum(&usb_xout_tx_buf[2], 2 + USB_XOUT_PAYLOAD_LEN);

//    if (!CDC_TxReady_HS())
//    {
//        return USBD_BUSY;
//    }

    return CDC_Transmit_HS(usb_xout_tx_buf, index);
}

void USB_Task(void *argument)
{
		osDelay(1000);
    while (1)
    {
        USB_Send_All_Xout();

        osDelay(1);
    }
}
/*----------------USB com end-----------------*/
