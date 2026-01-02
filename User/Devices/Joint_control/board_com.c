#include "board_com.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* ===================== 配置 ===================== */
#define UART_RX_DMA_BUF_SIZE  256   // >= 2~4倍帧长
#define CACHE_LINE_SIZE       32U

__attribute__((aligned(32))) uint8_t uart_rx_dma_buf[UART_RX_DMA_BUF_SIZE];

static volatile uint16_t dma_last_pos = 0;

/* ===================== 帧解析状态机 ===================== */
static volatile uint8_t rx_flag = 0;
controller_mess_t rx_controller_data;

typedef enum {
    RX_WAIT_HEAD1 = 0,
    RX_WAIT_HEAD2,
    RX_COLLECT_FRAME
} rx_state_t;

static rx_state_t rx_state = RX_WAIT_HEAD1;
static uint8_t frame_buf[sizeof(controller_mess_t)];
static uint16_t frame_idx = 0;

static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static void parse_byte(uint8_t b)
{
    switch (rx_state)
    {
        case RX_WAIT_HEAD1:
            if (b == FRAME_HEAD1) {
                frame_buf[0] = b;
                frame_idx = 1;
                rx_state = RX_WAIT_HEAD2;
            }
            break;

        case RX_WAIT_HEAD2:
            if (b == FRAME_HEAD2) {
                frame_buf[1] = b;
                frame_idx = 2;
                rx_state = RX_COLLECT_FRAME;
            } else if (b == FRAME_HEAD1) {
                // AA AA... 继续当头
                frame_buf[0] = b;
                frame_idx = 1;
            } else {
                rx_state = RX_WAIT_HEAD1;
                frame_idx = 0;
            }
            break;

        case RX_COLLECT_FRAME:
            frame_buf[frame_idx++] = b;
            if (frame_idx >= sizeof(controller_mess_t))
            {
                uint8_t sum = calc_checksum(frame_buf, (uint16_t)sizeof(controller_mess_t) - 1U);
                if (sum == frame_buf[sizeof(controller_mess_t) - 1U])
                {
                    memcpy(&rx_controller_data, frame_buf, sizeof(controller_mess_t));
                    rx_flag = 1;
                }
                rx_state = RX_WAIT_HEAD1;
                frame_idx = 0;
            }
            break;

        default:
            rx_state = RX_WAIT_HEAD1;
            frame_idx = 0;
            break;
    }
}

/* ===================== DCache 工具：必须 32B 对齐 ===================== */
static inline void dcache_invalidate(void *addr, uint32_t len)
{
    uint32_t a   = ((uint32_t)addr) & ~(CACHE_LINE_SIZE - 1U);
    uint32_t end = (uint32_t)addr + len;
    uint32_t l   = (end - a + (CACHE_LINE_SIZE - 1U)) & ~(CACHE_LINE_SIZE - 1U);
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, (int32_t)l);
}

/* ===================== 启动 DMA + IDLE ===================== */
void board_com_rx_init_dma(void)
{
    rx_flag = 0;
    rx_state = RX_WAIT_HEAD1;
    frame_idx = 0;
    dma_last_pos = 0;

    // 清错误 + flush RX
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

    // 启动 ReceiveToIdle DMA
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE);

    // 如果返回忙/错，直接强制恢复（避免“HAL_OK 但没开始”的边界状态）
    if (st != HAL_OK)
    {
        HAL_UART_AbortReceive(&huart1);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE);
    }

    // 关 DMA half-transfer 中断（降低中断）
    if (huart1.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    // 启动后先 invalidate 整个 buffer（避免 cache 旧数据）
    dcache_invalidate(uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE);
}

/* ===================== Rx Event Callback：只用 NDTR 推 dma_pos，增量处理 ===================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size; // 不用 Size，避免语义歧义
    if (huart->Instance != USART1) return;
    if (huart->hdmarx == NULL) return;

    // 先 invalidate 全 buffer（最稳，256B很小，开销可接受）
    dcache_invalidate(uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE);

    // 当前写入位置 = BUF_SIZE - NDTR
    uint16_t dma_pos = UART_RX_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (dma_pos == dma_last_pos) return;

    if (dma_pos > dma_last_pos)
    {
        // 连续段 [dma_last_pos, dma_pos)
        for (uint16_t i = dma_last_pos; i < dma_pos; i++) {
            parse_byte(uart_rx_dma_buf[i]);
        }
    }
    else
    {
        // 环回： [dma_last_pos, end) + [0, dma_pos)
        for (uint16_t i = dma_last_pos; i < UART_RX_DMA_BUF_SIZE; i++) {
            parse_byte(uart_rx_dma_buf[i]);
        }
        for (uint16_t i = 0; i < dma_pos; i++) {
            parse_byte(uart_rx_dma_buf[i]);
        }
    }

    dma_last_pos = dma_pos;
}

/* ===================== UART 错误回调：清错误 + flush + 重启 ===================== */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    // 清 UART 错误
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

    // flush RX，防止残留导致立刻再次 ORE
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

    // 复位解析状态
    rx_state = RX_WAIT_HEAD1;
    frame_idx = 0;
    dma_last_pos = 0;

    // 强制中止当前 DMA RX（如果卡死）
    HAL_UART_AbortReceive(&huart1);

    // 重新启动
    board_com_rx_init_dma();
}

/* ===================== 轮询处理：从 rx_controller_data 取帧 ===================== */
void board_com_poll(void)
{
    if (rx_flag)
    {
        rx_flag = 0;
        // process_received_frame(&rx_controller_data);
    }
}

/* ===================== RTOS Task ===================== */
void comTask(void *argument)
{
    board_com_rx_init_dma();

    while (1)
    {
        board_com_poll();
        osDelay(1);
    }
}
