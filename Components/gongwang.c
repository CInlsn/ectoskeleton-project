#include "gongwang.h"

#include "usart.h"
#include <string.h>

typedef struct {
    GongWangBusId_t bus_id;
    UART_HandleTypeDef *huart;
    __ALIGNED(32) uint8_t dma_rx_buf[GONGWANG_RX_DMA_BUF_SIZE];
    uint8_t angle_inited;
    float last_abs_rad;
    int32_t turn_count;
    uint32_t rx_ok_count;
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    uint32_t restart_count;
} GongWangBusContext_t;

volatile GongWangAbsState_t g_gongwang_abs_state[GONGWANG_BUS_COUNT];
volatile GongWangHipYaw_t hip_yaw[GONGWANG_BUS_COUNT];

static const uint8_t s_gongwang_abs_bits[GONGWANG_BUS_COUNT] = {
    GONGWANG_ABS_BITS_UART2,
    GONGWANG_ABS_BITS_UART3
};

GongWangBusContext_t s_gongwang_bus[GONGWANG_BUS_COUNT] = {
    {
        .bus_id = GONGWANG_BUS_UART2,
        .huart = &huart2
    },
    {
        .bus_id = GONGWANG_BUS_UART3,
        .huart = &huart3
    }
};

static GongWangBusContext_t *GongWang_GetContextByBus(GongWangBusId_t bus_id)
{
    if (bus_id >= GONGWANG_BUS_COUNT) {
        return NULL;
    }

    return &s_gongwang_bus[bus_id];
}

static GongWangBusContext_t *GongWang_GetContextByHandle(UART_HandleTypeDef *huart)
{
    uint32_t i;

    for (i = 0U; i < GONGWANG_BUS_COUNT; i++) {
        if (s_gongwang_bus[i].huart == huart) {
            return &s_gongwang_bus[i];
        }
    }

    return NULL;
}

static void GongWang_DCacheInvalidate(void *addr, uint32_t len)
{
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    uintptr_t end;
    uint32_t size;

    start = ((uintptr_t)addr) & ~(GONGWANG_CACHE_LINE_SIZE - 1U);
    end = (uintptr_t)addr + len;
    size = (uint32_t)((end - start + (GONGWANG_CACHE_LINE_SIZE - 1U)) &
                      ~(GONGWANG_CACHE_LINE_SIZE - 1U));

    if (size != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)size);
    }
#else
    (void)addr;
    (void)len;
#endif
}

static uint8_t GongWang_CalcCrc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc;
    uint16_t i;

    crc = 0U;
    for (i = 0U; i < len; i++) {
        crc ^= data[i];
    }

    return crc;
}

static uint32_t GongWang_GetAbsValidMask(GongWangBusId_t bus_id)
{
    uint8_t bits;

    if (bus_id >= GONGWANG_BUS_COUNT) {
        return 0xFFFFFFFFUL;
    }

    bits = s_gongwang_abs_bits[bus_id];
    if (bits >= 32U) {
        return 0xFFFFFFFFUL;
    }

    return (1UL << bits) - 1UL;
}

static float GongWang_GetCountsPerTurn(GongWangBusId_t bus_id)
{
    uint8_t bits;

    if (bus_id >= GONGWANG_BUS_COUNT) {
        return 1.0f;
    }

    bits = s_gongwang_abs_bits[bus_id];
    return (float)(1UL << bits);
}

static void GongWang_StartReceiveBus(GongWangBusContext_t *ctx)
{
    HAL_StatusTypeDef status;

    if ((ctx == NULL) || (ctx->huart == NULL)) {
        return;
    }

    (void)HAL_UART_AbortReceive(ctx->huart);
    __HAL_UART_CLEAR_FLAG(ctx->huart,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF |
                          UART_CLEAR_FEF |
                          UART_CLEAR_PEF |
                          UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(ctx->huart, UART_RXDATA_FLUSH_REQUEST);

    status = HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart,
                                          ctx->dma_rx_buf,
                                          GONGWANG_RX_DMA_BUF_SIZE);
    if (status == HAL_OK) {
        if (ctx->huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
        }
        GongWang_DCacheInvalidate(ctx->dma_rx_buf, GONGWANG_RX_DMA_BUF_SIZE);
        ctx->restart_count++;
    } else {
        ctx->rx_error_count++;
        g_gongwang_abs_state[ctx->bus_id].online = 0U;
    }
}

static void GongWang_SendAbsCommand(GongWangBusContext_t *ctx)
{
    uint8_t tx_buf[1];
    HAL_StatusTypeDef status;

    if ((ctx == NULL) || (ctx->huart == NULL)) {
        return;
    }

    /* Tamagawa 0x02 request frame is a single CF byte without request CRC. */
    tx_buf[0] = GONGWANG_CMD_ABS;

    status = HAL_UART_Transmit(ctx->huart, tx_buf, sizeof(tx_buf), GONGWANG_UART_TIMEOUT_MS);
    if (status != HAL_OK) {
        ctx->tx_error_count++;
    }
}

static void GongWang_ParseAbsFrame(GongWangBusContext_t *ctx, const uint8_t *data, uint16_t len)
{
    volatile GongWangAbsState_t *state;
    float abs_rad;
    float delta;
    uint8_t crc_calc;

    if ((ctx == NULL) || (data == NULL) || (len != GONGWANG_ABS_FRAME_LEN)) {
        if (ctx != NULL) {
            ctx->rx_error_count++;
            g_gongwang_abs_state[ctx->bus_id].online = 0U;
        }
        return;
    }

    if (data[0] != GONGWANG_CMD_ABS) {
        ctx->rx_error_count++;
        g_gongwang_abs_state[ctx->bus_id].online = 0U;
        return;
    }

    crc_calc = GongWang_CalcCrc8(data, (uint16_t)(len - 1U));
    if (crc_calc != data[len - 1U]) {
        ctx->rx_error_count++;
        g_gongwang_abs_state[ctx->bus_id].crc_ok = 0U;
        g_gongwang_abs_state[ctx->bus_id].online = 0U;
        return;
    }

    state = &g_gongwang_abs_state[ctx->bus_id];
    state->bus_id = ctx->bus_id;
    state->sf = data[1];
    state->ea0 = (uint8_t)((data[1] >> 4) & 0x01U);
    state->ea1 = (uint8_t)((data[1] >> 5) & 0x01U);
    state->ca0 = (uint8_t)((data[1] >> 6) & 0x01U);
    state->ca1 = (uint8_t)((data[1] >> 7) & 0x01U);
    state->crc_ok = 1U;
    state->online = 1U;
    state->abs_raw = ((uint32_t)data[2]) |
                     ((uint32_t)data[3] << 8) |
                     ((uint32_t)data[4] << 16);
    state->abs_raw &= GongWang_GetAbsValidMask(ctx->bus_id);
    abs_rad = ((float)state->abs_raw) * GONGWANG_TWO_PI / GongWang_GetCountsPerTurn(ctx->bus_id);
    state->abs_rad = abs_rad;

    if (ctx->angle_inited == 0U) {
        ctx->turn_count = 0;
        ctx->angle_inited = 1U;
    } else {
        delta = abs_rad - ctx->last_abs_rad;

        if (delta < -GONGWANG_PI) {
            ctx->turn_count++;
        } else if (delta > GONGWANG_PI) {
            ctx->turn_count--;
        }
    }

    ctx->last_abs_rad = abs_rad;
    state->turn_count = ctx->turn_count;
    state->total_rad = ((float)ctx->turn_count * GONGWANG_TWO_PI) + abs_rad;
    state->tick_ms = HAL_GetTick();
    state->update_count++;

    hip_yaw[ctx->bus_id].pos = state->abs_rad;
    hip_yaw[ctx->bus_id].totalpos = state->total_rad;
    hip_yaw[ctx->bus_id].turn_count = state->turn_count;
    hip_yaw[ctx->bus_id].online = state->online;
    hip_yaw[ctx->bus_id].abs_raw = state->abs_raw;

    ctx->rx_ok_count++;
}

void GongWang_Init(void)
{
    uint32_t i;

    memset((void *)g_gongwang_abs_state, 0, sizeof(g_gongwang_abs_state));
    memset((void *)hip_yaw, 0, sizeof(hip_yaw));

    for (i = 0U; i < GONGWANG_BUS_COUNT; i++) {
        memset(s_gongwang_bus[i].dma_rx_buf, 0, sizeof(s_gongwang_bus[i].dma_rx_buf));
        s_gongwang_bus[i].rx_ok_count = 0U;
        s_gongwang_bus[i].rx_error_count = 0U;
        s_gongwang_bus[i].tx_error_count = 0U;
        s_gongwang_bus[i].restart_count = 0U;
        s_gongwang_bus[i].angle_inited = 0U;
        s_gongwang_bus[i].last_abs_rad = 0.0f;
        s_gongwang_bus[i].turn_count = 0;
        g_gongwang_abs_state[i].bus_id = s_gongwang_bus[i].bus_id;
        GongWang_StartReceiveBus(&s_gongwang_bus[i]);
    }
}

void GongWang_Task(void *argument)
{
    (void)argument;

    while (1) {
        GongWang_SendAbsCommand(&s_gongwang_bus[GONGWANG_BUS_UART2]);
        GongWang_SendAbsCommand(&s_gongwang_bus[GONGWANG_BUS_UART3]);
        osDelay(2);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    GongWangBusContext_t *ctx;

    ctx = GongWang_GetContextByHandle(huart);
    if (ctx == NULL) {
        return;
    }

    if ((Size == 0U) || (Size > GONGWANG_RX_DMA_BUF_SIZE)) {
        ctx->rx_error_count++;
        g_gongwang_abs_state[ctx->bus_id].online = 0U;
        GongWang_StartReceiveBus(ctx);
        return;
    }

    GongWang_DCacheInvalidate(ctx->dma_rx_buf, Size);
    GongWang_ParseAbsFrame(ctx, ctx->dma_rx_buf, Size);
    GongWang_StartReceiveBus(ctx);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    GongWangBusContext_t *ctx;

    ctx = GongWang_GetContextByHandle(huart);
    if (ctx == NULL) {
        return;
    }

    ctx->rx_error_count++;
    g_gongwang_abs_state[ctx->bus_id].online = 0U;
    GongWang_StartReceiveBus(ctx);
}
