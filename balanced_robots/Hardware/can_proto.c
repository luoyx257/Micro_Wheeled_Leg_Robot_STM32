/**
 ******************************************************************************
 * @file    can_proto.c
 * @brief   TJA1050 CAN 通信驱动 + 指令解析 + 状态上报 实现
 ******************************************************************************
 */

#include "can_proto.h"
#include "can.h"
#include "hardware.h"
#include <string.h>

/*==============================================================================
 * 私有变量
 *============================================================================*/

/** @brief 最近一次收到的控制指令 */
static CanCommand_t s_command;

/** @brief 最近一次收到指令的时间戳（ms） */
static volatile uint32_t s_last_rx_time = 0U;

/** @brief 最近一次上报时间戳（ms） */
static uint32_t s_last_tx_time = 0U;

/** @brief 上报节流标志 */
static uint8_t s_telemetry_enable = 0U;

/*==============================================================================
 * 打包/解包辅助（CAN 报文统一使用大端）
 *============================================================================*/

void CAN_PackInt16BE(uint8_t *buf, uint16_t offset, int16_t value)
{
    buf[offset]     = (uint8_t)((uint16_t)value >> 8);
    buf[offset + 1] = (uint8_t)((uint16_t)value & 0xFFU);
}

void CAN_PackUint16BE(uint8_t *buf, uint16_t offset, uint16_t value)
{
    buf[offset]     = (uint8_t)(value >> 8);
    buf[offset + 1] = (uint8_t)(value & 0xFFU);
}

int16_t CAN_UnpackInt16BE(const uint8_t *buf, uint16_t offset)
{
    return (int16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

uint16_t CAN_UnpackUint16BE(const uint8_t *buf, uint16_t offset)
{
    return (uint16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

/*==============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief  发送一帧标准数据帧
 * @param  std_id 标准帧 ID（11 位）
 * @param  data   数据指针（必须 8 字节）
 * @return 1 = 成功，0 = 失败
 */
static uint8_t CAN_Proto_Send8(uint32_t std_id, const uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t            mailbox;
    uint32_t            start;

    if (data == NULL) {
        return 0U;
    }

    tx_header.StdId              = std_id;
    tx_header.ExtId              = 0U;
    tx_header.IDE                = CAN_ID_STD;      /* 标准帧（11 位 ID） */
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 等待空闲邮箱，超时则放弃本帧（不阻塞控制循环） */
    start = millis();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
        if ((millis() - start) > CAN_TX_TIMEOUT_MS) {
            return 0U;
        }
    }

    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)data, &mailbox) != HAL_OK) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  解析 0x200 控制指令（字段与原版 robot.cpp::parseBasic 一一对应）
 */
static void CAN_Proto_ParseCommand(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 8U) {
        return;     /* 长度不足，丢弃 */
    }

    s_command.dir      = data[0];
    s_command.height   = data[1];
    s_command.roll     = (int8_t)data[2];
    s_command.linear   = (int8_t)data[3];
    s_command.angular  = (int8_t)data[4];
    s_command.stable   = data[5];
    s_command.joy_x    = (int8_t)data[6];
    s_command.joy_y    = (int8_t)data[7];

    /* 方向字段做合法性检查（与原版 parseBasic 的 else 分支等价：
       未知方向一律按 STOP 处理） */
    if (s_command.dir > (uint8_t)JUMP) {
        s_command.dir = (uint8_t)STOP;
    }

    s_command.updated = 1U;
    s_last_rx_time    = millis();
}

/*==============================================================================
 * 接口实现
 *============================================================================*/

/**
 * @brief  初始化 CAN
 * @note   使用 16 位掩码列表模式 + FIFO0，只接收两条指令 ID，
 *         避免无关报文打断中断。
 */
void CAN_Proto_Init(void)
{
    CAN_FilterTypeDef filter;

    memset(&s_command, 0, sizeof(s_command));
    s_command.dir     = (uint8_t)STOP;
    s_command.height  = (uint8_t)WROBOT_DEFAULT_HEIGHT;
    s_command.updated = 0U;

    s_last_rx_time     = 0U;
    s_last_tx_time     = 0U;
    s_telemetry_enable = 1U;

    /* ---- 过滤器：只放行 0x200 / 0x201 两个 ID -------------------------------
     * FilterMode   : 32 位掩码模式（用 32 位 ID 寄存器，标准帧需左移 5 位）
     * FilterScale  : 32 位
     * FilterIdHigh : ID 高 16 位
     * FilterMaskId : 掩码，只比较高 11 位
     */
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((CAN_ID_COMMAND << 5) & 0xFFFFU);
    filter.FilterIdLow          = 0U;
    filter.FilterMaskIdHigh     = (uint16_t)((0x7FFU << 5) & 0xFFFFU);
    filter.FilterMaskIdLow      = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;      /* F103 只有 CAN1，此项无效但需赋值 */

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        /* 过滤器配置失败：不致命，继续启动 CAN */
    }

    /* 放宽一点：第二个过滤器放行 0x100~0x1FF（便于监听总线上的其他节点，调试用） */
    filter.FilterBank           = 1U;
    filter.FilterIdHigh         = (uint16_t)((CAN_ID_TELEMETRY << 5) & 0xFFFFU);
    filter.FilterMaskIdHigh     = (uint16_t)((0x700U << 5) & 0xFFFFU);
    (void)HAL_CAN_ConfigFilter(&hcan, &filter);

    (void)HAL_CAN_Start(&hcan);

    /* 开启 FIFO0 新报文中断 */
    (void)HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 确保 NVIC 中 CAN1_RX0 中断已使能。
       标准做法是在 CubeMX 里勾选 "CAN1 RX0 interrupts"（会生成到 can.c 的 MspInit 中），
       这里再补一次，避免漏勾导致 CAN 完全收不到帧。
       抢占优先级 0（最高），保证 CAN 帧不会因为控制循环而丢失。 */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/**
 * @brief  获取最新控制指令
 */
const CanCommand_t *CAN_Proto_GetCommand(void)
{
    return &s_command;
}

/**
 * @brief  清除"新指令"标志
 */
void CAN_Proto_ClearUpdated(void)
{
    s_command.updated = 0U;
}

/**
 * @brief  最近一次收到指令的时间戳
 */
uint32_t CAN_Proto_LastRxTime(void)
{
    return s_last_rx_time;
}

/**
 * @brief  FIFO0 收到报文的处理（由 HAL_CAN_RxFifo0MsgPendingCallback 调用）
 */
void CAN_Proto_RxCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];

    if (hcan_ptr == NULL) {
        return;
    }

    /* 一次中断可能积压多帧，全部取出来 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan_ptr, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA)) {
            continue;       /* 只处理标准数据帧 */
        }

        if (rx_header.StdId == CAN_ID_CMD_BASIC) {
            CAN_Proto_ParseCommand(rx_data, (uint8_t)rx_header.DLC);
        }
        /* 0x201 参数调试帧可在此扩展（PID 在线调参） */
    }
}

/**
 * @brief  上报姿态帧（0x100）
 */
void CAN_Proto_SendAttitude(float angle_y, float gyro_y,
                            float speed_left, float speed_right)
{
    uint8_t buf[8];

    /* 定点化：×100，范围 ±327.67，足够覆盖角度与转速 */
    CAN_PackInt16BE(buf, 0U, (int16_t)(angle_y * 100.0f));
    CAN_PackInt16BE(buf, 2U, (int16_t)(gyro_y * 100.0f));
    CAN_PackInt16BE(buf, 4U, (int16_t)(speed_left * 100.0f));
    CAN_PackInt16BE(buf, 6U, (int16_t)(speed_right * 100.0f));

    (void)CAN_Proto_Send8(CAN_ID_TX_ATTITUDE, buf);
}

/**
 * @brief  上报状态帧（0x101）
 */
void CAN_Proto_SendStatus(float cur_left, float cur_right, float battery_v,
                          uint8_t status, uint32_t servo_err)
{
    uint8_t buf[8];
    float   cl = cur_left;
    float   cr = cur_right;

    /* 电流可能为负（再生），取绝对值后以 mA 上报 */
    if (cl < 0.0f) { cl = -cl; }
    if (cr < 0.0f) { cr = -cr; }

    CAN_PackUint16BE(buf, 0U, (uint16_t)(cl * 1000.0f));
    CAN_PackUint16BE(buf, 2U, (uint16_t)(cr * 1000.0f));
    CAN_PackUint16BE(buf, 4U, (uint16_t)(battery_v * 1000.0f));
    buf[6] = status;
    buf[7] = (uint8_t)(servo_err & 0xFFU);

    (void)CAN_Proto_Send8(CAN_ID_TX_STATUS, buf);
}

/**
 * @brief  上报周期调度 + 上报（按 CAN_TELEMETRY_PERIOD_MS 节流）
 */
void CAN_Proto_TelemetryTick(float angle_y, float gyro_y,
                             float speed_left, float speed_right,
                             float cur_left, float cur_right, float battery_v,
                             uint8_t status, uint32_t servo_err)
{
    if (!s_telemetry_enable) {
        return;
    }

    /* 未到上报周期直接返回，避免每 1ms 都占用 CAN 总线 */
    if ((millis() - s_last_tx_time) < CAN_TELEMETRY_PERIOD_MS) {
        return;
    }
    s_last_tx_time = millis();

    CAN_Proto_SendAttitude(angle_y, gyro_y, speed_left, speed_right);
    CAN_Proto_SendStatus(cur_left, cur_right, battery_v, status, servo_err);
}

/**
 * @brief  HAL CAN 接收 FIFO0 回调（由 HAL 弱函数覆盖）
 * @note   放在本文件里，避免污染 CubeMX 生成的 can.c。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_Proto_RxCallback(hcan_ptr);
}
