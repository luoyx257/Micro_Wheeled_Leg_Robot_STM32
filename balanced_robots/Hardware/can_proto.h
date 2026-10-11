/**
 ******************************************************************************
 * @file    can_proto.h
 * @brief   TJA1050 CAN 通信驱动 + 上位机指令解析 + 机器人状态上报
 *
 * 硬件说明：
 *   TJA1050T（U1，SOP-8，5V 供电）
 *     U1-1 TXD ← PA12 (CAN1_TX)
 *     U1-4 RXD → PA11 (CAN1_RX)
 *     U1-6 CANL / U1-7 CANH → H2 接口，R11(120Ω) 终端电阻经 H2-3 跳线可选接入
 *
 * 位时序（已在 can.c 配好）：
 *   APB1 = 36MHz，Prescaler = 4，BS1 = 10TQ，BS2 = 5TQ，SJW = 1TQ
 *   位时间 = 1 + 10 + 5 = 16 TQ → 36MHz/(4×16) = 500 kbps，采样点 68.75%
 *
 * ★ 改动点说明（相对原版 ESP32 的 WiFi/WebSocket 方案）：
 *   原版通过 WebSocket 传 JSON（RobotProtocol::parseBasic 解析
 *   dir/height/roll/linear/angular/stable/joy_x/joy_y 共 8 个字段）。
 *   本移植把同样的 8 个字段原封不动搬进 CAN 报文 0x200 的 8 个数据字节里，
 *   字段顺序与含义完全一致，因此上位机侧的协议语义不变。
 ******************************************************************************
 */

#ifndef __CAN_PROTO_H__
#define __CAN_PROTO_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 报文 ID 定义
 *============================================================================*/

/** @brief 0x200：上位机 → 机器人，控制指令（8 字节，字段与 WebSocket JSON 一一对应） */
#define CAN_ID_CMD_BASIC        CAN_ID_COMMAND          /* 0x200 */

/** @brief 0x100：机器人 → 上位机，第一帧（姿态 + 速度） */
#define CAN_ID_TX_ATTITUDE      CAN_ID_TELEMETRY        /* 0x100 */

/** @brief 0x101：机器人 → 上位机，第二帧（电流 + 电池 + 状态） */
#define CAN_ID_TX_STATUS        (CAN_ID_TELEMETRY + 1U) /* 0x101 */

/*==============================================================================
 * 数据结构
 *============================================================================*/

/**
 * @brief 控制指令（承载原版 WebSocket JSON 的全部字段）
 *
 * 0x200 报文数据布局（与原版 robot.cpp::parseBasic 的字段一一对应）：
 *   Byte0 : dir     机器人运动状态（QR_State_t：0=FORWARD … 5=JUMP）
 *   Byte1 : height  机身高度（原版 doc["height"]）
 *   Byte2 : roll    期望横滚（int8，有符号）
 *   Byte3 : linear  前后平移量（int8，有符号）
 *   Byte4 : angular 旋转量（int8，有符号）
 *   Byte5 : stable  1 = 使能平衡输出（原版 doc["stable"] → wrobot.go）
 *   Byte6 : joy_x   左右摇杆（int8，有符号，对应 wrobot.joyx）
 *   Byte7 : joy_y   前后摇杆（int8，有符号，对应 wrobot.joyy）
 */
typedef struct {
    uint8_t dir;            /**< 运动状态（QR_State_t） */
    uint8_t height;         /**< 机身高度 */
    int8_t  roll;           /**< 期望横滚 */
    int8_t  linear;         /**< 前后平移 */
    int8_t  angular;        /**< 旋转量 */
    uint8_t stable;         /**< 1 = 使能输出 */
    int8_t  joy_x;          /**< 左右摇杆 → wrobot.joyx */
    int8_t  joy_y;          /**< 前后摇杆 → wrobot.joyy */
    uint8_t updated;        /**< 1 = 收到过至少一帧新指令（供上层判断） */
} CanCommand_t;

/**
 * @brief 姿态帧（0x100）数据布局，共 8 字节
 */
typedef struct {
    int16_t angle_y;        /**< [0..1] 平衡角（pitch，×100，单位 0.01°） */
    int16_t gyro_y;         /**< [2..3] 平衡角速度（×100，单位 0.01°/s） */
    int16_t speed_left;     /**< [4..5] 左轮转速（×100，单位 0.01 rad/s） */
    int16_t speed_right;    /**< [6..7] 右轮转速（×100，单位 0.01 rad/s） */
} CanTxAttitude_t;

/**
 * @brief 状态帧（0x101）数据布局，共 8 字节
 */
typedef struct {
    uint16_t current_left;  /**< [0..1] 左电机电流（×1000，单位 mA） */
    uint16_t current_right; /**< [2..3] 右电机电流（×1000，单位 mA） */
    uint16_t battery_mv;    /**< [4..5] 电池电压（mV） */
    uint8_t  status;        /**< [6] bit0=IMU在线 bit1=左编码器在线 bit2=右编码器在线
                                        bit3=左电机过流保护 bit4=右电机过流保护
                                        bit5=低电量 bit6=舵机总线异常 bit7=保留 */
    uint8_t  error_count;   /**< [7] 舵机通信错误累计次数（低 8 位） */
} CanTxStatus_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  初始化 CAN（配置过滤器 + 启动 + 开启接收中断）
 * @note   必须在 MX_CAN_Init() 之后调用。
 */
void CAN_Proto_Init(void);

/**
 * @brief  获取最新收到的控制指令
 * @return 控制指令指针（只读使用）
 */
const CanCommand_t *CAN_Proto_GetCommand(void);

/**
 * @brief  清除"新指令"标志（上层处理完指令后调用）
 */
void CAN_Proto_ClearUpdated(void);

/**
 * @brief  上报姿态帧（0x100）
 * @param  angle_y      平衡角（度）
 * @param  gyro_y       平衡角速度（度/秒）
 * @param  speed_left   左轮转速（rad/s）
 * @param  speed_right  右轮转速（rad/s）
 */
void CAN_Proto_SendAttitude(float angle_y, float gyro_y,
                            float speed_left, float speed_right);

/**
 * @brief  上报状态帧（0x101）
 * @param  cur_left     左电机电流（A）
 * @param  cur_right    右电机电流（A）
 * @param  battery_v    电池电压（V）
 * @param  status       状态位
 * @param  servo_err    舵机通信错误计数
 */
void CAN_Proto_SendStatus(float cur_left, float cur_right, float battery_v,
                          uint8_t status, uint32_t servo_err);

/**
 * @brief  上报周期调度 + 上报（由 1kHz 控制循环每周期调用）
 * @param  angle_y      平衡角（度）
 * @param  gyro_y       平衡角速度（度/秒）
 * @param  speed_left   左轮转速（rad/s）
 * @param  speed_right  右轮转速（rad/s）
 * @param  cur_left     左电机电流（A）
 * @param  cur_right    右电机电流（A）
 * @param  battery_v    电池电压（V）
 * @param  status       状态位（见 CanTxStatus_t 注释）
 * @param  servo_err    舵机通信错误计数
 *
 * @note   为避免每 1ms 都发帧占用总线，内部按 CAN_TELEMETRY_PERIOD_MS 节流：
 *         只有累计时间到点时才真正发送 0x100 + 0x101 两帧。
 *         这是"机器人 → 上位机"的唯一出口，便于统一管理总线占用率。
 */
void CAN_Proto_TelemetryTick(float angle_y, float gyro_y,
                             float speed_left, float speed_right,
                             float cur_left, float cur_right, float battery_v,
                             uint8_t status, uint32_t servo_err);

/**
 * @brief  打包/解包辅助：把 int16（×100 定点）写入大端
 */
void CAN_PackInt16BE(uint8_t *buf, uint16_t offset, int16_t value);
void CAN_PackUint16BE(uint8_t *buf, uint16_t offset, uint16_t value);
int16_t CAN_UnpackInt16BE(const uint8_t *buf, uint16_t offset);
uint16_t CAN_UnpackUint16BE(const uint8_t *buf, uint16_t offset);

/**
 * @brief  CAN 接收中断回调的宿主函数
 * @note   在 stm32f1xx_it.c 的 USB_HP_CAN1_TX_IRQHandler /
 *         USB_LP_CAN1_RX0_IRQHandler 中调用，或在 HAL_CAN_RxFifo0MsgPendingCallback
 *         中调用。本工程用后者（HAL 回调）。
 * @param  hcan CAN 句柄
 */
void CAN_Proto_RxCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  最近一次收到指令的毫秒时间戳（用于判断上位机是否掉线）
 */
uint32_t CAN_Proto_LastRxTime(void);

#endif /* __CAN_PROTO_H__ */
