/**
 ******************************************************************************
 * @file    sts3032.h
 * @brief   FEETECH STS3032 串行总线舵机驱动（半双工单总线，逐字节复刻 SCS/SMS_STS 协议）
 *
 * 硬件说明（最终网表确认）：
 *      +3.3V ──R6(1k)── DATA ──┬── CN1-3 / CN2-3（两颗 STS3032 并联在同一总线）
 *                              └── R7(1k) ── PB6 (USART1_TX)
 *   · USART1 配置为 Half-Duplex(single_wire_mode)：TX/RX 内部短接，只用 PB6
 *   · PB6 配置为复用开漏(AF_OD)：拉低靠 MCU，拉高靠 R6 上拉
 *   · 因此【不需要任何方向控制引脚】——原设计的 Q2/PB7 已删除
 *   · 波特率 1 Mbps（STS3032 出厂默认，8N1）
 *
 * 对应原版：
 *   Servo_STS3032.cpp（SCS / SCSerial / SMS_STS 三层）
 *   + SCServo 库的 SCS::syncWrite / SCS::Read / SCS::Ping / SCS::Ack
 *   协议字节序、校验和算法、超时机制均与原版保持一致。
 ******************************************************************************
 */

#ifndef __STS3032_H__
#define __STS3032_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

/**
 * @brief STS3032 舵机驱动实例（对应原版 SMP_STS sms_sts）
 */
typedef struct {
    UART_HandleTypeDef *huart;          /**< 半双工 UART 句柄（&huart1） */
    uint8_t  end;                       /**< 0 = 小端(默认)，1 = 大端（对应 SCS::End） */
    uint8_t  level;                     /**< 1 = 除广播外所有指令都等应答（对应 SCS::Level） */
    uint8_t  error;                     /**< 最近一次应答里的错误码（对应 SCS::Error） */
    uint32_t io_timeout_ms;             /**< 单字节收发超时（ms） */
    uint32_t error_count;               /**< 通信失败累计次数（调试观察用） */
} STS3032_t;

/*==============================================================================
 * 全局实例（定义在 sts3032.c 中）
 *============================================================================*/

/** @brief 舵机驱动实例 */
extern STS3032_t sts;

/*==============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief  初始化舵机总线（挂载 UART、清零状态）
 * @note   必须在 MX_USART1_UART_Init() 之后调用。
 *         注意：本函数的 UART 是单线半双工，收发方向由硬件自动切换，
 *         软件只需直接调用 HAL_UART_Transmit / HAL_UART_Receive。
 */
void STS3032_Init(void);

/*==============================================================================
 * 底层：字节收发（对应 SCSerial::writeSCS / readSCS / rFlushSCS）
 *============================================================================*/

/**
 * @brief  向总线写若干字节（对应 SCSerial::writeSCS(nDat, nLen)）
 * @param  data 数据指针
 * @param  len  长度
 * @return 实际写出的字节数；失败返回 0
 */
int STS3032_Write(const uint8_t *data, uint16_t len);

/**
 * @brief  向总线写一个字节
 */
int STS3032_WriteByte(uint8_t data);

/**
 * @brief  从总线读一个字节（对应 SCSerial::readSCS）
 * @param  data 输出缓冲
 * @return 1 = 成功，0 = 超时
 */
int STS3032_ReadByte(uint8_t *data);

/**
 * @brief  读取若干字节
 * @param  data 输出缓冲
 * @param  len  期望长度
 * @return 实际读到的字节数
 */
int STS3032_Read(uint8_t *data, uint16_t len);

/**
 * @brief  清空接收缓冲（对应 SCSerial::rFlushSCS）
 * @note   发指令前清掉总线上残留的半截数据，避免应答解析错位。
 */
void STS3032_FlushRx(void);

/*==============================================================================
 * 协议层（对应 SCS 类的公有方法）
 *============================================================================*/

/**
 * @brief  16 位数拆分为两个 8 位（对应 SCS::Host2SCS）
 * @param  data_l 输出：低位
 * @param  data_h 输出：高位
 * @param  data   输入 16 位数
 */
void STS3032_Host2SCS(uint8_t *data_l, uint8_t *data_h, uint16_t data);

/**
 * @brief  16 位数合成（对应 SCS::SCS2Host）
 */
uint16_t STS3032_SCS2Host(uint8_t data_l, uint8_t data_h);

/**
 * @brief  Ping 指令（对应 SCS::Ping）
 * @param  id 舵机 ID；0xFE = 广播
 * @return 舵机 ID；超时返回 -1
 * @note   这是测试单总线通断最直接的指令，测试例程会用到。
 */
int STS3032_Ping(uint8_t id);

/**
 * @brief  读指令（对应 SCS::Read）
 * @param  id       舵机 ID
 * @param  mem_addr 内存表地址
 * @param  data     输出缓冲
 * @param  len      读取长度
 * @return 读到的字节数；失败返回 0
 */
int STS3032_ReadReg(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t len);

/**
 * @brief  读 1 字节寄存器（对应 SCS::readByte）
 * @return 值；失败返回 -1
 */
int STS3032_ReadByteReg(uint8_t id, uint8_t mem_addr);

/**
 * @brief  读 2 字节寄存器（对应 SCS::readWord）
 * @return 值；失败返回 -1
 */
int STS3032_ReadWordReg(uint8_t id, uint8_t mem_addr);

/**
 * @brief  普通写指令（对应 SCS::genWrite）
 * @param  id       舵机 ID
 * @param  mem_addr 内存表地址
 * @param  data     待写数据
 * @param  len      数据长度
 * @return 1 = 成功（且收到正确应答），0 = 失败
 */
int STS3032_GenWrite(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t len);

/**
 * @brief  同步写指令（对应 SCS::syncWrite）
 * @param  id       舵机 ID 数组
 * @param  id_num   舵机数量
 * @param  mem_addr 内存表起始地址
 * @param  data     数据块（每颗舵机 n_len 字节，连续排列）
 * @param  n_len    每颗舵机的数据长度
 * @note   同步写是广播指令（ID = 0xFE），舵机不应答，因此无返回值。
 *         这是腿部控制每周期都在跑的指令，必须尽量轻量。
 */
int STS3032_SyncWrite(uint8_t id[], uint8_t id_num, uint8_t mem_addr,
                      uint8_t *data, uint8_t n_len);

/*==============================================================================
 * 应用层（对应 SMS_STS::SyncWritePosEx）
 *============================================================================*/

/**
 * @brief  同步写位置+速度+加速度（对应 SMS_STS::SyncWritePosEx）
 * @param  id       舵机 ID 数组
 * @param  id_num   舵机数量
 * @param  position 目标位置数组（可负：负值表示反向，内部会置 bit15）
 * @param  speed    目标速度数组（可传 NULL）
 * @param  acc      目标加速度数组（可传 NULL）
 *
 * @note   数据布局（每颗舵机 7 字节，与原版完全一致）：
 *           byte0     : 加速度 ACC
 *           byte1~2   : 目标位置（低、高）
 *           byte3~4   : 运行时间 0（位置+速度模式下忽略）
 *           byte5~6   : 目标速度（低、高）
 *         注意 position 数组会被就地修改（负值取绝对值并置 bit15），
 *         这与原版行为一致。
 */
void STS3032_SyncWritePosEx(uint8_t id[], uint8_t id_num, int16_t position[],
                            uint16_t speed[], uint8_t acc[]);

/**
 * @brief  便捷接口：读写单颗舵机当前位置
 * @param  id 舵机 ID
 * @return 当前位置（0~4095）；失败返回 -1
 * @note   测试例程"舵机角度读写"使用。
 */
int STS3032_ReadPosition(uint8_t id);

/**
 * @brief  便捷接口：设置单颗舵机的目标位置
 * @param  id       舵机 ID
 * @param  position 目标位置（0~4095）
 * @param  speed    目标速度（0 = 最大速度）
 * @param  acc      加速度（0 = 不设加速度）
 * @return 1 = 成功，0 = 失败
 * @note   测试例程"舵机角度读写"使用。
 */
int STS3032_SetPosition(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);

#endif /* __STS3032_H__ */
