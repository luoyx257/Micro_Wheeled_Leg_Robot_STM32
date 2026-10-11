/**
 ******************************************************************************
 * @file    sts3032.c
 * @brief   FEETECH STS3032 串行总线舵机驱动实现（逐字节复刻 SCS/SMS_STS 协议）
 *
 * 协议帧格式（与原版 SCServo 库完全一致）：
 *
 *   主机 → 舵机（普通指令）：
 *     FF FF ID LEN INST P1 ... Pn CHECKSUM
 *     LEN      = 参数长度 + 2
 *     CHECKSUM = ~(ID + LEN + INST + P1 + ... + Pn)
 *
 *   舵机 → 主机（应答）：
 *     FF FF ID LEN ERR CHECKSUM       （LEN 固定为 2）
 *     CHECKSUM = ~(ID + LEN + ERR)
 *
 *   同步写（广播，无应答）：
 *     FF FF FE LEN 83 ADDR LEN_DATA
 *        [ID1 D1..Dn] [ID2 D1..Dn] ...
 *     CHECKSUM
 *     LEN = (n + 1) × id_num + 4
 *
 * ★ 改动点说明（相对原版 Servo_STS3032.cpp + SCServo 库）：
 *   1. 原版 SCSerial 通过 Arduino Stream(pSerial->write/read) 收发，
 *      本移植用 HAL_UART_Transmit / HAL_UART_Receive 直接操作单线半双工 UART。
 *   2. 原版 rFlushSCS() 是 while(pSerial->read()!=-1) 无限清空，
 *      本移植用"带超时的读干净"实现，避免总线安静时死循环。
 *   3. 协议字节、校验和、超时重试逻辑、位置负值置 bit15 的处理全部保持原样。
 ******************************************************************************
 */

#include "sts3032.h"
#include "usart.h"
#include "hardware.h"
#include <stddef.h>

/*==============================================================================
 * 全局实例
 *============================================================================*/

/** @brief 舵机驱动实例 */
STS3032_t sts = {
    .huart         = &huart1,
    .end           = 0U,           /* 小端（与原版默认一致） */
    .level         = 1U,           /* 除广播外都等应答（与原版默认一致） */
    .error         = 0U,
    .io_timeout_ms = SCS_IO_TIMEOUT_MS,
    .error_count   = 0U,
};

/*==============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief  初始化舵机总线
 * @note   UART 已由 CubeMX 配好（1Mbps / 8N1 / 单线半双工 / PB6 复用开漏），
 *         这里只需要做状态清零。
 */
void STS3032_Init(void)
{
    sts.error       = 0U;
    sts.error_count = 0U;
    STS3032_FlushRx();
}

/*==============================================================================
 * 底层：字节收发
 *============================================================================*/

/**
 * @brief  向总线写若干字节
 * @note   单线半双工模式下，HAL_UART_Transmit 会自动使能发送器、
 *         发完后自动释放总线，无需手动切换方向。
 */
int STS3032_Write(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    if (HAL_UART_Transmit(sts.huart, (uint8_t *)data, len, sts.io_timeout_ms + len) != HAL_OK) {
        return 0;
    }
    return (int)len;
}

/**
 * @brief  向总线写一个字节
 */
int STS3032_WriteByte(uint8_t data)
{
    return STS3032_Write(&data, 1U);
}

/**
 * @brief  从总线读一个字节（带超时）
 */
int STS3032_ReadByte(uint8_t *data)
{
    if (data == NULL) {
        return 0;
    }

    if (HAL_UART_Receive(sts.huart, data, 1U, sts.io_timeout_ms) != HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief  读取若干字节
 */
int STS3032_Read(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    if (HAL_UART_Receive(sts.huart, data, len, sts.io_timeout_ms + len) != HAL_OK) {
        return 0;
    }
    return (int)len;
}

/**
 * @brief  清空接收缓冲
 * @note   原版是 while(pSerial->read() != -1) 的无限循环；
 *         这里改为"最多清 SCS_BUF_SIZE 次且每次超时很短"，
 *         既能清干净残留字节，又不会在总线安静时卡死。
 */
void STS3032_FlushRx(void)
{
    uint8_t  dummy;
    uint8_t  i;

    for (i = 0U; i < SCS_BUF_SIZE; i++) {
        if (HAL_UART_Receive(sts.huart, &dummy, 1U, 0U) != HAL_OK) {
            break;      /* 没有数据了 */
        }
    }
}

/*==============================================================================
 * 协议层
 *============================================================================*/

/**
 * @brief  16 位数拆分为两个 8 位（对应 SCS::Host2SCS）
 */
void STS3032_Host2SCS(uint8_t *data_l, uint8_t *data_h, uint16_t data)
{
    if ((data_l == NULL) || (data_h == NULL)) {
        return;
    }

    if (sts.end) {
        *data_l = (uint8_t)(data >> 8);
        *data_h = (uint8_t)(data & 0xFFU);
    } else {
        *data_h = (uint8_t)(data >> 8);
        *data_l = (uint8_t)(data & 0xFFU);
    }
}

/**
 * @brief  16 位数合成（对应 SCS::SCS2Host）
 */
uint16_t STS3032_SCS2Host(uint8_t data_l, uint8_t data_h)
{
    uint16_t data;

    if (sts.end) {
        data  = (uint16_t)data_l;
        data  = (uint16_t)(data << 8);
        data |= (uint16_t)data_h;
    } else {
        data  = (uint16_t)data_h;
        data  = (uint16_t)(data << 8);
        data |= (uint16_t)data_l;
    }
    return data;
}

/**
 * @brief  等待并校验应答头 FF FF（对应 SCS::checkHead）
 * @return 1 = 找到头，0 = 失败
 */
static int STS3032_CheckHead(void)
{
    uint8_t  dat;
    uint8_t  buf[2] = {0U, 0U};
    uint8_t  cnt = 0U;

    while (1) {
        if (!STS3032_ReadByte(&dat)) {
            return 0;
        }
        buf[1] = buf[0];
        buf[0] = dat;
        if ((buf[0] == 0xFFU) && (buf[1] == 0xFFU)) {
            break;
        }
        cnt++;
        if (cnt > 10U) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief  组包并发送普通指令（对应 SCS::writeBuf）
 * @param  id       舵机 ID
 * @param  mem_addr 内存表地址
 * @param  data     数据指针（可为 NULL，表示无参数指令如 Ping）
 * @param  len      数据长度
 * @param  fun      指令码（INST_xxx）
 * @return 1 = 发送成功
 */
static int STS3032_WriteBuf(uint8_t id, uint8_t mem_addr, uint8_t *data,
                            uint8_t len, uint8_t fun)
{
    uint8_t  msg_len = 2U;
    uint8_t  buf[6];
    uint8_t  checksum = 0U;
    uint8_t  i;
    uint8_t  tmp[8];

    buf[0] = 0xFFU;
    buf[1] = 0xFFU;
    buf[2] = id;
    buf[4] = fun;

    if (data != NULL) {
        msg_len = (uint8_t)(msg_len + len + 1U);
        buf[3] = msg_len;
        buf[5] = mem_addr;
        if (STS3032_Write(buf, 6U) != 6) {
            return 0;
        }
    } else {
        buf[3] = msg_len;
        if (STS3032_Write(buf, 5U) != 5) {
            return 0;
        }
    }

    checksum = (uint8_t)(id + msg_len + fun + mem_addr);

    if (data != NULL) {
        for (i = 0U; i < len; i++) {
            checksum = (uint8_t)(checksum + data[i]);
        }
        if (STS3032_Write(data, len) != (int)len) {
            return 0;
        }
    }

    tmp[0] = (uint8_t)(~checksum);
    if (STS3032_Write(tmp, 1U) != 1) {
        return 0;
    }

    return 1;
}

/**
 * @brief  等待指令应答（对应 SCS::Ack）
 * @return 1 = 收到合法应答，0 = 失败
 */
static int STS3032_Ack(uint8_t id)
{
    uint8_t buf[4];
    uint8_t checksum;

    sts.error = 0U;

    /* 广播指令（0xFE）不应答；level=0 时也不等应答 */
    if ((id != 0xFEU) && (sts.level != 0U)) {
        if (!STS3032_CheckHead()) {
            sts.error_count++;
            return 0;
        }
        if (STS3032_Read(buf, 4U) != 4) {
            sts.error_count++;
            return 0;
        }
        if (buf[0] != id) {
            sts.error_count++;
            return 0;
        }
        if (buf[1] != 2U) {
            sts.error_count++;
            return 0;
        }
        checksum = (uint8_t)(~(uint8_t)(buf[0] + buf[1] + buf[2]));
        if (checksum != buf[3]) {
            sts.error_count++;
            return 0;
        }
        sts.error = buf[2];
    }
    return 1;
}

/**
 * @brief  Ping 指令
 */
int STS3032_Ping(uint8_t id)
{
    uint8_t buf[4];
    uint8_t checksum;

    STS3032_FlushRx();
    if (!STS3032_WriteBuf(id, 0U, NULL, 0U, INST_PING)) {
        return -1;
    }

    sts.error = 0U;
    if (!STS3032_CheckHead()) {
        sts.error_count++;
        return -1;
    }
    if (STS3032_Read(buf, 4U) != 4) {
        sts.error_count++;
        return -1;
    }
    if ((buf[0] != id) && (id != 0xFEU)) {
        sts.error_count++;
        return -1;
    }
    if (buf[1] != 2U) {
        sts.error_count++;
        return -1;
    }
    checksum = (uint8_t)(~(uint8_t)(buf[0] + buf[1] + buf[2]));
    if (checksum != buf[3]) {
        sts.error_count++;
        return -1;
    }
    sts.error = buf[2];
    return (int)buf[0];
}

/**
 * @brief  读寄存器（对应 SCS::Read）
 */
int STS3032_ReadReg(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t len)
{
    uint8_t buf[4];
    uint8_t cal_sum = 0U;
    uint8_t i;
    int     size;

    if ((data == NULL) || (len == 0U)) {
        return 0;
    }

    STS3032_FlushRx();

    /* 读指令的参数是"要读的字节数" */
    if (!STS3032_WriteBuf(id, mem_addr, &len, 1U, INST_READ)) {
        return 0;
    }

    if (!STS3032_CheckHead()) {
        sts.error_count++;
        return 0;
    }

    sts.error = 0U;

    if (STS3032_Read(buf, 3U) != 3) {
        sts.error_count++;
        return 0;
    }

    size = STS3032_Read(data, len);
    if (size != (int)len) {
        sts.error_count++;
        return 0;
    }

    if (STS3032_Read(buf + 3, 1U) != 1) {
        sts.error_count++;
        return 0;
    }

    cal_sum = (uint8_t)(buf[0] + buf[1] + buf[2]);
    for (i = 0U; i < (uint8_t)size; i++) {
        cal_sum = (uint8_t)(cal_sum + data[i]);
    }
    cal_sum = (uint8_t)(~cal_sum);
    if (cal_sum != buf[3]) {
        sts.error_count++;
        return 0;
    }

    sts.error = buf[2];
    return size;
}

/**
 * @brief  读 1 字节寄存器
 */
int STS3032_ReadByteReg(uint8_t id, uint8_t mem_addr)
{
    uint8_t dat;
    int     size = STS3032_ReadReg(id, mem_addr, &dat, 1U);

    if (size != 1) {
        return -1;
    }
    return (int)dat;
}

/**
 * @brief  读 2 字节寄存器
 */
int STS3032_ReadWordReg(uint8_t id, uint8_t mem_addr)
{
    uint8_t dat[2];
    int     size = STS3032_ReadReg(id, mem_addr, dat, 2U);

    if (size != 2) {
        return -1;
    }
    return (int)STS3032_SCS2Host(dat[0], dat[1]);
}

/**
 * @brief  普通写指令（对应 SCS::genWrite）
 */
int STS3032_GenWrite(uint8_t id, uint8_t mem_addr, uint8_t *data, uint8_t len)
{
    STS3032_FlushRx();
    if (!STS3032_WriteBuf(id, mem_addr, data, len, INST_WRITE)) {
        return 0;
    }
    return STS3032_Ack(id);
}

/**
 * @brief  同步写指令（对应 SCS::syncWrite）
 * @note   广播指令，舵机不应答，因此不需要等 Ack，速度最快。
 */
int STS3032_SyncWrite(uint8_t id[], uint8_t id_num, uint8_t mem_addr,
                      uint8_t *data, uint8_t n_len)
{
    uint8_t  mes_len;
    uint8_t  sum = 0U;
    uint8_t  head[7];
    uint8_t  i;
    uint8_t  j;
    uint8_t  tail;

    if ((id == NULL) || (data == NULL) || (id_num == 0U) || (n_len == 0U)) {
        return 0;
    }

    STS3032_FlushRx();

    mes_len = (uint8_t)(((uint8_t)(n_len + 1U) * id_num) + 4U);

    head[0] = 0xFFU;
    head[1] = 0xFFU;
    head[2] = 0xFEU;                /* 广播 ID */
    head[3] = mes_len;
    head[4] = INST_SYNC_WRITE;
    head[5] = mem_addr;
    head[6] = n_len;

    if (STS3032_Write(head, 7U) != 7) {
        return 0;
    }

    sum = (uint8_t)(0xFEU + mes_len + INST_SYNC_WRITE + mem_addr + n_len);

    for (i = 0U; i < id_num; i++) {
        if (STS3032_WriteByte(id[i]) != 1) {
            return 0;
        }
        if (STS3032_Write(data + (i * n_len), n_len) != (int)n_len) {
            return 0;
        }
        sum = (uint8_t)(sum + id[i]);
        for (j = 0U; j < n_len; j++) {
            sum = (uint8_t)(sum + data[i * n_len + j]);
        }
    }

    tail = (uint8_t)(~sum);
    if (STS3032_WriteByte(tail) != 1) {
        return 0;
    }

    return 1;
}

/*==============================================================================
 * 应用层
 *============================================================================*/

/**
 * @brief  同步写位置 + 速度 + 加速度（对应 SMS_STS::SyncWritePosEx）
 *
 * 数据布局（每颗舵机 7 字节）：
 *   [0]      ACC
 *   [1][2]   Position（低、高）
 *   [3][4]   0（位置+速度模式下忽略）
 *   [5][6]   Speed（低、高）
 */
void STS3032_SyncWritePosEx(uint8_t id[], uint8_t id_num, int16_t position[],
                            uint16_t speed[], uint8_t acc[])
{
    /* 原版用 u8 offbuf[7*IDN]，这里用固定缓冲 + 边界保护 */
    uint8_t offbuf[7U * SERVO_ID_NUM];
    uint8_t i;
    uint16_t v;

    if ((id == NULL) || (position == NULL) || (id_num == 0U)) {
        return;
    }
    if (id_num > SERVO_ID_NUM) {
        id_num = SERVO_ID_NUM;
    }

    for (i = 0U; i < id_num; i++) {
        /* 位置为负时取绝对值并把 bit15 置 1（舵机用 bit15 表示方向） */
        if (position[i] < 0) {
            position[i] = (int16_t)(-position[i]);
            position[i] |= (int16_t)(1U << 15);
        }

        if (speed != NULL) {
            v = speed[i];
        } else {
            v = 0U;
        }

        if (acc != NULL) {
            offbuf[i * 7U] = acc[i];
        } else {
            offbuf[i * 7U] = 0U;
        }

        STS3032_Host2SCS(offbuf + i * 7U + 1U, offbuf + i * 7U + 2U, (uint16_t)position[i]);
        STS3032_Host2SCS(offbuf + i * 7U + 3U, offbuf + i * 7U + 4U, 0U);
        STS3032_Host2SCS(offbuf + i * 7U + 5U, offbuf + i * 7U + 6U, v);
    }

    (void)STS3032_SyncWrite(id, id_num, SMS_STS_ACC, offbuf, 7U);
}

/**
 * @brief  读取舵机当前位置（测试例程用）
 */
int STS3032_ReadPosition(uint8_t id)
{
    return STS3032_ReadWordReg(id, SMS_STS_PRESENT_POSITION);
}

/**
 * @brief  设置单颗舵机位置（测试例程用）
 */
int STS3032_SetPosition(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    uint8_t data[7];

    if (position < 0) {
        position = (int16_t)(-position);
        position |= (int16_t)(1U << 15);
    }

    data[0] = acc;
    STS3032_Host2SCS(data + 1U, data + 2U, (uint16_t)position);
    STS3032_Host2SCS(data + 3U, data + 4U, 0U);
    STS3032_Host2SCS(data + 5U, data + 6U, speed);

    return STS3032_GenWrite(id, SMS_STS_ACC, data, 7U);
}
