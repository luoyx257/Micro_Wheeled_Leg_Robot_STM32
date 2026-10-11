/**
 ******************************************************************************
 * @file    as5600.c
 * @brief   AS5600 磁性编码器驱动实现
 *
 * 关键点：
 *   1. AS5600 的 RAW_ANGLE 是 12bit（0~4095），0.0879°/LSB
 *   2. 用"±半圈判定"做跨圈累计，保证角度连续（对应 SimpleFOC 的多圈扩展）
 *   3. I2C 读失败时保持上一次角度、置 online=0，绝不返回 0 造成角度跳变
 ******************************************************************************
 */

#include "as5600.h"
#include "i2c.h"
#include "hardware.h"
#include <math.h>

/*==============================================================================
 * 全局实例
 *============================================================================*/

/** @brief 左轮编码器（插口 3，I2C2） */
AS5600_t encoder_left = {
    .hi2c            = &hi2c2,
    .direction       = ENCODER_L_DIRECTION,
    .angle           = 0.0f,
    .angle_last      = 0.0f,
    .full_rotations  = 0.0f,
    .total_rotations = 0,
    .magnet_ok       = 0U,
    .online          = 0U,
};

/** @brief 右轮编码器（插口 4，I2C1） */
AS5600_t encoder_right = {
    .hi2c            = &hi2c1,
    .direction       = ENCODER_R_DIRECTION,
    .angle           = 0.0f,
    .angle_last      = 0.0f,
    .full_rotations  = 0.0f,
    .total_rotations = 0,
    .magnet_ok       = 0U,
    .online          = 0U,
};

/*==============================================================================
 * 接口实现
 *============================================================================*/

/**
 * @brief  读取 AS5600 状态寄存器（0x0B）
 * @note   bit5 = MD（Magnet Detect）：1 表示磁铁在位
 */
uint8_t AS5600_ReadStatus(AS5600_t *enc)
{
    uint8_t status = 0U;

    if (enc == NULL) {
        return 0xFFU;
    }

    if (HAL_I2C_Mem_Read(enc->hi2c,
                         (uint16_t)AS5600_I2C_ADDR,
                         (uint16_t)AS5600_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT,
                         &status, 1U,
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK) {
        enc->online = 0U;
        return 0xFFU;
    }
    enc->online = 1U;
    return status;
}

/**
 * @brief  读取 AS5600 原始角度（0x0C，12bit）
 * @note   RAW_ANGLE 是 2 字节：bit11~bit8 在高字节低 4 位，bit7~bit0 在低字节。
 */
uint16_t AS5600_ReadRaw(AS5600_t *enc)
{
    uint8_t  buf[2] = {0U, 0U};
    uint16_t raw;

    if (enc == NULL) {
        return 0xFFFFU;
    }

    if (HAL_I2C_Mem_Read(enc->hi2c,
                         (uint16_t)AS5600_I2C_ADDR,
                         (uint16_t)AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT,
                         buf, 2U,
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK) {
        enc->online = 0U;
        /* 通信故障时尝试恢复总线，避免死锁拖死整个控制循环 */
        HW_I2C_BusRecover(enc->hi2c);
        return 0xFFFFU;
    }
    enc->online = 1U;

    /* 高字节低 4 位 + 低字节 */
    raw = (uint16_t)(((uint16_t)(buf[0] & 0x0FU) << 8) | (uint16_t)buf[1]);
    return raw;
}

/**
 * @brief  更新编码器角度与累计圈数
 *
 * 跨圈算法（对应 SimpleFOC MagneticSensor 的多圈扩展）：
 *   设上一角度为 a_last、本次为 a_now，单圈差值 d = a_now - a_last。
 *   若 d > π，说明实际上是从 2π 附近"倒退"到了 0 附近，d 应减去 2π；
 *   若 d < -π，说明实际上是从 0 附近"前进"到了 2π 附近，d 应加上 2π。
 *   把修正后的 d 累加到 full_rotations 上，即可得到连续角度。
 */
void AS5600_Update(AS5600_t *enc)
{
    uint16_t raw;
    float    angle;
    float    delta;

    if (enc == NULL) {
        return;
    }

    raw = AS5600_ReadRaw(enc);
    if (raw == 0xFFFFU) {
        /* 通信失败：保持上一次值不动，避免角度跳变污染速度环 */
        return;
    }

    /* 原始值 → 机械角（rad） */
    angle = (float)raw * (TWO_PI / AS5600_RAW_MAX);
    if (angle >= TWO_PI) {
        angle -= TWO_PI;
    }

    /* 方向修正（若实测方向相反，改 robot_config.h 中的宏即可） */
    angle *= enc->direction;
    if (angle < 0.0f) {
        angle += TWO_PI;
    }

    enc->angle = angle;

    /* 计算并修正单圈差值 */
    delta = enc->angle - enc->angle_last;
    if (delta > PI) {
        delta -= TWO_PI;            /* 2π → 0 的倒退 */
    } else if (delta < -PI) {
        delta += TWO_PI;            /* 0 → 2π 的前进 */
    }

    enc->full_rotations += delta;   /* 累计连续角度（rad） */
    enc->angle_last = enc->angle;

    /* 记录整圈数（供调试观察用） */
    enc->total_rotations = (int32_t)(enc->full_rotations / TWO_PI);
}

/**
 * @brief  同时更新左右编码器
 */
void AS5600_UpdateAll(void)
{
    AS5600_Update(&encoder_left);
    AS5600_Update(&encoder_right);
}

/**
 * @brief  初始化两片 AS5600
 */
void AS5600_Init(void)
{
    uint8_t status;

    /* ---- 左编码器（I2C2） ---- */
    status = AS5600_ReadStatus(&encoder_left);
    if (status == 0xFFU) {
        encoder_left.magnet_ok = 0U;
        encoder_left.online    = 0U;
    } else {
        encoder_left.magnet_ok = ((status & AS5600_STATUS_MD) != 0U) ? 1U : 0U;
    }
    /* 读一次角度，建立角度基准 */
    encoder_left.angle      = 0.0f;
    encoder_left.angle_last = 0.0f;
    encoder_left.full_rotations  = 0.0f;
    encoder_left.total_rotations = 0;
    (void)AS5600_ReadRaw(&encoder_left);

    /* ---- 右编码器（I2C1） ---- */
    status = AS5600_ReadStatus(&encoder_right);
    if (status == 0xFFU) {
        encoder_right.magnet_ok = 0U;
        encoder_right.online    = 0U;
    } else {
        encoder_right.magnet_ok = ((status & AS5600_STATUS_MD) != 0U) ? 1U : 0U;
    }
    encoder_right.angle      = 0.0f;
    encoder_right.angle_last = 0.0f;
    encoder_right.full_rotations  = 0.0f;
    encoder_right.total_rotations = 0;
    (void)AS5600_ReadRaw(&encoder_right);

    /* 用第一帧真实数据初始化，避免首次跨圈判定把 0 当成基准 */
    AS5600_Update(&encoder_left);
    AS5600_Update(&encoder_right);
    AS5600_ResetFullAngle(&encoder_left);
    AS5600_ResetFullAngle(&encoder_right);
}

/**
 * @brief  获取累计机械角（rad）
 */
float AS5600_GetFullAngle(const AS5600_t *enc)
{
    if (enc == NULL) {
        return 0.0f;
    }
    return enc->full_rotations;
}

/**
 * @brief  获取电角度（rad，0~2π）
 * @note   电角度 = 机械角 × 极对数，取模到 [0, 2π)。
 *         B 方案用它做电压矢量的角度输入。
 */
float AS5600_GetElectricalAngle(const AS5600_t *enc)
{
    float elec;

    if (enc == NULL) {
        return 0.0f;
    }

    elec = enc->full_rotations * MOTOR_POLE_PAIRS;

    /* 取模到 [0, 2π) */
    elec = (float)fmod((double)elec, (double)TWO_PI);
    if (elec < 0.0f) {
        elec += TWO_PI;
    }
    return elec;
}

/**
 * @brief  清零累计角度（上电对齐结束后调用）
 */
void AS5600_ResetFullAngle(AS5600_t *enc)
{
    if (enc == NULL) {
        return;
    }
    enc->full_rotations  = 0.0f;
    enc->total_rotations = 0;
    enc->angle_last      = enc->angle;
}
