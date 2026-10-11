/**
 ******************************************************************************
 * @file    as5600.h
 * @brief   AS5600 磁性编码器驱动（I2C 接口，12bit 绝对角度）
 *
 * 硬件说明：
 *   插口3 左 AS5600 → I2C2（PB10 = SCL，PB11 = SDA）
 *   插口4 右 AS5600 + MPU6050 → I2C1（PB8 = SCL，PB9 = SDA）
 *   AS5600 从机地址固定 0x36（7 位），两片在不同总线上，无地址冲突。
 *   两片 AS5600 均直连电机轴（1:1），机械角 = 转子角。
 *
 * 对应原版：
 *   SimpleFOC 的 MagneticSensorI2C(AS5600_I2C) + sensor.init() +
 *   sensor.getAngle()（返回 0~2π 机械角）
 ******************************************************************************
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

/**
 * @brief AS5600 编码器实例
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /**< 所属 I2C 句柄 */
    float  direction;               /**< 角度方向修正系数（+1 / -1） */
    float  angle;                   /**< 当前机械角（rad，0 ~ 2π） */
    float  angle_last;              /**< 上一次机械角（用于跨圈检测） */
    float  full_rotations;          /**< 累计整圈数（有符号） */
    int32_t total_rotations;        /**< 累计整圈数（整数，与 full_rotations 同步） */
    uint8_t magnet_ok;              /**< 1 = 磁铁在位（AS5600 STATUS.MD） */
    uint8_t online;                 /**< 1 = I2C 通信正常 */
} AS5600_t;

/*==============================================================================
 * 全局实例（定义在 as5600.c 中）
 *============================================================================*/

/** @brief 左轮编码器（I2C2，插口 3） */
extern AS5600_t encoder_left;

/** @brief 右轮编码器（I2C1，插口 4） */
extern AS5600_t encoder_right;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  初始化两片 AS5600（检测磁铁状态）
 * @note   必须在 MX_I2C1_Init()/MX_I2C2_Init() 之后调用。
 */
void AS5600_Init(void);

/**
 * @brief  更新单个编码器的角度与累计圈数（由 1kHz 控制循环调用）
 * @param  enc 编码器实例
 * @note   内部处理 0↔2π 跨圈，保证 getAngle() 连续可导。
 */
void AS5600_Update(AS5600_t *enc);

/**
 * @brief  同时更新左右两个编码器
 */
void AS5600_UpdateAll(void);

/**
 * @brief  读取 AS5600 原始 12bit 角度值
 * @param  enc 编码器实例
 * @return 0 ~ 4095；通信失败返回 0xFFFF
 */
uint16_t AS5600_ReadRaw(AS5600_t *enc);

/**
 * @brief  读取 AS5600 状态寄存器
 * @param  enc 编码器实例
 * @return 状态字节；通信失败返回 0xFF
 */
uint8_t AS5600_ReadStatus(AS5600_t *enc);

/**
 * @brief  获取累计机械角（rad，可超过 2π，带符号）
 * @param  enc 编码器实例
 * @return 连续的可累积角度，对应 SimpleFOC 的 sensor.getAngle() 多圈扩展
 */
float AS5600_GetFullAngle(const AS5600_t *enc);

/**
 * @brief  获取电角度（rad，0 ~ 2π）
 * @param  enc 编码器实例
 * @return 电角度 = 机械角 × 极对数
 * @note   这是 B 方案"AS5600 电角度闭环"使用的关键量。
 */
float AS5600_GetElectricalAngle(const AS5600_t *enc);

/**
 * @brief  把累计角度清零（对齐流程结束后调用）
 * @param  enc 编码器实例
 */
void AS5600_ResetFullAngle(AS5600_t *enc);

#endif /* __AS5600_H__ */
