/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   MPU6050 IMU 驱动 + 姿态解算（逐字复刻 MPU6050_tockn 库算法）
 *
 * 硬件说明：
 *   插口4（I2C1，PB8=SCL / PB9=SDA），与右 AS5600 共用总线
 *   从机地址 0x68（AD0 接地）
 *
 * ★ 改动点说明（重要）：
 *   本文件的姿态解算部分【完全按原版 MPU6050_tockn 库实现】，包括：
 *     · 低通系数      accCoef = 0.02f, gyroCoef = 0.98f
 *     · 陀螺仪灵敏度   / 65.5  (GYRO_CONFIG = 0x08 → ±500°/s)
 *     · 加速度计灵敏度 / 16384 (ACCEL_CONFIG = 0x00 → ±2g)
 *     · 加速度角度公式 atan2(accY, accZ + |accX|) * 360 / -2 / PI（Y 轴取负）
 *     · 零偏标定      连续采样取平均（原版 3000 次，本移植 1000 次以缩短启动）
 *   唯一差异：原版用 millis() 差分得到可变 dt，本移植固定 dt = 1/1000s，
 *   因为 STM32 侧控制循环是严格 1kHz，固定 dt 更稳定（已与用户确认）。
 ******************************************************************************
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

/**
 * @brief MPU6050 实例
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;            /**< 所属 I2C 句柄 */

    /* ---- 原始数据 ---- */
    int16_t raw_acc_x;                  /**< 加速度 X 原始值 */
    int16_t raw_acc_y;                  /**< 加速度 Y 原始值 */
    int16_t raw_acc_z;                  /**< 加速度 Z 原始值 */
    int16_t raw_temp;                   /**< 温度原始值 */
    int16_t raw_gyro_x;                 /**< 陀螺仪 X 原始值 */
    int16_t raw_gyro_y;                 /**< 陀螺仪 Y 原始值 */
    int16_t raw_gyro_z;                 /**< 陀螺仪 Z 原始值 */

    /* ---- 物理量 ---- */
    float acc_x;                        /**< 加速度 X（g） */
    float acc_y;                        /**< 加速度 Y（g） */
    float acc_z;                        /**< 加速度 Z（g） */
    float temp;                         /**< 温度（℃） */
    float gyro_x;                       /**< 角速度 X（°/s），已扣除零偏 */
    float gyro_y;                       /**< 角速度 Y（°/s），已扣除零偏 */
    float gyro_z;                       /**< 角速度 Z（°/s），已扣除零偏 */

    /* ---- 零偏 ---- */
    float gyro_x_offset;                /**< X 轴零偏 */
    float gyro_y_offset;                /**< Y 轴零偏 */
    float gyro_z_offset;                /**< Z 轴零偏 */

    /* ---- 姿态角（与原版 getAngleX/Y/Z 语义一致，单位：度） ---- */
    float angle_acc_x;                  /**< 加速度计解算的 X 角 */
    float angle_acc_y;                  /**< 加速度计解算的 Y 角 */
    float angle_gyro_x;                 /**< 陀螺仪积分 X 角 */
    float angle_gyro_y;                 /**< 陀螺仪积分 Y 角 */
    float angle_gyro_z;                 /**< 陀螺仪积分 Z 角（偏航） */
    float angle_x;                      /**< 互补滤波后 X 角（roll） */
    float angle_y;                      /**< 互补滤波后 Y 角（pitch，平衡用） */
    float angle_z;                      /**< Z 角（yaw） */

    uint8_t online;                     /**< 1 = I2C 通信正常 */
    uint8_t initialized;                /**< 1 = begin() 已完成 */
} MPU6050_t;

/*==============================================================================
 * 全局实例（定义在 mpu6050.c 中）
 *============================================================================*/

/** @brief 机身 IMU（插口 4，I2C1） */
extern MPU6050_t imu;

/*==============================================================================
 * 接口（命名与原版 MPU6050_tockn 一致，便于上层代码复用）
 *============================================================================*/

/**
 * @brief  初始化 MPU6050（寄存器配置 + 初始角度）
 * @note   对应原版 mpu6050.begin()。寄存器配置与原版逐条一致：
 *           SMPLRT_DIV    = 0x00
 *           CONFIG        = 0x00
 *           GYRO_CONFIG   = 0x08   (±500°/s)
 *           ACCEL_CONFIG  = 0x00   (±2g)
 *           PWR_MGMT_1    = 0x01   (PLL with X axis gyroscope reference)
 */
void MPU6050_Begin(void);

/**
 * @brief  读取一帧 14 字节数据并完成姿态解算
 * @note   对应原版 mpu6050.update()。必须按固定周期调用（本工程 1kHz）。
 * @param  dt 本次积分时间（s），固定传 CONTROL_DT
 */
void MPU6050_Update(float dt);

/**
 * @brief  读取 14 字节原始数据并换算为物理量（不做互补滤波）
 * @return 1 = 成功，0 = 失败
 * @note   对应原版 update() 里 requestFrom(0x3B, 14) 那一段；
 *         显式暴露出来是为了方便上层做原始数据观测与故障定位。
 */
uint8_t MPU6050_ReadRawFrame(void);

/**
 * @brief  陀螺仪零偏标定
 * @param  samples 采样次数（原版 3000；本工程建议 1000）
 * @note   对应原版 mpu6050.calcGyroOffsets(true)。标定期间机器人必须静止！
 */
void MPU6050_CalcGyroOffsets(uint16_t samples);

/* ---- 以下 get 接口与原版一一对应（单位：度、度每秒） ---- */

/** @brief 获取互补滤波后的 X 角（roll，单位 °） */
float MPU6050_GetAngleX(void);
/** @brief 获取互补滤波后的 Y 角（pitch，平衡控制用，单位 °） */
float MPU6050_GetAngleY(void);
/** @brief 获取 Z 角（yaw，单位 °） */
float MPU6050_GetAngleZ(void);
/** @brief 获取 X 轴角速度（单位 °/s） */
float MPU6050_GetGyroX(void);
/** @brief 获取 Y 轴角速度（平衡控制用，单位 °/s） */
float MPU6050_GetGyroY(void);
/** @brief 获取 Z 轴角速度（yaw 控制用，单位 °/s） */
float MPU6050_GetGyroZ(void);
/** @brief 获取温度（℃） */
float MPU6050_GetTemp(void);

/**
 * @brief  手动设置陀螺仪零偏
 * @note   对应原版 setGyroOffsets(x, y, z)
 */
void MPU6050_SetGyroOffsets(float x, float y, float z);

#endif /* __MPU6050_H__ */
