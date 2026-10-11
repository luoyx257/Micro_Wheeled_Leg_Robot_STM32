/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 IMU 驱动 + 姿态解算实现（复刻 MPU6050_tockn）
 *
 * ★ 改动点说明：
 *   1. 原版用 Arduino Wire 库的 beginTransmission/requestFrom（重复起始），
 *      这里用 HAL_I2C_Mem_Read（内部也是"写寄存器地址 + 重复起始 + 读"），
 *      时序完全等价。
 *   2. 原版 dt 来自 millis() 差分；本移植固定 dt（由调用方传入 CONTROL_DT），
 *      因为控制循环严格 1kHz。
 *   3. 所有滤波系数、灵敏度换算、角度公式【一字未改】。
 ******************************************************************************
 */

#include "mpu6050.h"
#include "i2c.h"
#include "hardware.h"
#include <math.h>

/*==============================================================================
 * MPU6050 寄存器地址（与原版 MPU6050_tockn.h 一致）
 *============================================================================*/

#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_PWR_MGMT_1      0x6BU

/*==============================================================================
 * 全局实例
 *============================================================================*/

/** @brief 机身 IMU（插口 4，I2C1 与右 AS5600 共用总线） */
MPU6050_t imu = {
    .hi2c           = &hi2c1,
    .online         = 0U,
    .initialized    = 0U,
};

/*==============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief  写一个寄存器
 * @param  reg  寄存器地址
 * @param  data 数据
 * @return 1 = 成功，0 = 失败
 */
static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    if (HAL_I2C_Mem_Write(imu.hi2c,
                          (uint16_t)MPU6050_I2C_ADDR,
                          (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &data, 1U,
                          MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        imu.online = 0U;
        return 0U;
    }
    imu.online = 1U;
    return 1U;
}

/*==============================================================================
 * 接口实现
 *============================================================================*/

/**
 * @brief  初始化 MPU6050
 *
 * 寄存器配置与原版 mpu6050.begin() 完全一致：
 *   SMPLRT_DIV   = 0x00  → 采样率 8kHz/1（内部）
 *   CONFIG       = 0x00  → DLPF 关闭（与原版一致，保持算法输入特性不变）
 *   GYRO_CONFIG  = 0x08  → 陀螺仪 ±500°/s（灵敏度 65.5 LSB/(°/s)）
 *   ACCEL_CONFIG = 0x00  → 加速度 ±2g（灵敏度 16384 LSB/g）
 *   PWR_MGMT_1   = 0x01  → 唤醒 + 选择 X 轴陀螺仪作为时钟源
 */
void MPU6050_Begin(void)
{
    (void)MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV,   0x00U);
    (void)MPU6050_WriteReg(MPU6050_REG_CONFIG,       0x00U);
    (void)MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG,  0x08U);
    (void)MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1,   0x01U);

    /* 原版：this->update(); angleGyroX = 0; angleGyroY = 0;
               angleX = getAccAngleX(); angleY = getAccAngleY();         */
    (void)MPU6050_ReadRawFrame();

    imu.angle_gyro_x = 0.0f;
    imu.angle_gyro_y = 0.0f;
    imu.angle_gyro_z = 0.0f;
    imu.angle_x      = imu.angle_acc_x;
    imu.angle_y      = imu.angle_acc_y;
    imu.angle_z      = imu.angle_gyro_z;

    imu.initialized = 1U;
}

/**
 * @brief  读取 14 字节原始数据并换算为物理量
 * @return 1 = 成功，0 = 失败
 * @note   对应原版 update() 中 requestFrom(0x3B, 14) 那一段；
 *         本函数只做"读 + 换算"，不做互补滤波。
 */
uint8_t MPU6050_ReadRawFrame(void)
{
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(imu.hi2c,
                         (uint16_t)MPU6050_I2C_ADDR,
                         (uint16_t)MPU6050_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         buf, 14U,
                         MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        imu.online = 0U;
        /* 通信异常时尝试恢复总线，避免拖死 1kHz 控制循环 */
        HW_I2C_BusRecover(imu.hi2c);
        return 0U;
    }
    imu.online = 1U;

    /* 大端拼接（与原版 wire->read() << 8 | wire->read() 一致） */
    imu.raw_acc_x  = (int16_t)(((uint16_t)buf[0]  << 8) | (uint16_t)buf[1]);
    imu.raw_acc_y  = (int16_t)(((uint16_t)buf[2]  << 8) | (uint16_t)buf[3]);
    imu.raw_acc_z  = (int16_t)(((uint16_t)buf[4]  << 8) | (uint16_t)buf[5]);
    imu.raw_temp   = (int16_t)(((uint16_t)buf[6]  << 8) | (uint16_t)buf[7]);
    imu.raw_gyro_x = (int16_t)(((uint16_t)buf[8]  << 8) | (uint16_t)buf[9]);
    imu.raw_gyro_y = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
    imu.raw_gyro_z = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

    /* ---- 温度：原版 (rawTemp + 12412.0) / 340.0 ---- */
    imu.temp = ((float)imu.raw_temp + 12412.0f) / 340.0f;

    /* ---- 加速度：原版 / 16384.0 ---- */
    imu.acc_x = ((float)imu.raw_acc_x) / MPU6050_ACC_LSB_PER_G;
    imu.acc_y = ((float)imu.raw_acc_y) / MPU6050_ACC_LSB_PER_G;
    imu.acc_z = ((float)imu.raw_acc_z) / MPU6050_ACC_LSB_PER_G;

    /* ---- 加速度角度：原版公式（Y 轴系数取负，务必保持）
     *      angleAccX = atan2(accY, accZ + |accX|) * 360 / 2.0 / PI
     *      angleAccY = atan2(accX, accZ + |accY|) * 360 / -2.0 / PI       */
    imu.angle_acc_x = atan2f(imu.acc_y, imu.acc_z + fabsf(imu.acc_x))
                      * 360.0f / 2.0f / PI;
    imu.angle_acc_y = atan2f(imu.acc_x, imu.acc_z + fabsf(imu.acc_y))
                      * 360.0f / -2.0f / PI;

    /* ---- 陀螺仪：原版 / 65.5 ---- */
    imu.gyro_x = ((float)imu.raw_gyro_x) / MPU6050_GYRO_LSB_PER_DPS;
    imu.gyro_y = ((float)imu.raw_gyro_y) / MPU6050_GYRO_LSB_PER_DPS;
    imu.gyro_z = ((float)imu.raw_gyro_z) / MPU6050_GYRO_LSB_PER_DPS;

    /* ---- 扣除零偏（与原版一致：扣除后再参与积分） ---- */
    imu.gyro_x -= imu.gyro_x_offset;
    imu.gyro_y -= imu.gyro_y_offset;
    imu.gyro_z -= imu.gyro_z_offset;

    return 1U;
}

/**
 * @brief  读取并完成姿态解算（互补滤波）
 * @param  dt 积分时间（s），本工程固定 CONTROL_DT = 0.001
 *
 * 与原版 update() 的对应关系：
 *   interval      = (millis() - preInterval) * 0.001   → 本移植 dt（参数传入）
 *   angleGyroX   += gyroX * interval
 *   angleGyroY   += gyroY * interval
 *   angleGyroZ   += gyroZ * interval
 *   angleX = gyroCoef * (angleX + gyroX*interval) + accCoef * angleAccX
 *   angleY = gyroCoef * (angleY + gyroY*interval) + accCoef * angleAccY
 *   angleZ = angleGyroZ
 */
void MPU6050_Update(float dt)
{
    if (!MPU6050_ReadRawFrame()) {
        /* 读取失败：保持上一帧角度，不更新（避免角度跳变导致失控） */
        return;
    }

    /* 陀螺仪积分角度（原版 angleGyroX/Y/Z） */
    imu.angle_gyro_x += imu.gyro_x * dt;
    imu.angle_gyro_y += imu.gyro_y * dt;
    imu.angle_gyro_z += imu.gyro_z * dt;

    /* 互补滤波（原版 angleX/angleY） */
    imu.angle_x = (MPU6050_GYRO_COEF * (imu.angle_x + imu.gyro_x * dt))
                  + (MPU6050_ACC_COEF * imu.angle_acc_x);
    imu.angle_y = (MPU6050_GYRO_COEF * (imu.angle_y + imu.gyro_y * dt))
                  + (MPU6050_ACC_COEF * imu.angle_acc_y);

    /* 偏航角直接取陀螺仪积分（原版 angleZ = angleGyroZ） */
    imu.angle_z = imu.angle_gyro_z;
}

/**
 * @brief  陀螺仪零偏标定
 * @param  samples 采样次数
 * @note   对应原版 calcGyroOffsets()：连续采样取平均，灵敏度同样 / 65.5。
 *         标定期间绝对不能移动机器人，否则零偏错误会导致平衡漂移。
 */
void MPU6050_CalcGyroOffsets(uint16_t samples)
{
    float   x = 0.0f;
    float   y = 0.0f;
    float   z = 0.0f;
    uint8_t buf[6];
    int16_t rx;
    int16_t ry;
    int16_t rz;
    uint16_t i;
    uint16_t n = samples;

    if (n == 0U) {
        n = 1U;
    }

    for (i = 0U; i < n; i++) {
        if (HAL_I2C_Mem_Read(imu.hi2c,
                             (uint16_t)MPU6050_I2C_ADDR,
                             0x43U,                     /* GYRO_XOUT_H */
                             I2C_MEMADD_SIZE_8BIT,
                             buf, 6U,
                             MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
            continue;   /* 单次失败跳过 */
        }

        rx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
        ry = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
        rz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

        x += ((float)rx) / MPU6050_GYRO_LSB_PER_DPS;
        y += ((float)ry) / MPU6050_GYRO_LSB_PER_DPS;
        z += ((float)rz) / MPU6050_GYRO_LSB_PER_DPS;
    }

    imu.gyro_x_offset = x / (float)n;
    imu.gyro_y_offset = y / (float)n;
    imu.gyro_z_offset = z / (float)n;
}

/**
 * @brief  手动设置零偏
 */
void MPU6050_SetGyroOffsets(float x, float y, float z)
{
    imu.gyro_x_offset = x;
    imu.gyro_y_offset = y;
    imu.gyro_z_offset = z;
}

/*==============================================================================
 * 与原版一一对应的 get 接口
 *============================================================================*/

/** @brief 获取互补滤波后的 X 角（roll，°） */
float MPU6050_GetAngleX(void) { return imu.angle_x; }

/** @brief 获取互补滤波后的 Y 角（pitch，°）—— LQR 平衡控制使用 */
float MPU6050_GetAngleY(void) { return imu.angle_y; }

/** @brief 获取 Z 角（yaw，°） */
float MPU6050_GetAngleZ(void) { return imu.angle_z; }

/** @brief 获取 X 轴角速度（°/s） */
float MPU6050_GetGyroX(void) { return imu.gyro_x; }

/** @brief 获取 Y 轴角速度（°/s）—— LQR 平衡控制使用 */
float MPU6050_GetGyroY(void) { return imu.gyro_y; }

/** @brief 获取 Z 轴角速度（°/s）—— yaw 转向控制使用 */
float MPU6050_GetGyroZ(void) { return imu.gyro_z; }

/** @brief 获取温度（℃） */
float MPU6050_GetTemp(void) { return imu.temp; }
