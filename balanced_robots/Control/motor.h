/**
 ******************************************************************************
 * @file    motor.h
 * @brief   电机控制层（B 方案：电压模式 3 相开环 + AS5600 电角度闭环）
 *
 * ============================ 方案说明 ============================
 * 原版 ESP32 用 SimpleFOC 做完整 FOC（电角度对齐 + 电流/电压力矩环 + 速度环）。
 * 本移植按与用户商定的 B 方案实现：
 *
 *   · 输出层：三相正弦电压开环调制（SVPWM 简化版：三相正弦 + 零序注入），
 *             占空比 = 0.5 + U phase / Vs，经 L6234 的 EN1/EN2/EN3 输出
 *   · 反馈层：AS5600 提供机械角 → ×极对数 → 电角度，闭环到电压矢量角度
 *   · 接口层：对外暴露与原版 SimpleFOC 同名同义的接口
 *             init() / initFOC() / loopFOC() / move() / target / shaft_angle /
 *             shaft_velocity，使上层 LQR / 步态 / PID 代码可以零改动搬过来
 *
 * 这样"上层控制逻辑完全复用原版、只换底层硬件驱动"的目标就达成了。
 *
 * ============================ 改动点 ============================
 * 1. 新增：上电电角度对齐（MOTOR_Align）—— 原版由 SimpleFOC initFOC() 完成，
 *    这里自己实现：给 d 轴一个固定电压，让转子吸到 0 电角度，记录编码器偏移。
 * 2. 新增：速度低通滤波（Tf = 0.01s，与 SimpleFOC 默认一致）。
 * 3. 删除：SimpleFOC 的电流环 / 力矩控制器（B 方案用电压直驱）。
 *    上层 LQR 输出的数值语义仍是"电压"，与 SimpleFOC 的 voltage 模式一致。
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "robot_config.h"
#include "l6234.h"
#include "as5600.h"

/*==============================================================================
 * 数据结构（对应原版 SimpleFOC 的 BLDCMotor 对象）
 *============================================================================*/

/**
 * @brief 电机实例
 */
typedef struct {
    L6234_t  *driver;           /**< 绑定的 L6234 驱动（对应 motor.linkDriver） */
    AS5600_t *sensor;           /**< 绑定的 AS5600 编码器（对应 motor.linkSensor） */

    float pole_pairs;           /**< 极对数（原版 BLDCMotor(7)） */

    /* ---- 对外状态量（命名与 SimpleFOC 一致，上层直接读） ---- */
    float target;               /**< 目标（voltage 模式下 = 电压，原版 motor.target） */
    float shaft_angle;          /**< 累计机械角（rad），原版 motor.shaft_angle */
    float shaft_velocity;       /**< 机械角速度（rad/s，已低通），原版 motor.shaft_velocity */
    float electrical_angle;     /**< 电角度（rad），B 方案闭环输入 */

    /* ---- 内部标定量 ---- */
    float zero_electrical_angle;/**< 电角度零点偏置（对齐得到） */
    float velocity_lpf_alpha;   /**< 速度低通系数（由 Tf 与 dt 计算） */
    float voltage_limit;        /**< 输出电压限幅（V） */

    uint8_t initialized;        /**< 1 = init() 完成 */
    uint8_t aligned;            /**< 1 = initFOC() 对齐完成 */
    uint8_t enabled;            /**< 1 = 允许输出 */
} Motor_t;

/*==============================================================================
 * 全局实例（定义在 motor.c 中）
 *============================================================================*/

/** @brief 左电机（对应原版 motor1） */
extern Motor_t motor1;

/** @brief 右电机（对应原版 motor2） */
extern Motor_t motor2;

/*==============================================================================
 * 接口（与原版 SimpleFOC 调用序列一一对应）
 *============================================================================*/

/**
 * @brief  电机层初始化（绑定驱动与编码器）
 * @note   对应原版：
 *           motor1.linkSensor(&sensor1);
 *           motor1.linkDriver(&driver1);
 *           motor1.voltage_sensor_align = 6;
 *           driver1.init();
 *           motor1.init();
 *         调用前请确保 L6234_Init() 与 AS5600_Init() 已完成。
 */
void Motor_Init(Motor_t *motor);

/**
 * @brief  电角度对齐（对应原版 motor.initFOC()）
 * @param  motor     电机实例
 * @param  align_voltage 对齐电压（V），原版 voltage_sensor_align = 6
 * @return 1 = 对齐成功，0 = 失败（编码器不在线）
 *
 * 原理：
 *   1. 给电机施加"电角度 = 0"的电压矢量（d 轴对准），转子会被吸到该位置；
 *   2. 等待转子稳定；
 *   3. 读取此时编码器角度，换算成电角度，即为 zero_electrical_angle；
 *   4. 之后任意时刻：electrical_angle = 机械角 × 极对数 − zero_electrical_angle。
 *
 * ⚠️ 对齐过程中电机会抖动，请让机器人悬空或确保周围无障碍物。
 */
uint8_t Motor_InitFOC(Motor_t *motor, float align_voltage);

/**
 * @brief  读取传感器、更新 shaft_angle / shaft_velocity / electrical_angle
 * @note   对应原版 motor.loopFOC()。必须在 1kHz 循环里每周期调用。
 */
void Motor_LoopFOC(Motor_t *motor, float dt);

/**
 * @brief  按 target 输出三相电压（对应原版 motor.move()）
 * @note   必须在 Motor_LoopFOC() 之后调用。
 */
void Motor_Move(Motor_t *motor);

/**
 * @brief  使能 / 关闭输出
 */
void Motor_Enable(Motor_t *motor, uint8_t enable);

/**
 * @brief  设置指定电角度的电压矢量（底层接口，供对齐与测试使用）
 * @param  motor        电机实例
 * @param  Uq           交轴电压（V）—— 产生转矩的分量
 * @param  Ud           直轴电压（V）—— 产生磁吸的分量
 * @param  angle_el     电角度（rad）
 */
void Motor_SetPhaseVoltage(Motor_t *motor, float Uq, float Ud, float angle_el);

#endif /* __MOTOR_H__ */
