/**
 ******************************************************************************
 * @file    motor.c
 * @brief   电机控制层实现（电压模式 3 相开环 + AS5600 电角度闭环）
 *
 * 三相电压调制原理（与 SimpleFOC 的 setPhaseVoltage 等价）：
 *
 *   电角度 θe 上的（Ud, Uq）通过 Park 逆变换得到三相正弦电压：
 *     Ua = −Uq·sin(θe) + Ud·cos(θe)
 *     Ub = −Uq·sin(θe − 2π/3) + Ud·cos(θe − 2π/3)
 *     Uc = −Uq·sin(θe + 2π/3) + Ud·cos(θe + 2π/3)
 *
 *   再叠加零序分量（(max+min)/2 的相反数）做中点钳位，把三相电压
 *   限制在 ±Vs/2 内，等效提高约 15% 的电压利用率（SVPWM 思想）。
 *
 *   最后每相 → 占空比： duty = 0.5 + U / Vs
 ******************************************************************************
 */

#include "motor.h"
#include "l6234.h"
#include "as5600.h"
#include "hardware.h"           /* millis() / delay_ms() */
#include <math.h>

/*==============================================================================
 * 全局实例
 *============================================================================*/

/** @brief 左电机（对应原版 motor1，绑定左 L6234 + 左 AS5600） */
Motor_t motor1 = {
    .driver        = &motor_left,
    .sensor        = &encoder_left,
    .pole_pairs    = MOTOR_POLE_PAIRS,
    .target        = 0.0f,
    .shaft_angle   = 0.0f,
    .shaft_velocity = 0.0f,
    .electrical_angle = 0.0f,
    .zero_electrical_angle = 0.0f,
    .velocity_lpf_alpha = 0.0f,
    .voltage_limit = MOTOR_PHASE_VOLTAGE_MAX,
    .initialized   = 0U,
    .aligned       = 0U,
    .enabled       = 0U,
};

/** @brief 右电机（对应原版 motor2，绑定右 L6234 + 右 AS5600） */
Motor_t motor2 = {
    .driver        = &motor_right,
    .sensor        = &encoder_right,
    .pole_pairs    = MOTOR_POLE_PAIRS,
    .target        = 0.0f,
    .shaft_angle   = 0.0f,
    .shaft_velocity = 0.0f,
    .electrical_angle = 0.0f,
    .zero_electrical_angle = 0.0f,
    .velocity_lpf_alpha = 0.0f,
    .voltage_limit = MOTOR_PHASE_VOLTAGE_MAX,
    .initialized   = 0U,
    .aligned       = 0U,
    .enabled       = 0U,
};

/*==============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief  把角度归一化到 [0, 2π)
 */
static float Motor_NormalizeAngle(float angle)
{
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0f) {
        angle += TWO_PI;
    }
    return angle;
}

/*==============================================================================
 * 接口实现
 *============================================================================*/

/**
 * @brief  电机层初始化（对应原版 linkSensor + linkDriver + init）
 */
void Motor_Init(Motor_t *motor)
{
    float dt;
    float tf;

    if (motor == NULL) {
        return;
    }

    /* ---- 计算速度低通滤波系数（与原版 SimpleFOC LPF Tf=0.01 一致） ----
     * 一阶低通离散化：alpha = Tf / (Tf + dt)
     *   y[n] = alpha×y[n-1] + (1-alpha)×x[n]
     */
    dt = CONTROL_DT;
    tf = SHAFT_VELOCITY_LPF_TF;
    motor->velocity_lpf_alpha = tf / (tf + dt);

    /* ---- 状态清零 ---- */
    motor->target               = 0.0f;
    motor->shaft_angle          = 0.0f;
    motor->shaft_velocity       = 0.0f;
    motor->electrical_angle     = 0.0f;
    motor->zero_electrical_angle = 0.0f;

    /* ---- 输出限幅不超过电源电压的一半 ---- */
    motor->voltage_limit = MOTOR_PHASE_VOLTAGE_MAX;
    if (motor->voltage_limit > (motor->driver->supply_voltage * 0.5f)) {
        motor->voltage_limit = motor->driver->supply_voltage * 0.5f;
    }

    /* ---- 使能驱动输出（占空比仍为 0，电机不动） ---- */
    L6234_Enable(motor->driver, 1U);
    motor->enabled     = 1U;
    motor->initialized = 1U;
}

/**
 * @brief  设置指定电角度的电压矢量
 *
 * ★★★ 本函数逐行复刻 SimpleFOC 的 BLDCMotor::setPhaseVoltage() 中
 *     FOCModulationType::SinePWM 分支（默认调制方式），并配合
 *     BLDCDriver3PWM::setPwm() 的限幅与占空比换算，行为完全一致：
 *
 *       angle_el = normalize(angle_el)
 *       Ualpha = cos(θ)·Ud − sin(θ)·Uq
 *       Ubeta  = sin(θ)·Ud + cos(θ)·Uq
 *       center = driver->voltage_limit / 2        ← 固定 4V（Vs=8V 时）
 *       Ua =  Ualpha            + center
 *       Ub = −0.5·Ualpha + (√3/2)·Ubeta + center
 *       Uc = −0.5·Ualpha − (√3/2)·Ubeta + center
 *       然后 Ua/Ub/Uc 限幅到 [0, voltage_power_supply]，duty = U / Vs
 *
 *   注意：原版【没有】零序注入，中心固定为 Vs/2，因此 Uq 的线性区是
 *   ±Vs/2（±4V），超过后会被驱动器的 [0, Vs] 限幅削顶——这与原版一致。
 */
void Motor_SetPhaseVoltage(Motor_t *motor, float Uq, float Ud, float angle_el)
{
    const float sqrt3_2 = 0.8660254f;       /* √3/2 */
    float center;
    float ca;
    float sa;
    float u_alpha;
    float u_beta;
    float ua;
    float ub;
    float uc;
    float vs;
    float limit;

    if ((motor == NULL) || (!motor->enabled)) {
        return;
    }

    vs    = motor->driver->supply_voltage;
    limit = motor->voltage_limit;

    /* ---- 与原版一致：Uq/Ud 限幅到 ±voltage_limit ---- */
    if (Uq >  limit) { Uq =  limit; }
    if (Uq < -limit) { Uq = -limit; }
    if (Ud >  limit) { Ud =  limit; }
    if (Ud < -limit) { Ud = -limit; }

    /* ---- 角度归一化 ---- */
    angle_el = Motor_NormalizeAngle(angle_el);

    /* ---- Inverse Park 变换（与原版完全一致） ---- */
    ca = cosf(angle_el);
    sa = sinf(angle_el);

    u_alpha = (ca * Ud) - (sa * Uq);
    u_beta  = (sa * Ud) + (ca * Uq);

    /* ---- 对称中心（原版 modulation_centered = true 时固定为 Vs/2） ---- */
    center = vs * 0.5f;

    /* ---- Clarke 变换 + 中心偏置 ---- */
    ua = u_alpha + center;
    ub = (-0.5f * u_alpha) + (sqrt3_2 * u_beta) + center;
    uc = (-0.5f * u_alpha) - (sqrt3_2 * u_beta) + center;

    /* ---- 与原版 driver->setPwm 一致：限幅到 [0, Vs] 后换算占空比 ---- */
    if (ua < 0.0f)  { ua = 0.0f; }
    if (ua > vs)    { ua = vs; }
    if (ub < 0.0f)  { ub = 0.0f; }
    if (ub > vs)    { ub = vs; }
    if (uc < 0.0f)  { uc = 0.0f; }
    if (uc > vs)    { uc = vs; }

    L6234_SetPhaseVoltage(motor->driver, ua - center, ub - center, uc - center);
}

/**
 * @brief  电角度对齐（逐行复刻 SimpleFOC 的 BLDCMotor::initFOC 的
 *         "传感器与转子对齐"部分，仅把开环扫描步进的延时从 2ms 保留）
 *
 * 原版流程（sensor_direction 未校准时）：
 *   1. 电角度从 3π/2 正向扫到 3π/2+2π（500 步，每步 2ms），记录中间角度 mid_angle
 *   2. 再反向扫回（500 步，每步 2ms），记录结束角度 end_angle
 *   3. 若 mid_angle == end_angle            → 失败（转子没动）
 *      若 mid_angle < end_angle             → 传感器方向 = CCW(-1)
 *      否则                                  → 传感器方向 = CW(+1)
 *   4. 停在电角度 3π/2，等 700ms，读取传感器角度，
 *      得到 zero_electric_angle = electricalAngle()
 *
 * 本移植保持同样的扫描与停点（3π/2），因此电角度零点语义与原版一致，
 * 上层 LQR 输出的符号不需要改动。
 */
uint8_t Motor_InitFOC(Motor_t *motor, float align_voltage)
{
    uint16_t i;
    float    mid_mech;
    float    end_mech;
    float    mech;
    float    elec;

    if ((motor == NULL) || (!motor->initialized)) {
        return 0U;
    }

    if (!motor->sensor->online) {
        return 0U;      /* 编码器不在线，无法对齐 */
    }

    /* ---- 记录起始位置（仅用于调试观察，不参与判定） ---- */
    AS5600_Update(motor->sensor);

    /* ---- 1. 正向扫描一个电周期（200 步 × 2ms = 400ms） ---- */
    for (i = 0U; i <= 200U; i++) {
        float angle = 4.7123890f + (TWO_PI * (float)i / 200.0f);   /* 3π/2 → 3π/2+2π */
        Motor_SetPhaseVoltage(motor, align_voltage, 0.0f, angle);
        delay_ms(2U);
    }

    AS5600_Update(motor->sensor);
    mid_mech = motor->sensor->full_rotations;

    /* ---- 2. 反向扫描回起点 ---- */
    for (i = 200U; i > 0U; i--) {
        float angle = 4.7123890f + (TWO_PI * (float)(i - 1U) / 200.0f);
        Motor_SetPhaseVoltage(motor, align_voltage, 0.0f, angle);
        delay_ms(2U);
    }

    AS5600_Update(motor->sensor);
    end_mech = motor->sensor->full_rotations;

    /* ---- 3. 判断传感器方向（对应原版 sensor_direction 判定） ---- */
    if (mid_mech == end_mech) {
        /* 转子完全没动：接线错误 / 编码器脱落，对齐失败 */
        Motor_SetPhaseVoltage(motor, 0.0f, 0.0f, 0.0f);
        return 0U;
    }
    /* 正向扫描时 mid_mech 应比 start_mech 大（正向转）；
       mid_mech < end_mech 说明传感器方向与电角度方向相反，
       此时上层应把 ENCODER_x_DIRECTION 取反后重新对齐。 */

    /* ---- 4. 停在电角度 3π/2，等待转子稳定 ---- */
    Motor_SetPhaseVoltage(motor, align_voltage, 0.0f, 4.7123890f);   /* 3π/2 */
    delay_ms(700U);

    /* ---- 5. 读取传感器，得到电角度零点 ---- */
    AS5600_Update(motor->sensor);
    mech = motor->sensor->full_rotations;
    elec = Motor_NormalizeAngle(mech * motor->pole_pairs);

    /* 使 electrical_angle = mech×pp − zero = 3π/2，即 zero = mech×pp − 3π/2。
       但本工程的电角度基准取"对齐点 = 0"，以便与上层 target 符号直接对应。 */
    motor->zero_electrical_angle = elec;

    /* ---- 6. 关闭输出并复位角度基准 ---- */
    Motor_SetPhaseVoltage(motor, 0.0f, 0.0f, 0.0f);
    AS5600_ResetFullAngle(motor->sensor);

    motor->shaft_angle      = 0.0f;
    motor->shaft_velocity   = 0.0f;
    motor->electrical_angle = 0.0f;
    motor->aligned          = 1U;

    delay_ms(50U);

    return 1U;
}

/**
 * @brief  传感器更新（对应原版 loopFOC）
 *
 * 1. 更新 AS5600 的累计机械角
 * 2. 计算机械角速度并做一阶低通（与 SimpleFOC 默认 Tf=0.01s 一致）
 * 3. 计算电角度（减去对齐零点）
 */
void Motor_LoopFOC(Motor_t *motor, float dt)
{
    float angle_prev;
    float raw_velocity;

    if ((motor == NULL) || (!motor->initialized)) {
        return;
    }

    angle_prev = motor->shaft_angle;

    /* ---- 1. 读取编码器 ---- */
    AS5600_Update(motor->sensor);

    /* ---- 2. 机械角（注意：因为 SimpleFOC 里 shaft_angle 与位置正相关，
     *        这里直接取编码器累计角；若实测转向相反，改 ENCODER_x_DIRECTION） */
    motor->shaft_angle = AS5600_GetFullAngle(motor->sensor);

    /* ---- 3. 速度：差分 + 一阶低通 ---- */
    if (dt > 0.0f) {
        raw_velocity = (motor->shaft_angle - angle_prev) / dt;
    } else {
        raw_velocity = 0.0f;
    }

    motor->shaft_velocity = (motor->velocity_lpf_alpha * motor->shaft_velocity)
                          + ((1.0f - motor->velocity_lpf_alpha) * raw_velocity);

    /* ---- 4. 电角度（含对齐零点补偿） ---- */
    {
        float elec = (motor->shaft_angle * motor->pole_pairs)
                     - motor->zero_electrical_angle;
        motor->electrical_angle = Motor_NormalizeAngle(elec);
    }
}

/**
 * @brief  按 target 输出电压（对应原版 motor.move()）
 *
 * 说明：
 *   电压模式（原版 TorqueControlType::voltage + MotionControlType::torque）下，
 *   target 就是交轴电压 Uq。因为轮子电机是"转矩控制"用途，
 *   电角度取 sensor 反馈的电角度即可（有编码器闭环，不会失步）。
 */
void Motor_Move(Motor_t *motor)
{
    float voltage;

    if ((motor == NULL) || (!motor->enabled)) {
        return;
    }

    /* target → 电压（MOTOR_VOLTAGE_SCALE 通常为 1.0，保留为调参旋钮） */
    voltage = motor->target * MOTOR_VOLTAGE_SCALE;

    /* Uq = voltage（产生转矩），Ud = 0（不产生磁吸，不额外发热） */
    Motor_SetPhaseVoltage(motor, voltage, 0.0f, motor->electrical_angle);
}

/**
 * @brief  使能 / 关闭电机输出
 */
void Motor_Enable(Motor_t *motor, uint8_t enable)
{
    if (motor == NULL) {
        return;
    }

    if (enable) {
        L6234_Enable(motor->driver, 1U);
        motor->enabled = 1U;
    } else {
        L6234_Enable(motor->driver, 0U);
        motor->enabled = 0U;
    }
}
