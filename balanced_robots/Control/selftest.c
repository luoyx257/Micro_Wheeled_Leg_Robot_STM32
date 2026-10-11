/**
 ******************************************************************************
 * @file    selftest.c
 * @brief   单模块测试例程实现
 *
 * 设计原则：
 *   · 每个用例只依赖它要测的那个模块，互不干扰
 *   · 观测手段统一走 CAN（0x100 姿态帧 / 0x101 状态帧）与舵机/LED 现象，
 *     因为本 PCB 的唯一硬件 UART 已被舵机总线占用，没有多余串口打印
 *   · 所有用例都不跑平衡控制，保证安全
 ******************************************************************************
 */

#include "selftest.h"
#include "hardware.h"
#include "l6234.h"
#include "as5600.h"
#include "mpu6050.h"
#include "sts3032.h"
#include "can_proto.h"
#include "motor.h"
#include <math.h>

/*==============================================================================
 * 私有辅助
 *============================================================================*/

/**
 * @brief  把当前所有可观测状态通过 CAN 上报一次（测试模式下每 20ms 调用）
 */
static void SelfTest_ReportCan(void)
{
    uint8_t status = 0U;

    if (imu.online)             { status |= 0x01U; }
    if (encoder_left.online)    { status |= 0x02U; }
    if (encoder_right.online)   { status |= 0x04U; }
    if (L6234_IsFaulted(&motor_left))  { status |= 0x08U; }
    if (L6234_IsFaulted(&motor_right)) { status |= 0x10U; }
    if (current_sense.battery_voltage < BAT_LOW_THRESHOLD_V) { status |= 0x20U; }

    CAN_Proto_TelemetryTick(MPU6050_GetAngleY(), MPU6050_GetGyroY(),
                            motor1.shaft_velocity, motor2.shaft_velocity,
                            current_sense.current_left, current_sense.current_right,
                            current_sense.battery_voltage,
                            status, sts.error_count);
}

/**
 * @brief  测试用的电机开环旋转：按给定电压矢量角速度旋转指定时间
 * @param  motor     电机实例（须已 Init + InitFOC）
 * @param  voltage   电压幅值（V）
 * @param  time_ms   持续时间（ms）
 * @param  direction +1 / -1
 * @note   直接驱动电角度斜坡旋转（开环速度模式），不依赖编码器。
 */
static void SelfTest_SpinOpenLoop(Motor_t *motor, float voltage,
                                  uint32_t time_ms, int direction)
{
    uint32_t t0 = millis();
    float    angle = 0.0f;
    float    dir = (direction >= 0) ? 1.0f : -1.0f;

    while ((millis() - t0) < time_ms) {
        /* 电角度以 2π rad/s 的 3 倍速旋转 → 慢速转动，肉眼可观察 */
        angle += (TWO_PI * 3.0f * dir * 0.001f);
        if (angle > TWO_PI) { angle -= TWO_PI; }
        if (angle < 0.0f)  { angle += TWO_PI; }

        Motor_SetPhaseVoltage(motor, voltage, 0.0f, angle);
        delay_ms(1U);
        SelfTest_ReportCan();
    }

    /* 结束时关闭输出 */
    Motor_SetPhaseVoltage(motor, 0.0f, 0.0f, 0.0f);
}

/*==============================================================================
 * 各测试用例
 *============================================================================*/

/** @brief 用例 1/2/3：电机正反转测试 */
static void SelfTest_Motor(uint8_t which)
{
    Motor_t *m1 = NULL;
    Motor_t *m2 = NULL;

    if ((which == SELFTEST_MOTOR_LEFT) || (which == SELFTEST_MOTOR_BOTH)) {
        m1 = &motor1;
    }
    if ((which == SELFTEST_MOTOR_RIGHT) || (which == SELFTEST_MOTOR_BOTH)) {
        m2 = &motor2;
    }

    while (1) {
        /* ---- 正转（电压幅值 1.5V，低速） ---- */
        if (m1 != NULL) { SelfTest_SpinOpenLoop(m1, 1.5f, 2000U,  1); }
        if (m2 != NULL) { SelfTest_SpinOpenLoop(m2, 1.5f, 2000U,  1); }

        /* ---- 停 1 秒 ---- */
        L6234_Stop(&motor_left);
        L6234_Stop(&motor_right);
        delay_ms(1000U);

        /* ---- 反转 ---- */
        if (m1 != NULL) { SelfTest_SpinOpenLoop(m1, 1.5f, 2000U, -1); }
        if (m2 != NULL) { SelfTest_SpinOpenLoop(m2, 1.5f, 2000U, -1); }

        /* ---- 停 1 秒 ---- */
        L6234_Stop(&motor_left);
        L6234_Stop(&motor_right);
        delay_ms(1000U);
    }
}

/** @brief 用例 4：舵机角度读写 */
static void SelfTest_Servo(void)
{
    uint8_t  id[SERVO_ID_NUM] = { SERVO_ID_LEFT, SERVO_ID_RIGHT };
    int16_t  pos[SERVO_ID_NUM];
    uint16_t spd[SERVO_ID_NUM] = { 300U, 300U };
    uint8_t  acc[SERVO_ID_NUM] = { 30U, 30U };
    int      p;
    uint8_t  i;

    while (1) {
        /* ---- 步骤 1：Ping 两颗舵机，检查总线是否连通 ---- */
        for (i = 0U; i < SERVO_ID_NUM; i++) {
            (void)STS3032_Ping(id[i]);
            delay_ms(10U);
        }

        /* ---- 步骤 2：读取两颗舵机当前位置（读指令验证半双工接收） ---- */
        for (i = 0U; i < SERVO_ID_NUM; i++) {
            p = STS3032_ReadPosition(id[i]);
            if (p < 0) {
                sts.error_count++;       /* 读取失败计数 */
            }
            delay_ms(10U);
        }

        /* ---- 步骤 3：左右交替摆动（同步写指令验证发送） ---- */
        pos[0] = 2200; pos[1] = 1900;
        STS3032_SyncWritePosEx(id, SERVO_ID_NUM, pos, spd, acc);
        delay_ms(1000U);

        pos[0] = 2300; pos[1] = 1800;
        STS3032_SyncWritePosEx(id, SERVO_ID_NUM, pos, spd, acc);
        delay_ms(1000U);

        pos[0] = 2148; pos[1] = 1948;
        STS3032_SyncWritePosEx(id, SERVO_ID_NUM, pos, spd, acc);
        delay_ms(1000U);

        SelfTest_ReportCan();
    }
}

/** @brief 用例 5：IMU 数据读取 */
static void SelfTest_Imu(void)
{
    while (1) {
        /* 1kHz 更新姿态，CAN 每 20ms 上报一次（上位机可看角度/角速度曲线） */
        MPU6050_Update(CONTROL_DT);
        SelfTest_ReportCan();
        delay_ms(1U);
    }
}

/** @brief 用例 6：CAN 收发（收到指令就回状态） */
static void SelfTest_Can(void)
{
    while (1) {
        /* 只要收到过 0x200，就持续回 0x100/0x101 */
        (void)CAN_Proto_GetCommand();
        SelfTest_ReportCan();
        delay_ms(1U);
    }
}

/** @brief 用例 7：编码器角度读取 */
static void SelfTest_Encoder(void)
{
    while (1) {
        AS5600_UpdateAll();

        /* 用 shaft_velocity 字段上报左右转速、用 angle_y 上报左编码器角度，
           方便在没有示波器的情况下也能通过 CAN 看到编码器是否在动 */
        {
            uint8_t status = 0U;
            if (encoder_left.online)  { status |= 0x02U; }
            if (encoder_right.online) { status |= 0x04U; }

            CAN_Proto_TelemetryTick(
                encoder_left.full_rotations,          /* 左编码器累计角度（rad） */
                encoder_left.angle,                   /* 左编码器机械角（rad） */
                encoder_right.full_rotations,         /* 右编码器累计角度（rad） */
                encoder_right.angle,                  /* 右编码器机械角（rad） */
                current_sense.current_left,
                current_sense.current_right,
                current_sense.battery_voltage,
                status, sts.error_count);
        }
        delay_ms(1U);
    }
}

/** @brief 用例 8：电流采样与过流保护验证 */
static void SelfTest_Current(void)
{
    uint32_t t0;
    uint32_t report_t0;

    /* 先让电机缓慢加载（1.0V 电压矢量旋转），观察电流上升与保护动作 */
    while (1) {
        t0 = millis();
        report_t0 = millis();

        while ((millis() - t0) < 10000U) {
            ADC_Sense_Update();

            /* 每 1ms 执行一次保护判定（与实际控制循环一致） */
            (void)L6234_CurrentProtect(&motor_left,  current_sense.current_left,  CONTROL_DT);
            (void)L6234_CurrentProtect(&motor_right, current_sense.current_right, CONTROL_DT);

            if ((millis() - report_t0) >= CAN_TELEMETRY_PERIOD_MS) {
                report_t0 = millis();
                SelfTest_ReportCan();
            }
            delay_ms(1U);
        }
    }
}

/*==============================================================================
 * 入口
 *============================================================================*/

/**
 * @brief  测试例程主入口（不返回）
 */
void SelfTest_Run(void)
{
    /* ---- 初始化所有被测模块 ---- */
    L6234_Init();
    ADC_Sense_Init();
    AS5600_Init();
    MPU6050_Begin();
    MPU6050_CalcGyroOffsets(MPU6050_GYRO_CALIB_N);
    STS3032_Init();
    CAN_Proto_Init();

    switch (SELFTEST_CASE) {
        case SELFTEST_MOTOR_LEFT:
        case SELFTEST_MOTOR_RIGHT:
        case SELFTEST_MOTOR_BOTH:
            /* 电机用例：先使能驱动（不做 FOC 对齐，纯开环电压旋转） */
            L6234_Enable(&motor_left, 1U);
            L6234_Enable(&motor_right, 1U);
            motor1.enabled = 1U;
            motor2.enabled = 1U;
            motor1.initialized = 1U;
            motor2.initialized = 1U;
            SelfTest_Motor(SELFTEST_CASE);
            break;

        case SELFTEST_SERVO:
            SelfTest_Servo();
            break;

        case SELFTEST_IMU:
            SelfTest_Imu();
            break;

        case SELFTEST_CAN:
            SelfTest_Can();
            break;

        case SELFTEST_ENCODER:
            SelfTest_Encoder();
            break;

        case SELFTEST_CURRENT:
            SelfTest_Current();
            break;

        default:
            /* 未知用例号：只做 CAN 心跳，方便确认程序在跑 */
            while (1) {
                SelfTest_ReportCan();
                delay_ms(1U);
            }
    }
}
