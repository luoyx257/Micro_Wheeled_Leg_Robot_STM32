/**
 ******************************************************************************
 * @file    controller.c
 * @brief   上层控制层实现：PID / LPF / LQR / YAW / 腿部 / 跳跃
 *
 * ★★★ 本文件是原版 ESP32 代码的逐行搬运，所有系数与公式保持不变 ★★★
 *   来源：wl_pro_robot.ino（Mu Shibo, 2024）
 *   仅替换硬件访问接口，控制数学一行未改。
 ******************************************************************************
 */

#include "controller.h"
#include "hardware.h"
#include "mpu6050.h"
#include "as5600.h"
#include "sts3032.h"
#include "can_proto.h"
#include <math.h>

/*==============================================================================
 * PID / LPF 实现（复刻 SimpleFOC 的数学行为）
 *============================================================================*/

/**
 * @brief PID 运算
 *
 * ★ 本函数逐行复刻 SimpleFOC 库的 PIDController::operator()（pid.cpp），
 *   行为完全一致，包括：
 *     · Ts 异常保护：Ts <= 0 或 Ts > 0.5 时取 1e-3
 *     · 积分采用 Tustin（梯形）变换： integral += I×Ts×0.5×(e + e_prev)
 *     · 积分值本身被限幅到 ±limit（抗饱和）
 *     · 输出 = P×e + integral + D×(e − e_prev)/Ts，再限幅到 ±limit
 *     · 输出变化率限制（output_ramp），注意是在限幅之后再判断速率
 *     · integral_prev 保存的是"积分项的值"，不是误差
 */
float PID_Compute(PIDController_t *pid, float error)
{
    uint32_t now;
    float    Ts;
    float    proportional;
    float    integral;
    float    derivative;
    float    output;
    float    output_rate;

    if (pid == NULL) {
        return 0.0f;
    }

    /* ---- 1. 计算时间步长（原版：Ts = (now - prev)×1e-6，异常时取 1e-3） ---- */
    now = micros();
    Ts  = (float)(now - pid->timestamp_prev) * 1e-6f;
    if ((Ts <= 0.0f) || (Ts > 0.5f)) {
        Ts = 1e-3f;
    }

    /* ---- 2. 比例项 ---- */
    proportional = pid->P * error;

    /* ---- 3. 积分项（Tustin 变换，与原版完全一致） ---- */
    integral = pid->integral_prev
             + (pid->I * Ts * 0.5f * (error + pid->error_prev));
    /* 抗饱和：积分值限幅到 ±limit */
    if (integral >  pid->limit) { integral =  pid->limit; }
    if (integral < -pid->limit) { integral = -pid->limit; }

    /* ---- 4. 微分项 ---- */
    derivative = pid->D * (error - pid->error_prev) / Ts;

    /* ---- 5. 求和并限幅 ---- */
    output = proportional + integral + derivative;
    if (output >  pid->limit) { output =  pid->limit; }
    if (output < -pid->limit) { output = -pid->limit; }

    /* ---- 6. 输出变化率限制（与原版一致：先限幅，再判速率） ---- */
    if (pid->ramp > 0.0f) {
        output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->ramp) {
            output = pid->output_prev + (pid->ramp * Ts);
        } else if (output_rate < -pid->ramp) {
            output = pid->output_prev - (pid->ramp * Ts);
        }
    }

    /* ---- 7. 保存状态（供下一次调用） ---- */
    pid->integral_prev  = integral;
    pid->output_prev    = output;
    pid->error_prev     = error;
    pid->timestamp_prev = now;

    return output;
}

/**
 * @brief 一阶低通滤波
 *
 * ★ 逐行复刻 SimpleFOC 的 LowPassFilter::operator()（lowpass_filter.cpp）：
 *     · dt < 0        → dt = 1e-3
 *     · dt > 0.3      → 认为间隔过长，直接输出输入值并重置（原版的特殊分支）
 *     · alpha = Tf/(Tf+dt)； y = alpha×y_prev + (1−alpha)×x
 */
float LPF_Compute(LowPassFilter_t *lpf, float x)
{
    uint32_t now;
    float    dt;
    float    alpha;
    float    y;

    if (lpf == NULL) {
        return x;
    }

    now = micros();
    dt  = (float)(now - lpf->timestamp_prev) * 1e-6f;

    if (dt < 0.0f) {
        dt = 1e-3f;
    } else if (dt > 0.3f) {
        /* 间隔过长：直接输出并重置，避免引入错误的历史权重 */
        lpf->y_prev          = x;
        lpf->timestamp_prev  = now;
        return x;
    }

    alpha = lpf->Tf / (lpf->Tf + dt);
    y = (alpha * lpf->y_prev) + ((1.0f - alpha) * x);

    lpf->y_prev         = y;
    lpf->timestamp_prev = now;

    return y;
}

/*==============================================================================
 * PID / LPF 实例定义（系数与原版 wl_pro_robot.ino 完全一致）
 *============================================================================*/

PIDController_t pid_angle      = { .P = 1.0f,    .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_gyro       = { .P = 0.06f,   .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_distance   = { .P = 0.5f,    .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_speed      = { .P = 0.7f,    .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_yaw_angle  = { .P = 1.0f,    .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_yaw_gyro   = { .P = 0.04f,   .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_lqr_u      = { .P = 1.0f,    .I = 15.0f, .D = 0.0f, .ramp = 100000.0f, .limit = 8.0f   };
PIDController_t pid_zeropoint  = { .P = 0.002f,  .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 4.0f   };
PIDController_t pid_roll_angle = { .P = 8.0f,    .I = 0.0f,  .D = 0.0f, .ramp = 100000.0f, .limit = 450.0f };

LowPassFilter_t lpf_joyy      = { .Tf = 0.2f };
LowPassFilter_t lpf_zeropoint = { .Tf = 0.1f };
LowPassFilter_t lpf_roll      = { .Tf = 0.3f };

/*==============================================================================
 * 全局控制量（与原版同名）
 *============================================================================*/

/** @brief 机器人遥控数据（原版 Wrobot wrobot，默认高度 38） */
Wrobot wrobot = {
    .height    = WROBOT_DEFAULT_HEIGHT,
    .roll      = 0,
    .linear    = 0,
    .angular   = 0,
    .dir       = 0,
    .dir_last  = 0,
    .joyy      = 0,
    .joyy_last = 0,
    .joyx      = 0,
    .joyx_last = 0,
    .go        = 0,
};

/* ---- LQR 平衡控制数据（原版初值：angle_zeropoint = -2.25，
 *      distance_zeropoint = -256.0 作为"未刷新"标志） ---- */
float LQR_angle        = 0.0f;
float LQR_gyro         = 0.0f;
float LQR_speed        = 0.0f;
float LQR_distance     = 0.0f;
float angle_control    = 0.0f;
float gyro_control     = 0.0f;
float speed_control    = 0.0f;
float distance_control = 0.0f;
float LQR_u            = 0.0f;
float angle_zeropoint  = -2.25f;
float distance_zeropoint = MAGIC_NOT_UPDATED;

/* ---- YAW 轴控制数据（原版初值：YAW_angle_zero_point = -10） ---- */
float YAW_gyro            = 0.0f;
float YAW_angle           = 0.0f;
float YAW_angle_last      = 0.0f;
float YAW_angle_total     = 0.0f;
float YAW_angle_zero_point = -10.0f;
float YAW_output          = 0.0f;

/* ---- 腿部舵机控制数据（原版 ID/Position/Speed/ACC） ---- */
uint8_t  servo_id[SERVO_ID_NUM]       = { SERVO_ID_LEFT, SERVO_ID_RIGHT };
int16_t  servo_position[SERVO_ID_NUM] = { 2148, 1948 };
uint16_t servo_speed[SERVO_ID_NUM]    = { 300U, 300U };
uint8_t  servo_acc[SERVO_ID_NUM]      = { 30U, 30U };

/* ---- 逻辑标志位 ---- */
float robot_speed        = 0.0f;
float robot_speed_last   = 0.0f;
int   wrobot_move_stop_flag = 0;
int   jump_flag          = 0;
float leg_position_add   = 0.0f;
int   uncontrolable      = 0;

/** @brief 首帧指令同步标志（避免开机时误判"摇杆归零"事件） */
static uint8_t s_first_cmd = 1U;

/*==============================================================================
 * 接口实现
 *============================================================================*/

/**
 * @brief  控制层初始化（对应原版 setup() 中的舵机中位设置部分）
 *
 * 原版 setup()：
 *   sms_sts.pSerial = &Serial2;
 *   ID[0]=1; ID[1]=2; ACC=30/30; Speed=300/300; Position=2148/1948;
 *   sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
 */
void Controller_Init(void)
{
    /* ---- 复位所有 PID 内部状态（原版上电即为零） ---- */
    PIDController_t *pid_list[] = {
        &pid_angle, &pid_gyro, &pid_distance, &pid_speed,
        &pid_yaw_angle, &pid_yaw_gyro, &pid_lqr_u,
        &pid_zeropoint, &pid_roll_angle
    };
    uint8_t i;

    for (i = 0U; i < (sizeof(pid_list) / sizeof(pid_list[0])); i++) {
        pid_list[i]->error_prev     = 0.0f;
        pid_list[i]->output_prev    = 0.0f;
        pid_list[i]->integral_prev  = 0.0f;
        pid_list[i]->timestamp_prev = micros();
    }

    lpf_joyy.y_prev            = 0.0f;
    lpf_joyy.timestamp_prev    = micros();
    lpf_zeropoint.y_prev       = 0.0f;
    lpf_zeropoint.timestamp_prev = micros();
    lpf_roll.y_prev            = 0.0f;
    lpf_roll.timestamp_prev    = micros();

    /* ---- 舵机初始位置（与原版 setup() 完全一致） ---- */
    servo_id[0]       = SERVO_ID_LEFT;
    servo_id[1]       = SERVO_ID_RIGHT;
    servo_acc[0]      = 30U;
    servo_acc[1]      = 30U;
    servo_speed[0]    = 300U;
    servo_speed[1]    = 300U;
    servo_position[0] = 2148;
    servo_position[1] = 1948;

    /* 舵机(ID1/ID2)以速度 300 步/秒、加速度 30 运行至各自 Position */
    STS3032_SyncWritePosEx(servo_id, SERVO_ID_NUM, servo_position,
                           servo_speed, servo_acc);
}

/**
 * @brief  从 CAN 指令更新 wrobot
 * @note   对应原版 web_loop() → rp.spinOnce() 的效果：
 *         把上位机指令写入 wrobot 结构，供各控制环使用。
 */
void Controller_UpdateFromCan(void)
{
    const CanCommand_t *cmd = CAN_Proto_GetCommand();

    if (cmd->updated == 0U) {
        return;
    }

    wrobot.dir     = (int)cmd->dir;
    wrobot.height  = (int)cmd->height;
    wrobot.roll    = (int)cmd->roll;
    wrobot.linear  = (int)cmd->linear;
    wrobot.angular = (int)cmd->angular;
    wrobot.go      = (int)cmd->stable;
    wrobot.joyx    = (int)cmd->joy_x;
    wrobot.joyy    = (int)cmd->joy_y;

    if (s_first_cmd) {
        /* 首帧：把"上一次"同步为当前值，避免开机瞬间触发
           "运动指令复零"的原地停车逻辑 */
        wrobot.dir_last  = wrobot.dir;
        wrobot.joyx_last = wrobot.joyx;
        wrobot.joyy_last = wrobot.joyy;
        s_first_cmd      = 0U;
    }

    CAN_Proto_ClearUpdated();
}

/**
 * @brief  LQR 自平衡控制
 * @note   与原版 lqr_balance_loop() 逐行对应，公式与分支完全一致。
 */
void Controller_LqrBalanceLoop(void)
{
    /* QR_u = LQR_k1*(angle - angle_zeropoint) + LQR_k2*gyro
              + LQR_k3*(distance - distance_zeropoint) + LQR_k4*speed
       给负值是因为按照当前的电机接线，正转矩会向后转 */
    LQR_distance = (-0.5f) * (motor1.shaft_angle + motor2.shaft_angle);
    LQR_speed    = (-0.5f) * (motor1.shaft_velocity + motor2.shaft_velocity);
    LQR_angle    = (float)MPU6050_GetAngleY();
    LQR_gyro     = (float)MPU6050_GetGyroY();

    /* 计算自平衡输出 */
    angle_control = PID_Compute(&pid_angle, LQR_angle - angle_zeropoint);
    gyro_control  = PID_Compute(&pid_gyro, LQR_gyro);

    /* 运动细节优化处理 */
    if (wrobot.joyy != 0) {             /* 有前后方向运动指令时的处理 */
        distance_zeropoint   = LQR_distance;    /* 位移零点重置 */
        pid_lqr_u.error_prev = 0.0f;            /* 输出积分清零 */
    }

    /* 运动指令复零时的原地停车处理 */
    if (((wrobot.joyx_last != 0) && (wrobot.joyx == 0)) ||
        ((wrobot.joyy_last != 0) && (wrobot.joyy == 0))) {
        wrobot_move_stop_flag = 1;
    }
    if ((wrobot_move_stop_flag == 1) && (fabsf(LQR_speed) < 0.5f)) {
        distance_zeropoint    = LQR_distance;   /* 位移零点重置 */
        wrobot_move_stop_flag = 0;
    }

    /* 被快速推动时的原地停车处理 */
    if (fabsf(LQR_speed) > 15.0f) {
        distance_zeropoint = LQR_distance;
    }

    /* 计算位移控制输出 */
    distance_control = PID_Compute(&pid_distance, LQR_distance - distance_zeropoint);
    speed_control    = PID_Compute(&pid_speed,
                                   LQR_speed - (0.1f * LPF_Compute(&lpf_joyy, (float)wrobot.joyy)));

    /* 轮部离地检测 */
    robot_speed_last = robot_speed;
    robot_speed      = LQR_speed;

    if ((fabsf(robot_speed - robot_speed_last) > 10.0f) ||
        (fabsf(robot_speed) > 50.0f) ||
        (jump_flag != 0)) {
        /* 轮部离地：位移零点重置，只输出角度+角速度分量 */
        distance_zeropoint   = LQR_distance;
        LQR_u                = angle_control + gyro_control;
        pid_lqr_u.error_prev = 0.0f;
    } else {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    /* 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期 */
    if ((fabsf(LQR_u) < 5.0f) && (wrobot.joyy == 0) &&
        (fabsf(distance_control) < 4.0f) && (jump_flag == 0)) {
        LQR_u = PID_Compute(&pid_lqr_u, LQR_u);     /* 补偿小转矩非线性 */
        /* 重心自适应 */
        angle_zeropoint -= PID_Compute(&pid_zeropoint,
                                       LPF_Compute(&lpf_zeropoint, distance_control));
    } else {
        pid_lqr_u.error_prev = 0.0f;                /* 输出积分清零 */
    }

    /* 平衡控制参数自适应 */
    if (wrobot.height < 50) {
        pid_speed.P = 0.7f;
    } else if (wrobot.height < 64) {
        pid_speed.P = 0.6f;
    } else {
        pid_speed.P = 0.5f;
    }
}

/**
 * @brief  腿部动作控制（对应原版 leg_loop）
 */
void Controller_LegLoop(void)
{
    Controller_JumpLoop();

    if (jump_flag == 0) {               /* 不处于跳跃状态 */
        /* 机身高度自适应控制 */
        servo_acc[0] = 8U;
        servo_acc[1] = 8U;
        servo_speed[0] = 200U;
        servo_speed[1] = 200U;

        {
            float roll_angle = (float)MPU6050_GetAngleX() + 2.0f;
            /* 原版两条实现，用后者（带 lpf_roll） */
            leg_position_add = PID_Compute(&pid_roll_angle,
                                           LPF_Compute(&lpf_roll, roll_angle));
        }

        servo_position[0] = (int16_t)(2048 + 12 + 8.4f * ((float)wrobot.height - 32.0f)
                                      - leg_position_add);
        servo_position[1] = (int16_t)(2048 - 12 - 8.4f * ((float)wrobot.height - 32.0f)
                                      - leg_position_add);

        /* 与原版一致的硬限幅 */
        if (servo_position[0] < 2110) {
            servo_position[0] = 2110;
        } else if (servo_position[0] > 2510) {
            servo_position[0] = 2510;
        }

        if (servo_position[1] < 1586) {
            servo_position[1] = 1586;
        } else if (servo_position[1] > 1986) {
            servo_position[1] = 1986;
        }

        STS3032_SyncWritePosEx(servo_id, SERVO_ID_NUM, servo_position,
                               servo_speed, servo_acc);
    }
}

/**
 * @brief  跳跃控制（对应原版 jump_loop）
 */
void Controller_JumpLoop(void)
{
    if ((wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0)) {
        servo_acc[0] = 0U;
        servo_acc[1] = 0U;
        servo_speed[0] = 0U;
        servo_speed[1] = 0U;
        servo_position[0] = (int16_t)(2048 + 12 + 8.4f * (80.0f - 32.0f));
        servo_position[1] = (int16_t)(2048 - 12 - 8.4f * (80.0f - 32.0f));
        STS3032_SyncWritePosEx(servo_id, SERVO_ID_NUM, servo_position,
                               servo_speed, servo_acc);

        jump_flag = 1;
    }

    if (jump_flag > 0) {
        jump_flag++;
        if ((jump_flag > 30) && (jump_flag < 35)) {
            servo_acc[0] = 0U;
            servo_acc[1] = 0U;
            servo_speed[0] = 0U;
            servo_speed[1] = 0U;
            servo_position[0] = (int16_t)(2048 + 12 + 8.4f * (40.0f - 32.0f));
            servo_position[1] = (int16_t)(2048 - 12 - 8.4f * (40.0f - 32.0f));
            STS3032_SyncWritePosEx(servo_id, SERVO_ID_NUM, servo_position,
                                   servo_speed, servo_acc);

            jump_flag = 40;
        }
        if (jump_flag > 200) {
            jump_flag = 0;              /* 已准备好再次跳跃 */
        }
    }
}

/**
 * @brief  YAW 轴转向控制（对应原版 yaw_loop）
 */
void Controller_YawLoop(void)
{
    /* YAW_output = 0.03*(YAW_Kp*YAW_angle_total + YAW_Kd*YAW_gyro); */
    Controller_YawAngleAddup();

    YAW_angle_total += (float)wrobot.joyx * 0.002f;

    {
        float yaw_angle_control = PID_Compute(&pid_yaw_angle, YAW_angle_total);
        float yaw_gyro_control  = PID_Compute(&pid_yaw_gyro, YAW_gyro);
        YAW_output = yaw_angle_control + yaw_gyro_control;
    }
}

/**
 * @brief  yaw 轴角度累加（对应原版 yaw_angle_addup）
 * @note   算法：跨 ±π 时选择"更短的转角"，避免 2π 跳变。
 */
void Controller_YawAngleAddup(void)
{
    float yaw_angle_1;
    float yaw_angle_2;
    float yaw_addup_angle;

    YAW_angle = (float)MPU6050_GetAngleZ();
    YAW_gyro  = (float)MPU6050_GetGyroZ();

    if (YAW_angle_zero_point == (-10.0f)) {
        YAW_angle_zero_point = YAW_angle;
    }

    if (YAW_angle > YAW_angle_last) {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last - (float)TWO_PI;
    } else {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last + (float)TWO_PI;
    }

    if (fabsf(yaw_angle_1) > fabsf(yaw_angle_2)) {
        yaw_addup_angle = yaw_angle_2;
    } else {
        yaw_addup_angle = yaw_angle_1;
    }

    YAW_angle_total += yaw_addup_angle;
    YAW_angle_last   = YAW_angle;
}

/**
 * @brief  把 LQR / YAW 输出下发到两个电机（对应原版 loop() 末尾）
 */
void Controller_ApplyMotorOutput(void)
{
    /* 将自平衡计算输出转矩赋给电机 */
    motor1.target = (-0.5f) * (LQR_u + YAW_output);
    motor2.target = (-0.5f) * (LQR_u - YAW_output);

    /* 倒地失控后关闭输出 */
    if (fabsf(LQR_angle) > UNCONTROL_ANGLE_DEG) {
        uncontrolable = 1;
    }

    if (uncontrolable != 0) {           /* 扶起后延时恢复 */
        if (fabsf(LQR_angle) < RECOVER_ANGLE_DEG) {
            uncontrolable++;
        }
        if (uncontrolable > RECOVER_COUNT) {
            uncontrolable = 0;
        }
    }

    /* 关停输出（遥控停止或角度过大失控） */
    if ((wrobot.go == 0) || (uncontrolable != 0)) {
        motor1.target = 0.0f;
        motor2.target = 0.0f;
        leg_position_add = 0.0f;
    }

    /* 记录上一次的遥控数据 */
    wrobot.dir_last  = wrobot.dir;
    wrobot.joyx_last = wrobot.joyx;
    wrobot.joyy_last = wrobot.joyy;
}
