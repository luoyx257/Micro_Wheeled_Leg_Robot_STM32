/**
 ******************************************************************************
 * @file    controller.h
 * @brief   上层控制层：LQR 自平衡 / YAW 转向 / 腿部动作 / 跳跃
 *
 * ★★★ 重要声明（改动点说明）★★★
 *   本文件是原版 ESP32 工程 wl_pro_robot.ino 中上层控制逻辑的【逐行搬运】：
 *     · PIDController / LowPassFilter 的数学实现与 SimpleFOC 库一致
 *     · 所有 PID 系数、低通时间常数、零点常量、限幅值【一字未改】
 *     · lqr_balance_loop / yaw_loop / leg_loop / jump_loop 的公式与分支
 *       【完全保持原版】，只把硬件相关的调用替换为 STM32 驱动接口
 *
 *   原版 → 本移植 的替换对照：
 *     mpu6050.getAngleY()      → MPU6050_GetAngleY()
 *     mpu6050.getGyroY()       → MPU6050_GetGyroY()
 *     mpu6050.getAngleX()      → MPU6050_GetAngleX()
 *     mpu6050.getAngleZ()      → MPU6050_GetAngleZ()
 *     mpu6050.getGyroZ()       → MPU6050_GetGyroZ()
 *     motor1.shaft_angle       → motor1.shaft_angle（同名同义）
 *     motor1.shaft_velocity    → motor1.shaft_velocity（同名同义）
 *     motor1.target / move()   → motor1.target / Motor_Move()
 *     sms_sts.SyncWritePosEx() → STS3032_SyncWritePosEx()
 *     wrobot（WebSocket 更新） → wrobot（CAN 指令更新）
 ******************************************************************************
 */

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"
#include "robot_config.h"
#include "motor.h"

/*==============================================================================
 * PID 控制器（复刻 SimpleFOC PIDController，数学行为完全一致）
 *============================================================================*/

/**
 * @brief PID 控制器
 * @note  参数命名沿用原版的指定初始化语法（.P / .I / .D / .ramp / .limit）
 */
typedef struct {
    float P;                    /**< 比例增益 */
    float I;                    /**< 积分增益 */
    float D;                    /**< 微分增益 */
    float ramp;                 /**< 输出变化率限制（单位/秒） */
    float limit;                /**< 输出上限（对称限幅） ±limit */

    /* ---- 运行时内部状态 ---- */
    float error_prev;           /**< 上一次误差（对应原版 pid.error_prev） */
    float output_prev;          /**< 上一次输出 */
    float integral_prev;        /**< 积分累计 */
    uint32_t timestamp_prev;    /**< 上次调用时间（us） */
} PIDController_t;

/**
 * @brief 一阶低通滤波器（复刻 SimpleFOC LowPassFilter）
 */
typedef struct {
    float Tf;                   /**< 时间常数（s） */
    float y_prev;               /**< 上一次输出 */
    uint32_t timestamp_prev;    /**< 上次调用时间（us） */
} LowPassFilter_t;

/**
 * @brief PID 运算（与原版 PIDController::operator() 数学等价）
 * @param  pid   控制器
 * @param  error 误差
 * @return 控制输出
 */
float PID_Compute(PIDController_t *pid, float error);

/**
 * @brief 低通滤波运算（与原版 LowPassFilter::operator() 数学等价）
 * @param  lpf   滤波器
 * @param  x     输入
 * @return 滤波输出
 */
float LPF_Compute(LowPassFilter_t *lpf, float x);

/*==============================================================================
 * 全局控制器对象（对应原版在 .ino 顶部定义的那一批实例）
 *============================================================================*/

/* ---- PID 实例（系数与原版完全一致，禁止修改） ---- */
extern PIDController_t pid_angle;       /* {.P=1,     .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_gyro;        /* {.P=0.06,  .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_distance;    /* {.P=0.5,   .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_speed;       /* {.P=0.7,   .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_yaw_angle;   /* {.P=1.0,   .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_yaw_gyro;    /* {.P=0.04,  .I=0,  .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_lqr_u;       /* {.P=1,     .I=15, .D=0, .ramp=100000, .limit=8}   */
extern PIDController_t pid_zeropoint;   /* {.P=0.002, .I=0,  .D=0, .ramp=100000, .limit=4}   */
extern PIDController_t pid_roll_angle;  /* {.P=8,     .I=0,  .D=0, .ramp=100000, .limit=450} */

/* ---- 低通滤波器实例（时间常数与原版一致） ---- */
extern LowPassFilter_t lpf_joyy;        /* {.Tf = 0.2} */
extern LowPassFilter_t lpf_zeropoint;   /* {.Tf = 0.1} */
extern LowPassFilter_t lpf_roll;        /* {.Tf = 0.3} */

/* ---- 机器人遥控数据结构（原版 Wrobot wrobot） ---- */
extern Wrobot wrobot;

/* ---- LQR 平衡控制中间量（与原版同名，便于对照调试） ---- */
extern float LQR_angle;
extern float LQR_gyro;
extern float LQR_speed;
extern float LQR_distance;
extern float angle_control;
extern float gyro_control;
extern float speed_control;
extern float distance_control;
extern float LQR_u;
extern float angle_zeropoint;
extern float distance_zeropoint;

/* ---- YAW 轴控制中间量 ---- */
extern float YAW_gyro;
extern float YAW_angle;
extern float YAW_angle_last;
extern float YAW_angle_total;
extern float YAW_angle_zero_point;
extern float YAW_output;

/* ---- 腿部舵机控制量（原版 ID[2] / Position[2] / Speed[2] / ACC[2]） ---- */
extern uint8_t  servo_id[SERVO_ID_NUM];
extern int16_t  servo_position[SERVO_ID_NUM];
extern uint16_t servo_speed[SERVO_ID_NUM];
extern uint8_t  servo_acc[SERVO_ID_NUM];

/* ---- 逻辑标志位 ---- */
extern float robot_speed;               /**< 当前轮部转速 */
extern float robot_speed_last;          /**< 上一时刻轮部转速 */
extern int   wrobot_move_stop_flag;     /**< 摇杆停止标识 */
extern int   jump_flag;                 /**< 跳跃时段标识 */
extern float leg_position_add;          /**< roll 轴平衡控制量 */
extern int   uncontrolable;             /**< 机身倾角过大导致失控 */

/*==============================================================================
 * 控制接口
 *============================================================================*/

/**
 * @brief  控制层初始化（舵机中位、零点复位等，对应原版 setup() 里的控制部分）
 */
void Controller_Init(void);

/**
 * @brief  LQR 自平衡控制（对应原版 lqr_balance_loop，公式一字未改）
 * @param  dt 控制周期（s）
 */
void Controller_LqrBalanceLoop(void);

/**
 * @brief  腿部动作控制（对应原版 leg_loop）
 */
void Controller_LegLoop(void);

/**
 * @brief  跳跃控制（对应原版 jump_loop）
 */
void Controller_JumpLoop(void);

/**
 * @brief  YAW 轴转向控制（对应原版 yaw_loop）
 */
void Controller_YawLoop(void);

/**
 * @brief  yaw 角度累加（对应原版 yaw_angle_addup）
 */
void Controller_YawAngleAddup(void);

/**
 * @brief  从 CAN 指令更新 wrobot（对应原版 web_loop 里 rp.spinOnce() 的作用）
 * @note   把 CAN 收到的 8 字节指令写入 wrobot 各字段，
 *         并在首帧时把 *_last 也同步，避免开机瞬间产生假的"指令归零"事件。
 */
void Controller_UpdateFromCan(void);

/**
 * @brief  把 LQR/YAW 输出下发到两个电机（对应原版 loop() 末尾那几行）
 */
void Controller_ApplyMotorOutput(void);

#endif /* __CONTROLLER_H__ */
