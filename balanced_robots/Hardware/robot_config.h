/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   轮腿机器人（STM32F103C8T6 版）全局硬件配置 / 参数宏定义
 *
 * 说明：
 *   本文件是全部驱动模块的"唯一配置入口"。所有引脚、外设参数、保护阈值、
 *   控制周期宏都集中在此，修改硬件或调参只需要改这一个文件。
 *
 * 硬件对应：
 *   主控  : STM32F103C8T6 (LQFP48, 72MHz, 8MHz HSE)
 *   电机  : 2× L6234PD013TR（三相无刷驱动器，SENSE 100mΩ 采样）
 *   编码器: 2× AS5600（I2C）
 *   IMU   : MPU6050（I2C，与右编码器共用 I2C1）
 *   舵机  : 2× FEETECH STS3032，半双工单总线（USART1 单线模式 + 开漏）
 *   通信  : TJA1050 CAN 收发器（500 kbps，接上位机 USB-CAN）
 *
 * 引脚来源：更新后的 PROTEL 网表（见 HARDWARE_REVIEW.md 第三节引脚确认表）
 ******************************************************************************
 */

#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#include "main.h"

/*==============================================================================
 * 一、全局常量
 *============================================================================*/

/** @brief 圆周率（与 Arduino/ESP32 原版一致，使用 7 位有效数字） */
#ifndef PI
#define PI                      3.1415927f
#endif

/** @brief 2π，减少运行时重复计算 */
#define TWO_PI                  6.2831853f

/** @brief 舵机/上位机协议里表示"无穷远"的哨兵值（沿用原版） */
#define MAGIC_NOT_UPDATED       (-256.0f)

/*==============================================================================
 * 二、系统时基
 *============================================================================*/

/**
 * @brief 控制循环频率（Hz）
 * @note  原版 ESP32 是"不固定周期"的 loop()，靠 millis() 求差分算 dt。
 *        STM32 侧改为固定 1kHz：因为 20kHz PWM 下 while 循环抖动会让
 *        MPU6050 互补滤波的积分项不稳定。所有 dt 取 1/1000。
 */
#define CONTROL_LOOP_HZ         1000U
#define CONTROL_DT              (1.0f / (float)CONTROL_LOOP_HZ)

/** @brief 控制循环周期，单位微秒（用于调度判断） */
#define CONTROL_PERIOD_US       (1000000U / CONTROL_LOOP_HZ)

/*==============================================================================
 * 三、电机与驱动（L6234 ×2）
 *----------------------------------------------------------------------------*/
/*  PCB 丝印对应：
 *    插口1 左电机 L6234：EN1=PA0  IN1=PA1  EN2=PA2  IN2=PB0  EN3=PA3  IN3=PB1
 *    插口2 右电机 L6234：EN1=PA8  IN1=PB12 EN2=PA9  IN2=PB13 EN3=PA10 IN3=PB14
 *============================================================================*/

/** @brief 左电机 PWM 定时器句柄（tim.c 中的 htim2） */
#define MOTOR_L_TIM_HANDLE      htim2
/** @brief 左电机 EN 通道 → 引脚 映射 */
#define MOTOR_L_EN1_PIN         GPIO_PIN_0      /* PA0  - TIM2_CH1 */
#define MOTOR_L_EN2_PIN         GPIO_PIN_2      /* PA2  - TIM2_CH3 */
#define MOTOR_L_EN3_PIN         GPIO_PIN_3      /* PA3  - TIM2_CH4 */
#define MOTOR_L_EN1_CHANNEL     TIM_CHANNEL_1
#define MOTOR_L_EN2_CHANNEL     TIM_CHANNEL_3
#define MOTOR_L_EN3_CHANNEL     TIM_CHANNEL_4
/** @brief 左电机方向脚 IN1/IN2/IN3（GPIO 推挽输出） */
#define MOTOR_L_IN1_PORT        GPIOA           /* PA1  */
#define MOTOR_L_IN1_PIN         GPIO_PIN_1
#define MOTOR_L_IN2_PORT        GPIOB           /* PB0  */
#define MOTOR_L_IN2_PIN         GPIO_PIN_0
#define MOTOR_L_IN3_PORT        GPIOB           /* PB1  */
#define MOTOR_L_IN3_PIN         GPIO_PIN_1

/** @brief 右电机 PWM 定时器句柄（tim.c 中的 htim1） */
#define MOTOR_R_TIM_HANDLE      htim1
/** @brief 右电机 EN 通道 → 引脚 映射 */
#define MOTOR_R_EN1_PIN         GPIO_PIN_8      /* PA8  - TIM1_CH1 */
#define MOTOR_R_EN2_PIN         GPIO_PIN_9      /* PA9  - TIM1_CH2 */
#define MOTOR_R_EN3_PIN         GPIO_PIN_10     /* PA10 - TIM1_CH3 */
#define MOTOR_R_EN1_CHANNEL     TIM_CHANNEL_1
#define MOTOR_R_EN2_CHANNEL     TIM_CHANNEL_2
#define MOTOR_R_EN3_CHANNEL     TIM_CHANNEL_3
/** @brief 右电机方向脚 IN1/IN2/IN3 */
#define MOTOR_R_IN1_PORT        GPIOB           /* PB12 */
#define MOTOR_R_IN1_PIN         GPIO_PIN_12
#define MOTOR_R_IN2_PORT        GPIOB           /* PB13 */
#define MOTOR_R_IN2_PIN         GPIO_PIN_13
#define MOTOR_R_IN3_PORT        GPIOB           /* PB14 */
#define MOTOR_R_IN3_PIN         GPIO_PIN_14

/** @brief PWM 定时器 ARR 值（tim.c: Period = 3599）
 *         72MHz / (3599+1) = 20 kHz，适合无刷电机，无啸叫 */
#define PWM_TIMER_ARR           3599U
/** @brief PWM 定时器时钟频率（TIM1/TIM2 都在 APB2/APB1 的 2 倍频域，均为 72MHz） */
#define PWM_TIMER_CLOCK         72000000U
/** @brief 实际 PWM 频率（Hz），仅供显示/校验 */
#define PWM_FREQUENCY_HZ        (PWM_TIMER_CLOCK / (PWM_TIMER_ARR + 1U))

/** @brief 电机极对数（原版 SimpleFOC: BLDCMotor(7)） */
#define MOTOR_POLE_PAIRS        7.0f

/** @brief 电源电压（原版 driver1.voltage_power_supply = 8，7~9V 电池） */
#define MOTOR_VOLTAGE_SUPPLY    8.0f

/** @brief 单相电压上限（V）。用于限幅，避免 100% 占空比导致相间直通风险 */
#define MOTOR_PHASE_VOLTAGE_MAX 4.5f

/**
 * @brief 控制输出 → 相电压 的标度系数
 * @note  原版 SimpleFOC 中 motor.target 是"电压"(V)，直接作为 Uq 使用。
 *        本移植保持同样语义：target(单位 V) × MOTOR_VOLTAGE_SCALE = 相电压。
 *        若上电调试发现整体增益不对，只改这一个数即可。
 */
#define MOTOR_VOLTAGE_SCALE     1.0f

/*==============================================================================
 * 四、电流采样与保护（L6234 SENSE，100mΩ 采样电阻）
 *----------------------------------------------------------------------------*/
/*  PCB 对应：左电机 PA4(ADC1_IN4/SENSE1+SENSE2)，右电机 PA5(ADC1_IN5/SENSE2)、
 *            电池电压 PA6(ADC1_IN6)
 *============================================================================*/

/** @brief 采样电阻阻值（Ω）。网表：R5/R8/R9/R10 = FRL0603FR100TS = 100mΩ */
#define CURRENT_SENSE_RESISTOR  0.1f

/** @brief ADC 参考电压（V）—— STM32F103 的 VDDA */
#define ADC_VREF                3.3f
/** @brief ADC 满量程（12bit） */
#define ADC_FULL_SCALE          4095.0f

/**
 * @brief ADC 原始值 → 电流（A）换算系数
 *        I = ADC_RAW × (ADC_VREF / ADC_FULL_SCALE) / CURRENT_SENSE_RESISTOR
 *          = ADC_RAW × (3.3 / 4095) / 0.1 = ADC_RAW × 0.008058…
 *        即 1 个 LSB ≈ 8.06 mA
 */
#define ADC_TO_CURRENT          (ADC_VREF / ADC_FULL_SCALE / CURRENT_SENSE_RESISTOR)

/** @brief 电流采样滑动平均窗口长度（用于抑制 20kHz PWM 开关噪声） */
#define CURRENT_FILTER_WINDOW   4U

/** @brief 过流保护阈值（A）—— 超过即限幅并把该电机输出钳到 0
 *         L6234 连续输出 4A、峰值 5A；取 3.0A 作为堵转保护点 */
#define MOTOR_CURRENT_LIMIT_A   3.0f

/** @brief 电流"持续超限"判定次数：连续 N 次超限才触发保护（防误触发） */
#define MOTOR_OVER_CURRENT_CNT  3U

/** @brief 保护触发后的锁定时间（ms）。此期间该电机输出被强制为 0 */
#define MOTOR_FAULT_LOCK_MS     1000U

/** @brief 电池电压换算系数（宏，便于打板后实测校准）
 *        Vbat = ADC_RAW × ADC_VREF / ADC_FULL_SCALE × BAT_ADC_DIV_RATIO
 *        等效分压比约 1/3（R3 10k 上臂 : R2 10k ∥ R3 10k 下臂），
 *        BOOT0 支路会使得实际值偏移，打板后请实测校准本宏。
 */
#define BAT_ADC_DIV_RATIO       3.0f

/** @brief 低电量报警阈值（V），对应原版 LED_BAT 判据 battery > 7.8 */
#define BAT_LOW_THRESHOLD_V     7.8f

/*==============================================================================
 * 五、AS5600 磁性编码器
 *----------------------------------------------------------------------------*/
/*  PCB 对应：插口3 左 AS5600（I2C2，PB10=SCL/PB11=SDA）
 *            插口4 右 AS5600 + MPU6050（I2C1，PB8=SCL/PB9=SDA）
 *============================================================================*/

/** @brief AS5600 7 位从机地址（固定，不可改） */
#define AS5600_I2C_ADDR         (0x36U << 1)

/** @brief AS5600 寄存器：原始角度（12bit，包含在 2 字节中） */
#define AS5600_REG_RAW_ANGLE    0x0CU

/** @brief AS5600 寄存器：状态（MD/ML/MH，用于判断磁铁是否正常） */
#define AS5600_REG_STATUS       0x0BU

/** @brief AS5600 状态位：检测到磁铁 */
#define AS5600_STATUS_MD        (1U << 5)

/** @brief AS5600 原始值满量程（12bit） */
#define AS5600_RAW_MAX          4096.0f

/** @brief I2C 超时（ms）。HAL I2C 卡死时靠它返回，避免控制循环被拖死 */
#define AS5600_I2C_TIMEOUT_MS   2U

/** @brief 编码器角度方向（+1 / -1）。若上电对齐发现方向相反，只改这里 */
#define ENCODER_L_DIRECTION     1.0f
#define ENCODER_R_DIRECTION     1.0f

/** @brief 轮部速度低通滤波时间常数（s）—— 原版 SimpleFOC 默认 LPF Tf = 0.01 */
#define SHAFT_VELOCITY_LPF_TF   0.01f

/*==============================================================================
 * 六、MPU6050 IMU / 姿态解算
 *----------------------------------------------------------------------------*/
/*  PCB 对应：插口4（I2C1，与右 AS5600 共用总线）
 *  从机地址 0x68（AD0 接地）
 *============================================================================*/

/** @brief MPU6050 7 位从机地址（AD0=0） */
#define MPU6050_I2C_ADDR        (0x68U << 1)

/** @brief I2C 超时（ms） */
#define MPU6050_I2C_TIMEOUT_MS  5U

/** @brief 陀螺仪零偏标定采样次数（原版 3000 次；STM32 上取 1000 次 ≈ 1s） */
#define MPU6050_GYRO_CALIB_N    1000U

/** @brief 互补滤波系数（必须与 MPU6050_tockn 库一致）
 *         angle = gyroCoef × (angle + gyro×dt) + accCoef × angleAcc
 *         原版构造函数：accCoef = 0.02f, gyroCoef = 0.98f */
#define MPU6050_ACC_COEF        0.02f
#define MPU6050_GYRO_COEF       0.98f

/** @brief 陀螺仪量程 ±500°/s 对应的灵敏度（LSB/(°/s)），原版 GYRO_CONFIG=0x08 */
#define MPU6050_GYRO_LSB_PER_DPS 65.5f

/** @brief 加速度计量程 ±2g 对应的灵敏度（LSB/g），原版 ACCEL_CONFIG=0x00 */
#define MPU6050_ACC_LSB_PER_G   16384.0f

/*==============================================================================
 * 七、STS3032 半双工总线舵机
 *----------------------------------------------------------------------------*/
/*  硬件拓扑（已按最终网表确认）：
 *      +3.3V ──R6(1k)── DATA ──┬── CN1-3 / CN2-3（两颗 STS3032 并联）
 *                              └── R7(1k) ── PB6 (USART1_TX)
 *    USART1 配置为 Half-Duplex(single_wire_mode)，PB6 为复用开漏(AF_OD)：
 *      · 发送：推挽/开漏拉低总线，空闲由 R6 上拉到 3.3V
 *      · 接收：TX 释放总线，舵机驱动 DATA，MCU 从同一根 PB6 收
 *    → 无需任何方向控制引脚（PB7 已取消）
 *============================================================================*/

/** @brief 舵机总线波特率（STS3032 出厂默认 1 Mbps，8N1） */
#define SCS_BAUDRATE            1000000U

/** @brief 两颗舵机 ID（左/右），沿用原版 */
#define SERVO_ID_LEFT           1U
#define SERVO_ID_RIGHT          2U
#define SERVO_ID_NUM            2U

/** @brief 舵机内存表地址（SMS_STS 系列） */
#define SMS_STS_ACC             41U     /* 加速度 */
#define SMS_STS_GOAL_POSITION   42U     /* 目标位置（2 字节） */
#define SMS_STS_GOAL_SPEED      46U     /* 目标速度（2 字节） */
#define SMS_STS_PRESENT_POSITION 56U    /* 当前位置（2 字节，只读） */
#define SMS_STS_PRESENT_SPEED   58U     /* 当前速度（2 字节，只读） */

/** @brief 指令码 */
#define INST_PING               0x01U
#define INST_READ               0x02U
#define INST_WRITE              0x03U
#define INST_REG_WRITE          0x04U
#define INST_REG_ACTION         0x05U
#define INST_SYNC_READ          0x82U
#define INST_SYNC_WRITE         0x83U

/** @brief 舵机中位与有效行程（原版：中位 2048，行程 450） */
#define SERVO_CENTER            2048
#define SERVO_TRAVEL            450

/** @brief 单字节收发超时（ms）。1Mbps 下 1 字节 ≈ 10us，1ms 足够宽松 */
#define SCS_IO_TIMEOUT_MS       2U

/** @brief 一帧最大长度（同步写 2 舵机 × 7 数据 + 7 头 = 21，留余量） */
#define SCS_BUF_SIZE            32U

/*==============================================================================
 * 八、CAN 通信（TJA1050，接上位机 USB-CAN）
 *----------------------------------------------------------------------------*/
/*  位时序（已在 can.c 配置好）：
 *      APB1 = 36MHz, Prescaler = 4, BS1 = 10TQ, BS2 = 5TQ, SJW = 1TQ
 *      位时间 = 1 + 10 + 5 = 16 TQ → 36MHz/(4×16) = 500 kbps，采样点 68.75%
 *============================================================================*/

/** @brief 机器人 → 上位机：状态上报的 CAN ID */
#define CAN_ID_TELEMETRY        0x100U
/** @brief 上位机 → 机器人：控制指令的 CAN ID */
#define CAN_ID_COMMAND          0x200U
/** @brief 上位机 → 机器人：参数调试（PID/LPF）的 CAN ID */
#define CAN_ID_PARAM            0x201U

/** @brief 状态上报周期（ms） */
#define CAN_TELEMETRY_PERIOD_MS 20U

/** @brief CAN 发送超时（ms） */
#define CAN_TX_TIMEOUT_MS       5U

/*==============================================================================
 * 九、上层控制参数（与原版 ESP32 代码一一对应，禁止随意改动）
 *============================================================================*/

/** @brief 机器人运动状态枚举（原版 robot.h 的 QR_State_t，顺序必须一致） */
typedef enum {
    FORWARD = 0,
    BACK,
    RIGHT,
    LEFT,
    STOP,
    JUMP,
} QR_State_t;

/** @brief 机器人模式枚举（原版 Robot_Mode_t） */
typedef enum {
    BASIC = 0,
} Robot_Mode_t;

/** @brief 摇杆/遥控数据结构（原版 Wrobot，字段与默认值完全一致） */
typedef struct {
    int height;         /**< 机身高度，默认 38 */
    int roll;
    int linear;
    int angular;
    int dir;            /**< 方向指令（QR_State_t） */
    int dir_last;
    int joyy;           /**< 前后摇杆 */
    int joyy_last;
    int joyx;           /**< 左右摇杆 */
    int joyx_last;
    int go;             /**< 1=使能输出（原版 bool go） */
} Wrobot;

/** @brief 机器人默认高度（原版 Wrobot.height = 38） */
#define WROBOT_DEFAULT_HEIGHT   38

/** @brief 失控保护：机身倾角超过该值判为倒地（原版 25.0°） */
#define UNCONTROL_ANGLE_DEG     25.0f
/** @brief 扶起恢复：倾角小于该值开始计数（原版 10.0°） */
#define RECOVER_ANGLE_DEG       10.0f
/** @brief 恢复所需循环次数（原版 200 次 @ 不定周期；1kHz 下对应 0.2s 量级） */
#define RECOVER_COUNT           200

/*==============================================================================
 * 十、调试开关
 *============================================================================*/

/** @brief 1 = 进入自检测试模式（不跑平衡控制），0 = 正常平衡控制
 *  @note  测试模式下的用例号见 selftest.h */
#define DEBUG_SELFTEST_ENABLE   0

/** @brief 过流保护累计触发次数（只读，供上报与调试观察） */
/* 定义在 l6234.c 中 */

#endif /* __ROBOT_CONFIG_H__ */
