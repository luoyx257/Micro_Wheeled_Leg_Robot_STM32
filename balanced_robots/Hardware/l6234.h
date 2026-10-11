/**
 ******************************************************************************
 * @file    l6234.h
 * @brief   L6234 三相无刷电机驱动模块（PWM 输出 + 方向控制 + ADC 电流采样 + 堵转保护）
 *
 * 硬件说明（L6234 真值表）：
 *   ENx = 0                        → 该半桥输出高阻（关闭）
 *   ENx = 1 & INx = 1              → OUTx 输出高（接 Vs）
 *   ENx = 1 & INx = 0              → OUTx 输出低（接 GND）
 *
 * 本移植采用"电压模式 3 相开环 + 电角度闭环"（B 方案）：
 *   · IN1/IN2/IN3 全部固定为高电平（正向模式）
 *   · EN1/EN2/EN3 输出 PWM，占空比 d 的等效相电压 = d × Vs
 *   · 三相都输出 50% 占空比时，相间电压差为 0（对应电压矢量 0）
 * 因此"相电压 Um ∈ [-Vs/2, +Vs/2]"映射为占空比 "0.5 + Um/Vs"，
 * 与 SimpleFOC 的 setPhaseVoltage 语义保持一致。
 *
 * PCB 引脚（丝印对应，详见 HARDWARE_REVIEW.md）：
 *   左电机：EN1=PA0(TIM2_CH1) IN1=PA1 | EN2=PA2(TIM2_CH3) IN2=PB0 | EN3=PA3(TIM2_CH4) IN3=PB1
 *   右电机：EN1=PA8(TIM1_CH1) IN1=PB12| EN2=PA9(TIM1_CH2) IN2=PB13| EN3=PA10(TIM1_CH3)IN3=PB14
 *   电流采样：左 PA4(ADC1_IN4)、右 PA5(ADC1_IN5)、电池 PA6(ADC1_IN6)
 ******************************************************************************
 */

#ifndef __L6234_H__
#define __L6234_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

/**
 * @brief L6234 驱动实例
 */
typedef struct {
    TIM_HandleTypeDef *htim;        /**< PWM 定时器句柄 */
    uint32_t ch1;                   /**< EN1 对应的定时器通道 */
    uint32_t ch2;                   /**< EN2 对应的定时器通道 */
    uint32_t ch3;                   /**< EN3 对应的定时器通道 */

    GPIO_TypeDef *in1_port;         /**< IN1 端口 */
    uint16_t      in1_pin;          /**< IN1 引脚 */
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    GPIO_TypeDef *in3_port;
    uint16_t      in3_pin;

    float supply_voltage;           /**< 电源电压 Vs（V） */
    uint8_t enabled;                /**< 1 = 已使能输出 */
} L6234_t;

/**
 * @brief 电流/电压采样数据
 */
typedef struct {
    uint16_t raw_left;              /**< 左电机电流 ADC 原始值 */
    uint16_t raw_right;             /**< 右电机电流 ADC 原始值 */
    uint16_t raw_battery;           /**< 电池电压 ADC 原始值 */

    float current_left;             /**< 左电机电流（A），已滤波 */
    float current_right;            /**< 右电机电流（A），已滤波 */
    float battery_voltage;          /**< 电池电压（V） */
} CurrentSense_t;

/*==============================================================================
 * 全局实例（定义在 l6234.c 中）
 *============================================================================*/

extern L6234_t       motor_left;
extern L6234_t       motor_right;
extern CurrentSense_t current_sense;

/** @brief 过流保护累计触发次数（调试观察用） */
extern volatile uint32_t over_current_count;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  初始化两个 L6234 驱动（启动 PWM、方向脚置正向、输出关闭）
 * @note   必须在 MX_TIM1_Init()/MX_TIM2_Init()/MX_GPIO_Init() 之后调用。
 */
void L6234_Init(void);

/**
 * @brief  使能/关闭某路 L6234 输出
 * @param  dev     驱动实例指针
 * @param  enable  1 = 使能，0 = 关闭（三相占空比归 0、方向脚拉低）
 */
void L6234_Enable(L6234_t *dev, uint8_t enable);

/**
 * @brief  设置单相输出电压（占空比形式）
 * @param  dev  驱动实例指针
 * @param  ch   定时器通道（EN1/EN2/EN3）
 * @param  duty 占空比 0.0 ~ 1.0（0.5 表示相电压 0）
 */
void L6234_SetDuty(L6234_t *dev, uint32_t ch, float duty);

/**
 * @brief  三相统一设置（电压模式核心接口）
 * @param  dev  驱动实例指针
 * @param  u1   第 1 相电压（V，范围 ±Vs/2）
 * @param  u2   第 2 相电压（V）
 * @param  u3   第 3 相电压（V）
 * @note   内部完成：电压 → 占空比换算 + 限幅 + 使能判断。
 */
void L6234_SetPhaseVoltage(L6234_t *dev, float u1, float u2, float u3);

/**
 * @brief  关闭三相输出（自由停车：占空比全 0，L6234 输出高阻）
 * @param  dev 驱动实例指针
 */
void L6234_Stop(L6234_t *dev);

/**
 * @brief  三相短路制动（占空比全 1，三相同时下管导通 → 电机被短路 → 快速停止）
 * @param  dev 驱动实例指针
 * @note   仅用于急停保护；正常停机请用 L6234_Stop()。
 */
void L6234_Brake(L6234_t *dev);

/*------------------------------------------------------------------------------
 * 电流 / 电压采样
 *------------------------------------------------------------------------------*/

/**
 * @brief  初始化 ADC1 多通道扫描与电流滤波器
 * @note   必须在 MX_ADC1_Init() 之后调用。
 */
void ADC_Sense_Init(void);

/**
 * @brief  执行一次三通道采样并换算（由 1kHz 控制循环调用）
 * @note   顺序必须与 CubeMX 中 ADC1 的 Rank 设置一致：IN4 → IN5 → IN6
 */
void ADC_Sense_Update(void);

/**
 * @brief  查询某路电机是否处于过流保护锁定状态
 * @param  dev 驱动实例指针
 * @return 1 = 处于保护锁定（输出被强制关闭），0 = 正常
 */
uint8_t L6234_IsFaulted(const L6234_t *dev);

/**
 * @brief  过流保护主逻辑：检查电流、限幅、必要时锁死输出
 * @param  dev     驱动实例指针
 * @param  current 当前实测电流（A）
 * @param  dt      控制周期（s）
 * @return 1 = 允许正常输出，0 = 已被保护（调用方应把目标置 0）
 * @note   保护策略：
 *           ① 瞬时限幅：电流超过 MOTOR_CURRENT_LIMIT_A 立即关闭该路输出
 *           ② 持续判定：连续 MOTOR_OVER_CURRENT_CNT 次超限才进入"故障锁定"
 *           ③ 锁定后 MOTOR_FAULT_LOCK_MS 内不再输出，防止反复冲击烧毁 L6234
 */
uint8_t L6234_CurrentProtect(L6234_t *dev, float current, float dt);

#endif /* __L6234_H__ */
