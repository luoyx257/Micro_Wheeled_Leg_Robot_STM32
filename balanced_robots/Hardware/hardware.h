/**
 ******************************************************************************
 * @file    hardware.h
 * @brief   公共硬件底座：毫秒/微秒时基、I2C 总线恢复
 *
 * 说明：
 *   本文件提供 Arduino 风格的 millis() / micros() / delay_us()，
 *   以便原版 ESP32 代码里的时间相关逻辑可以几乎原样搬过来。
 *   同时集中放置 I2C 从机无应答时的总线恢复函数（避免 NACK 后 HAL 卡死）。
 ******************************************************************************
 */

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 时基
 *============================================================================*/

/**
 * @brief  初始化微秒级延时（DWT 周期计数器）
 * @note   必须在 SystemCoreClockUpdate() 之后调用（main 里 MX_*_Init 之后即可）
 */
void HW_Init(void);

/**
 * @brief  获取系统运行毫秒数
 * @return 自启动以来经过的毫秒数（溢出周期约 49.7 天，与原版 millis() 行为一致）
 */
uint32_t millis(void);

/**
 * @brief  获取系统运行微秒数
 * @return 自启动以来经过的微秒数（32 位，约 71 分钟后溢出）
 */
uint32_t micros(void);

/**
 * @brief  微秒级忙等延时
 * @param  us 延时时间（微秒）
 */
void delay_us(uint32_t us);

/**
 * @brief  毫秒级延时（基于 millis，可被中断打断）
 * @param  ms 延时时间（毫秒）
 */
void delay_ms(uint32_t ms);

/**
 * @brief  由 SysTick 中断调用的毫秒计数递增（在 stm32f1xx_it.c 的
 *         SysTick_Handler 中调用，位于 HAL_IncTick() 旁边）
 */
void HW_IncMillis(void);

/*==============================================================================
 * I2C 总线异常恢复
 *============================================================================*/

/**
 * @brief  I2C 总线死锁恢复（9 个 SCL 脉冲 + STOP）
 * @param  hi2c 目标 I2C 句柄（&hi2c1 / &hi2c2）
 * @note   当从机在传输中被复位、或 MCU 复位时从机还拉着 SDA，
 *         会造成总线死锁。此函数把 SCL 手动打 9 个脉冲让从机释放 SDA。
 */
void HW_I2C_BusRecover(I2C_HandleTypeDef *hi2c);

#endif /* __HARDWARE_H__ */
