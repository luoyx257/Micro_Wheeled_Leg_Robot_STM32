/**
 ******************************************************************************
 * @file    selftest.h
 * @brief   单模块测试例程（电机 / 舵机 / IMU / CAN / 编码器）
 *
 * 使用方法：
 *   1. 在 robot_config.h 里把 DEBUG_SELFTEST_ENABLE 置 1；
 *   2. 在 main.c 里把 SELFTEST_CASE 改成要跑的用例号；
 *   3. 编译烧录，观察现象（详见 TEST_GUIDE.md）。
 *
 * 注意：测试模式下【不会】运行平衡控制，机器人不会自己站起来，
 *       请把机器人架起或放在安全位置再上电。
 ******************************************************************************
 */

#ifndef __SELFTEST_H__
#define __SELFTEST_H__

#include "main.h"
#include "robot_config.h"

/*==============================================================================
 * 测试用例编号
 *============================================================================*/

/** @brief 测试例程 1：左电机正反转（开环电压，先慢后快） */
#define SELFTEST_MOTOR_LEFT         1U
/** @brief 测试例程 2：右电机正反转 */
#define SELFTEST_MOTOR_RIGHT        2U
/** @brief 测试例程 3：两个电机同时正反转 */
#define SELFTEST_MOTOR_BOTH         3U
/** @brief 测试例程 4：舵机角度读写（Ping + 读位置 + 摆动） */
#define SELFTEST_SERVO              4U
/** @brief 测试例程 5：IMU 数据读取（角度/角速度通过 CAN 上报） */
#define SELFTEST_IMU                5U
/** @brief 测试例程 6：CAN 收发（收到 0x200 就回 0x100/0x101） */
#define SELFTEST_CAN                6U
/** @brief 测试例程 7：编码器读取（左右 AS5600 角度通过 CAN 上报） */
#define SELFTEST_ENCODER            7U
/** @brief 测试例程 8：电流采样与过流保护验证 */
#define SELFTEST_CURRENT            8U

/**
 * @brief  当前运行的测试用例（改这里切换）
 * @note   仅当 DEBUG_SELFTEST_ENABLE = 1 时有效。
 */
#ifndef SELFTEST_CASE
#define SELFTEST_CASE               SELFTEST_MOTOR_LEFT
#endif

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  测试例程入口（在 main 的初始化完成后调用一次）
 * @note   内部是一个 while(1) 死循环，不会返回。
 */
void SelfTest_Run(void);

#endif /* __SELFTEST_H__ */
