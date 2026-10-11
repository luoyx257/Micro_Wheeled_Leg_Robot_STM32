/**
 ******************************************************************************
 * @file    hardware.c
 * @brief   公共硬件底座实现：毫秒/微秒时基、I2C 总线恢复
 ******************************************************************************
 */

#include "hardware.h"

/*==============================================================================
 * 私有变量
 *============================================================================*/

/** @brief 毫秒计数器，由 SysTick 中断 (1ms) 递增 */
static volatile uint32_t s_millis = 0;

/** @brief micros() 溢出扩展：记录 DWT CYCCNT 上一次回绕时的微秒基准 */
static volatile uint32_t s_micros_base = 0;

/** @brief 上一次读取的 DWT CYCCNT 值，用于检测 32 位计数器回绕 */
static volatile uint32_t s_last_cycle_cnt = 0;

/** @brief CPU 每微秒的周期数（72MHz → 72） */
static uint32_t s_cycles_per_us = 72U;

/*==============================================================================
 * 时基实现
 *============================================================================*/

/**
 * @brief  初始化 DWT 周期计数器，为 delay_us()/micros() 提供时基
 * @note   DWT 是 Cortex-M3 内核自带的调试计数器，不占用任何外设定时器资源。
 */
void HW_Init(void)
{
    /* 依据当前主频计算每微秒周期数（SystemCoreClock 由 CubeMX 的
       SystemClock_Config + SystemCoreClockUpdate 维护，默认 72MHz） */
    s_cycles_per_us = SystemCoreClock / 1000000U;

    /* 使能 DWT 的周期计数器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 打开跟踪与调试模块 */
    DWT->CYCCNT = 0U;                                  /* 清计数 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              /* 使能计数 */
}

/**
 * @brief  毫秒计数递增（由 SysTick_Handler 调用）
 */
void HW_IncMillis(void)
{
    s_millis++;
}

/**
 * @brief  获取系统运行毫秒数
 */
uint32_t millis(void)
{
    return s_millis;
}

/**
 * @brief  获取系统运行微秒数（DWT 周期计数换算 + 回绕扩展）
 * @note   DWT->CYCCNT 是 32 位（ARMv7-M 定义），按 CPU 周期递增；
 *         72MHz 下约 59.6s 回绕一次。这里检测回绕并累加基数，
 *         保证在 71 分钟内（micros 自身 32 位溢出前）单调递增。
 *         PID / LPF 只用 micros() 的差值，因此对绝对值的溢出不敏感。
 */
uint32_t micros(void)
{
    uint32_t now = DWT->CYCCNT;

    if (now < s_last_cycle_cnt) {
        /* 检测到一次回绕，累加一个完整 32 位周期对应的微秒数 */
        s_micros_base += (0xFFFFFFFFUL / s_cycles_per_us);
    }
    s_last_cycle_cnt = now;

    return s_micros_base + (now / s_cycles_per_us);
}

/**
 * @brief  微秒级忙等延时
 * @param  us 延时微秒数
 * @note   使用 DWT 周期差做判断，不受中断影响，精度约 ±1us。
 */
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * s_cycles_per_us;

    while ((DWT->CYCCNT - start) < ticks) {
        /* busy wait */
    }
}

/**
 * @brief  毫秒级延时
 * @param  ms 延时毫秒数
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        /* busy wait（millis 由中断更新，可被中断打断） */
    }
}

/*==============================================================================
 * I2C 总线恢复
 *============================================================================*/

/**
 * @brief  I2C 总线死锁恢复
 * @param  hi2c I2C 句柄
 *
 * 原理：
 *   把 SCL/SDA 切成普通 GPIO 开漏输出，手动产生最多 9 个 SCL 时钟，
 *   让从机把剩余的 bit 发完并释放 SDA，最后补一个 STOP 时序，
 *   然后重新 HAL_I2C_Init() 让 HAL 重新接管引脚。
 */
void HW_I2C_BusRecover(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_TypeDef *port;
    uint16_t scl_pin;
    uint16_t sda_pin;
    uint8_t i;

    if (hi2c == NULL) {
        return;
    }

    /* 取出该 I2C 对应引脚；两块 AS5600/MPU6050 都挂在 GPIOB 上 */
    if (hi2c->Instance == I2C1) {
        port = GPIOB;
        scl_pin = GPIO_PIN_8;      /* I2C1_SCL */
        sda_pin = GPIO_PIN_9;      /* I2C1_SDA */
    } else {
        port = GPIOB;
        scl_pin = GPIO_PIN_10;     /* I2C2_SCL */
        sda_pin = GPIO_PIN_11;     /* I2C2_SDA */
    }

    /* 先停止 HAL 状态机，避免恢复过程中 HAL 又去操作寄存器 */
    (void)HAL_I2C_DeInit(hi2c);

    /* 切成开漏输出，两根线都释放为高 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = scl_pin;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = sda_pin;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(port, scl_pin | sda_pin, GPIO_PIN_SET);
    delay_us(10);

    /* 打 9 个 SCL 时钟，把从机未发完的数据位读走 */
    for (i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
        delay_us(5);
    }

    /* 补一个 STOP 时序：SCL 高时把 SDA 由低拉高 */
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);
    delay_us(5);

    /* 重新初始化 HAL 的 I2C（会重新把引脚配成 AF_OD） */
    (void)HAL_I2C_Init(hi2c);
}
