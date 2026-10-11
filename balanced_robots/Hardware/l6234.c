/**
 ******************************************************************************
 * @file    l6234.c
 * @brief   L6234 三相无刷电机驱动模块实现
 *
 * 模块职责：
 *   1. 三相 PWM 输出（20kHz，TIM1 驱动右电机 / TIM2 驱动左电机）
 *   2. 方向脚 IN1/IN2/IN3 控制（本方案固定正向，预留接口）
 *   3. ADC 电流 / 电池电压采样（PA4 / PA5 / PA6）
 *   4. 过流限幅保护（避免堵转烧毁 L6234）
 ******************************************************************************
 */

#include "l6234.h"
#include "tim.h"
#include "adc.h"
#include <math.h>

/*==============================================================================
 * 全局实例
 *============================================================================*/

/** @brief 左电机驱动实例（插口 1，TIM2：PA0/PA2/PA3，方向 PA1/PB0/PB1） */
L6234_t motor_left = {
    .htim            = &htim2,
    .ch1             = MOTOR_L_EN1_CHANNEL,
    .ch2             = MOTOR_L_EN2_CHANNEL,
    .ch3             = MOTOR_L_EN3_CHANNEL,
    .in1_port        = MOTOR_L_IN1_PORT,
    .in1_pin         = MOTOR_L_IN1_PIN,
    .in2_port        = MOTOR_L_IN2_PORT,
    .in2_pin         = MOTOR_L_IN2_PIN,
    .in3_port        = MOTOR_L_IN3_PORT,
    .in3_pin         = MOTOR_L_IN3_PIN,
    .supply_voltage  = MOTOR_VOLTAGE_SUPPLY,
    .enabled         = 0U,
};

/** @brief 右电机驱动实例（插口 2，TIM1：PA8/PA9/PA10，方向 PB12/PB13/PB14） */
L6234_t motor_right = {
    .htim            = &htim1,
    .ch1             = MOTOR_R_EN1_CHANNEL,
    .ch2             = MOTOR_R_EN2_CHANNEL,
    .ch3             = MOTOR_R_EN3_CHANNEL,
    .in1_port        = MOTOR_R_IN1_PORT,
    .in1_pin         = MOTOR_R_IN1_PIN,
    .in2_port        = MOTOR_R_IN2_PORT,
    .in2_pin         = MOTOR_R_IN2_PIN,
    .in3_port        = MOTOR_R_IN3_PORT,
    .in3_pin         = MOTOR_R_IN3_PIN,
    .supply_voltage  = MOTOR_VOLTAGE_SUPPLY,
    .enabled         = 0U,
};

/** @brief 电流/电压采样全局数据 */
CurrentSense_t current_sense = {0};

/** @brief 过流保护累计触发次数 */
volatile uint32_t over_current_count = 0U;

/*==============================================================================
 * 私有变量
 *============================================================================*/

/** @brief 左电机电流滑动平均缓冲 */
static uint16_t s_buf_left[CURRENT_FILTER_WINDOW];
/** @brief 右电机电流滑动平均缓冲 */
static uint16_t s_buf_right[CURRENT_FILTER_WINDOW];
/** @brief 滑动平均写指针 */
static uint8_t  s_filter_idx = 0U;
/** @brief 滤波器是否已填满一轮 */
static uint8_t  s_filter_ready = 0U;

/** @brief 左电机连续过流计数 */
static uint8_t  s_oc_cnt_left = 0U;
/** @brief 右电机连续过流计数 */
static uint8_t  s_oc_cnt_right = 0U;
/** @brief 左电机保护锁定剩余时间（ms） */
static uint32_t s_lock_left = 0U;
/** @brief 右电机保护锁定剩余时间（ms） */
static uint32_t s_lock_right = 0U;

/*==============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief  把占空比写入定时器比较寄存器
 * @param  dev  驱动实例
 * @param  ch   定时器通道
 * @param  duty 占空比 0.0~1.0
 */
static void L6234_WriteCCR(L6234_t *dev, uint32_t ch, float duty)
{
    uint32_t ccr;

    /* 限幅，防止浮点误差越界 */
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    /* 占空比 → 比较值（四舍五入，避免长期偏小） */
    ccr = (uint32_t)(duty * (float)PWM_TIMER_ARR + 0.5f);

    __HAL_TIM_SET_COMPARE(dev->htim, ch, ccr);
}

/**
 * @brief  根据保护状态取得电流保护计数器指针（左/右）
 * @param  dev 驱动实例
 * @return 对应的连续过流计数器指针；未知实例返回 NULL
 */
static uint8_t *L6234_GetOcCounter(const L6234_t *dev)
{
    if (dev == &motor_left) {
        return &s_oc_cnt_left;
    }
    if (dev == &motor_right) {
        return &s_oc_cnt_right;
    }
    return NULL;
}

/**
 * @brief  取得保护锁定剩余时间指针（左/右）
 */
static uint32_t *L6234_GetLockTimer(const L6234_t *dev)
{
    if (dev == &motor_left) {
        return &s_lock_left;
    }
    if (dev == &motor_right) {
        return &s_lock_right;
    }
    return NULL;
}

/*==============================================================================
 * 对外接口：初始化与输出控制
 *============================================================================*/

/**
 * @brief  初始化两个 L6234 驱动
 * @note   上电默认"输出关闭"，防止意外转动。
 */
void L6234_Init(void)
{
    uint8_t i;

    /* ---- 1. 方向脚 IN1/IN2/IN3 固定置高（正向模式） -------------------------
     * L6234：EN=1 & IN=1 → OUT 输出高。三相 PWM 调制下等效于半桥正向斩波，
     * 相电压 = 占空比 × Vs。这与 SimpleFOC 3PWM 驱动的电压语义一致。
     */
    HAL_GPIO_WritePin(motor_left.in1_port,  motor_left.in1_pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_left.in2_port,  motor_left.in2_pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_left.in3_port,  motor_left.in3_pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_right.in1_port, motor_right.in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_right.in2_port, motor_right.in2_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_right.in3_port, motor_right.in3_pin, GPIO_PIN_SET);

    /* ---- 2. 三相占空比清零（L6234 输出高阻，电机不转） -------------------- */
    L6234_SetDuty(&motor_left,  motor_left.ch1,  0.0f);
    L6234_SetDuty(&motor_left,  motor_left.ch2,  0.0f);
    L6234_SetDuty(&motor_left,  motor_left.ch3,  0.0f);
    L6234_SetDuty(&motor_right, motor_right.ch1, 0.0f);
    L6234_SetDuty(&motor_right, motor_right.ch2, 0.0f);
    L6234_SetDuty(&motor_right, motor_right.ch3, 0.0f);

    /* ---- 3. 启动 PWM 输出 ------------------------------------------------- */
    (void)HAL_TIM_PWM_Start(motor_left.htim,  motor_left.ch1);
    (void)HAL_TIM_PWM_Start(motor_left.htim,  motor_left.ch2);
    (void)HAL_TIM_PWM_Start(motor_left.htim,  motor_left.ch3);
    (void)HAL_TIM_PWM_Start(motor_right.htim, motor_right.ch1);
    (void)HAL_TIM_PWM_Start(motor_right.htim, motor_right.ch2);
    (void)HAL_TIM_PWM_Start(motor_right.htim, motor_right.ch3);

    /* ---- 4. 清零统计与滤波器 --------------------------------------------- */
    motor_left.enabled  = 0U;
    motor_right.enabled = 0U;

    over_current_count = 0U;
    s_oc_cnt_left  = 0U;
    s_oc_cnt_right = 0U;
    s_lock_left    = 0U;
    s_lock_right   = 0U;

    for (i = 0U; i < CURRENT_FILTER_WINDOW; i++) {
        s_buf_left[i]  = 0U;
        s_buf_right[i] = 0U;
    }
    s_filter_idx   = 0U;
    s_filter_ready = 0U;
}

/**
 * @brief  使能 / 关闭某路 L6234 输出
 */
void L6234_Enable(L6234_t *dev, uint8_t enable)
{
    if (dev == NULL) {
        return;
    }

    if (enable) {
        /* 使能方向脚为正向 */
        HAL_GPIO_WritePin(dev->in1_port, dev->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dev->in2_port, dev->in2_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dev->in3_port, dev->in3_pin, GPIO_PIN_SET);
        dev->enabled = 1U;
    } else {
        /* 关闭：三相占空比清零 + 方向脚拉低（彻底关断输出） */
        L6234_SetDuty(dev, dev->ch1, 0.0f);
        L6234_SetDuty(dev, dev->ch2, 0.0f);
        L6234_SetDuty(dev, dev->ch3, 0.0f);
        HAL_GPIO_WritePin(dev->in1_port, dev->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->in2_port, dev->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dev->in3_port, dev->in3_pin, GPIO_PIN_RESET);
        dev->enabled = 0U;
    }
}

/**
 * @brief  设置单相 PWM 占空比
 */
void L6234_SetDuty(L6234_t *dev, uint32_t ch, float duty)
{
    if (dev == NULL) {
        return;
    }
    L6234_WriteCCR(dev, ch, duty);
}

/**
 * @brief  三相统一设置相电压（电压模式核心接口）
 * @param  dev 驱动实例
 * @param  u1  第 1 相电压（V），范围 ±Vs/2
 * @param  u2  第 2 相电压（V）
 * @param  u3  第 3 相电压（V）
 *
 * 换算：duty = 0.5 + u / Vs
 *   三相均为 0V  → 三路占空比都是 0.5 → 相间电压差 0（零电压矢量）
 *   相 1 为 +Vs/2 → 占空比 1.0（全高）
 *   相 1 为 -Vs/2 → 占空比 0.0（全低）
 */
void L6234_SetPhaseVoltage(L6234_t *dev, float u1, float u2, float u3)
{
    float vs;
    float half_vs;
    float limit;

    if (dev == NULL) {
        return;
    }

    /* 未使能时不允许输出，保证安全 */
    if (!dev->enabled) {
        L6234_WriteCCR(dev, dev->ch1, 0.0f);
        L6234_WriteCCR(dev, dev->ch2, 0.0f);
        L6234_WriteCCR(dev, dev->ch3, 0.0f);
        return;
    }

    vs = dev->supply_voltage;
    if (vs < 1.0f) {
        /* 电源电压配置异常，直接关闭输出，避免除零 */
        L6234_WriteCCR(dev, dev->ch1, 0.0f);
        L6234_WriteCCR(dev, dev->ch2, 0.0f);
        L6234_WriteCCR(dev, dev->ch3, 0.0f);
        return;
    }
    half_vs = vs * 0.5f;

    /* 单相限幅：不超过 ±MOTOR_PHASE_VOLTAGE_MAX，并保证不越出 ±Vs/2 */
    limit = MOTOR_PHASE_VOLTAGE_MAX;
    if (limit > half_vs) {
        limit = half_vs;
    }

    if (u1 >  limit) { u1 =  limit; }
    if (u1 < -limit) { u1 = -limit; }
    if (u2 >  limit) { u2 =  limit; }
    if (u2 < -limit) { u2 = -limit; }
    if (u3 >  limit) { u3 =  limit; }
    if (u3 < -limit) { u3 = -limit; }

    /* 相电压 → 占空比 */
    L6234_WriteCCR(dev, dev->ch1, 0.5f + u1 / vs);
    L6234_WriteCCR(dev, dev->ch2, 0.5f + u2 / vs);
    L6234_WriteCCR(dev, dev->ch3, 0.5f + u3 / vs);
}

/**
 * @brief  关闭三相输出（自由停车）
 */
void L6234_Stop(L6234_t *dev)
{
    if (dev == NULL) {
        return;
    }
    L6234_WriteCCR(dev, dev->ch1, 0.0f);
    L6234_WriteCCR(dev, dev->ch2, 0.0f);
    L6234_WriteCCR(dev, dev->ch3, 0.0f);
}

/**
 * @brief  三相短路制动（急停用）
 * @note   三相占空比全 1 → 三个 OUT 都输出高，电机三相被短接到 Vs，
 *         反电动势形成制动电流。仅用于紧急停止，长时间使用会过热。
 */
void L6234_Brake(L6234_t *dev)
{
    if (dev == NULL) {
        return;
    }
    L6234_WriteCCR(dev, dev->ch1, 1.0f);
    L6234_WriteCCR(dev, dev->ch2, 1.0f);
    L6234_WriteCCR(dev, dev->ch3, 1.0f);
}

/*==============================================================================
 * ADC 采样
 *============================================================================*/

/**
 * @brief  初始化电流采样
 * @note   ADC1 已在 CubeMX 中配置为 3 通道扫描（IN4/IN5/IN6），
 *         采样时间 55.5 周期（12MHz ADC 时钟下 ≈4.6us/通道）。
 */
void ADC_Sense_Init(void)
{
    uint8_t i;

    for (i = 0U; i < CURRENT_FILTER_WINDOW; i++) {
        s_buf_left[i]  = 0U;
        s_buf_right[i] = 0U;
    }
    s_filter_idx   = 0U;
    s_filter_ready = 0U;

    current_sense.current_left    = 0.0f;
    current_sense.current_right   = 0.0f;
    current_sense.battery_voltage = 0.0f;

    /* 先做一次校准（HAL 会用内部参考做偏移校准，提高精度） */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        /* 校准失败不致命，继续工作 */
    }
}

/**
 * @brief  执行一次三通道采样（顺序必须与 CubeMX Rank 一致：IN4→IN5→IN6）
 *
 * 调用时机：1kHz 控制循环开头。总共耗时约 3×(1.5+55.5)/12MHz ≈ 14us。
 */
void ADC_Sense_Update(void)
{
    uint32_t sum_l = 0U;
    uint32_t sum_r = 0U;
    uint8_t  i;

    /* ---- 规则通道扫描：依次读取 3 个 Rank ---- */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return;
    }

    /* Rank 1：PA4 → 左电机电流 */
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
        current_sense.raw_left = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    /* Rank 2：PA5 → 右电机电流 */
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
        current_sense.raw_right = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    /* Rank 3：PA6 → 电池电压分压 */
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
        current_sense.raw_battery = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    (void)HAL_ADC_Stop(&hadc1);

    /* ---- 滑动平均滤波（抑制 20kHz 开关噪声） ---- */
    s_buf_left[s_filter_idx]  = current_sense.raw_left;
    s_buf_right[s_filter_idx] = current_sense.raw_right;

    s_filter_idx++;
    if (s_filter_idx >= CURRENT_FILTER_WINDOW) {
        s_filter_idx   = 0U;
        s_filter_ready = 1U;
    }

    for (i = 0U; i < CURRENT_FILTER_WINDOW; i++) {
        sum_l += s_buf_left[i];
        sum_r += s_buf_right[i];
    }

    if (s_filter_ready) {
        sum_l /= CURRENT_FILTER_WINDOW;
        sum_r /= CURRENT_FILTER_WINDOW;
    } else {
        /* 滤波器未满时，用"已写入样本数"做平均，避免开机阶段读数偏小 */
        sum_l /= (uint32_t)(s_filter_idx == 0U ? CURRENT_FILTER_WINDOW : s_filter_idx);
        sum_r /= (uint32_t)(s_filter_idx == 0U ? CURRENT_FILTER_WINDOW : s_filter_idx);
    }

    /* ---- 换算为物理量 ---- */
    current_sense.current_left  = (float)sum_l * ADC_TO_CURRENT;
    current_sense.current_right = (float)sum_r * ADC_TO_CURRENT;

    current_sense.battery_voltage =
        (float)current_sense.raw_battery * (ADC_VREF / ADC_FULL_SCALE) * BAT_ADC_DIV_RATIO;
}

/*==============================================================================
 * 过流保护
 *============================================================================*/

/**
 * @brief  查询电机是否处于过流保护锁定
 */
uint8_t L6234_IsFaulted(const L6234_t *dev)
{
    const uint32_t *lock = L6234_GetLockTimer(dev);

    if (lock == NULL) {
        return 0U;
    }
    return (*lock > 0U) ? 1U : 0U;
}

/**
 * @brief  过流保护主逻辑
 *
 * 三级保护策略：
 *   ① 瞬时保护：单次采样电流 > MOTOR_CURRENT_LIMIT_A → 立即关闭该路输出
 *                （PWM 关断是最快的响应手段，不等软件循环）
 *   ② 持续判定：连续 MOTOR_OVER_CURRENT_CNT 次超限 → 判定为真堵转/短路
 *   ③ 故障锁定：锁定 MOTOR_FAULT_LOCK_MS，期间拒绝输出，避免反复冲击
 */
uint8_t L6234_CurrentProtect(L6234_t *dev, float current, float dt)
{
    uint8_t  *oc_cnt;
    uint32_t *lock_timer;
    uint32_t  lock_ms;

    if (dev == NULL) {
        return 0U;
    }

    oc_cnt     = L6234_GetOcCounter(dev);
    lock_timer = L6234_GetLockTimer(dev);
    if ((oc_cnt == NULL) || (lock_timer == NULL)) {
        return 1U;      /* 未知实例不做保护判断 */
    }

    /* ---- 锁定倒计时递减 ---- */
    if (*lock_timer > 0U) {
        lock_ms = (uint32_t)(dt * 1000.0f);
        if (lock_ms == 0U) {
            lock_ms = 1U;
        }
        if (*lock_timer > lock_ms) {
            *lock_timer -= lock_ms;
        } else {
            *lock_timer = 0U;
            *oc_cnt     = 0U;       /* 解锁后清计数，重新开始判定 */
        }

        /* 锁定期间强制关闭输出 */
        L6234_Stop(dev);
        return 0U;
    }

    /* ---- 电流判定 ---- */
    if ((current > MOTOR_CURRENT_LIMIT_A) || (current < -MOTOR_CURRENT_LIMIT_A)) {
        /* ① 瞬时限幅：立即关闭该路输出 */
        L6234_Stop(dev);

        /* ② 持续判定 */
        if (*oc_cnt < 0xFFU) {
            (*oc_cnt)++;
        }

        if (*oc_cnt >= MOTOR_OVER_CURRENT_CNT) {
            /* ③ 进入故障锁定 */
            *lock_timer = MOTOR_FAULT_LOCK_MS;
            *oc_cnt     = 0U;
            over_current_count++;
        }
        return 0U;
    }

    /* 电流正常，逐步清掉超限计数（半衰，避免一次干扰后立刻又锁） */
    if (*oc_cnt > 0U) {
        (*oc_cnt)--;
    }
    return 1U;
}
