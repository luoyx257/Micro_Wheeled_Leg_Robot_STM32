/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 轮腿机器人主程序（STM32F103C8T6 版）
  *
  * 移植说明（ESP32/Arduino → STM32 HAL）：
  *   · 原版 setup()  → MX_*_Init() + Robot_Setup()
  *   · 原版 loop()   → Robot_Loop()，由 TIM4 的 1kHz 节拍驱动
  *   · 原版 WiFi/WebSocket → CAN（TJA1050，500kbps，与上位机 USB-CAN 通信）
  *
  * 控制流程（严格对应原版 loop() 的顺序）：
  *   1. 更新 CAN 指令（替代原版 web_loop）
  *   2. 采样电流/电压（替代原版 bat_check，改为逐周期采样）
  *   3. 更新 IMU（原版 mpu6050.update()）
  *   4. 更新编码器 + FOC（原版 motor1/2.loopFOC()）
  *   5. LQR 自平衡（原版 lqr_balance_loop()）
  *   6. YAW 转向（原版 yaw_loop()）
  *   7. 腿部/跳跃（原版 leg_loop()，降频到 200Hz 以减轻总线负担）
  *   8. 输出到电机（原版 motor1/2.move()）+ 过流保护
  *   9. CAN 状态上报（替代原版 Serial 打印）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_config.h"
#include "hardware.h"
#include "l6234.h"
#include "as5600.h"
#include "mpu6050.h"
#include "sts3032.h"
#include "can_proto.h"
#include "motor.h"
#include "controller.h"
#if (DEBUG_SELFTEST_ENABLE != 0)
#include "selftest.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @brief 电角度对齐电压（对应原版 motor.voltage_sensor_align = 6） */
#define ROBOT_ALIGN_VOLTAGE     6.0f

/** @brief 舵机控制降频周期（ms）。原版每循环都发同步写；1Mbps 下一帧约 110us，
 *         为把总线占用压到 2%，这里改为每 5ms 发一次（200Hz），
 *         对腿部动作的响应完全没有影响。 */
#define SERVO_UPDATE_PERIOD_MS  5U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/** @brief 控制节拍标志：由 TIM4 中断置位，主循环消费 */
static volatile uint8_t s_control_tick = 0U;

/** @brief 上次舵机更新时间 */
static uint32_t s_servo_last_ms = 0U;

/** @brief TIM4 控制节拍定时器句柄（必须在文件作用域，中断里也要用） */
static TIM_HandleTypeDef s_htim4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ControlTimer_Init(void);
static void Robot_Setup(void);
static void Robot_Loop(void);
static void Robot_UpdateLegServo(void);
static void Robot_SendTelemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  初始化 1kHz 控制节拍定时器（TIM4）
 *
 * TIM4 挂在 APB1（36MHz），但 APB1 分频系数为 2，定时器时钟自动 ×2 = 72MHz。
 *   Prescaler = 71  → 计数时钟 1MHz
 *   Period    = 999 → 每 1000 个计数溢出一次 = 1kHz
 *
 * 这样控制循环的周期抖动只有几十个 ns，远优于 HAL_Delay(1)（抖动可达 1ms）。
 */
static void ControlTimer_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    s_htim4.Instance               = TIM4;
    s_htim4.Init.Prescaler         = 71U;
    s_htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period            = 999U;
    s_htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&s_htim4) != HAL_OK) {
        Error_Handler();
    }

    /* 抢占优先级 1，低于 CAN 接收（0），高于 SysTick（15） */
    HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    (void)HAL_TIM_Base_Start_IT(&s_htim4);
}

/**
 * @brief  TIM4 更新中断（1kHz 控制节拍）
 * @note   中断服务函数本体放在本文件（CubeMX 生成的 stm32f1xx_it.c 中
 *         没有 TIM4，这里直接提供强定义，避免额外改动生成代码）。
 *         中断内只置标志，所有耗时操作都放在主循环里做。
 */
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&s_htim4, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&s_htim4, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&s_htim4, TIM_IT_UPDATE);
            s_control_tick = 1U;
        }
    }
}

/**
 * @brief  机器人初始化（对应原版 setup() 的机器人部分）
 *
 * 顺序严格对应原版：
 *   1. 舵机总线初始化（原版 Serial2.begin(1000000)）
 *   2. 舵机中位设置（原版 sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC)）
 *   3. 编码器初始化（原版 sensor1/sensor2.init()）
 *   4. IMU 初始化 + 陀螺仪零偏标定（原版 mpu6050.begin()/calcGyroOffsets(true)）
 *   5. 电机驱动使能 + FOC 对齐（原版 driver.init() / motor.init() / initFOC()）
 */
static void Robot_Setup(void)
{
    uint8_t aligned_l;
    uint8_t aligned_r;

    /* ---- 1. 舵机总线 ---- */
    STS3032_Init();

    /* ---- 2. 控制层初始化（含舵机中位，对应原版 setup 的舵机部分） ---- */
    Controller_Init();

    /* ---- 3. 编码器 ---- */
    AS5600_Init();

    /* ---- 4. IMU：初始化 + 零偏标定（标定期间绝对不能动机器人） ---- */
    MPU6050_Begin();
    MPU6050_CalcGyroOffsets(MPU6050_GYRO_CALIB_N);

    /* ---- 5. 电机：驱动使能 + 传感器/驱动绑定 + 电角度对齐 ---- */
    Motor_Init(&motor1);            /* 对应 motor1.linkSensor/linkDriver/init() */
    Motor_Init(&motor2);

    aligned_l = Motor_InitFOC(&motor1, ROBOT_ALIGN_VOLTAGE);   /* 对应 motor1.initFOC() */
    aligned_r = Motor_InitFOC(&motor2, ROBOT_ALIGN_VOLTAGE);   /* 对应 motor2.initFOC() */

    /* 对齐失败（编码器离线/接线错误）时关闭输出，避免失控 */
    if ((aligned_l == 0U) || (aligned_r == 0U)) {
        Motor_Enable(&motor1, 0U);
        Motor_Enable(&motor2, 0U);
    }

    /* ---- 6. 控制节拍定时器 ---- */
    ControlTimer_Init();

    /* 对应原版 setup() 末尾的 delay(500)，等待一切稳定 */
    delay_ms(500U);
}

/**
 * @brief  腿部舵机更新（原版 leg_loop 的调用点，降频到 200Hz）
 */
static void Robot_UpdateLegServo(void)
{
    if ((millis() - s_servo_last_ms) < SERVO_UPDATE_PERIOD_MS) {
        return;
    }
    s_servo_last_ms = millis();

    Controller_LegLoop();
}

/**
 * @brief  CAN 状态上报（替代原版 Serial 打印）
 */
static void Robot_SendTelemetry(void)
{
    uint8_t status = 0U;

    if (imu.online)           { status |= 0x01U; }
    if (encoder_left.online)  { status |= 0x02U; }
    if (encoder_right.online) { status |= 0x04U; }
    if (L6234_IsFaulted(&motor_left))  { status |= 0x08U; }
    if (L6234_IsFaulted(&motor_right)) { status |= 0x10U; }
    if (current_sense.battery_voltage < BAT_LOW_THRESHOLD_V) { status |= 0x20U; }
    if (sts.error_count > 0U) { status |= 0x40U; }

    CAN_Proto_TelemetryTick(LQR_angle, LQR_gyro,
                            motor1.shaft_velocity, motor2.shaft_velocity,
                            current_sense.current_left, current_sense.current_right,
                            current_sense.battery_voltage,
                            status, sts.error_count);
}

/**
 * @brief  主控制循环（对应原版 loop()，顺序完全一致）
 */
static void Robot_Loop(void)
{
    /* ===== 1. 更新上位机指令（原版 web_loop → rp.spinOnce） ===== */
    Controller_UpdateFromCan();

    /* ===== 2. 电流/电压采样（原版 bat_check，改为逐周期） ===== */
    ADC_Sense_Update();

    /* ===== 3. IMU 更新（原版 mpu6050.update()） ===== */
    MPU6050_Update(CONTROL_DT);

    /* ===== 4. 编码器 + FOC（原版 motor1/2.loopFOC()） ===== */
    Motor_LoopFOC(&motor1, CONTROL_DT);
    Motor_LoopFOC(&motor2, CONTROL_DT);

    /* ===== 5. LQR 自平衡（原版 lqr_balance_loop()） ===== */
    Controller_LqrBalanceLoop();

    /* ===== 6. YAW 转向（原版 yaw_loop()） ===== */
    Controller_YawLoop();

    /* ===== 7. 腿部动作（原版 leg_loop()，降频执行） ===== */
    Robot_UpdateLegServo();

    /* ===== 8. 下发输出到电机（原版 loop() 末尾的 target 计算 + move()） ===== */
    Controller_ApplyMotorOutput();

    /* 过流保护：任一电机触发保护则强制目标为 0（避免堵转烧 L6234） */
    if ((L6234_CurrentProtect(&motor_left, current_sense.current_left, CONTROL_DT) == 0U) ||
        (L6234_CurrentProtect(&motor_right, current_sense.current_right, CONTROL_DT) == 0U)) {
        motor1.target = 0.0f;
        motor2.target = 0.0f;
    }

    Motor_Move(&motor1);
    Motor_Move(&motor2);

    /* ===== 9. 状态上报 ===== */
    Robot_SendTelemetry();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 提高中断优先级分组灵活性：使用 4 位抢占优先级 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 由 TIM4 的 1kHz 中断置位，主循环按 1kHz 节拍执行控制 */
    if (s_control_tick != 0U) {
      s_control_tick = 0U;
      Robot_Loop();
    }
  }
#endif
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
