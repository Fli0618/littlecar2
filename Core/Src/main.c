/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "drive_bus_servo.h"
#include "sensor_ops.h"
#include "sensor_wit.h"
#include "drive_emm.h"
#include "advance_chassis.h"
#include "advance_control.h"
#include "advance_visual.h"
#include "advance_motion.h"
#include "advance_holonomic_position.h"
#include "advance_world.h"
#include "advance_material_task.h"
#include "advance_arm.h"
#include "car_pose.h"
#include "comm_jetson.h"
#include "comm_stdio.h"
#include "comm_tuner.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * 设为 1 时进入 USART1 在线调参模式，并自动禁用 printf；
 * 设为 0 时关闭在线调参，恢复 USART1 printf 输出并运行比赛主流程。
 */
#define ONLINE_DEBUG_MODE (1U)

/* TIM6 提供 1 ms 调度节拍，所有业务周期统一在这里配置。 */
#define APP_WORLD_PERIOD_MS ((uint32_t)10U)
#define APP_ORIGIN_PERIOD_MS ((uint32_t)1000U)
#define APP_LED_PERIOD_MS ((uint32_t)1500U)

/* TIM6 负责系统后台周期任务；阻塞式视觉伺服由其接口内部自行调度。 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_ToggleLed(void)
{
  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10);
}

static void App_TryResetWorldOrigin(void)
{
  WorldPose2D_t pose = {0};

  (void)AdvanceWorld_GetPoseCopy(&pose);
  if (pose.origin_ready == 0U)
  {
    (void)AdvanceWorld_ResetOrigin();
  }
}

static void App_RunTask(Competition_StartArea_t start_area) // 传入启停区编号，有两个启停区需要弄不同的流程，还没有实现
{
  /* 后续比赛流程将根据 start_area 选择对应启停区的业务路径。 */

  printf("[START] area=%u\r\n", (unsigned int)start_area);
  // 丝杆回零部分！！！
  // if (!AdvanceArm_HomeBlocking())
  // {
  //   AdvanceArm_EStop();
  //   printf("[ARM] home failed\r\n");
  //   return;
  // }
  printf("[ARM] home success\r\n");

  char code[DETECT_QR_CODE_LENGTH + 1U] = {0};
  Detect_TargetList_t targets = {0};
  uint8_t i;

  /* 1. 二维码识别 */
  (void)detect_qr_read_blocking(code);
  printf("[QR] %s\r\n", code);

  /*
   * detect_qr_read_blocking() 退出前已经发送 STOP。
   * 稍等 USART6 DMA 发送完成，再启动下一项任务。
   */
  HAL_Delay(10U);

  /* 2. 颜色识别 */
  (void)detect_color_start();

  while (1)
  {
    if (detect_get_targets(&targets) != 0U)
    {
      printf("[COLOR] count=%u\r\n",
             (unsigned int)targets.count);

      for (i = 0U; i < targets.count; ++i)
      {
        printf("type=%u x=%d y=%d conf=%u\r\n",
               (unsigned int)targets.targets[i].type,
               (int)targets.targets[i].x,
               (int)targets.targets[i].y,
               (unsigned int)targets.targets[i].confidence);
      }

      /* 检测到至少一个目标后进入下一阶段 */
      if (targets.count > 0U)
      {
        break;
      }
    }

    __WFI();
  }

  (void)detect_stop();
  HAL_Delay(10U);

  /* 清空上一个阶段的本地变量 */
  targets = (Detect_TargetList_t){0};

  /* 3. 数字圆环识别 */
  (void)detect_circle_start();

  while (1)
  {
    if (detect_get_targets(&targets) != 0U)
    {
      printf("[CIRCLE] count=%u\r\n",
             (unsigned int)targets.count);

      for (i = 0U; i < targets.count; ++i)
      {
        printf("type=%u x=%d y=%d conf=%u\r\n",
               (unsigned int)targets.targets[i].type,
               (int)targets.targets[i].x,
               (int)targets.targets[i].y,
               (unsigned int)targets.targets[i].confidence);
      }

      if (targets.count > 0U)
      {
        break;
      }
    }
    __WFI();
  }
  (void)detect_stop();
  printf("[TEST] finished\r\n");
}

static void App_TimerUpdate(void)
{
  static uint16_t world_elapsed_ms = 0U;
  static uint16_t drive_elapsed_ms = 0U;
  static uint16_t control_elapsed_ms = 0U;
  static uint16_t origin_elapsed_ms = 0U;
  static uint16_t led_elapsed_ms = 0U;

  CommJetson_Update();
#if (ONLINE_DEBUG_MODE != 0U)
  CommTuner_Update();
#endif

  if (++world_elapsed_ms >= APP_WORLD_PERIOD_MS)
  {
    world_elapsed_ms = 0U;
    OPS_Update();
    WIT_Update();
    AdvanceWorld_Update();
  }

  if (++drive_elapsed_ms >= DRIVE_EMM_UPDATE_PERIOD_MS)
  {
    drive_elapsed_ms = 0U;
    drive_emm_Update();
  }

  if (++control_elapsed_ms >= ADVANCE_MOTION_CONTROL_PERIOD_MS)
  {
    control_elapsed_ms = 0U;

    /* PID pending 配置必须在每个 20 ms 周期边界检查一次。 */
    AdvanceMotion_Update();
    /* 全向 pending 配置与控制轮廓也在固定 20 ms 边界推进。 */
    AdvanceHolonomic_Update();
  }

  if (++origin_elapsed_ms >= APP_ORIGIN_PERIOD_MS)
  {
    origin_elapsed_ms = 0U;
    App_TryResetWorldOrigin();
  }

  if (++led_elapsed_ms >= APP_LED_PERIOD_MS)
  {
    led_elapsed_ms = 0U;
    App_ToggleLed();
  }
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
  // 提前初始化 GPIO，确认程序是否启动
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStructInit = {0};

  // 1. 针对你这款蓝色 M144Z-M4 板子，尝试常见的 PA0(LED)
  GPIO_InitStructInit.Pin = GPIO_PIN_0;
  GPIO_InitStructInit.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructInit.Pull = GPIO_NOPULL;
  GPIO_InitStructInit.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructInit);

  // 2. 同时保留之前的 PF9/PF10 和 PC13
  GPIO_InitStructInit.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStructInit);

  GPIO_InitStructInit.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructInit);

  // 全量闪烁测试
  for (int i = 0; i < 10; i++)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile int j = 0; j < 300000; j++)
      ; // 缩短延时，亮暗更明显
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  CommStdio_Init(&huart1, (uint8_t)(ONLINE_DEBUG_MODE == 0U));

#if (ONLINE_DEBUG_MODE != 0U)
  if (CommTuner_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  CommJetson_Init(&huart6);

  // 传感器初始化
  OPS_Init(&huart5);
  WIT_Init();

  /* 上层模块先绑定传感器数据视图，再初始化自身状态。 */
  CarPose_Init();
  AdvanceWorld_Init();
  AdvanceControl_Init();
  AdvanceVisual_Init();
  AdvanceMotion_Init();
  AdvanceHolonomic_Init();
  if (drive_emm_Init() != HAL_OK)
  {
    Error_Handler();
  }
  drive_emm_ConfigureChassisFeedback(
      CHASSIS_MOTOR_LF_ID,
      CHASSIS_MOTOR_RF_ID,
      CHASSIS_MOTOR_LR_ID,
      CHASSIS_MOTOR_RR_ID);
  AdvanceArm_Init();

  // 外设初始化
  BusServo_Init(&huart4);

  /* 原点建立由 1 s 调度任务重试，不阻塞等待 OPS 数据。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  printf("STM32 init finish\r\n");
  // 1. 初始化电机驱动底层 (开启 DMA 接收等)
  // 2. 启动后立即闪烁 3 次作为“板子活了”的信号
  for (int i = 0; i < 6; i++)
  {
    App_ToggleLed();
    HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if (ONLINE_DEBUG_MODE != 0U)
  printf("ONLINE_DEBUG_MODE\r\n");
  while (1)
  {
    CommTuner_Process();
    __WFI();
  }
#else

  // 测试
  // 电机基础
  // Test_Chassis_Sign();
  // Test_Chassis_SetBodyVelocityEx();
  // Test_Chassis_MoveMecanumEx();
  // Test_MMCL();

  // wit ops
  // while(1)
  // {
  //   AdvanceTest_PrintImuOpsData();
  //   HAL_Delay(200);
  // }

  // Test_Servo1();

  // motion 依赖世界坐标数据
  // Test_Motion_SetWorldVelocityEx();
  // Test_Motion_GotoPoseBlocking();
  // Test_Motion_GotoPoseYawAndCancel();

  Competition_StartArea_t start_area;

  // 比赛尚未开始时只等待 Jetson 的有效启动请求
  while (CommJetson_TakeCompetitionStart(&start_area) == 0U)
  {
    __WFI();
  }
  // 主流程
  // Test_jetson(start_area);
  // App_RunTask(start_area);

  AdvanceVisual_State_t state;
  Chassis_Enable(true);
  HAL_Delay(100U);
  state = AdvanceVisual_AlignColorBlocking(RED);
  printf("[VISUAL] state=%u\r\n", (unsigned int)state);

  while (1)
  {
    __WFI();
  }
#endif

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LIFT_UP_LIMIT_Pin LIFT_DOWN_LIMIT_Pin SLIDE_FRONT_LIMIT_Pin */
  GPIO_InitStruct.Pin = LIFT_UP_LIMIT_Pin | LIFT_DOWN_LIMIT_Pin | SLIDE_FRONT_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SLIDE_REAR_LIMIT_Pin */
  GPIO_InitStruct.Pin = SLIDE_REAR_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SLIDE_REAR_LIMIT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART5)
  {
    OPS_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART2)
  {
    WIT_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART3)
  {
    drive_emm_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART6)
  {
    CommJetson_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART1)
  {
    CommTuner_OnUartRxEvent(huart, Size);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    drive_emm_OnTxComplete(huart);
  }

  if (huart->Instance == USART6)
  {
    CommJetson_OnUartTxComplete(huart);
  }

  if (huart->Instance == USART1)
  {
    CommTuner_OnUartTxComplete(huart);
  }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    drive_emm_OnTxAbortComplete(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    drive_emm_OnUartError(huart);
  }

  if (huart->Instance == UART5)
  {
    OPS_OnUartError(huart);
  }

  if (huart->Instance == USART2)
  {
    WIT_OnUartError(huart);
  }

  if (huart->Instance == USART6)
  {
    CommJetson_OnUartError(huart);
  }

  if (huart->Instance == USART1)
  {
    CommTuner_OnUartError(huart);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    App_TimerUpdate();
  }
}

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
