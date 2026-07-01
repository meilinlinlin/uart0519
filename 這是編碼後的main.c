/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 IR TX main program controlled by UART command
  *
  * IR protocol:
  *   NEC timing-based simplified packet
  *   Start + 8-bit Payload + Stop
  *
  * Payload format:
  *   TTT LL CCC
  *   TTT = 3-bit event type
  *   LL  = 2-bit level
  *   CCC = 3-bit check code = TTT XOR 0LL
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN PD */

/* ===== NEC timing reference =====
 * Start : 9 ms ON + 4.5 ms OFF
 * bit 0 : 562 us ON + 562 us OFF
 * bit 1 : 562 us ON + 1687 us OFF
 * Stop  : 562 us ON
 */
#define IR_START_ON_US       9000U
#define IR_START_OFF_US      4500U
#define IR_BIT_MARK_US       562U
#define IR_ZERO_SPACE_US     562U
#define IR_ONE_SPACE_US      1687U
#define IR_STOP_MARK_US      562U

// ===== 發送策略 =====
// 按 0 代表「無事件」，只送 SAFE 封包數次讓接收端確認安全，送完就停止紅外線。
// 按 1/2/3/E 代表有效事件，才週期性重複送出。
#define IR_SAFE_CONFIRM_TX_COUNT   3U     // SAFE 連續送 3 次，讓接收端確認安全
#define IR_SAFE_BURST_GAP_MS       120U   // SAFE 封包彼此間隔
#define IR_EVENT_PERIOD_MS         150U   // Level 1/2/3 事件重複發送間隔
#define IR_ERROR_PERIOD_MS         300U   // 系統錯誤重複發送間隔
#define IR_MAIN_LOOP_DELAY_MS      5U

// TIM1_CH1 的 PWM duty，需對應 MX_TIM1_Init() 裡的 Period = 52
// Pulse = 26 約為 50% duty
#define IR_PWM_DUTY          26U

/* ===== Event Type: TTT, 3 bit ===== */
#define IR_TYPE_NONE         0x00U   // 000：無事件
#define IR_TYPE_VRU_CROSS    0x01U   // 001：弱勢用路人穿越
#define IR_TYPE_BRAKE        0x02U   // 010：前車急煞，未來擴充
#define IR_TYPE_OBSTACLE     0x03U   // 011：障礙物 / 施工，未來擴充
#define IR_TYPE_ACCIDENT     0x04U   // 100：事故，未來擴充
#define IR_TYPE_LANE         0x05U   // 101：車道異常，未來擴充
#define IR_TYPE_RESERVED     0x06U   // 110：保留
#define IR_TYPE_ERROR        0x07U   // 111：系統錯誤

/* ===== Level: LL, 2 bit ===== */
#define IR_LEVEL_NONE        0x00U   // 00：無事件 / 無等級
#define IR_LEVEL_1           0x01U   // 01：Level 1 注意
#define IR_LEVEL_2           0x02U   // 10：Level 2 警告
#define IR_LEVEL_3           0x03U   // 11：Level 3 緊急

/* USER CODE END PD */

TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
static uint8_t current_event_type = IR_TYPE_NONE;
static uint8_t current_level = IR_LEVEL_NONE;

// SAFE 不要一直送，只在按 0 後送固定次數
static uint8_t safe_tx_remaining = 0;

// 控制週期性發送用
static uint32_t last_send_tick = 0;
static uint8_t force_send_now = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
static void UART_Print(const char *msg);

static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);

static void IR_CarrierOn(void);
static void IR_CarrierOff(void);
static void IR_SendStart(void);
static void IR_SendBit(uint8_t bitValue);
static void IR_SendStop(void);
static uint8_t IR_MakePayload(uint8_t event_type, uint8_t level);
static void IR_SendPayload8(uint8_t payload);
static void IR_SendCurrentEvent(void);

static void Panel_ShowSafe(void);
static void Panel_ShowLevel1(void);
static void Panel_ShowLevel2(void);
static void Panel_ShowLevel3(void);
static void Panel_ShowError(void);
static void Panel_Update(uint8_t event_type, uint8_t level);

static void Set_Event(uint8_t event_type, uint8_t level);
static void Check_UART_Command(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void UART_Print(const char *msg)
{
  uint16_t len = 0;

  while (msg[len] != '\0')
  {
    len++;
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)msg, len, HAL_MAX_DELAY);
}

/*
 * DWT cycle counter delay
 * STM32F407 是 Cortex-M4，可以使用 DWT 做 us 等級 delay。
 * 注意：NEC timing 需要微秒，不能用 HAL_Delay() 取代。
 */
static void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - startTick) < delayTicks)
  {
    // wait
  }
}

/* ===== Panel LED display =====
 * F407 DISCOVERY 常見 LED：
 * LD4 = Green, LD3 = Orange, LD5 = Red, LD6 = Blue
 */
static void Panel_ShowSafe(void)
{
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
}

static void Panel_ShowLevel1(void)
{
  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
}

static void Panel_ShowLevel2(void)
{
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD6_Pin, GPIO_PIN_RESET);
}

static void Panel_ShowLevel3(void)
{
  HAL_GPIO_WritePin(GPIOD, LD5_Pin | LD6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin, GPIO_PIN_RESET);
}

static void Panel_ShowError(void)
{
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
}

static void Panel_Update(uint8_t event_type, uint8_t level)
{
  if (event_type == IR_TYPE_ERROR)
  {
    Panel_ShowError();
  }
  else if (event_type == IR_TYPE_NONE && level == IR_LEVEL_NONE)
  {
    Panel_ShowSafe();
  }
  else if (level == IR_LEVEL_1)
  {
    Panel_ShowLevel1();
  }
  else if (level == IR_LEVEL_2)
  {
    Panel_ShowLevel2();
  }
  else if (level == IR_LEVEL_3)
  {
    Panel_ShowLevel3();
  }
  else
  {
    Panel_ShowError();
  }
}

/*
 * PA8 / TIM1_CH1 輸出 38kHz PWM 給紅外線發射器。
 * 這裡不反覆 Start/Stop Timer，而是讓 PWM 一直啟動，
 * 用 CCR1 = IR_PWM_DUTY / 0 控制載波是否輸出，時間比較穩。
 */
static void IR_CarrierOn(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, IR_PWM_DUTY);
}

static void IR_CarrierOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

static void IR_SendStart(void)
{
  IR_CarrierOn();
  delay_us(IR_START_ON_US);

  IR_CarrierOff();
  delay_us(IR_START_OFF_US);
}

static void IR_SendBit(uint8_t bitValue)
{
  IR_CarrierOn();
  delay_us(IR_BIT_MARK_US);

  IR_CarrierOff();

  if (bitValue)
  {
    delay_us(IR_ONE_SPACE_US);
  }
  else
  {
    delay_us(IR_ZERO_SPACE_US);
  }
}

static void IR_SendStop(void)
{
  IR_CarrierOn();
  delay_us(IR_STOP_MARK_US);

  IR_CarrierOff();
}

/*
 * 8-bit payload 格式：
 *   TTT LL CCC
 *
 *   TTT = event_type[2:0]
 *   LL  = level[1:0]
 *   CCC = TTT XOR 0LL
 *
 * 範例：
 *   無事件：TTT=000, LL=00, CCC=000 => 00000000 = 0x00
 *   弱勢用路人 L1：TTT=001, LL=01, CCC=000 => 00101000 = 0x28
 *   弱勢用路人 L2：TTT=001, LL=10, CCC=011 => 00110011 = 0x33
 *   弱勢用路人 L3：TTT=001, LL=11, CCC=010 => 00111010 = 0x3A
 *   系統錯誤：TTT=111, LL=00, CCC=111 => 11100111 = 0xE7
 */
static uint8_t IR_MakePayload(uint8_t event_type, uint8_t level)
{
  uint8_t ttt = event_type & 0x07U;
  uint8_t ll  = level & 0x03U;
  uint8_t ccc = (ttt ^ ll) & 0x07U;   // ll 等同 0LL

  return (uint8_t)((ttt << 5) | (ll << 3) | ccc);
}

/*
 * 我們的 payload 表格是從左到右讀 TTT LL CCC，
 * 所以這裡用 MSB first 傳送。
 */
static void IR_SendPayload8(uint8_t payload)
{
  for (int8_t i = 7; i >= 0; i--)
  {
    IR_SendBit((payload >> i) & 0x01U);
  }
}

static void IR_SendCurrentEvent(void)
{
  uint8_t payload = IR_MakePayload(current_event_type, current_level);

  IR_SendStart();
  IR_SendPayload8(payload);
  IR_SendStop();
}

static void Set_Event(uint8_t event_type, uint8_t level)
{
  current_event_type = event_type;
  current_level = level;

  // 每次收到新指令，都讓下一輪立即送一次
  force_send_now = 1;
  last_send_tick = HAL_GetTick();

  if (event_type == IR_TYPE_NONE && level == IR_LEVEL_NONE)
  {
    // 無事件：只送幾次 SAFE 封包給接收端確認安全，送完就停止紅外線
    safe_tx_remaining = IR_SAFE_CONFIRM_TX_COUNT;
  }
  else
  {
    // 有事件或系統錯誤：持續週期性送出
    safe_tx_remaining = 0;
  }

  Panel_Update(current_event_type, current_level);
}

static void Check_UART_Command(void)
{
  uint8_t rxData;

  // timeout 1ms，不要卡死等 UART
  if (HAL_UART_Receive(&huart4, &rxData, 1, 1) == HAL_OK)
  {
    if (rxData == '0')
    {
      Set_Event(IR_TYPE_NONE, IR_LEVEL_NONE);

      UART_Print("\r\nCMD 0 received\r\n");
      UART_Print("Mode: SAFE / NO EVENT, send 00000000 three times, then IR OFF\r\n");
    }
    else if (rxData == '1')
    {
      Set_Event(IR_TYPE_VRU_CROSS, IR_LEVEL_1);

      UART_Print("\r\nCMD 1 received\r\n");
      UART_Print("Mode: VRU CROSS Level 1, send payload 00101000\r\n");
    }
    else if (rxData == '2')
    {
      Set_Event(IR_TYPE_VRU_CROSS, IR_LEVEL_2);

      UART_Print("\r\nCMD 2 received\r\n");
      UART_Print("Mode: VRU CROSS Level 2, send payload 00110011\r\n");
    }
    else if (rxData == '3')
    {
      Set_Event(IR_TYPE_VRU_CROSS, IR_LEVEL_3);

      UART_Print("\r\nCMD 3 received\r\n");
      UART_Print("Mode: VRU CROSS Level 3, send payload 00111010\r\n");
    }
    else if (rxData == 'E' || rxData == 'e')
    {
      Set_Event(IR_TYPE_ERROR, IR_LEVEL_NONE);

      UART_Print("\r\nCMD E received\r\n");
      UART_Print("Mode: SYSTEM ERROR, send payload 11100111\r\n");
    }
    else if (rxData == '\r' || rxData == '\n')
    {
      // ignore Enter
    }
    else
    {
      UART_Print("\r\nInvalid command.\r\n");
      UART_Print("Use: 0=SAFE, 1=Level1, 2=Level2, 3=Level3, E=ERROR\r\n");
    }
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_UART4_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  DWT_Delay_Init();

  // PWM Timer 啟動一次，之後用 CCR1 控制載波 ON/OFF
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  IR_CarrierOff();

  Set_Event(IR_TYPE_NONE, IR_LEVEL_NONE);

  // 開機閃一下 LD5，確認新程式真的有跑
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
  HAL_Delay(300);

  Set_Event(IR_TYPE_NONE, IR_LEVEL_NONE);

  UART_Print("\r\nSTM32 IR TX Ready\r\n");
  UART_Print("Protocol: NEC timing-based simplified 8-bit packet\r\n");
  UART_Print("Packet: Start + TTT LL CCC + Stop\r\n");
  UART_Print("Commands:\r\n");
  UART_Print("0 = SAFE / NO EVENT    -> 00000000, send 3 times then IR OFF\r\n");
  UART_Print("1 = VRU CROSS Level 1  -> 00101000\r\n");
  UART_Print("2 = VRU CROSS Level 2  -> 00110011\r\n");
  UART_Print("3 = VRU CROSS Level 3  -> 00111010\r\n");
  UART_Print("E = SYSTEM ERROR       -> 11100111\r\n\r\n");

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    Check_UART_Command();

    Panel_Update(current_event_type, current_level);

    uint32_t now = HAL_GetTick();

    if (current_event_type == IR_TYPE_NONE && current_level == IR_LEVEL_NONE)
    {
      // 無事件：只送固定次數 SAFE 封包，送完之後保持 IR 關閉，不再閃爍
      if (safe_tx_remaining > 0)
      {
        if (force_send_now || ((now - last_send_tick) >= IR_SAFE_BURST_GAP_MS))
        {
          IR_SendCurrentEvent();      // payload = 00000000
          IR_CarrierOff();

          safe_tx_remaining--;
          last_send_tick = now;
          force_send_now = 0;

          UART_Print("TX SAFE payload 00000000\r\n");

          if (safe_tx_remaining == 0)
          {
            UART_Print("SAFE burst finished, IR OFF\r\n");
          }
        }
      }
      else
      {
        // SAFE 確認封包送完後，真的不再發紅外線
        IR_CarrierOff();
        force_send_now = 0;
      }
    }
    else
    {
      // 有事件才週期性重複發送，讓後車持續收到警示
      uint32_t period_ms = (current_event_type == IR_TYPE_ERROR) ?
                            IR_ERROR_PERIOD_MS : IR_EVENT_PERIOD_MS;

      if (force_send_now || ((now - last_send_tick) >= period_ms))
      {
        IR_SendCurrentEvent();
        IR_CarrierOff();

        last_send_tick = now;
        force_send_now = 0;
      }
    }

    HAL_Delay(IR_MAIN_LOOP_DELAY_MS);

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 52;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  // 一開始先關閉載波，之後用 IR_CarrierOn() 設成 IR_PWM_DUTY
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
  __HAL_RCC_UART4_CLK_ENABLE();

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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // PA8 = TIM1_CH1，用來輸出 38kHz PWM 給紅外線發射器
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PC10 = UART4_TX, PC11 = UART4_RX
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 初始化板上 LED
  HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // PB0 保留成紅外線接收輸入，發射端不使用也沒關係
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
    // 錯誤時 LD6 亮
    HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
