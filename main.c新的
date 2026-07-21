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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== NEC timing ===== */
#define IR_START_ON_US             9000U
#define IR_START_OFF_US            4500U
#define IR_BIT_MARK_US             562U
#define IR_ZERO_SPACE_US           562U
#define IR_ONE_SPACE_US            1687U
#define IR_STOP_MARK_US            562U

/* ===== Send strategy ===== */

#define IR_SAFE_PERIOD_MS       120U
#define IR_EVENT_PERIOD_MS         150U
#define IR_ERROR_PERIOD_MS         300U
#define IR_MAIN_LOOP_DELAY_MS      5U

#define IR_PWM_DUTY                26U

/* ===== Event type：2 bit ===== */
#define IR_TYPE_NONE               0x00U  /* 00：安全 */
#define IR_TYPE_VRU_CROSS          0x01U  /* 01：弱勢用路人穿越 */
#define IR_TYPE_BRAKE              0x02U  /* 10：前前車緊急煞車 */
#define IR_TYPE_ERROR              0x03U  /* 11：系統錯誤 */

/* ===== Warning level ===== */
#define IR_LEVEL_NONE              0x00U
#define IR_LEVEL_1                 0x01U
#define IR_LEVEL_2                 0x02U
#define IR_LEVEL_3                 0x03U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t current_event_type = IR_TYPE_NONE;
static uint8_t current_level = IR_LEVEL_NONE;


static uint32_t last_send_tick = 0U;
static uint8_t force_send_now = 0U;

/* 避免相同事件重複刷新 LCD */
static uint8_t lcd_last_event_type = 0xFFU;
static uint8_t lcd_last_level = 0xFFU;
/* ===== 延遲量測變數 ===== */
static uint32_t timing_event_start_cycle = 0U;

static uint32_t timing_lcd_us = 0U;
static uint32_t timing_ir_wait_us = 0U;
static uint32_t timing_ir_tx_us = 0U;

/* 從進入Set_Event到LCD完成的總時間 */
static uint32_t timing_total_to_lcd_us = 0U;

/* 每次新事件的流水編號，方便和Python送出的事件對照 */
static uint32_t timing_sequence = 0U;

static uint8_t timing_first_tx_pending = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void UART_Print(const char *msg);

static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);
static uint32_t DWT_ElapsedUs(uint32_t start_cycle);

static void IR_CarrierOn(void);
static void IR_CarrierOff(void);
static void IR_SendStart(void);
static void IR_SendBit(uint8_t bitValue);
static void IR_SendStop(void);
static uint8_t IR_CalculateCRC2(uint8_t data4);
static uint8_t IR_MakePayload(uint8_t event_type, uint8_t level);
static void IR_SendPayload6(uint8_t payload);
static void IR_SendCurrentEvent(void);
static void IR_Process(void);

static void LCD_UpdateEvent(uint8_t event_type, uint8_t level);
static void Timing_PrintResult(uint8_t event_type, uint8_t level);
static void Set_Event(uint8_t event_type, uint8_t level);
static void Check_UART_Command(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Print(const char *msg)
{
  uint16_t len = 0U;

  while (msg[len] != '\0')
  {
    len++;
  }

  HAL_UART_Transmit(&huart4,
                    (uint8_t *)msg,
                    len,
                    HAL_MAX_DELAY);
}

/*
 * STM32F407 Cortex-M4 DWT 微秒延遲。
 * NEC 時序需要 us 等級，不能使用 HAL_Delay() 取代。
 */
static void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start_tick;
  uint32_t delay_ticks;

  start_tick = DWT->CYCCNT;
  delay_ticks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start_tick) < delay_ticks)
  {
  }
}
static uint32_t DWT_ElapsedUs(uint32_t start_cycle)
{
  uint32_t elapsed_cycles;

  elapsed_cycles = DWT->CYCCNT - start_cycle;

  return elapsed_cycles /
         (SystemCoreClock / 1000000U);
}
/* PA8 / TIM1_CH1 38 kHz carrier */
static void IR_CarrierOn(void)
{
  __HAL_TIM_SET_COMPARE(&htim1,
                        TIM_CHANNEL_1,
                        IR_PWM_DUTY);
}

static void IR_CarrierOff(void)
{
  __HAL_TIM_SET_COMPARE(&htim1,
                        TIM_CHANNEL_1,
                        0U);
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

  if (bitValue != 0U)
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
static void LCD_UpdateEvent(uint8_t event_type, uint8_t level)
{
  uint32_t lcd_start_cycle;

  /*
   * 預設這次沒有重新更新 LCD。
   * 如果相同畫面直接 return，LCD 時間會是 0。
   */
  timing_lcd_us = 0U;

  /*
   * 狀態相同就不重新繪圖，避免 LCD 閃爍。
   */
  if ((lcd_last_event_type == event_type) &&
      (lcd_last_level == level))
  {
    return;
  }

  lcd_last_event_type = event_type;
  lcd_last_level = level;

  /*
   * 開始測量整張畫面的更新時間。
   */
  lcd_start_cycle = DWT->CYCCNT;

  if (event_type == IR_TYPE_ERROR)
  {
    LCD_ShowSystemErrorScreen();
  }
  else if ((event_type == IR_TYPE_NONE) &&
           (level == IR_LEVEL_NONE))
  {
    LCD_ShowSafeScreen();
  }
  else if ((event_type == IR_TYPE_VRU_CROSS) &&
           (level == IR_LEVEL_1))
  {
    LCD_ShowLevel1Screen();
  }
  else if ((event_type == IR_TYPE_VRU_CROSS) &&
           (level == IR_LEVEL_2))
  {
    LCD_ShowLevel2Screen();
  }
  else if ((event_type == IR_TYPE_VRU_CROSS) &&
           (level == IR_LEVEL_3))
  {
    LCD_ShowLevel3Screen();
  }
  else if ((event_type == IR_TYPE_BRAKE) &&
          (level == IR_LEVEL_1))
  {
    LCD_ShowBrakeLevel1Screen();
  }
  else if ((event_type == IR_TYPE_BRAKE) &&
           (level == IR_LEVEL_2))
  {
    LCD_ShowBrakeLevel2Screen();
  }
  else if ((event_type == IR_TYPE_BRAKE) &&
          (level == IR_LEVEL_3))
  {
    LCD_ShowBrakeLevel3Screen();
  }
  else
  {
    LCD_ShowSystemErrorScreen();
  }

  /*
   * 畫面函式執行結束後，SPI 資料已經傳送完成。
   */
  timing_lcd_us =
      DWT_ElapsedUs(lcd_start_cycle);
}
/*
 * Payload：
 *
 * TTT LL CCC
 *
 * TTT：event type
 * LL ：warning level
 * CCC：TTT XOR 0LL
 */
/*
 * CRC-2 生成多項式：
 *
 * G(x) = x^2 + x + 1
 * 二進位表示為 111，也就是 0x07。
 *
 * 傳入資料格式：
 * TTLL，共4 bit。
 *
 * 回傳：
 * 2 bit CRC。
 */
static uint8_t IR_CalculateCRC2(uint8_t data4)
{
  uint8_t remainder;
  int8_t bit;

  /*
   * 只保留低4 bit，並在後面補2個0。
   *
   * 例如：
   * data4 = 0110
   * remainder = 011000
   */
  remainder = (uint8_t)((data4 & 0x0FU) << 2);

  /*
   * 使用 111 做模二除法。
   * 模二除法中的減法就是 XOR。
   */
  for (bit = 5; bit >= 2; bit--)
  {
    if ((remainder & (1U << bit)) != 0U)
    {
      remainder ^= (uint8_t)(0x07U << (bit - 2));
    }
  }

  /*
   * 最後只保留2 bit餘數。
   */
  return remainder & 0x03U;
}

/*
 * 6-bit Payload：
 *
 * TT LL CC
 *
 * TT：事件種類，2 bit
 * LL：事件等級，2 bit
 * CC：CRC-2，2 bit
 */
static uint8_t IR_MakePayload(uint8_t event_type,
                              uint8_t level)
{
  uint8_t data4;
  uint8_t crc2;

  /*
   * 將事件種類與等級組成4 bit：
   *
   * TTLL
   */
  data4 = (uint8_t)(((event_type & 0x03U) << 2) |
                     (level & 0x03U));

  crc2 = IR_CalculateCRC2(data4);

  /*
   * 組成：
   *
   * TTLLCC
   */
  return (uint8_t)((data4 << 2) | crc2);
}

/*
 * 6-bit Payload，MSB first。
 *
 * 傳送順序：
 * bit5 → bit4 → bit3 → bit2 → bit1 → bit0
 */
static void IR_SendPayload6(uint8_t payload)
{
  int8_t i;

  payload &= 0x3FU;

  for (i = 5; i >= 0; i--)
  {
    IR_SendBit((payload >> i) & 0x01U);
  }
}

static void IR_SendCurrentEvent(void)
{
  uint8_t payload;
  uint32_t ir_start_cycle;
  uint8_t is_first_tx;

  payload = IR_MakePayload(current_event_type,
                           current_level);

  /*
   * 記住這是不是新事件的第一包紅外線。
   */
  is_first_tx = timing_first_tx_pending;

  /*
   * 只有第一包才測量：
   * 從進入Set_Event到真正開始發射IR的等待時間。
   */
  if (is_first_tx != 0U)
  {
    timing_ir_wait_us =
        DWT_ElapsedUs(timing_event_start_cycle);
  }

  /*
   * 開始測量一包完整紅外線的發送時間。
   */
  ir_start_cycle = DWT->CYCCNT;

  IR_SendStart();
  IR_SendPayload6(payload);
  IR_SendStop();

  IR_CarrierOff();

  /*
   * 最後一個紅外線脈波結束後停止計時。
   */
  timing_ir_tx_us =
      DWT_ElapsedUs(ir_start_cycle);

  /*
   * 第一包已完成。
   * 完整時間稍後等LCD完成後再一起列印。
   */
  if (is_first_tx != 0U)
  {
    timing_first_tx_pending = 0U;
  }
}
/* 把新函式加在這裡 */
static void Timing_PrintResult(uint8_t event_type,
                               uint8_t level)
{
  char log_msg[220];
  uint8_t payload;

  payload = IR_MakePayload(event_type, level);

  timing_sequence++;

  snprintf(
      log_msg,
      sizeof(log_msg),
      "TIME,SEQ=%lu,TYPE=%u,LEVEL=%u,PAYLOAD=0x%02X,"
      "WAIT_TO_IR=%lu us,IR_TX=%lu us,LCD=%lu us,"
      "TOTAL_TO_LCD=%lu us\r\n",
      (unsigned long)timing_sequence,
      (unsigned int)event_type,
      (unsigned int)level,
      (unsigned int)payload,
      (unsigned long)timing_ir_wait_us,
      (unsigned long)timing_ir_tx_us,
      (unsigned long)timing_lcd_us,
      (unsigned long)timing_total_to_lcd_us
  );

  UART_Print(log_msg);
}
static void Set_Event(uint8_t event_type,
                      uint8_t level)
{
  /*
   * 從STM32已經取得事件、開始處理時計時。
   *
   * 注意：這不包含Nano影像辨識時間，
   * 也不包含UART字元尚未到達STM32之前的時間。
   */
  timing_event_start_cycle = DWT->CYCCNT;

  /*
   * 清除上一筆量測結果，
   * 避免誤把上一個事件的值印出來。
   */
  timing_lcd_us = 0U;
  timing_ir_wait_us = 0U;
  timing_ir_tx_us = 0U;
  timing_total_to_lcd_us = 0U;

  /*
   * 下一次紅外線是新事件的第一包。
   */
  timing_first_tx_pending = 1U;

  /*
   * 儲存目前事件。
   */
  current_event_type = event_type;
  current_level = level;

  /*
   * 新事件立即發送第一包IR。
   */
  force_send_now = 1U;
  last_send_tick = HAL_GetTick();

  /*
   * 先發送第一包紅外線。
   */
  IR_Process();

  /*
   * 紅外線完成後，再更新前車LCD。
   */
  LCD_UpdateEvent(event_type, level);

  /*
   * 從事件開始處理，到前車LCD完成的總時間。
   */
  timing_total_to_lcd_us =
      DWT_ElapsedUs(timing_event_start_cycle);

  /*
   * 此時IR和LCD都是這一次事件的結果，
   * 最後再一起列印。
   */
  Timing_PrintResult(event_type, level);
}
static void Check_UART_Command(void)
{
  uint8_t rx_data;

  /*
   * 只等待 1 ms，避免程式一直卡在 UART。
   */
  if (HAL_UART_Receive(&huart4,
                       &rx_data,
                       1U,
                       1U) != HAL_OK)
  {
    return;
  }

  switch (rx_data)
  {
    case '0':
      Set_Event(IR_TYPE_NONE,
                IR_LEVEL_NONE);

      //UART_Print("\r\nCMD 0: SAFE\r\n");
      break;

    case '1':
      Set_Event(IR_TYPE_VRU_CROSS,
                IR_LEVEL_1);

      //UART_Print("\r\nCMD 1: VRU Level 1\r\n");
      break;

    case '2':
      Set_Event(IR_TYPE_VRU_CROSS,
                IR_LEVEL_2);

      //UART_Print("\r\nCMD 2: VRU Level 2\r\n");
      break;

    case '3':
      Set_Event(IR_TYPE_VRU_CROSS,
                IR_LEVEL_3);

      //UART_Print("\r\nCMD 3: VRU Level 3\r\n");
      break;
      case '4':
      Set_Event(IR_TYPE_BRAKE,
                IR_LEVEL_1);

      //UART_Print("\r\nCMD 4: BRAKE Level 1\r\n");
      break;

    case '5':
      Set_Event(IR_TYPE_BRAKE,
                IR_LEVEL_2);

      //UART_Print("\r\nCMD 5: BRAKE Level 2\r\n");
      break;

    case '6':
      Set_Event(IR_TYPE_BRAKE,
                IR_LEVEL_3);

      //UART_Print("\r\nCMD 6: BRAKE Level 3\r\n");
      break;

    case 'E':
    case 'e':
      Set_Event(IR_TYPE_ERROR,
                IR_LEVEL_NONE);

      //UART_Print("\r\nCMD E: SYSTEM ERROR\r\n");
      break;

    case '\r':
    case '\n':
      /*
       * 忽略 Enter。
       */
      break;

    default:
      UART_Print("\r\nInvalid command\r\n");
      UART_Print("Use: 0, 1, 2, 3, 4, 5, 6 or E\r\n");
      break;
  }
}
static void IR_Process(void)
{
  uint32_t now;
  uint32_t period_ms;

  now = HAL_GetTick();

  /*
 * SAFE 狀態：
 * 每隔固定時間持續發送 00000000，
 * 直到目前狀態改變。
 */
if ((current_event_type == IR_TYPE_NONE) &&
    (current_level == IR_LEVEL_NONE))
{
  if ((force_send_now != 0U) ||
      ((now - last_send_tick) >= IR_SAFE_PERIOD_MS))
  {
    IR_SendCurrentEvent();

    last_send_tick = now;
    force_send_now = 0U;

    //UART_Print("TX SAFE 00000000\r\n");
  }

  return;
}

  /*
   * 系統錯誤每 300 ms 發送一次；
   * 一般事件每 150 ms 發送一次。
   */
  if (current_event_type == IR_TYPE_ERROR)
  {
    period_ms = IR_ERROR_PERIOD_MS;
  }
  else
  {
    period_ms = IR_EVENT_PERIOD_MS;
  }

  if ((force_send_now != 0U) ||
      ((now - last_send_tick) >= period_ms))
  {
    IR_SendCurrentEvent();

    last_send_tick = now;
    force_send_now = 0U;
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


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

/*
 * 等待 LCD 電源及控制器穩定。
 */
HAL_Delay(1000);

ILI9341_Init();
HAL_Delay(100);

/*
 * 初始化 DWT 微秒延遲。
 * NEC 紅外線時序需要 us 等級延遲。
 */
DWT_Delay_Init();

/*
 * 啟動 TIM1 Channel 1 PWM。
 */
if (HAL_TIM_PWM_Start(&htim1,
                      TIM_CHANNEL_1) != HAL_OK)
{
  Error_Handler();
}

/*
 * 開機預設先關閉紅外線載波。
 */
IR_CarrierOff();

/*
 * 開機初始狀態設定為 SAFE。
 * 這裡會同時：
 * 1. 顯示安全畫面
 * 2. 開始週期性發送 SAFE 封包
 */
Set_Event(IR_TYPE_NONE,
          IR_LEVEL_NONE);

UART_Print("\r\nLCD + IR TX Ready\r\n");
UART_Print("0 = SAFE\r\n");
UART_Print("1 = VRU Level 1\r\n");
UART_Print("2 = VRU Level 2\r\n");
UART_Print("3 = VRU Level 3\r\n");

UART_Print("4 = BRAKE Level 1\r\n");
UART_Print("5 = BRAKE Level 2\r\n");
UART_Print("6 = BRAKE Level 3\r\n");
UART_Print("E = SYSTEM ERROR\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
{
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

  /*
   * 檢查 UART4 是否收到 0、1、2、3、E。
   */
  Check_UART_Command();

  /*
   * 根據目前狀態發送紅外線。
   */
  IR_Process();

  /*
   * 避免主迴圈跑得過快。
   */
  HAL_Delay(IR_MAIN_LOOP_DELAY_MS);
  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
