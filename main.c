/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Automated Irrigation System
  *                   - Capacitive soil moisture sensor on PA0 (ADC1 CH0)
  *                   - Relay (pump control) on PA1
  *                   - External manual-override button on PA4 (EXTI4, active-low)
  *                   - LCD1602 in 4-bit mode on PB4-PB9
  *                   - TIM2 periodic ISR every 500 ms drives the main event queue
  *                   - UART2 streams ADC values to PuTTY at 115200 baud
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Simple software event-queue flags processed in the main loop */
typedef struct
{
  uint8_t doAdcRead      : 1;   /* TIM2 tick: time to sample the sensor      */
  uint8_t doLcdUpdate    : 1;   /* Time to refresh the LCD display            */
  uint8_t doUartPrint    : 1;   /* Time to send ADC value over UART           */
  uint8_t doRelayEval    : 1;   /* Time to evaluate relay on/off decision     */
  uint8_t manualOverride : 1;   /* External button pressed - force pump ON    */
} EventQueue_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -- Moisture threshold -----------------------------------------------------
 * ADC is 12-bit (0-4095).
 * Capacitive sensors output HIGH voltage (high ADC) when DRY and
 * LOW voltage (low ADC) when WET.
 * Pump engages when ADC reading EXCEEDS this value (soil is too dry).
 *
 * Calibrated values:
 *   DRY_VAL = 2775  (sensor in open air)
 *   WET_VAL = 1500  (sensor in water)
 *   Threshold = 2138 -> pump engages below 50% moisture
 * --------------------------------------------------------------------------*/
#define MOISTURE_THRESHOLD      2138u

/* Pump stays ON for at least this many 500 ms ticks before re-evaluating   */
#define PUMP_ON_TICKS           6u      /* 3 seconds                         */

/* Manual override keeps the pump ON for this many 500 ms ticks             */
#define MANUAL_OVERRIDE_TICKS   20u     /* 10 seconds                        */

/* -- LCD1602 GPIO mapping (4-bit mode) ------------------------------------- */
#define LCD_RS_PIN              GPIO_PIN_8
#define LCD_RS_PORT             GPIOB
#define LCD_EN_PIN              GPIO_PIN_9
#define LCD_EN_PORT             GPIOB
#define LCD_D4_PIN              GPIO_PIN_4
#define LCD_D4_PORT             GPIOB
#define LCD_D5_PIN              GPIO_PIN_5
#define LCD_D5_PORT             GPIOB
#define LCD_D6_PIN              GPIO_PIN_6
#define LCD_D6_PORT             GPIOB
#define LCD_D7_PIN              GPIO_PIN_7
#define LCD_D7_PORT             GPIOB

/* -- Relay GPIO ------------------------------------------------------------ */
#define RELAY_PIN               GPIO_PIN_1
#define RELAY_PORT              GPIOA
/* Relay module is active-LOW; flip these if yours is active-HIGH           */
#define RELAY_ON()              HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET)
#define RELAY_OFF()             HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET)

/* -- Manual override button (external, breadboard, active-LOW) ------------ */
#define BTN_PIN                 GPIO_PIN_4
#define BTN_PORT                GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static volatile EventQueue_t gEvents = {0};

static uint32_t gAdcValue      = 0;    /* Latest raw ADC reading            */
static uint32_t gPumpOnCounter = 0;    /* Ticks remaining for pump ON       */
static uint8_t  gPumpActive    = 0;    /* 1 = pump currently running        */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* -- LCD helper prototypes ------------------------------------------------- */
static void LCD_Init(void);
static void LCD_SendNibble(uint8_t nibble, uint8_t rs);
static void LCD_SendByte(uint8_t byte, uint8_t rs);
static void LCD_Command(uint8_t cmd);
static void LCD_Char(uint8_t ch);
static void LCD_SetCursor(uint8_t row, uint8_t col);
static void LCD_String(const char *str);
static void LCD_Clear(void);

/* -- Application helper prototypes ----------------------------------------- */
static uint32_t App_ReadADC(void);
static uint8_t  App_MoisturePercent(uint32_t adcVal);
static void     App_UpdateLCD(uint32_t adcVal, uint8_t pumpOn);
static void     App_UartPrintStatus(uint32_t adcVal, uint8_t moisture, uint8_t pumpOn);
static void     App_EvaluateRelay(uint32_t adcVal);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 *  LCD1602 - 4-bit mode driver
 * ============================================================ */

static void LCD_Pulse_EN(void)
{
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
}

static void LCD_SendNibble(uint8_t nibble, uint8_t rs)
{
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (nibble & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (nibble & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (nibble & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (nibble & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  LCD_Pulse_EN();
}

static void LCD_SendByte(uint8_t byte, uint8_t rs)
{
  LCD_SendNibble(byte >> 4, rs);
  LCD_SendNibble(byte & 0x0F, rs);
  HAL_Delay(2);
}

static void LCD_Command(uint8_t cmd)  { LCD_SendByte(cmd, 0); }
static void LCD_Char(uint8_t ch)      { LCD_SendByte(ch,  1); }

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
  LCD_Command(addr);
}

static void LCD_String(const char *str)
{
  while (*str)
  {
    LCD_Char((uint8_t)(*str++));
  }
}

static void LCD_Clear(void)
{
  LCD_Command(0x01);
  HAL_Delay(2);
}

static void LCD_Init(void)
{
  HAL_Delay(50);

  /* Special 3-pulse init sequence required by HD44780 datasheet           */
  LCD_SendNibble(0x03, 0);
  HAL_Delay(5);
  LCD_SendNibble(0x03, 0);
  HAL_Delay(1);
  LCD_SendNibble(0x03, 0);
  HAL_Delay(1);

  /* Switch to 4-bit mode                                                  */
  LCD_SendNibble(0x02, 0);
  HAL_Delay(1);

  LCD_Command(0x28);   /* Function set: 4-bit, 2 lines, 5x8 dots          */
  LCD_Command(0x0C);   /* Display ON, cursor OFF, blink OFF                */
  LCD_Command(0x06);   /* Entry mode: increment cursor, no display shift   */
  LCD_Clear();
}

/* ============================================================
 *  Application helpers
 * ============================================================ */

static uint32_t App_ReadADC(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t val = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return val;
}

/**
  * @brief  Convert raw ADC to 0-100 moisture percentage.
  *         Calibrated: DRY_VAL=2775 (air), WET_VAL=1500 (water).
  *         High ADC = dry, low ADC = wet.
  */
static uint8_t App_MoisturePercent(uint32_t adcVal)
{
  const uint32_t DRY_VAL = 2775u;
  const uint32_t WET_VAL = 1500u;

  if (adcVal >= DRY_VAL) return 0u;
  if (adcVal <= WET_VAL) return 100u;

  return (uint8_t)(((DRY_VAL - adcVal) * 100u) / (DRY_VAL - WET_VAL));
}

/**
  * @brief  Refresh both lines of the LCD.
  *         Line 1: "Moisture: XX %"
  *         Line 2: "Pump: ON " or "Pump: OFF"
  */
static void App_UpdateLCD(uint32_t adcVal, uint8_t pumpOn)
{
  char buf[17];
  uint8_t pct = App_MoisturePercent(adcVal);

  LCD_SetCursor(0, 0);
  snprintf(buf, sizeof(buf), "Moisture:%3u %%  ", pct);
  LCD_String(buf);

  LCD_SetCursor(1, 0);
  if (pumpOn)
  {
    LCD_String("Pump: ON        ");
  }
  else
  {
    LCD_String("Pump: OFF       ");
  }
}

/**
  * @brief  Send a formatted status line over UART2 (visible in PuTTY).
  *         Format: "ADC=XXXX  Moisture=XX%  Threshold=XXXX  Pump=ON\r\n"
  */
static void App_UartPrintStatus(uint32_t adcVal, uint8_t moisture, uint8_t pumpOn)
{
  char buf[64];
  int len = snprintf(buf, sizeof(buf),
                     "ADC=%4lu  Moisture=%3u%%  Threshold=%4u  Pump=%s\r\n",
                     (unsigned long)adcVal,
                     moisture,
                     (unsigned int)MOISTURE_THRESHOLD,
                     pumpOn ? "ON " : "OFF");
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

/**
  * @brief  Single source of truth for relay control.
  *         Once the pump turns ON it stays ON for PUMP_ON_TICKS ticks
  *         to avoid rapid relay cycling.
  */
static void App_EvaluateRelay(uint32_t adcVal)
{
  if (gPumpOnCounter > 0u)
  {
    /* Still inside the minimum-ON window - keep pump running              */
    gPumpOnCounter--;
    RELAY_ON();
    gPumpActive = 1u;
  }
  else if (adcVal > MOISTURE_THRESHOLD)
  {
    /* Soil is too dry - engage pump and start minimum-ON window           */
    gPumpOnCounter = PUMP_ON_TICKS;
    RELAY_ON();
    gPumpActive = 1u;
  }
  else
  {
    /* Moisture is adequate - disengage pump                               */
    RELAY_OFF();
    gPumpActive = 0u;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Ensure relay starts OFF                                                */
  RELAY_OFF();

  /* Boot message over UART                                                 */
  const char *bootMsg =
      "\r\n===== Automated Irrigation System =====\r\n"
      "Sampling every 500 ms.\r\n"
      "Adjust MOISTURE_THRESHOLD in main.c to set trigger point.\r\n"
      "ADC range: 0 (wet) - 4095 (dry)\r\n"
      "Calibration: DRY=2775  WET=1500  Threshold=2138 (50%%)\r\n"
      "========================================\r\n\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)bootMsg, (uint16_t)strlen(bootMsg), HAL_MAX_DELAY);

  /* Initialise LCD                                                         */
  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_String("  Irrigation    ");
  LCD_SetCursor(1, 0);
  LCD_String("   System v1.0  ");
  HAL_Delay(2000);
  LCD_Clear();

  /* Start TIM2 in interrupt mode - fires every 500 ms                     */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (gEvents.doAdcRead)
    {
      gEvents.doAdcRead   = 0u;
      gAdcValue           = App_ReadADC();
      gEvents.doRelayEval = 1u;
      gEvents.doLcdUpdate = 1u;
      gEvents.doUartPrint = 1u;
    }

    if (gEvents.manualOverride)
    {
      gEvents.manualOverride = 0u;
      gPumpOnCounter = MANUAL_OVERRIDE_TICKS;
      RELAY_ON();
      gPumpActive = 1u;

      const char *overrideMsg = "[MANUAL OVERRIDE] Pump forced ON!\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)overrideMsg,
                        (uint16_t)strlen(overrideMsg), HAL_MAX_DELAY);

      gEvents.doLcdUpdate = 1u;
    }

    if (gEvents.doRelayEval)
    {
      gEvents.doRelayEval = 0u;
      App_EvaluateRelay(gAdcValue);
    }

    if (gEvents.doLcdUpdate)
    {
      gEvents.doLcdUpdate = 0u;
      App_UpdateLCD(gAdcValue, gPumpActive);
    }

    if (gEvents.doUartPrint)
    {
      gEvents.doUartPrint = 0u;
      uint8_t moisture = App_MoisturePercent(gAdcValue);
      App_UartPrintStatus(gAdcValue, moisture, gPumpActive);
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution,
   *  Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding
   *  rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;      /* PA0                             */
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function - fires interrupt every 500 ms
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  /* TIM2 is on APB1 bus.
   * SYSCLK = 84 MHz, APB1 prescaler = /2 -> APB1 = 42 MHz.
   * TIM clock = 42 MHz x 2 (timer multiplier) = 84 MHz.
   * PSC = 8399 -> tick = 100 us ; ARR = 4999 -> 500 ms                  */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Relay output - PA1 starts LOW (relay OFF for active-HIGH module)      */
  HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin (onboard blue button)                      */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin                                           */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 - Relay output                                */
  GPIO_InitStruct.Pin = RELAY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 - External manual-override button (active-LOW)
   *  Wire: PA4 -> one side of button -> GND
   *  Internal pull-up keeps pin HIGH when button is not pressed.          */
  GPIO_InitStruct.Pin = BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

  /* -- LCD GPIO: PB4-PB7 (D4-D7), PB8 (RS), PB9 (EN) ------------------- */
  GPIO_InitStruct.Pin = LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN
                      | LCD_RS_PIN | LCD_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable EXTI interrupt for PA4 (EXTI4)                                 */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* Enable EXTI interrupt for PC13 / B1 (EXTI15_10)                       */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* Enable TIM2 interrupt                                                  */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM2 period-elapsed callback - fires every 500 ms.
  *         Sets the doAdcRead flag to kick off the event queue.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    gEvents.doAdcRead = 1u;

    /* Mirror pump state on the onboard LED for easy visual confirmation   */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                      gPumpActive ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

/**
  * @brief  GPIO EXTI callback - handles the manual button (PA4) and
  *         the onboard blue button (PC13 / B1).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t lastTick_BTN = 0;
  static uint32_t lastTick_B1  = 0;

  if (GPIO_Pin == BTN_PIN)
  {
    if ((HAL_GetTick() - lastTick_BTN) > 300u)
    {
      lastTick_BTN = HAL_GetTick();
      gEvents.manualOverride = 1u;
    }
  }
  else if (GPIO_Pin == B1_Pin)
  {
    if ((HAL_GetTick() - lastTick_B1) > 300u)
    {
      lastTick_B1 = HAL_GetTick();
      gEvents.manualOverride = 1u;
    }
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
