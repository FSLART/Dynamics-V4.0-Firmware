/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>

#define TIMER_FREQ 1000000.0f  // 1 MHz após prescaler de 71
#define Timer_Period (1.0f/TIMER_FREQ)  // 1μs
#define MAX_Speed_Measures 10
#define MAX_TIME_CAPTURE 5000 // Protection for Stoped Car
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Global Variables
uint32_t blink_time = 100, previus_blink = 0;
uint16_t ADC_VALUE[4];    // Array for ADC values~
uint32_t speed_capture_L; // Captured value of speed of left wheel
uint32_t speed_capture_R; // Captured value of speed of right wheel

// Variáveis para controle de captura alternada
volatile uint8_t captureState_L = 0, captureState_R = 0;
volatile uint32_t lowPeriod_L = 0, lowPeriod_R = 0;

// Moving Average for ADC
#define ADC_BUFFER_SIZE 20                // Tamanho do buffer para média móvel
uint16_t adc_buffers[4][ADC_BUFFER_SIZE]; // Buffer para cada canal ADC
uint8_t adc_buffer_index = 0;             // Índice circular do buffer
uint16_t adc_filtered[4];                 // Valores filtrados por média móvel

// Capture
uint32_t speed_measures_left[MAX_Speed_Measures];  // Vector to store values whitout filter
uint32_t speed_measures_right[MAX_Speed_Measures]; // Vector to store values whitout filter
uint32_t posL = 0, posR = 0;                       // Circular Index of the array
volatile uint32_t last_captured_pulseL, last_captured_pulseR;

// CAN
uint32_t can_send_interval = 50; // ms
uint32_t last_can_send = 0;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// Functions
float MeasureBrakePressure(uint16_t);
float MeasureSuspensionPosition(uint16_t);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim); // Input Capture Callback
float calculate_speed_wheel(capture);
void ADC_UpdateMovingAverage(void); // Função para atualizar média móvel do ADC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *data, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_VALUE, 4); // READ ADC VALUES TO DMA
  // Start Timer Interruptions for Input Captures
  HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);

  // Inicializar buffers para média móvel com zeros
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < ADC_BUFFER_SIZE; j++)
    {
      adc_buffers[i][j] = 0;
    }
    adc_filtered[i] = 0;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // LED HEARTBEAT
    int32_t current_time = HAL_GetTick();
    if ((current_time - previus_blink) >= blink_time)
    {
      previus_blink = current_time;
      HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    }
    // LED HEARTBEAT END

    // Atualizar médias móveis dos ADCs
    ADC_UpdateMovingAverage();

    // ADC VARIABLES
    //uint16_t adcST_Angle = adc_filtered[0]; // Steering Angle (Only Dynamics Front) - Filtrado
    uint16_t adcBRK_PRESS=adc_filtered[1]; //Brake Pressure (Only Dynamics Rear) - Filtrado
    uint16_t adcSuspL = adc_filtered[2]; // Suspension Left - Filtrado
    uint16_t adcSuspR = adc_filtered[3]; // Suspension Right - Filtrado

    // Steering Angle
    /*float ST_ANGLE;
    float inclive = 270.0f / 3072.0f;
    ST_ANGLE = (inclive * adcST_Angle) - 180.0;*/
    // End Steering Angle

    // Brake Pressure
    float BRK_PRESS;
    BRK_PRESS= MeasureBrakePressure(adcBRK_PRESS);
    // End Brake Pressure

    // Suspension Right
    float SUSP_R;
    SUSP_R = MeasureSuspensionPosition(adcSuspR);
    // End Suspension Right

    // Suspension Left
    float SUSP_L;
    SUSP_L = MeasureSuspensionPosition(adcSuspL);
    // End Suspension Left

    // Começa aqui os INPUT CAPTURES
    /*float filtered_time_left, filtered_time_right;
    Speed_Measures();
    filtered_time_left = Moving_Average_Filter_Left();
    filtered_time_right = Moving_Average_Filter_Right();
    if (last_captured_pulseL > MAX_TIME_CAPTURE)
    {
      int a = 0;
    }
    if (last_captured_pulseR > MAX_TIME_CAPTURE)
    {
      int B = 0;
    }*/
    // Termina aqui os INPUT CAPTURES

    // Começa aqui a parte de CAN

    uint32_t now = HAL_GetTick();
    if ((now - last_can_send) >= can_send_interval)
    {
      last_can_send = now;

      int16_t brake = (int16_t)(BRK_PRESS * 10);
      int16_t susp_r = (int16_t)(SUSP_R * 10);
      int16_t susp_l = (int16_t)(SUSP_L * 10);

      TxHeader.IDE = CAN_ID_STD;
      TxHeader.StdId = 0x456; // Confirmar
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.DLC = 6;

      TxData[0] = brake & 0xFF;        // Least significant byte first
      TxData[1] = (brake >> 8) & 0xFF; // Most significant byte
      TxData[2] = susp_r & 0xFF;
      TxData[3] = (susp_r >> 8) & 0xFF;
      TxData[4] = susp_l & 0xFF;
      TxData[5] = (susp_l >> 8) & 0xFF;

      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
    }
    // Termina aqui a parte de CAN

    // Bluetooth messages
    //printf("Steering Angle: %.2f ADC: %u \n", ST_ANGLE, adcST_Angle);
    printf("Pressure Brake %.2f bar  ADC: %u \n", BRK_PRESS, adcBRK_PRESS);
    printf("Suspension Right: %.2fmm ADC: %u \n", SUSP_R, adcSuspR);
    printf("Suspension Left: %.2fmm ADC: %u \n", SUSP_L, adcSuspL);

    // Bluetooth messages end

    // debug capture
    float tempo_real_D = speed_capture_R * (Timer_Period);
    float tempo_real_L = speed_capture_L * (Timer_Period);

    // Valores do período em nível baixo (quando o sinal está em 0)
    printf("Período em nível baixo direita: %.6f s (%lu ticks)\n", tempo_real_D, speed_capture_R);
    printf("Período em nível baixo esquerda: %.6f s (%lu ticks)\n", tempo_real_L, speed_capture_L);
    
    // Calcular frequência (2 * período do nível baixo para 50% duty cycle)
    if (tempo_real_D > 0) {
      float freq_D = 1.0f / (tempo_real_D * 2.0f);  // Para duty cycle de 50%
      printf("Frequência roda direita: %.2f Hz\n", freq_D);
    }
    
    if (tempo_real_L > 0) {
      float freq_L = 1.0f / (tempo_real_L * 2.0f);  // Para duty cycle de 50%
      printf("Frequência roda esquerda: %.2f Hz\n", freq_L);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterConfig1();
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterConfig2();
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 71;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim13, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 71;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPS_RX_Pin */
  GPIO_InitStruct.Pin = GPS_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPS_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_TX_Pin */
  GPIO_InitStruct.Pin = GPS_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPS_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_SDA_Pin */
  GPIO_InitStruct.Pin = GPS_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPS_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_SCL_Pin */
  GPIO_InitStruct.Pin = GPS_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPS_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_HEARTBEAT_Pin */
  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EEPROM_SCL_Pin EEPROM_SDA_Pin */
  GPIO_InitStruct.Pin = EEPROM_SCL_Pin|EEPROM_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float MeasureBrakePressure(uint16_t bits)
{
  // Constants for clarity
  const float ADC_MAX = 4095.0f;
  const float MCU_VREF = 3.3f;            // MCU reference voltage
  const float SENSOR_VREF = 5.0f;         // Sensor reference voltage
  const float CONVERSION_FACTOR = 0.667f; // Factor to convert 3.3V to 5V scale
  const float OFFSET_VOLTAGE = 0.5f;      // 0 bar = 0.5V
  const float SENSITIVITY = 0.02857f;     // 28.57mV/bar

  // Calculate voltage from ADC reading (0-3.3V range)
  float volts = (float)bits * MCU_VREF / ADC_MAX;

  // Scale to sensor voltage range (0-5V)
  volts = volts / CONVERSION_FACTOR;

  // Apply boundary checking for voltage
  if (volts < 0.0f)
  {
    volts = 0.0f;
  }
  else if (volts > SENSOR_VREF)
  {
    volts = SENSOR_VREF;
  }

  // Calculate pressure from voltage using calibration formula:
  // P(bar) = (V - 0.5V) / 0.02857V/bar
  float pressure = 0.0f;
  if (volts <= OFFSET_VOLTAGE)
  {
    pressure = 0.0f; // Anything below offset voltage is 0 bar
  }
  else
  {
    pressure = (volts - OFFSET_VOLTAGE) / SENSITIVITY;
  }

  // Limit maximum pressure if needed
  const float MAX_PRESSURE = 140.0f; // Maximum measurable pressure
  if (pressure > MAX_PRESSURE)
  {
    pressure = MAX_PRESSURE;
  }

  return pressure; // Return the brake pressure in bar
}

float MeasureSuspensionPosition(uint16_t bits)
{
  // Constants and Variables
  float V_SUSP;                // Voltage Signal from sensor
  float sensor_voltage = 5;    // MAX Voltage level from sensor
  float MCU_voltage = 3.3;     // MAX MCU voltage level
  float Eletrical_stroke = 75; // mm
  float SUSPENSION_POSITION;   // Suspension level in mm
  float Conversion_Factor = MCU_voltage / sensor_voltage;
  float volts; // converted voltage
  // Calculate Voltage from ADC
  V_SUSP = (3.3 * bits) / 4096;
  volts = V_SUSP / Conversion_Factor;
  // Calculate Position
  SUSPENSION_POSITION = (Eletrical_stroke * volts) / sensor_voltage;

  return SUSPENSION_POSITION; // return suspension level in millimeters
}

/**
 * @brief  Atualiza a média móvel para todos os canais ADC
 * @retval None
 */
void ADC_UpdateMovingAverage(void)
{
  // Para cada canal ADC
  for (int channel = 0; channel < 4; channel++)
  {
    // Adiciona o novo valor ao buffer circular
    adc_buffers[channel][adc_buffer_index] = ADC_VALUE[channel];

    // Calcular soma de todos os valores no buffer
    uint32_t sum = 0;
    for (int i = 0; i < ADC_BUFFER_SIZE; i++)
    {
      sum += adc_buffers[channel][i];
    }

    // Calcular média e atualizar o valor filtrado
    adc_filtered[channel] = (uint16_t)(sum / ADC_BUFFER_SIZE);
  }

  // Atualiza o índice circular
  adc_buffer_index = (adc_buffer_index + 1) % ADC_BUFFER_SIZE;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM13 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (captureState_L == 0) // Acabamos de detectar borda de descida (1→0)
    {
      // Resetamos o contador para começar a medir o tempo em nível baixo
      __HAL_TIM_SET_COUNTER(htim, 0);
      
      // Configuramos para detectar a próxima borda de subida (0→1)
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      captureState_L = 1;
    }
    else // Acabamos de detectar borda de subida (0→1)
    {
      // Capturamos o tempo que o sinal ficou em nível baixo
      speed_capture_L = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      lowPeriod_L = speed_capture_L; // Guardamos para usar depois
      
      // Configuramos para detectar a próxima borda de descida (1→0)
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      captureState_L = 0;
    }
  }
  else if (htim->Instance == TIM14 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (captureState_R == 0) // Acabamos de detectar borda de descida (1→0)
    {
      // Resetamos o contador para começar a medir o tempo em nível baixo
      __HAL_TIM_SET_COUNTER(htim, 0);
      
      // Configuramos para detectar a próxima borda de subida (0→1)
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      captureState_R = 1;
    }
    else // Acabamos de detectar borda de subida (0→1)
    {
      // Capturamos o tempo que o sinal ficou em nível baixo
      speed_capture_R = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      lowPeriod_R = speed_capture_R; // Guardamos para usar depois
      
      // Configuramos para detectar a próxima borda de descida (1→0)
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      captureState_R = 0;
    }
  }
}
/*
void HAL_SYSTICK_CALLBACK()
{
  last_captured_pulseL++; // Increments 1ms
  last_captured_pulseR++; // Increments 1ms
}
void Speed_Measures()
{
  speed_measures_left[posL] = time_capture_L;
  speed_measures_right[posR] = time_capture_R;
  posL = (posL + 1) % MAX_Speed_Measures;
  posR = (posR + 1) % MAX_Speed_Measures;
}
float Moving_Average_Filter_Left()
{
  uint32_t sum = 0;
  float filtered_speed_left;
  for (int i = 0; i < MAX_Speed_Measures; i++)
  {
    sum += speed_measures_left[i];
  }
  filtered_speed_left = sum / MAX_Speed_Measures;
  return filtered_speed_left;
}
float Moving_Average_Filter_Right()
{
  uint32_t sum = 0;
  float filtered_speed_right;
  for (int i = 0; i < MAX_Speed_Measures; i++)
  {
    sum += speed_measures_right[i];
  }
  filtered_speed_right = sum / MAX_Speed_Measures;
  return filtered_speed_right;
}
float Calculate_Speed_in_Wheel()
{
}
float calculate_speed_wheel(capture){

}*/

void CAN_FilterConfig1()
{
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x456 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x456 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 18; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}
void CAN_FilterConfig2()
{
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x456 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x456 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
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

#ifdef  USE_FULL_ASSERT
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
