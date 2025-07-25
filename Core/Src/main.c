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

// Dynamics Front or Dynamics Rear?

#define DYNAMICS_FRONT
//#define DYNAMICS_REAR

/************************************************************************/

#include <stdio.h>
#include <math.h>

#define TIMER_FREQ 62937.0f              // 72MHz/(1143+1) = 62.937 kHz
#define Timer_Period (1.0f / TIMER_FREQ) // 1μs
#define MAX_Speed_Measures 10

#define MAX_VALID_PERIOD 65534           // Máximo período válido do timer


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

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Global Variables
uint32_t blink_time = 100, previus_blink = 0;
uint16_t ADC_VALUE[4];                       // Array for ADC values
uint32_t time_capture_L;                     // Captured value of speed of left wheel
uint32_t time_capture_R;                     // Captured value of speed of right wheel

// Variables for alternated capture control
volatile uint8_t captureState_L = 0, captureState_R = 0;
volatile uint32_t lowPeriod_L = 0, lowPeriod_R = 0;

volatile uint32_t last_capture_time_L = 0;   // Last capture timestamp for left wheel
volatile uint32_t last_capture_time_R = 0;   // Last capture timestamp for right wheel
const uint32_t WHEEL_TIMEOUT_MS = 500;      // Wheel timeout in milliseconds
// Moving Average for ADC 
#define ADC_BUFFER_SIZE 50                   // Buffer size for moving average
uint16_t adc_buffers[4][ADC_BUFFER_SIZE];    // Buffer for each ADC channel
uint8_t adc_buffer_index = 0;               // Circular buffer index
uint16_t adc_filtered[4];                   // Filtered values by moving average

// Capture
uint32_t speed_measures_left[MAX_Speed_Measures];  // Vector to store values without filter
uint32_t speed_measures_right[MAX_Speed_Measures]; // Vector to store values without filter
uint32_t posL = 0, posR = 0;                       // Circular index of the array

// CAN
uint32_t can_send_interval = 50;                    // CAN send interval in milliseconds
uint32_t last_can_send = 0;                        // Last CAN send timestamp

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// Functions
float MeasureSteeringAngle(uint16_t);
float MeasureBrakePressure(uint16_t);
float MeasureSuspensionPosition(uint16_t);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);  // Input Capture Callback
void Speed_Measures(void);
float Calculate_Speed_in_Wheel(float,uint32_t);           // Calculate Speed in Wheel
void ADC_UpdateMovingAverage(void);                       // Function to update ADC moving average
float Moving_Average_Filter_Right(void);
float Moving_Average_Filter_Left(void);
void CAN_FilterConfig1(void);
void CAN_FilterConfig2(void); 

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
static void MX_IWDG_Init(void);
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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  
  //Watchdog
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADC_Start_DMA(&hadc1, ADC_VALUE, 4);    // Start ADC DMA for reading values
  // Start Timer Interruptions for Input Captures
  HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);

  // Initialize moving average buffers with zeros
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
    //WatchDog
    HAL_IWDG_Refresh(&hiwdg);
    
    // LED HEARTBEAT
    int32_t current_time = HAL_GetTick();
    if ((current_time - previus_blink) >= blink_time)
    {
      previus_blink = current_time;
      HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    }
    // LED HEARTBEAT END

    // Update ADC moving averages
    ADC_UpdateMovingAverage();

// ADC VARIABLES
#ifdef DYNAMICS_FRONT
    uint16_t adcST_Angle = adc_filtered[0];         // Steering Angle (Only Dynamics Front) - Filtered
#endif
#ifdef DYNAMICS_REAR
    uint16_t adcBRK_PRESS = adc_filtered[1];        // Brake Pressure (Only Dynamics Rear) - Filtered
#endif
    uint16_t adcSuspL = adc_filtered[2];            // Suspension Left - Filtered
    uint16_t adcSuspR = adc_filtered[3];            // Suspension Right - Filtered

// Steering Angle
#ifdef DYNAMICS_FRONT
    float ST_ANGLE;
    ST_ANGLE=MeasureSteeringAngle(adcST_Angle);
#endif
// End Steering Angle

// Brake Pressure
#ifdef DYNAMICS_REAR
    float BRK_PRESS;
    BRK_PRESS = MeasureBrakePressure(adcBRK_PRESS);
#endif
    // End Brake Pressure

    // Suspension Right
    float SUSP_R;
    SUSP_R = MeasureSuspensionPosition(adcSuspR);
    // End Suspension Right

    // Suspension Left
    float SUSP_L;
    SUSP_L = MeasureSuspensionPosition(adcSuspL);
    // End Suspension Left

    // Input Capture processing starts here
    float filtered_time_left, filtered_time_right;
    float speed_km_left, speed_km_right;
    Speed_Measures();
    filtered_time_left = Moving_Average_Filter_Left();
    filtered_time_right = Moving_Average_Filter_Right();
    float time_R = filtered_time_right * Timer_Period;
    float time_L = filtered_time_left * Timer_Period;
    speed_km_left = Calculate_Speed_in_Wheel(time_L, last_capture_time_L);
    speed_km_right = Calculate_Speed_in_Wheel(time_R, last_capture_time_R);
    if (speed_km_left < 0.1f) speed_km_left = 0.0f;
    if (speed_km_right < 0.1f) speed_km_right = 0.0f;

// Input Capture processing ends here

// CAN communication starts here
#ifdef DYNAMICS_FRONT
    uint32_t now = HAL_GetTick();
    if ((now - last_can_send) >= can_send_interval)
    {
      last_can_send = now;

      int16_t angle = (int16_t)(ST_ANGLE * 10);
      uint16_t susp_r = (uint16_t)(SUSP_R * 10);
      uint16_t susp_l = (uint16_t)(SUSP_L * 10);
      uint16_t spd_left = (uint16_t)(speed_km_left * 10);
      uint16_t spd_right = (uint16_t)(speed_km_right * 10);

      TxHeader.IDE = CAN_ID_STD;
      TxHeader.StdId = 0x446;                       // Confirm ID
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.DLC = 6;

      TxData[0] = angle & 0xFF;                     // Least significant byte first
      TxData[1] = (angle >> 8) & 0xFF;              // Most significant byte
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

      TxHeader.StdId = 0x456;
      TxHeader.DLC = 4;

      TxData[0] = spd_left & 0xFF;
      TxData[1] = (spd_left >> 8) & 0xFF;
      TxData[2] = spd_right & 0xFF;
      TxData[3] = (spd_right >> 8) & 0xFF;

      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
    }

#endif

#ifdef DYNAMICS_REAR
    uint32_t now = HAL_GetTick();
    if ((now - last_can_send) >= can_send_interval)
    {
      last_can_send = now;

      uint16_t brake = (uint16_t)(BRK_PRESS * 10);
      uint16_t susp_r = (uint16_t)(SUSP_R * 10);
      uint16_t susp_l = (uint16_t)(SUSP_L * 10);
      uint16_t spd_left = (uint16_t)(speed_km_left * 10);
      uint16_t spd_right = (uint16_t)(speed_km_right * 10);

      TxHeader.IDE = CAN_ID_STD;
      TxHeader.StdId = 0x546;                       // Confirm ID
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.DLC = 6;

      TxData[0] = brake & 0xFF;                     // Least significant byte first
      TxData[1] = (brake >> 8) & 0xFF;              // Most significant byte
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

      TxHeader.StdId = 0x556;
      TxHeader.DLC = 4;

      TxData[0] = spd_left & 0xFF;
      TxData[1] = (spd_left >> 8) & 0xFF;
      TxData[2] = spd_right & 0xFF;
      TxData[3] = (spd_right >> 8) & 0xFF;

      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
      if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        Error_Handler();
      }
    }

#endif
// CAN communication ends here

// Bluetooth messages
static uint32_t last_debug_time = 0;
uint32_t current_debug_time = HAL_GetTick();

// Imprimir apenas a cada 500ms
if ((current_debug_time - last_debug_time) >= 500) {
    last_debug_time = current_debug_time;
    
    #ifdef DYNAMICS_FRONT
    printf("Steering Angle: %.2f ADC: %u \n", ST_ANGLE, adcST_Angle);
    #endif
    #ifdef DYNAMICS_REAR
    printf("Pressure Brake %.2f bar  ADC: %u \n", BRK_PRESS, adcBRK_PRESS);
    #endif
    printf("Susp R: %.2fmm Susp L: %.2fmm\n", SUSP_R, SUSP_L);
    printf("Speed L: %.2f Speed R: %.2f km/h\n", speed_km_left, speed_km_right);
}

    // Bluetooth messages end
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim13.Init.Prescaler = 1143;
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
  htim14.Init.Prescaler = 1143;
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
float MeasureSteeringAngle(uint16_t bits)
{
// Constants and variables
  const float ADC_MAX = 4095.0f;
  const float MCU_VREF = 3.3f;                    // MCU reference voltage
  const float SENSOR_VREF_Max = 4.5f;             // Maximum sensor reference voltage
  const float SENSOR_VREF_Min = 0.5f;             // Minimum sensor reference voltage
  const float Resolution = 180.0f;                // Resolution of sensor -180 to 180
  const float OFFSET = -31.3f;                    // In case of mechanical problems, put offset angle
  
  // Calculate Function Slope
  float max_v = SENSOR_VREF_Max * (2.0f/3.0f);
  float min_v = SENSOR_VREF_Min * (2.0f/3.0f);
  float inclination = 360.0f / (max_v - min_v);
  
  // Calculate Steering Angle
  float V_STA = (MCU_VREF * bits) / ADC_MAX;
  float ST_Angle = inclination * V_STA - 45 - Resolution;

  // If necessary add OFFSET
  ST_Angle += OFFSET;
  return ST_Angle;
}

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
  float V_SUSP;                                    // Voltage Signal from sensor
  float sensor_voltage = 5;                       // MAX Voltage level from sensor
  float MCU_voltage = 3.3;                        // MAX MCU voltage level
  float Electrical_stroke = 75;                   // mm
  float SUSPENSION_POSITION;                      // Suspension level in mm
  float Conversion_Factor = MCU_voltage / sensor_voltage;
  float volts;                                    // converted voltage
  // Calculate Voltage from ADC
  V_SUSP = (MCU_voltage * bits) / 4095;
  volts = V_SUSP / Conversion_Factor;
  // Calculate Position
  SUSPENSION_POSITION = (Electrical_stroke * volts) / sensor_voltage;

  return SUSPENSION_POSITION;                     // return suspension level in millimeters
}

/**
 * @brief  Updates the moving average for all ADC channels
 * @retval None
 */
void ADC_UpdateMovingAverage(void)
{
  // For each ADC channel
  for (int channel = 0; channel < 4; channel++)
  {
    // Add the new value to the circular buffer
    adc_buffers[channel][adc_buffer_index] = ADC_VALUE[channel];

    // Calculate sum of all values in the buffer
    uint32_t sum = 0;
    for (int i = 0; i < ADC_BUFFER_SIZE; i++)
    {
      sum += adc_buffers[channel][i];
    }

    // Calculate average and update the filtered value
    adc_filtered[channel] = (uint16_t)(sum / ADC_BUFFER_SIZE);
  }

  // Update circular index
  adc_buffer_index = (adc_buffer_index + 1) % ADC_BUFFER_SIZE;
}

/**
 * @brief  Timer Input Capture Callback - Handles wheel speed sensor pulses
 * @param  htim: Timer handle that triggered the interrupt
 * @note   This function processes falling edge pulses from wheel speed sensors
 *         TIM13 - Left wheel speed sensor (Hall sensor or encoder)
 *         TIM14 - Right wheel speed sensor (Hall sensor or encoder)
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t lastCapture_L = 0;               // Previous capture value for left wheel
  static uint32_t lastCapture_R = 0;               // Previous capture value for right wheel
  static uint8_t firstCapture_L = 1;               // Flag for first capture on left wheel
  static uint8_t firstCapture_R = 1;               // Flag for first capture on right wheel

  // Left wheel speed sensor processing (TIM13)
  if (htim->Instance == TIM13 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t calculated_period;
    
    // Update timestamp for timeout detection
    last_capture_time_L = HAL_GetTick();

    // Handle first capture - just store the value for reference
    if (firstCapture_L)
    {
      lastCapture_L = currentCapture;
      firstCapture_L = 0;
      time_capture_L = 0;                          // No valid period yet
    }
    else
    {
      // Calculate period between consecutive pulses
      // Handle timer overflow case (16-bit timer with 65535 max value)
      if (currentCapture >= lastCapture_L)
      {
        calculated_period = currentCapture - lastCapture_L;
      }
      else
      {
        // Timer overflow occurred
        calculated_period = (65535 - lastCapture_L) + currentCapture + 1;
      }

      // Validate period is within acceptable range
      if (calculated_period > 0 && calculated_period <= MAX_VALID_PERIOD)
      {
        time_capture_L = calculated_period;        // Store valid period
      }
      else
      {
        time_capture_L = 0;                        // Invalid period, likely noise
      }

      lastCapture_L = currentCapture;              // Update reference for next capture
    }
  }

  // Right wheel speed sensor processing (TIM14)
  else if (htim->Instance == TIM14 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t calculated_period;
    
    // Update timestamp for timeout detection
    last_capture_time_R = HAL_GetTick();

    // Handle first capture - just store the value for reference
    if (firstCapture_R)
    {
      lastCapture_R = currentCapture;
      firstCapture_R = 0;
      time_capture_R = 0;                          // No valid period yet
    }
    else
    {
      // Calculate period between consecutive pulses
      // Handle timer overflow case (16-bit timer with 65535 max value)
      if (currentCapture >= lastCapture_R)
      {
        calculated_period = currentCapture - lastCapture_R;
      }
      else
      {
        // Timer overflow occurred
        calculated_period = (65535 - lastCapture_R) + currentCapture + 1;
      }

      // Validate period is within acceptable range
      if (calculated_period > 0 && calculated_period <= MAX_VALID_PERIOD)
      {
        time_capture_R = calculated_period;        // Store valid period
      }
      else
      {
        time_capture_R = 0;                        // Invalid period, likely noise
      }
      
      lastCapture_R = currentCapture;              // Update reference for next capture
    }
  }
}

void Speed_Measures()
{
  speed_measures_left[posL] = time_capture_L;
  speed_measures_right[posR] = time_capture_R;
  posL = (posL + 1) % MAX_Speed_Measures;
  posR = (posR + 1) % MAX_Speed_Measures;
}
float Moving_Average_Filter_Left(void)
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
float Moving_Average_Filter_Right(void)
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
float Calculate_Speed_in_Wheel(float time, uint32_t last_capture_time)
{
  // Constants and Variables
  const int teeth = 36;                            // Number of teeth in CogWheel
  const float inches_diameter = 20.5;              // Diameter of Wheels in inches
  const float diameter = 25.4 * inches_diameter;   // Diameter of Wheels in mm
  const float perimeter = diameter * 3.141592;     // Perimeter of wheel in mm
  const float perimeter_m = perimeter / 1000.0f;   // Perimeter in Meters
  
  uint32_t current_time = HAL_GetTick();
  if ((current_time - last_capture_time) > WHEEL_TIMEOUT_MS)
  {
    return 0.0f;                                   // Wheel stopped by timeout
  }

   if (time <= 0)
  {
    return 0.0f;                                   // Stopped Wheel
  }
  
  // Calculate Frequency
  float frequency = 1.0f / time;                   // Hz

  // Calculate Wheel RPM
  float wheel_rpm = (frequency * 60) / teeth;

  // Calculate Speed
  float speed_km_per_hour = (perimeter_m * wheel_rpm) * 0.06;

  return speed_km_per_hour;                        // Return value of Speed in Km/h
}

void CAN_FilterConfig1()
{
#ifdef DYNAMICS_FRONT
  CAN_FilterTypeDef canfilterconfig;

  // Filter for 0x446
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;                  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x446 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x446 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 18;       // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  
  // Filter for 0x456
   canfilterconfig.FilterBank = 1;
  canfilterconfig.FilterIdHigh = 0x456 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  
#endif

#ifdef DYNAMICS_REAR
  CAN_FilterTypeDef canfilterconfig;

  // Filter for 0x546
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;                  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x546 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x546 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 18;       // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  // Filter for 0x556
   canfilterconfig.FilterBank = 1;
  canfilterconfig.FilterIdHigh = 0x556 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
#endif
}
void CAN_FilterConfig2()
{
#ifdef DYNAMICS_FRONT
  CAN_FilterTypeDef canfilterconfig;

  // Filter for 0x446
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;                 // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x446 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x446 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  // Filter for 0x456
  canfilterconfig.FilterBank = 19;                 // Different bank for CAN2
  canfilterconfig.FilterIdHigh = 0x456 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
#endif
#ifdef DYNAMICS_REAR
  CAN_FilterTypeDef canfilterconfig;

  // Filter for 0x546
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;                 // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x546 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x546 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  // Filter for 0x556
  canfilterconfig.FilterBank = 19;
  canfilterconfig.FilterIdHigh = 0x556 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x7FF << 5;
  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
#endif
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
