/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ST_ANGLE_Pin GPIO_PIN_1
#define ST_ANGLE_GPIO_Port GPIOA
#define BRK_PRESS_Pin GPIO_PIN_2
#define BRK_PRESS_GPIO_Port GPIOA
#define SPEED_L_Pin GPIO_PIN_6
#define SPEED_L_GPIO_Port GPIOA
#define SPEED_R_Pin GPIO_PIN_7
#define SPEED_R_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_5
#define GPS_RX_GPIO_Port GPIOC
#define SUSP_R_Pin GPIO_PIN_0
#define SUSP_R_GPIO_Port GPIOB
#define SUSP_L_Pin GPIO_PIN_1
#define SUSP_L_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOB
#define CH2_RX_Pin GPIO_PIN_12
#define CH2_RX_GPIO_Port GPIOB
#define CH2_TX_Pin GPIO_PIN_13
#define CH2_TX_GPIO_Port GPIOB
#define GPS_SDA_Pin GPIO_PIN_9
#define GPS_SDA_GPIO_Port GPIOC
#define GPS_SCL_Pin GPIO_PIN_8
#define GPS_SCL_GPIO_Port GPIOA
#define BL_TX_Pin GPIO_PIN_9
#define BL_TX_GPIO_Port GPIOA
#define BL_RX_Pin GPIO_PIN_10
#define BL_RX_GPIO_Port GPIOA
#define CH1_TX_Pin GPIO_PIN_12
#define CH1_TX_GPIO_Port GPIOA
#define LED_HEARTBEAT_Pin GPIO_PIN_11
#define LED_HEARTBEAT_GPIO_Port GPIOC
#define EEPROM_SCL_Pin GPIO_PIN_6
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
