/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOA
#define ST_DOZ_Pin GPIO_PIN_1
#define ST_DOZ_GPIO_Port GPIOA
#define MQTT_TX_Pin GPIO_PIN_2
#define MQTT_TX_GPIO_Port GPIOA
#define MQTT_RX_Pin GPIO_PIN_3
#define MQTT_RX_GPIO_Port GPIOA
#define DWIN_TX_Pin GPIO_PIN_10
#define DWIN_TX_GPIO_Port GPIOB
#define DWIN_RX_Pin GPIO_PIN_11
#define DWIN_RX_GPIO_Port GPIOB
#define NASOS1_Pin GPIO_PIN_8
#define NASOS1_GPIO_Port GPIOA
#define ETH_TX_Pin GPIO_PIN_9
#define ETH_TX_GPIO_Port GPIOA
#define ETH_RX_Pin GPIO_PIN_10
#define ETH_RX_GPIO_Port GPIOA
#define POPLAVOK_Pin GPIO_PIN_11
#define POPLAVOK_GPIO_Port GPIOA
#define POPLAVOK_EXTI_IRQn EXTI15_10_IRQn
#define DOLIV_Pin GPIO_PIN_3
#define DOLIV_GPIO_Port GPIOB
#define HEATING_Pin GPIO_PIN_4
#define HEATING_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define UFO_Pin GPIO_PIN_6
#define UFO_GPIO_Port GPIOB
#define NASOS2_Pin GPIO_PIN_7
#define NASOS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
