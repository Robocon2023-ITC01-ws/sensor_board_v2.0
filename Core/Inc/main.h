/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BTN3_Pin GPIO_PIN_2
#define BTN3_GPIO_Port GPIOA
#define BTN3_EXTI_IRQn EXTI2_IRQn
#define BTN4_Pin GPIO_PIN_3
#define BTN4_GPIO_Port GPIOA
#define BTN4_EXTI_IRQn EXTI3_IRQn
#define BTN5_Pin GPIO_PIN_4
#define BTN5_GPIO_Port GPIOA
#define BTN5_EXTI_IRQn EXTI4_IRQn
#define BTN6_Pin GPIO_PIN_5
#define BTN6_GPIO_Port GPIOA
#define BTN6_EXTI_IRQn EXTI9_5_IRQn
#define BTN7_Pin GPIO_PIN_0
#define BTN7_GPIO_Port GPIOB
#define BTN7_EXTI_IRQn EXTI0_IRQn
#define BTN8_Pin GPIO_PIN_1
#define BTN8_GPIO_Port GPIOB
#define BTN8_EXTI_IRQn EXTI1_IRQn
#define BTN10_Pin GPIO_PIN_10
#define BTN10_GPIO_Port GPIOB
#define BTN10_EXTI_IRQn EXTI15_10_IRQn
#define BTN11_Pin GPIO_PIN_11
#define BTN11_GPIO_Port GPIOB
#define BTN11_EXTI_IRQn EXTI15_10_IRQn
#define BTN12_Pin GPIO_PIN_12
#define BTN12_GPIO_Port GPIOB
#define BTN12_EXTI_IRQn EXTI15_10_IRQn
#define BTN13_Pin GPIO_PIN_13
#define BTN13_GPIO_Port GPIOB
#define BTN13_EXTI_IRQn EXTI15_10_IRQn
#define BTN14_Pin GPIO_PIN_14
#define BTN14_GPIO_Port GPIOB
#define BTN14_EXTI_IRQn EXTI15_10_IRQn
#define BTN15_Pin GPIO_PIN_15
#define BTN15_GPIO_Port GPIOB
#define BTN15_EXTI_IRQn EXTI15_10_IRQn
#define Buzzer_Pin GPIO_PIN_5
#define Buzzer_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_8
#define BTN1_GPIO_Port GPIOB
#define BTN1_EXTI_IRQn EXTI9_5_IRQn
#define BTN2_Pin GPIO_PIN_9
#define BTN2_GPIO_Port GPIOB
#define BTN2_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
