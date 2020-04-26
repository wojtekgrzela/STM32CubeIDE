/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_READY_Pin GPIO_PIN_13
#define LED_READY_GPIO_Port GPIOC
#define LED_WORKING_Pin GPIO_PIN_14
#define LED_WORKING_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_15
#define LED_ERROR_GPIO_Port GPIOC
#define ENCODER_A_Pin GPIO_PIN_0
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_1
#define ENCODER_B_GPIO_Port GPIOA
#define ENCODER_BUTTON_Pin GPIO_PIN_2
#define ENCODER_BUTTON_GPIO_Port GPIOA
#define ENCODER_BUTTON_EXTI_IRQn EXTI2_IRQn
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOA
#define EN_ENG_Pin GPIO_PIN_0
#define EN_ENG_GPIO_Port GPIOB
#define IN2_ENG_Pin GPIO_PIN_2
#define IN2_ENG_GPIO_Port GPIOB
#define IN1_ENG_Pin GPIO_PIN_10
#define IN1_ENG_GPIO_Port GPIOB
#define EN_EMAG_Pin GPIO_PIN_11
#define EN_EMAG_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define BRAKE_Pin GPIO_PIN_13
#define BRAKE_GPIO_Port GPIOB
#define BRAKE_EXTI_IRQn EXTI15_10_IRQn
#define CLUTCH_Pin GPIO_PIN_14
#define CLUTCH_GPIO_Port GPIOB
#define CLUTCH_EXTI_IRQn EXTI15_10_IRQn
#define RPM_Pin GPIO_PIN_8
#define RPM_GPIO_Port GPIOA
#define RPM_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
