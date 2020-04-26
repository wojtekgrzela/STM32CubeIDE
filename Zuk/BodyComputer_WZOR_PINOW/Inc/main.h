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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_3_Pin GPIO_PIN_2
#define GPIO_3_GPIO_Port GPIOE
#define GPIO_4_Pin GPIO_PIN_3
#define GPIO_4_GPIO_Port GPIOE
#define GPIO_5_Pin GPIO_PIN_4
#define GPIO_5_GPIO_Port GPIOE
#define GPIO_6_Pin GPIO_PIN_5
#define GPIO_6_GPIO_Port GPIOE
#define GPIO_7_Pin GPIO_PIN_6
#define GPIO_7_GPIO_Port GPIOE
#define GPIO_10_Pin GPIO_PIN_13
#define GPIO_10_GPIO_Port GPIOC
#define GPIO_11_Pin GPIO_PIN_14
#define GPIO_11_GPIO_Port GPIOC
#define GPIO_12_Pin GPIO_PIN_15
#define GPIO_12_GPIO_Port GPIOC
#define MEASURE_VIN_Pin GPIO_PIN_0
#define MEASURE_VIN_GPIO_Port GPIOC
#define MEASURE_5V_Pin GPIO_PIN_1
#define MEASURE_5V_GPIO_Port GPIOC
#define BOARD_TEMP_1_Pin GPIO_PIN_2
#define BOARD_TEMP_1_GPIO_Port GPIOC
#define BOARD_TEMP_2_Pin GPIO_PIN_3
#define BOARD_TEMP_2_GPIO_Port GPIOC
#define ADC_7_Pin GPIO_PIN_0
#define ADC_7_GPIO_Port GPIOA
#define ADC_8_Pin GPIO_PIN_1
#define ADC_8_GPIO_Port GPIOA
#define ADC_1_Pin GPIO_PIN_2
#define ADC_1_GPIO_Port GPIOA
#define ADC_2_Pin GPIO_PIN_3
#define ADC_2_GPIO_Port GPIOA
#define ADC_DAC_1_Pin GPIO_PIN_4
#define ADC_DAC_1_GPIO_Port GPIOA
#define ADC_DAC_2_Pin GPIO_PIN_5
#define ADC_DAC_2_GPIO_Port GPIOA
#define ADC_3_Pin GPIO_PIN_6
#define ADC_3_GPIO_Port GPIOA
#define ADC_4_Pin GPIO_PIN_7
#define ADC_4_GPIO_Port GPIOA
#define ADC_5_Pin GPIO_PIN_4
#define ADC_5_GPIO_Port GPIOC
#define ADC_6_Pin GPIO_PIN_5
#define ADC_6_GPIO_Port GPIOC
#define TIM3_CH3_Pin GPIO_PIN_0
#define TIM3_CH3_GPIO_Port GPIOB
#define TIM3_CH4_Pin GPIO_PIN_1
#define TIM3_CH4_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define GPIO_8_Pin GPIO_PIN_7
#define GPIO_8_GPIO_Port GPIOE
#define GPIO_9_Pin GPIO_PIN_8
#define GPIO_9_GPIO_Port GPIOE
#define TIM1_CH1_Pin GPIO_PIN_9
#define TIM1_CH1_GPIO_Port GPIOE
#define LED_8_Pin GPIO_PIN_10
#define LED_8_GPIO_Port GPIOE
#define TIM1_CH2_Pin GPIO_PIN_11
#define TIM1_CH2_GPIO_Port GPIOE
#define LED_7_Pin GPIO_PIN_12
#define LED_7_GPIO_Port GPIOE
#define TIM1_CH3_Pin GPIO_PIN_13
#define TIM1_CH3_GPIO_Port GPIOE
#define TIM1_CH4_Pin GPIO_PIN_14
#define TIM1_CH4_GPIO_Port GPIOE
#define LED_6_Pin GPIO_PIN_15
#define LED_6_GPIO_Port GPIOE
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define ENC_BUTTON_Pin GPIO_PIN_12
#define ENC_BUTTON_GPIO_Port GPIOB
#define MOSFET_16_Pin GPIO_PIN_10
#define MOSFET_16_GPIO_Port GPIOD
#define MOSFET_15_Pin GPIO_PIN_11
#define MOSFET_15_GPIO_Port GPIOD
#define TIM4_CH1_Pin GPIO_PIN_12
#define TIM4_CH1_GPIO_Port GPIOD
#define TIM4_CH2_Pin GPIO_PIN_13
#define TIM4_CH2_GPIO_Port GPIOD
#define TIM4_CH3_Pin GPIO_PIN_14
#define TIM4_CH3_GPIO_Port GPIOD
#define TIM4_CH4_Pin GPIO_PIN_15
#define TIM4_CH4_GPIO_Port GPIOD
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define PPS_GPS_IN_Pin GPIO_PIN_8
#define PPS_GPS_IN_GPIO_Port GPIOA
#define USART1_TX_GPS_Pin GPIO_PIN_9
#define USART1_TX_GPS_GPIO_Port GPIOA
#define USART1_RX_GPS_Pin GPIO_PIN_10
#define USART1_RX_GPS_GPIO_Port GPIOA
#define TIM2_CH1_Pin GPIO_PIN_15
#define TIM2_CH1_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_0
#define LED_5_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_1
#define LED_4_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOD
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_7
#define LED_1_GPIO_Port GPIOD
#define TIM2_CH2_Pin GPIO_PIN_3
#define TIM2_CH2_GPIO_Port GPIOB
#define TIM3_CH1_Pin GPIO_PIN_4
#define TIM3_CH1_GPIO_Port GPIOB
#define TIM3_CH2_Pin GPIO_PIN_5
#define TIM3_CH2_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_6
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB
#define EEPROM_WP1_Pin GPIO_PIN_8
#define EEPROM_WP1_GPIO_Port GPIOB
#define EEPROM_WP2_Pin GPIO_PIN_9
#define EEPROM_WP2_GPIO_Port GPIOB
#define GPIO_1_Pin GPIO_PIN_0
#define GPIO_1_GPIO_Port GPIOE
#define GPIO_2_Pin GPIO_PIN_1
#define GPIO_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
