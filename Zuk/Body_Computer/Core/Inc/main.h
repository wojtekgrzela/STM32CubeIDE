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

#include "../ErrorCodes/ErrorCodes.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Diagnostic codes */
typedef enum
{
	DIAGNOSTICS_OK								= 0,

	WATER_TEMP_HIGH_WARNING						= 10,
	WATER_TEMP_HIGH_ALARM						= 11,

	OIL_TEMP_HIGH_WARNING						= 20,
	OIL_TEMP_HIGH_ALARM							= 21,

	LOW_VOLTAGE_MAIN_BATTERY_ALARM				= 30,
	HIGH_VOLTAGE_MAIN_BATTERY_ALARM				= 31,

	LOW_VOLTAGE_AUXILIARY_BATTERY_ALARM			= 40,
	HIGH_VOLTAGE_AUXILIARY_BATTERY_ALARM		= 41,

	UNKNOWN_ISSUE								= 0xF0F0F0F0
}Diagnostic_Code;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Diagnostic Snapshot */
typedef struct
{
	Diagnostic_Code snapshotIdentificator;				//4 bytes total
	uint8_t RawTime[6/*Size of raw time from GPS*/];	//12 bytes total (2 unused)
	uint16_t value;										//12 bytes total
}Diagnostic_Snapshot_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Internal Error Snapshot */
typedef struct
{
	Error_Code ErrorIdentificator;						//4 bytes total
	uint8_t RawTime[6/*Size of raw time from GPS*/];	//12 bytes total (2 unused)

}Error_Snapshot_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ENCODER BUTTON STRUCT */
typedef union
{
	uint8_t allFlags;
	struct
	{
		uint8_t shortPressDetected		:1;	/* LSB!!! */
		uint8_t longPressDetected		:1;

		uint8_t longPressInfoForISR		:1;	/* This is an info for the external
												interrupt callback that there
												was a long press detected and
												not to set the short press on */
	};
}ENCButton_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


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
#define MEASURE_VIN_Pin GPIO_PIN_0
#define MEASURE_VIN_GPIO_Port GPIOC
#define MEASURE_5V_Pin GPIO_PIN_1
#define MEASURE_5V_GPIO_Port GPIOC
#define BOARD_TEMP_STABILIZER_Pin GPIO_PIN_2
#define BOARD_TEMP_STABILIZER_GPIO_Port GPIOC
#define BOARD_TEMP_DCDC_Pin GPIO_PIN_3
#define BOARD_TEMP_DCDC_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOB
#define LED_8_Pin GPIO_PIN_10
#define LED_8_GPIO_Port GPIOE
#define LED_7_Pin GPIO_PIN_12
#define LED_7_GPIO_Port GPIOE
#define LED_6_Pin GPIO_PIN_15
#define LED_6_GPIO_Port GPIOE
#define I2C2_SCL_LCD_Pin GPIO_PIN_10
#define I2C2_SCL_LCD_GPIO_Port GPIOB
#define I2C2_SDA_LCD_Pin GPIO_PIN_11
#define I2C2_SDA_LCD_GPIO_Port GPIOB
#define ENC_BUTTON_Pin GPIO_PIN_12
#define ENC_BUTTON_GPIO_Port GPIOB
#define ENC_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define USART1_TX_GPS_Pin GPIO_PIN_9
#define USART1_TX_GPS_GPIO_Port GPIOA
#define USART1_RX_GPS_Pin GPIO_PIN_10
#define USART1_RX_GPS_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_0
#define LED_5_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_1
#define LED_4_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_7
#define LED_1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_EEPROM_Pin GPIO_PIN_6
#define I2C1_SCL_EEPROM_GPIO_Port GPIOB
#define I2C1_SDA_EEPROM_Pin GPIO_PIN_7
#define I2C1_SDA_EEPROM_GPIO_Port GPIOB
#define EEPROM_WP_1_Pin GPIO_PIN_8
#define EEPROM_WP_1_GPIO_Port GPIOB
#define EEPROM_WP_2_Pin GPIO_PIN_9
#define EEPROM_WP_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define RUNTIME_STATS_AND_DEBUG

#ifdef RUNTIME_STATS_AND_DEBUG
#define RUNTIME_STATS_TIMER_CONFIG
#define RUNTIME_STATS_QUEUES
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* TASKS & TIMERS FREQUENCIES */
#define MY_LCD_TASK_TIME_PERIOD				(TickType_t)(200)	/* MINIMUM: 100, for the 4x20 LCD it is needed approximately 80-85ms to send all data */
#define MY_GPS_TASK_TIME_PERIOD				(TickType_t)(1000)
#define MY_EEPROM_TASK_TIME_PERIOD			(TickType_t)(1000)	/* Would be good to decrease this number significantly, as it only executes when it has data from a Queue */
#define MY_DUMP_TO_EEPROM_TASK_TIME_PERIOD 	(TickType_t)(1000)

#define ENC_BUTTON_LONG_PRESS_TIME			((uint32_t)(3000))	/* Value in milliseconds */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* FUNCTIONALITIES AND FEATURES */
#define BOARD_TEMPERATURE_MEASUREMENT
#define BOARD_VOLTAGE_MEASUREMENT

#define GPS_RECEIVING
#ifdef GPS_RECEIVING
	#define GPS_PARSING
	#define GPS_ON_MICROSD_WRITE
#endif
#ifdef GPS_PARSING
	#define GPS_LCD_PRINT
#endif
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* ADCs */
#define MAX_ADC_VALUE							((uint32_t)(4095))
#define NO_OF_ADC3_MEASURES						((uint32_t)(4))
#define MEASURE_VIN_VOLTAGE_DIVIDER_x10000		((uint32_t)(1601))	/* R1/(R1+R2) = 1k/(1k+4.7k) ~ 0.17543859649 BUT after measuring: 0.16016260*/
#define MEASURE_5V_VOLTAGE_DIVIDER_x10000		((uint32_t)(4443))	/* R1/(R1+R2) = 1.6k/(1.6k+1.6k) = 0.5  BUT after measuring: 0.444342629*/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Symbols */
#define TRUE									((uint8_t)(1))
#define True									((uint8_t)(1))
#define FALSE									((uint8_t)(0))
#define False									((uint8_t)(0))
#define DEGREE_SYMBOL_LCD						((uint8_t)(0b11011111))
#define SPACE_IN_ASCII							((uint8_t)(0x20))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* EEPROM */
#define EEPROM_CAR_ADDRESS							((uint16_t)(0b10100000))
#define EEPROM_BOARD_ADDRESS						((uint16_t)(0b10100010))

#define EEPROM_CAR_BLOCK_PORT						(EEPROM_WP_1_GPIO_Port)
#define EEPROM_BOARD_BLOCK_PORT						(EEPROM_WP_2_GPIO_Port)
#define EEPROM_CAR_BLOCK_PIN						(EEPROM_WP_1_Pin)
#define EEPROM_BOARD_BLOCK_PIN						(EEPROM_WP_2_Pin)

/* EEPROM CAR ADDRESSES AND SETTINGS */
#define TOTAL_MILEAGE_START_ADDRESS					((uint16_t)(100))
#define TRIP_MILEAGE_START_ADDRESS					((uint16_t)(140))
#define TOTAL_MILEAGE_TABLE_SIZE					((uint8_t)(10))
#define TRIP_MILEAGE_TABLE_SIZE						((uint8_t)(10))
#define TOTAL_SNAPCHOTS_NUMBER_ADDRESS				((uint16_t)(999))
#define DIAGNOSTIC_SNAPSHOTS_START_ADDRESS			((uint16_t)(1000))

#define WATER_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS	((uint16_t)(200))
#define WATER_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(201))
#define WATER_HIGH_TEMP_FAN_ON_THRESHOLD_ADDRESS	((uint16_t)(202))
#define WATER_HIGH_TEMP_FAN_OFF_THRESHOLD_ADDRESS	((uint16_t)(203))
#define WATER_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(204))

#define OIL_HIGH_TEMP_WARNING_THRESHOLD_ADDRESS		((uint16_t)(210))
#define OIL_HIGH_TEMP_ALARM_THRESHOLD_ADDRESS		((uint16_t)(211))
#define OIL_TEMP_ALL_SETTINGS_ADDRESS				((uint16_t)(212))

#define MAIN_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(220))	/* 2 bytes value! */
#define MAIN_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS		((uint16_t)(222))	/* 2 bytes value! */
#define MAIN_BATTERY_ALL_SETTINGS_ADDRESS						((uint16_t)(224))

#define AUXILIARY_BATTERY_LOW_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(230))	/* 2 bytes value */
#define AUXILIARY_BATTERY_HIGH_VOLTAGE_ALARM_THRESHOLD_ADDRESS	((uint16_t)(232))	/* 2 bytes value */
#define AUXILIARY_BATTERY_ALL_SETTINGS_ADDRESS					((uint16_t)(234))

#define FUEL_LOW_LEVEL_WARNING_THRESHOLD_ADDRESS				((uint16_t)(240))
#define FUEL_ALL_SETTINGS_ADDRESS								((uint16_t)(241))

#define OIL_HIGH_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(250))
#define OIL_LOW_PRESSURE_ALARM_THRESHOLD_ADDRESS				((uint16_t)(251))
#define OIL_PRESSURE_ALL_SETTINGS_ADDRESS						((uint16_t)(252))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
