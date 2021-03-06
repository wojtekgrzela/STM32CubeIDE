/*
 * Task_1000ms.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Wojciech Grzelinski
 */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Includes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "defines.h"
#include "../../VariousFunctions/Functions.h"
#include "../../lcd_hd44780_i2c/lcd_hd44780_i2c.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines And Typedefs */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

extern volatile uint16_t ADC1Measures[NO_OF_ADC1_MEASURES];
extern volatile uint16_t ADC3Measures[NO_OF_ADC3_MEASURES];

extern CAR_mileage_struct CAR_mileage;

extern volatile ENCButton_struct ENC_button_menu;
extern volatile int8_t EncoderCounterDiff;

extern boardTemperature_type temperature3V3DCDC;
extern boardTemperature_type temperature5VDCDC;

extern NTC_parameters_struct NTC;

extern LCD_message mainBatteryVoltageValueForLCD;
extern LCD_message auxiliaryBatteryVoltageValueForLCD;
extern LCD_message waterTemperatureValueForLCD;
extern LCD_message totalMileageForLCD;
extern LCD_message tripMileageForLCD;

extern LCD_message voltage3V3ForLCD;
extern LCD_message voltage5VForLCD;
extern LCD_message voltageInForLCD;
extern LCD_message temperature3V3DCDCForLCD;
extern LCD_message temperature5VDCDCForLCD;


extern boardVoltage_type voltage3V3;
extern boardVoltage_type voltage5V;
extern boardVoltage_type voltageIn;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void Start1000msTask(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_TASK_1000_MS_TIME_PERIOD;
	Error_Code error = NO_ERROR;

	static uint8_t totalMileageMessage[7] = "";
	static uint8_t tripMileageMessage[8] = "";
	totalMileageForLCD.messageHandler = totalMileageMessage;
	tripMileageForLCD.messageHandler = tripMileageMessage;

	static uint8_t temperature3V3DCDC_message[6] =
		{ SPACE_IN_ASCII };
	static uint8_t temperature5VDCDC_message[6] =
		{ SPACE_IN_ASCII };
	temperature3V3DCDCForLCD.messageHandler = temperature3V3DCDC_message;
	temperature5VDCDCForLCD.messageHandler = temperature5VDCDC_message;

	static uint8_t voltage3V3_message[5] =
		{ SPACE_IN_ASCII };
	static uint8_t voltage5V_message[5] =
		{ SPACE_IN_ASCII };
	static uint8_t voltageIn_message[6] =
		{ SPACE_IN_ASCII };
	voltage3V3ForLCD.messageHandler = voltage3V3_message;
	voltage5VForLCD.messageHandler = voltage5V_message;
	voltageInForLCD.messageHandler = voltageIn_message;

	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		error = calculate_NTC_temperature(&temperature5VDCDC,
				ADC3Measures[3]/*NTC near 5V stabilizer*/, &NTC);

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			temperature5VDCDCForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)temperature5VDCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (uint16_t)temperature5VDCDC, DEGREE_SYMBOL_LCD);
			temperature5VDCDCForLCD.size = strlen((char*)temperature5VDCDCForLCD.messageHandler);
			temperature5VDCDCForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_NTC_temperature(&temperature3V3DCDC, ADC3Measures[2]/*NTC near DC/DC*/,
				&NTC);

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			temperature3V3DCDCForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)temperature3V3DCDCForLCD.messageHandler, 6, "%" PRIi16 "%cC", (uint16_t)temperature3V3DCDC, DEGREE_SYMBOL_LCD);
			temperature3V3DCDCForLCD.size = strlen((char*)temperature3V3DCDCForLCD.messageHandler);
			temperature3V3DCDCForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_voltage(&voltageIn, ADC3Measures[0]/*Vin*/, MEASURE_VIN_VOLTAGE_DIVIDER);

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			voltageInForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)voltageInForLCD.messageHandler, 7, "%01" PRIu16 ".%02" PRIu16 "V", (uint16_t)voltageIn, (uint16_t)(voltageIn*100)%100);
			voltageInForLCD.size = strlen((char*)voltageInForLCD.messageHandler);
			voltageInForLCD.messageReadyFLAG = TRUE;
		}

		error = calculate_voltage(&voltage5V, ADC3Measures[1]/*5V*/, MEASURE_5V_VOLTAGE_DIVIDER);

		if (NO_ERROR != error)
		{
			my_error_handler(error);
		}
		else
		{
			voltage5VForLCD.messageReadyFLAG = FALSE;
			snprintf((char*)voltage5VForLCD.messageHandler, 6, "%01" PRIu16 ".%02" PRIi16 "V", (uint16_t)voltage5V, (uint16_t)(voltage5V*100)%100);
			voltage5VForLCD.size = strlen((char*)voltage5VForLCD.messageHandler);
			voltage5VForLCD.messageReadyFLAG = TRUE;
			;
		}

		totalMileageForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)totalMileageForLCD.messageHandler, 6, "%01" PRIu32, CAR_mileage.totalMileage);
		totalMileageForLCD.size = strlen((char*)totalMileageForLCD.messageHandler);
		totalMileageForLCD.messageReadyFLAG = TRUE;

		tripMileageForLCD.messageReadyFLAG = FALSE;
		snprintf((char*)tripMileageForLCD.messageHandler, 5, "%01" PRIu32 ".%01" PRIu32, (CAR_mileage.tripMileage/10), (CAR_mileage.tripMileage)%10);
		tripMileageForLCD.size = strlen((char*)tripMileageForLCD.messageHandler);
		tripMileageForLCD.messageReadyFLAG = TRUE;

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}
