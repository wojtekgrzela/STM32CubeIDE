/*
 * Task_AlarmControl.c
 *
 *  Created on: Jan 24, 2021
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
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Defines */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BUZZER_SIGNAL_SHORT_ONE_TIME	50u
#define BUZZER_SIGNAL_LONG_ONE_TIME		1000u
#define BUZZER_SIGNAL_SHORT_TWO_TIME	500u
#define BUZZER_SIGNAL_SHORT_FIVE_TIME	500u
#define BUZZER_SIGNAL_LONG_THREE_TIME	1000u
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Extern variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern volatile ENCButton_struct ENC_button_menu;
extern buzzerMainSettings_struct BUZZER_settings;
extern osTimerId My_Timer_BuzzerHandle;

extern WDGCounter task_AlarmControl_WDG;

extern waterTempSettings_struct CAR_waterTemp;
extern oilTempSettings_struct CAR_oilTemp;
extern oilPressureSettings_struct CAR_oilPressure;
extern batterySettings_struct CAR_mainBattery;
extern batterySettings_struct CAR_auxiliaryBattery;
extern fuelSettings_struct CAR_fuel;
extern boardVoltagesSettings_struct BOARD_voltage;
extern boardTemperaturesSettings_struct BOARD_temperature;

extern Diagnostic_ValueTooLowHigh_struct valueStatesLowHigh;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Local variables */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
valueSignal_type carWaterTemp_warning;
valueSignal_type carWaterTemp_alarm;
valueSignal_type carOilTemp_warning;
valueSignal_type carOilTemp_alarm;
valueSignal_type carOilAnalogPressure_alarm;
valueSignal_type carOilBinaryPressure_alarm;
valueSignal_type carMainBattVoltage_alarm;
valueSignal_type carAuxBattVoltage_alarm;
valueSignal_type carFuelLevel_warning;

valueSignal_type board3V3Voltage_alarm;
valueSignal_type board5VVoltage_alarm;
valueSignal_type boardVinVoltage_alarm;
valueSignal_type board3V3Temp_alarm;
valueSignal_type board5VTemp_alarm;
valueSignal_type boardHBridgeTemp_alarm;

static valueSignal_type* const valueSignalPtrTable[] =
{
	&carWaterTemp_warning,
	&carWaterTemp_alarm,
	&carOilTemp_warning,
	&carOilTemp_alarm,
	&carOilAnalogPressure_alarm,
	&carOilBinaryPressure_alarm,
	&carOilBinaryPressure_alarm,
	&carAuxBattVoltage_alarm,
	&carFuelLevel_warning,

	&board3V3Voltage_alarm,
	&board5VVoltage_alarm,
	&boardVinVoltage_alarm,
	&board3V3Temp_alarm,
	&board5VTemp_alarm
};

const uint8_t valueSignalPtrTableSize = sizeof(valueSignalPtrTable)/sizeof(valueSignalPtrTable[0]);

static valueSignal_type* currentValueSignalTypePtr = NULL;
static boolean BuzzerSignalInProgress = FALSE;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Functions prototypes */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline void BuzzerSignal_On(void);
static inline void BuzzerSignal_Off(void);
static void BuzzerSignal_shortOne(void);
static void BuzzerSignal_longOne(void);
static void BuzzerSignal_shortTwo(void);
static void BuzzerSignal_shortFive(void);
static void BuzzerSignal_longTree(void);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



void StartAlarmControlTask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = MY_ALARM_CONTROL_TASK_TIME_PERIOD;

	/* Setting the proper Buzzer Signal to every possible alarm from value */
	carWaterTemp_warning.BuzzerSignal = BuzzerSignal_shortTwo;
	carWaterTemp_alarm.BuzzerSignal = BuzzerSignal_shortFive;
	carOilTemp_warning.BuzzerSignal = BuzzerSignal_shortTwo;
	carOilTemp_alarm.BuzzerSignal = BuzzerSignal_shortFive;
	carOilAnalogPressure_alarm.BuzzerSignal = BuzzerSignal_longTree;
	carOilBinaryPressure_alarm.BuzzerSignal = BuzzerSignal_longTree;
	carMainBattVoltage_alarm.BuzzerSignal = BuzzerSignal_shortFive;
	carAuxBattVoltage_alarm.BuzzerSignal = BuzzerSignal_shortFive;
	carFuelLevel_warning.BuzzerSignal = BuzzerSignal_longOne;

	board3V3Voltage_alarm.BuzzerSignal = BuzzerSignal_longOne;
	board5VVoltage_alarm.BuzzerSignal = BuzzerSignal_longOne;
	boardVinVoltage_alarm.BuzzerSignal = BuzzerSignal_longOne;
	board3V3Temp_alarm.BuzzerSignal = BuzzerSignal_shortTwo;
	board5VTemp_alarm.BuzzerSignal = BuzzerSignal_shortTwo;
	boardHBridgeTemp_alarm.BuzzerSignal = BuzzerSignal_shortTwo;


	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	for (;;)
	{
		carWaterTemp_warning.signalSetting1 = CAR_waterTemp.waterTempWarningOn;
		carWaterTemp_warning.signalSetting2 = CAR_waterTemp.waterTempWarningBuzzerOn;
		carWaterTemp_alarm.signalSetting1 = CAR_waterTemp.waterTempAlarmOn;
		carWaterTemp_alarm.signalSetting2 = CAR_waterTemp.waterTempAlarmBuzzerOn;

		carOilTemp_warning.signalSetting1 = CAR_oilTemp.oilTempWarningOn;
		carOilTemp_warning.signalSetting2 = CAR_oilTemp.oilTempWarningBuzzerOn;
		carOilTemp_alarm.signalSetting1 = CAR_oilTemp.oilTempAlarmOn;
		carOilTemp_alarm.signalSetting2 = CAR_oilTemp.oilTempAlarmBuzzerOn;

		if(valueStatesLowHigh.carOilAnalogPressure_ValueState == val_tooHigh)
		{
			carOilAnalogPressure_alarm.signalSetting1 = CAR_oilPressure.oilHighPressureAlarmOn;
			carOilAnalogPressure_alarm.signalSetting2 = CAR_oilPressure.oilPressureAlarmBuzzerOn;
		}
		else
		{
			carOilAnalogPressure_alarm.signalSetting1 = CAR_oilPressure.oilLowPressureAlarmOn;
			carOilAnalogPressure_alarm.signalSetting2 = CAR_oilPressure.oilPressureAlarmBuzzerOn;
		}
		carOilBinaryPressure_alarm.signalSetting1 = TRUE;	/* Hardcoded because it should be always TRUE */
		carOilBinaryPressure_alarm.signalSetting2 = TRUE;	/* Hardcoded because it should be always TRUE */

		if(valueStatesLowHigh.carMainBattVoltage_ValueState == val_tooHigh)
		{
			carMainBattVoltage_alarm.signalSetting1 = CAR_mainBattery.highVoltageAlarmOn;
			carMainBattVoltage_alarm.signalSetting2 = CAR_mainBattery.highVoltageAlarmBuzzerOn;
		}
		else
		{
			carMainBattVoltage_alarm.signalSetting1 = CAR_mainBattery.lowVoltageAlarmOn;
			carMainBattVoltage_alarm.signalSetting2 = CAR_mainBattery.lowVoltageAlarmBuzzerOn;
		}

		if(valueStatesLowHigh.carAuxBattVoltage_ValueState == val_tooHigh)
		{
			carAuxBattVoltage_alarm.signalSetting1 = CAR_auxiliaryBattery.highVoltageAlarmOn;
			carAuxBattVoltage_alarm.signalSetting2 = CAR_auxiliaryBattery.highVoltageAlarmBuzzerOn;
		}
		else
		{
			carAuxBattVoltage_alarm.signalSetting1 = CAR_auxiliaryBattery.lowVoltageAlarmOn;
			carAuxBattVoltage_alarm.signalSetting2 = CAR_auxiliaryBattery.lowVoltageAlarmBuzzerOn;
		}

		carFuelLevel_warning.signalSetting1 = CAR_fuel.lowFuelLevelWarningOn;
		carFuelLevel_warning.signalSetting2 = CAR_fuel.lowFuelLevelWarningBuzzerOn;

		board3V3Voltage_alarm.signalSetting1 = BOARD_voltage.board3V3SupplyAlarmOn;
		board3V3Voltage_alarm.signalSetting2 = BOARD_voltage.board3V3SupplyBuzzerAlarmOn;
		board5VVoltage_alarm.signalSetting1 = BOARD_voltage.board5VSupplyAlarmOn;
		board5VVoltage_alarm.signalSetting2 = BOARD_voltage.board5VSupplyBuzzerAlarmON;
		boardVinVoltage_alarm.signalSetting1 = BOARD_voltage.boardVinSupplyAlarmOn;
		boardVinVoltage_alarm.signalSetting2 = BOARD_voltage.boardVinSupplyBuzzerAlarmOn;

		board3V3Temp_alarm.signalSetting1 = BOARD_temperature.board3V3DCDCTemperatureHighAlarmOn;
		board3V3Temp_alarm.signalSetting2 = BOARD_temperature.board3V3DCDCTemperatureHighBuzzerAlarmOn;
		board5VTemp_alarm.signalSetting1 = BOARD_temperature.board5VDCDCTemperatureHighAlarmON;
		board5VTemp_alarm.signalSetting2 = BOARD_temperature.board5VDCDCTemperatureHighBuzzerAlarmOn;
		boardHBridgeTemp_alarm.signalSetting1 = BOARD_temperature.boardHBridgeTemperatureHighAlarmOn;
		boardHBridgeTemp_alarm.signalSetting2 = BOARD_temperature.boardHBridgeTemperatureHighBuzzerAlarmOn;

		/* Go through the list of possible alarms and check if there is any */
		for(uint8_t i = 0u; i < valueSignalPtrTableSize; ++i)
		{
			currentValueSignalTypePtr = NULL;
			if(TRUE == valueSignalPtrTable[i]->signalBuzzerIndication) /* If there is an indication alarm */
			{
				if(FALSE == BuzzerSignalInProgress)	/* Check if there is another alarm in progress */
				{
					currentValueSignalTypePtr = valueSignalPtrTable[i];	/* Write an alarm structure pointer to current variable */
					break;
				}
			}
			else /* If there is no indication of alarm then clear the "done" flag - it is used to make the alarm go only once */
			{
				valueSignalPtrTable[i]->signalBuzzerDone = FALSE;	/* Set "done" flag to FALSE */
			}
		}

		/* Check if the buzzer is ON in current alarm signal structure in the 1st setting.
		 * Check if the buzzer is ON in current alarm signal structure in the 2nd setting.
		 * Check if main switch for the buzzer is ON.
		 * Check if main alarm switch for buzzer is ON. */
		if( (currentValueSignalTypePtr ? currentValueSignalTypePtr->signalSetting1 : FALSE) &&
				(currentValueSignalTypePtr ? currentValueSignalTypePtr->signalSetting2 : FALSE) &&
				(BUZZER_settings.buzzerMainSwitchOn) &&
				(BUZZER_settings.buzzerMainAlarmsSwitchOn))
		{
			if(FALSE == currentValueSignalTypePtr->signalBuzzerDone)	/* If there was no alarm so far from this indication */
			{
				/* Execute current signal with Buzzer */
				currentValueSignalTypePtr->BuzzerSignal();
			}
		}

		/* Check if there was a long press of the button detected and if the buzzer for long press is ON or
		 * check if there was a short press of the button detected and if the buzzer for short press in ON. */
		if(((ENC_button_menu.longPressDetectedBuzzer) && (BUZZER_settings.buzzerWhenLognPressOn)) ||
				((ENC_button_menu.shortPressDetectedBuzzer) && (BUZZER_settings.buzzerWhenShortPressOn)))
		{
			/* Check if there is Buzzer Signal already going.
			 * Check if the main switch for the buzzer is ON.
			 * Check if the main switch for the buttons is ON. */
			if((FALSE == BuzzerSignalInProgress) && (TRUE == BUZZER_settings.buzzerMainSwitchOn) && (TRUE == BUZZER_settings.buzzerMainButtonsSwitchOn))
			{
					BuzzerSignal_shortOne();
			}
			ENC_button_menu.allFlags &= 0b11100111;	/* Clear the button alarms bits (long and short press buzzer) */
		}

		++task_AlarmControl_WDG;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
}


/* A function to permanently turn the buzzer ON. */
static inline void BuzzerSignal_On(void)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);	/* Turn the buzzer ON. */
}



/* A function to permanently turn the buzzer OFF. */
static inline void BuzzerSignal_Off(void)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);	/* Turn the buzzer OFF. */
}



/* A function to turn the buzzer ON for a short while (like a beep). */
/* This function is used for button presses only - no signaling of the done signal. */
static void BuzzerSignal_shortOne(void)
{
	BuzzerSignalInProgress = TRUE;	/* Indicate the buzzer is occupied. */
	BuzzerSignal_On();	/* Turn the buzzer ON. */
	(void)osTimerStart(My_Timer_BuzzerHandle, BUZZER_SIGNAL_SHORT_ONE_TIME);	/* Start the timer. */
	BuzzerSignalInProgress = FALSE;	/* Indicate the buzzer is no longer in use */
}



/* A function to turn the buzzer ON for a long while (like a whistle, ~seconds). */
static void BuzzerSignal_longOne(void)
{
	static uint8_t i = 0u;
	BuzzerSignalInProgress = TRUE;	/* Indicate the buzzer is occupied. */

	if(0u == i)
	{
		BuzzerSignal_On();	/* Turn the buzzer ON. */
		(void)osTimerStart(My_Timer_BuzzerHandle, BUZZER_SIGNAL_LONG_ONE_TIME);	/* Start the timer. */
	}

	++i;

	if(4u <= i)
	{
		i = 0u;	/* Set the iterator to 0. */
		BuzzerSignal_Off();	/* Turn the buzzer OFF. */
		currentValueSignalTypePtr->signalBuzzerDone = TRUE;	/* Indicate the signal was already done. */
		BuzzerSignalInProgress = FALSE;	/* Indicate the buzzer is no longer in use */
	}
}



/* A function to turn the buzzer ON for two short times (beeping with pauses). */
static void BuzzerSignal_shortTwo(void)
{
	static uint8_t i = 0u;	/* An iterator through the beeping. */

	BuzzerSignalInProgress = TRUE;	/* Indicate the buzzer is occupied. */

	if((0u == i) || (4u == i))	/* If it is the first time in this function. */
	{
		BuzzerSignal_On();	/* Turn the buzzer ON. */
		(void)osTimerStart(My_Timer_BuzzerHandle, BUZZER_SIGNAL_SHORT_TWO_TIME);	/* Start the timer. */
	}

	++i;	/* Increment the iterator. */

	/* If the number of beeps and pauses is greater than the value:
	 * Set the iterator back to 0u.
	 * Set the "done" flag to TRUE - indicate the alarm was already done. */
	if(6u <= i)
	{
		i = 0u;	/* Set the iterator to 0. */
		BuzzerSignal_Off();	/* Turn the buzzer OFF. */
		currentValueSignalTypePtr->signalBuzzerDone = TRUE;	/* Indicate the signal was already done. */
		BuzzerSignalInProgress = FALSE;	/* Indicate the buzzer is no longer in use */
	}
}



/* A function to turn the buzzer ON for five short times (beeping with pauses). */
static void BuzzerSignal_shortFive(void)
{
	static uint8_t i = 0u;	/* An iterator through the beeping. */

	BuzzerSignalInProgress = TRUE;	/* Indicate the buzzer is occupied. */

	if(0u == (i%4))
	{
		BuzzerSignal_On();	/* Turn the buzzer ON. */
		(void)osTimerStart(My_Timer_BuzzerHandle, BUZZER_SIGNAL_SHORT_FIVE_TIME);	/* Start the timer. */
	}

	++i;	/* Increment the iterator. */

	/* If the number of beeps and pauses is greater than the value:
	 * Set the iterator back to 0u.
	 * Set the "done" flag to TRUE - indicate the alarm was already done. */
	if(12u <= i)
	{
		i = 0u;	/* Set the iterator to 0. */
		BuzzerSignal_Off();	/* Turn the buzzer OFF. */
		currentValueSignalTypePtr->signalBuzzerDone = TRUE;	/* Indicate the signal was already done. */
		BuzzerSignalInProgress = FALSE;	/* Indicate the buzzer is no longer in use */
	}
}



/* A function to turn the buzzer ON for three long times (beeping with pauses). */
static void BuzzerSignal_longTree(void)
{
	static uint8_t i = 0u;	/* An iterator through the beeping. */

	BuzzerSignalInProgress = TRUE;	/* Indicate the buzzer is occupied. */

	if(0u == (i%8))
	{
		BuzzerSignal_On();	/* Turn the buzzer ON. */
		(void)osTimerStart(My_Timer_BuzzerHandle, BUZZER_SIGNAL_LONG_THREE_TIME);	/* Start the timer. */
	}

	++i;	/* Increment the iterator. */

	/* If the number of beeps and pauses is greater than the value:
	 * Set the iterator back to 0u.
	 * Set the "done" flag to TRUE - indicate the alarm was already done. */
	if(32u <= i)
	{
		i = 0u;	/* Set the iterator to 0. */
		BuzzerSignal_Off();	/* Turn the buzzer OFF. */
		currentValueSignalTypePtr->signalBuzzerDone = TRUE;	/* Indicate the signal was already done. */
		BuzzerSignalInProgress = FALSE;	/* Indicate the buzzer is no longer in use */
	}
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Timer_Buzzer(void const * argument)
{
	/* Change the state of the Buzzer (On/Off) after a certain time.
	 * How to use: Firstly turn on the alarm, then start the timer for
	 * a wanted time. You can do that in a loop - will give an interrupted signal
	 */
	BuzzerSignal_Off();	/* Turn the buzzer OFF. */
}
