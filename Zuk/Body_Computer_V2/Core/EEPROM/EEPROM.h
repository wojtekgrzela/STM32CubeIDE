/*
 * EEPROM.h
 *
 *  Created on: May 6, 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_


#include "main.h"
#include "defines.h"
#include "../ErrorCodes/ErrorCodes.h"


#define DATA_READY		(TRUE)
#define DATA_NOT_READY	(FALSE)
#define DATA_COPIED		(TRUE)
#define DATA_NOT_COPIED	(FALSE)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	I2C_HandleTypeDef *EEPROM_hi2c;
	uint16_t address;
	uint16_t pin;
	GPIO_TypeDef *port;
	boolean isLocked;
} EEPROM_parameters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint16_t memAddressSize;
	uint16_t memAddress;
	uint16_t size;
	boolean isReady;
	boolean dataIsCopied;
	boolean *isReadyPtr;
	boolean *dataIsCopiedPtr;
	uint8_t *data;
	EEPROM_parameters_struct *EEPROMParameters;
} EEPROM_data_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Unions for read data conversions (variables, structures, communicates) */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef union data32bit_union
{
	uint8_t u8bit[4];
	uint16_t ud16bit[2];
	uint32_t u32bit;

	int8_t i8bit[4];
	int16_t i16bit[2];
	int32_t i32bit;

	boolean bool8bit[4];
	float f32bit;

	carTemperature_type carTemperature;
	carOilAnalogPressure_type carAnalogPressure;
	cafFuelLevel_type carFuelLevel;
	carVoltage_type carVoltage;

	boardVoltage_type boardVoltage;
	boardTemperature_type boardTemperature;

	LCDSettings_type LCDSettings;
	LCDBacklightSettings_type LCDBacklightSettings;
	timeHours_type timeHours;
	Enum_Layer screenLayer;
} data32bit_union;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*** A structure with all the info for diagnostic snapshot ***/
typedef struct
{
	union
	{
		uint8_t data[MAX_DIAGNOSTIC_SNAPSHOT_SIZE];
		struct
		{
			Diagnostic_Snapshot_struct diag_mess_from_queue;	//8 bytes total
			uint8_t clockTime[8/*Size of clock time from GPS*/];//16 bytes total
			uint8_t latitude[10];								//26 bytes total
			uint8_t latitudeIndicator;							//27 bytes total
			uint8_t longitude[11];								//38 bytes total
			uint8_t longitudeIndicator;							//39 bytes total
		};
	};
	EEPROM_data_struct DiagnosticDataForEEPROM;
}DiagnosticDataToEEPROM_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*** A structure with all the info for error snapshot ***/
typedef struct
{
	union
	{
		uint8_t data[10];
		struct
		{
			Error_Code error_mess_from_queue;					//4 bytes total
			uint8_t clockTime[8/*Size of raw time from GPS*/];	//12 bytes total (2 unused) = 10 bytes of data
		};
	};
	EEPROM_data_struct ErrorDataForEEPROM;
}ErrorDataToEEPROM_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define CREATE_EEPROM_data_struct(X) EEPROM_data_struct X = {0};\
									X.isReadyPtr = &(X.isReady);\
									X.dataIsCopiedPtr = &(X.dataIsCopied);\
									*(X.isReadyPtr) = DATA_NOT_READY;\
									*(X.dataIsCopiedPtr) = DATA_NOT_COPIED;\
									X.EEPROMParameters = NULL;

#define INITIALIZE_EEPROM_data_struct(X) X.isReadyPtr = &(X.isReady);\
									X.dataIsCopiedPtr = &(X.dataIsCopied);\
									*(X.isReadyPtr) = DATA_NOT_READY;\
									*(X.dataIsCopiedPtr) = DATA_NOT_COPIED;

Error_Code LockEEPROM(EEPROM_parameters_struct *EEPROM);
Error_Code UnlockEEPROM(EEPROM_parameters_struct *EEPROM);

Error_Code ReadEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);
Error_Code WriteEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);


#endif /* INC_EEPROM_H_ */
