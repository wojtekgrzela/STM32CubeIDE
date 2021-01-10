/*
 * EEPROM.h
 *
 *  Created on: May 6, 2020
 *      Author: Wojciech Grzelinski
 */


#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_


#include "main.h"
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
	uint8_t	isLocked :1;
}EEPROM_parameters_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct
{
	uint16_t memAddressSize;
	uint16_t memAddress;
	uint16_t size;
	boolean isReady;
	boolean dataIsCopied;
	boolean* isReadyPtr;
	boolean* dataIsCopiedPtr;
	uint8_t* data;
	EEPROM_parameters_struct *EEPROMParameters;
}EEPROM_data_struct;
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

	float f32bit;
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
			uint8_t rawTime[6/*Size of raw time from GPS*/];	//16 bytes total (2 unused)
			uint8_t latitude[10];								//24 bytes total
			uint8_t latitudeIndicator;							//28 bytes total (3 unused)
			uint8_t longitude[11];								//36 bytes total
			uint8_t longitudeIndicator;							//40 bytes total (3 unused) = 37 bytes of data
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
			uint8_t rawTime[6/*Size of raw time from GPS*/];	//12 bytes total (2 unused) = 10 bytes of data
		};
	};
	EEPROM_data_struct ErrorDataForEEPROM;
}ErrorDataToEEPROM_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define CREATE_EEPROM_data_struct(X) EEPROM_data_struct X = {0};\
									X.isReadyPtr = &(X.isReady);\
									X.dataIsCopiedPtr = &(X.dataIsCopied);\
									*(X.isReadyPtr) = DATA_NOT_READY;\
									*(X.dataIsCopiedPtr) = DATA_NOT_COPIED;

#define INITIALIZE_EEPROM_data_struct(X) X.isReadyPtr = &(X.isReady);\
									X.dataIsCopiedPtr = &(X.dataIsCopied);\
									*(X.isReadyPtr) = DATA_NOT_READY;\
									*(X.dataIsCopiedPtr) = DATA_NOT_COPIED;

Error_Code LockEEPROM(EEPROM_parameters_struct *const EEPROM);
Error_Code UnlockEEPROM(EEPROM_parameters_struct *const EEPROM);

Error_Code ReadEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);
Error_Code WriteEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);


#endif /* INC_EEPROM_H_ */
