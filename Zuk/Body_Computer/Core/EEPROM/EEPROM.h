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
	uint8_t isReady :1;
	uint8_t *data;
	EEPROM_parameters_struct *EEPROMParameters;
}EEPROM_data_struct;
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


Error_Code LockEEPROM(EEPROM_parameters_struct *const EEPROM);
Error_Code UnlockEEPROM(EEPROM_parameters_struct *const EEPROM);

Error_Code ReadEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);
Error_Code WriteEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data);


#endif /* INC_EEPROM_H_ */
