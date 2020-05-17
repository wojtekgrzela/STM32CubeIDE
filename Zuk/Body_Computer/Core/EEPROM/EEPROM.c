/*
 * EEPROM.c
 *
 *  Created on: May 6, 2020
 *      Author: Wojciech Grzelinski
 */


#include "EEPROM.h"



/**
 * A function that sets the Write Protect Pin HIGH for the EEPROM
 * blocking the writing to it.
 *
 * @param EEPROM: a pointer to a structure with EEPROM parameters
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code LockEEPROM(EEPROM_parameters_struct *const EEPROM)
{
	Error_Code error = NO_ERROR;

	HAL_GPIO_WritePin(EEPROM->port, EEPROM->pin, SET);

	EEPROM->isLocked = TRUE;

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that sets the Write Protect Pin LOW for the EEPROM
 * unlocking the writing to it.
 *
 * @param EEPROM: a pointer to a structure with EEPROM parameters
 * 			(look in EEPROM.h)
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code UnlockEEPROM(EEPROM_parameters_struct *const EEPROM)
{
	Error_Code error = NO_ERROR;
	HAL_GPIO_WritePin(EEPROM->port, EEPROM->pin, RESET);

	EEPROM->isLocked = FALSE;

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that reads from EEPROM. The number of bytes, address and other
 * parameters are given through pointers to structures with parameters. It
 * is possible to read data only through an EEPROM_data_struct type.
 *
 * @param EEPROM: a pointer to a structure with EEPROM parameters
 * 			(look in EEPROM.h)
 * @param data: a pointer to a structure with memory address, size and buffer
 * 			(look in EEPROM.h)
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code ReadEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data)
{
	Error_Code error = NO_ERROR;
	error = HAL_I2C_Mem_Read_IT(EEPROM->EEPROM_hi2c, ((EEPROM->address)|(0x0001))
			/*"OR" to set the last bit to 1 (in order to read)*/,
			data->memAddress, data->memAddressSize, data->data, data->size);

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/**
 * A function that writes to EEPROM. The number of bytes, address and other
 * parameters as well as data to write are given through pointers to
 * structures with parameters. It is possible to write data only through
 * an EEPROM_data_struct type.
 *
 * @param temperature: a pointer to the temperature it will calculate
 * @param ADC_value: a value from ADC
 * @param NTC: a pointer to the NTC structure with NTC parameters
 * @retval Error_Code: gives a value of error if one occurs
 */
Error_Code WriteEEPROM(const EEPROM_parameters_struct *const EEPROM, EEPROM_data_struct *const data)
{
	Error_Code error = NO_ERROR;

	if(FALSE == EEPROM->isLocked)
	{
		error = HAL_I2C_Mem_Write_IT(EEPROM->EEPROM_hi2c, ((EEPROM->address)&(0xFFFE))
				/*"AND" to set the last bit to 0 (in order to write)*/,
				data->memAddress, data->memAddressSize, data->data, data->size);
	}
	else
	{
		error = EEPROM__TRIED_TO_WRITE_WHILE_LOCKED;
	}

	return error;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

