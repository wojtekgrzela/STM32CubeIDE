/*
 * Init_Functions.h
 *
 *  Created on: May 16, 2020
 *      Author: Dell
 */

#ifndef INC_INIT_FUNCTIONS_H_
#define INC_INIT_FUNCTIONS_H_

#include "../ErrorCodes/ErrorCodes.h"
#include "../EEPROM/EEPROM.h"
#include "main.h"


extern EEPROM_parameters_struct EEPROM_car;
extern CAR_EEPROM_counters_struct CAR_EEPROM_counters;
extern CAR_mileage_struct CAR_mileage;
extern waterTempSettings_struct CAR_waterTemp;
extern oilTempSettings_struct CAR_oilTemp;
extern batterySettings_struct CAR_mainBattery;
extern batterySettings_struct CAR_auxiliaryBattery;
extern fuelSettings_struct CAR_fuel;
extern oilPressureSettings_struct CAR_oilPressure;

extern EEPROM_parameters_struct EEPROM_board;


Error_Code EraseWholeEEPROM(EEPROM_parameters_struct * EEPROMParameters);
Error_Code InitVariablesFromEEPROMCar(void);


#endif /* INC_INIT_FUNCTIONS_H_ */
