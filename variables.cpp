/*
 * variables.cpp
 *
 *  Created on: 11 mar 2023
 *      Author: joaquin
 */

#include "ch.hpp"
#include "hal.h"

using namespace chibios_rt;

#include "variables.h"
#include "eeprom.h"
#include "string.h"

/*
    #define POS_POSABIERTO     10
    #define POS_POSCERRADO     12
    #define POS_INCR_AD_PORMIL 14
 */

uint16_t posAbierto;
uint16_t posCerrado;
int16_t incAdPormil;

extern "C" {
    void leeVariablesC(void);
}

/*
 * Si la EEPROM es valida, los dos primeros bytes seran 0x7851
 *
 */

uint16_t reseteaEeprom(void)
{
    uint16_t error = 0;
    eepromUnLock();
    eeprom_write_u16(POS_POSABIERTO, 95, &error);
    eeprom_write_u16(POS_POSCERRADO, 0, &error);
    eeprom_write_i16(POS_INCR_AD_PORMIL,0, &error);
    eeprom_write_u16(0, 0x7851, &error);
    eepromLock();
    return error;
}


void leeVariables(void)
{
    uint16_t claveEEprom = eeprom_read_u16(0);
    if (claveEEprom != 0x7851)
        reseteaEeprom();
    posAbierto = eeprom_read_u16(POS_POSABIERTO);
    posCerrado = eeprom_read_u16(POS_POSCERRADO);
    incAdPormil = eeprom_read_i16(POS_INCR_AD_PORMIL);
}

uint16_t escribeVariables(void)
{
    uint16_t error = 0;
    eepromUnLock();
    eeprom_write_u16(POS_POSABIERTO, posAbierto, &error);
    eeprom_write_u16(POS_POSCERRADO, posCerrado, &error);
    eeprom_write_i16(POS_INCR_AD_PORMIL,incAdPormil, &error);
    eeprom_write_u16(0, 0x7851, &error);
    eepromLock();
    return error;
}

void leeVariablesC(void)
{
  leeVariables();
}
