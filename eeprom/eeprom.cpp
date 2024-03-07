// https://github.com/tegesoft/flash-stm32f407/blob/master/flash/flash.c

#include "eeprom.h"

#include <ch.h>
#include <hal.h>
#include <stdint.h>
#include <string.h>
#include "eeprom.h"



bool eepromUnLock(void)
{
    //Unlocking the data EEPROM and FLASH_PECR register code example
    /* (1) Wait till no operation is on going */
    /* (2) Check if the PELOCK is unlocked */
    /* (3) Perform unlock sequence */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {
        /* For robust implementation, add here time-out management */
        }
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) {
        FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
        FLASH->PEKEYR = FLASH_PEKEY2;
    }
    return ((FLASH->PECR & FLASH_PECR_PELOCK) == 0);
}


bool eepromLock(void)
{
    /* (1) Wait till no operation is on going */
    /* (2) Locks the NVM by setting PELOCK in PECR */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
    {
        /* For robust implementation, add here time-out management */
    }
    FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
    return ((FLASH->PECR & FLASH_PECR_PELOCK) == 1);
}


bool eepromIsLock(void)
{
    return ((FLASH->PECR & FLASH_PECR_PELOCK) == 1);
}


int16_t eeprom_read_i16(int numPosicion)
{
    int16_t *direccion;
    direccion = (int16_t *) EEPROMORG + numPosicion;
    return (int16_t) *direccion;
}

uint16_t eeprom_read_u16(int numPosicion)
{
    uint16_t *direccion;
    direccion = (uint16_t *) EEPROMORG + numPosicion;
    return (uint16_t) *direccion;
}

void eeprom_write_i16(int numPosicion, int16_t valor, uint16_t *error)
{
    int16_t *direccion;
    direccion = (int16_t *) EEPROMORG + numPosicion;
    *direccion = valor;
    if (*direccion != valor)
        *error = 1;
}

void eeprom_write_u16(int numPosicion, uint16_t valor, uint16_t *error)
{
    uint16_t *direccion;
    direccion = (uint16_t *) EEPROMORG + numPosicion;
    *direccion = valor;
    if (*direccion != valor)
        *error = 1;
}
