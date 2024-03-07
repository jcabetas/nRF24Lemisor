#ifndef EEPROM_H
#define EEPROM_H

#include <ch.h>
#include <hal.h>

#define FLASH_PEKEY1 0x89ABCDEF
#define FLASH_PEKEY2 0x02030405
#define EEPROMORG 0x08080000

bool eepromUnLock(void);
bool eepromLock(void);
bool eepromIsLock(void);
int16_t eeprom_read_i16(int numPosicion);
uint16_t eeprom_read_u16(int numPosicion);
void eeprom_write_i16(int numPosicion, int16_t valor, uint16_t *error);
void eeprom_write_u16(int numPosicion, uint16_t valor, uint16_t *error);

#endif /* EEPROM_H */
