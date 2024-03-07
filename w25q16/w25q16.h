/*
  W25Q16.h - Arduino library for communicating with the Winbond W25Q16 Serial Flash.
  Created by Derek Evans, July 17, 2016.
*/

#ifndef W25Q16_h
#define W25Q16_h

#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define PAGE_PROGRAM 0x02
#define READ_STATUS_REGISTER_1 0x05
#define READ_DATA 0x03
#define SECTOR_ERASE 0x20
#define CHIP_ERASE 0xC7
#define POWER_DOWN 0xB9
#define RELEASE_POWER_DOWN 0xAB
#define MANUFACTURER_ID 0x90

uint8_t  W25Q16_init(void);
void W25Q16_write(uint16_t page, uint8_t pageAddress, uint8_t val);
void W25Q16_write_u16(uint16_t page, uint8_t pageAddress, uint16_t val);
void W25Q16_write_i16(uint16_t page, uint8_t pageAddress, int16_t val);
uint8_t W25Q16_read(uint16_t page, uint8_t pageAddress);
uint16_t W25Q16_read_u16(uint16_t page, uint8_t pageAddress);
int16_t W25Q16_read_i16(uint16_t page, uint8_t pageAddress) ;
void W25Q16_initStreamWrite(uint16_t page, uint8_t pageAddress);
void W25Q16_streamWrite(uint16_t *page, uint8_t *pageAddress, uint8_t val);
void W25Q16_closeStreamWrite(void);
void W25Q16_initStreamRead(uint16_t page, uint8_t pageAddress);
uint8_t W25Q16_streamRead(uint16_t *page, uint8_t *pageAddress);
void W25Q16_closeStreamRead(void);
void W25Q16_powerDown(void);
void W25Q16_releasePowerDown(void);
void W25Q16_chipErase(void);
void W25Q16_sectorErase(uint16_t page);
uint16_t W25Q16_manufacturerID(void);


void W25Q16_writeEnable(void);
void W25Q16_writeDisable(void);
void W25Q16_notBusy(void);


#define w25q16_PAGESIZE 0xFF
#define w25q16_NUMPAGES 0x2000

//int16_t leeVariableU8(uint16_t posParam, const char *nombParam, uint8_t valorMin, uint8_t valorMax, uint8_t *valor);
//int16_t escribeVarU8(uint16_t posParam, const char *nombVar, uint8_t valor);
//int16_t leeVariable8(uint16_t posParam, const char *nombParam, int8_t valorMin, int8_t valorMax, int8_t *valor);
//int16_t escribeVar8(uint16_t posParam, const char *nombVar, int8_t valor);
//int16_t leeVariableU16(uint16_t posParam, const char *nombParam, uint16_t valorMin, uint16_t valorMax, uint16_t *valor);
//int16_t escribeVarU16(uint16_t posParam, const char *nombVar, uint16_t valor);


#endif /* W25Q16_h */
