/*
 * variables.cpp
 *
 *  Created on: 11 mar 2023
 *      Author: joaquin
 */

#include "ch.hpp"
#include "hal.h"

using namespace chibios_rt;

#include <w25q16/w25q16.h>
#include "variables.h"
#include "flashVars.h"
#include "flash.h"
#include "string.h"

/*
    #define POS_ADDAMANECER     2
    #define POS_ADDATARDECER    4
    #define POS_AUTOPUERTA      6
    #define POS_DSADDPORDIA     8
    #define POS_POSABIERTO     10
    #define POS_POSCERRADO     12
    #define POS_INCR_AD_PORMIL 14
 */

int16_t addAmanecer;
int16_t addAtardecer;
uint16_t autoPuerta;  // 0:cerrada, 1:abierta, 2: automatico, 3: autoConMargen
int16_t dsAddPordia;
uint16_t posAbierto;
uint16_t posCerrado;
int16_t incAdPormil;
uint8_t hayW25q16;

extern "C" {
    uint8_t leeVariablesC(void);
}


/*
 * Si la FLASH es valida, los dos primeros bytes seran 0x7851
 *
 */
void reseteaEEprom(void)
{
    W25Q16_sectorErase(0);
    W25Q16_write_i16(0, POS_ADDAMANECER, 30);
    W25Q16_write_i16(0, POS_ADDATARDECER, -30);
    W25Q16_write_u16(0, POS_AUTOPUERTA, 2);
    W25Q16_write_i16(0, POS_DSADDPORDIA, 0);
    W25Q16_write_u16(0, POS_POSABIERTO, 95);
    W25Q16_write_u16(0, POS_POSCERRADO, 0);
    W25Q16_write_i16(0, POS_INCR_AD_PORMIL,0);
    W25Q16_write_u16(0, 0, 0x7851);
}


void reseteaFlash(void)
{
    borroMemoriaVars();
    int error;
    uC_write_i16(POS_ADDAMANECER, 30, &error);
    uC_write_i16(POS_ADDATARDECER, -30, &error);
    uC_write_u16(POS_AUTOPUERTA, 2, &error);
    uC_write_i16(POS_DSADDPORDIA, 0, &error);
    uC_write_u16(POS_POSABIERTO, 95, &error);
    uC_write_u16(POS_POSCERRADO, 0, &error);
    uC_write_i16(POS_INCR_AD_PORMIL,0, &error);
    uC_write_u16(0, 0x7851, &error);
}


uint8_t leeVariables(void)
{
  hayW25q16 = initW25q16();
  if (hayW25q16)
  {
      uint16_t claveEEprom = W25Q16_read_u16(0, 0);
      if (claveEEprom != 0x7851)
          reseteaEEprom();
      addAmanecer = W25Q16_read_i16(0, POS_ADDAMANECER);
      addAtardecer = W25Q16_read_i16(0, POS_ADDATARDECER);
      autoPuerta = W25Q16_read_u16(0, POS_AUTOPUERTA);
      dsAddPordia = W25Q16_read_i16(0, POS_DSADDPORDIA);
      posAbierto = W25Q16_read_u16(0, POS_POSABIERTO);
      posCerrado = W25Q16_read_u16(0, POS_POSCERRADO);
      incAdPormil = W25Q16_read_i16(0, POS_INCR_AD_PORMIL);
      sleepW25q16();
  }
  else
  {
      uint16_t error = 0;
      uint16_t claveEEprom = uC_read_u16(0, 0, &error);
      if (claveEEprom != 0x7851)
          reseteaFlash();
      addAmanecer = uC_read_i16(POS_ADDAMANECER, 30, &error);
      addAtardecer = uC_read_i16(POS_ADDATARDECER, -30, &error);
      autoPuerta = uC_read_u16(POS_AUTOPUERTA, 2, &error);
      dsAddPordia = uC_read_i16(POS_DSADDPORDIA, 0, &error);
      posAbierto = uC_read_u16(POS_POSABIERTO, 0, &error);
      posCerrado = uC_read_u16(POS_POSCERRADO,100, &error);
      incAdPormil = uC_read_i16(POS_INCR_AD_PORMIL, 0, &error);
  }
  return hayW25q16;
}

void escribeVariables(void)
{
  if (initW25q16()) // hay w25q16 instalado?
  {
      W25Q16_sectorErase(0);
      W25Q16_write_i16(0, POS_ADDAMANECER, addAmanecer);
      W25Q16_write_i16(0, POS_ADDATARDECER, addAtardecer);
      W25Q16_write_u16(0, POS_AUTOPUERTA, autoPuerta);
      W25Q16_write_i16(0, POS_DSADDPORDIA, dsAddPordia);
      W25Q16_write_u16(0, POS_POSABIERTO, posAbierto);
      W25Q16_write_u16(0, POS_POSCERRADO, posCerrado);
      W25Q16_write_i16(0, POS_INCR_AD_PORMIL, incAdPormil);
      W25Q16_write_u16(0, 0, 0x7851);
      sleepW25q16();
  }
  else
  {
      int error = 0;
      uC_write_i16(POS_ADDAMANECER, addAmanecer, &error);
      uC_write_i16(POS_ADDATARDECER, addAtardecer, &error);
      uC_write_u16(POS_AUTOPUERTA, autoPuerta, &error);
      uC_write_i16(POS_DSADDPORDIA, dsAddPordia, &error);
      uC_write_u16(POS_POSABIERTO, posAbierto, &error);
      uC_write_u16(POS_POSCERRADO, posCerrado, &error);
      uC_write_i16(POS_INCR_AD_PORMIL, incAdPormil, &error);
      uC_write_u16(0, 0x7851, &error);
  }
  leeVariables();
}

uint8_t leeVariablesC(void)
{
  return leeVariables();
}


