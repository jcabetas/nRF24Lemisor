/*
 * volcarFlash.cpp
 *
 *  Created on: 26 jul. 2020
 *      Author: jcabe
 */


/*
 * testFlash.c
 *
 *  Created on: 12/9/2019
 *      Author: joaquin
 */


#include "w25q16Vars.h"

#include "ch.hpp"
#include "hal.h"
#include "varsFlash.h"

using namespace chibios_rt;

#include "w25q16.h"
#include "chprintf.h"
//#include "tty.h"

/*
 * Formato opcional:
 * - Cada sector (16 paginas) tiene una sola variable. Se indexa con la "posicion"
 *
 * Cada versi�n PLC
 * - Los primeros dos sectores guardan dos versiones PLC
 * - Comienzan con una variable uint16_t con valor 12345
 * - Otra variable uint16_t con longitud de datos
 * - Los datos empiezan en posici�n 4 (0-base)
 *
 * Cada sector siguiente:
 * - Tipo de variable: 0/FF (no ocupada) 1 (No/Si)  2 (No/Si/Auto) 3 (Baud) 4 (ModoPozo)
 *                                       5 (uint8) 6 (int8) 7 (uint16) 8 (int16) 9 (int32)
 *                                      10 (str50)
 * - Checksum (sum) del nombre uint16_t
 * - Nombre de variable (str16)
 * - 1 byte: posvalida 'J' o no (0x00)
 * - N bytes con el valor
 *   Si no es valido, continuar leyendo hasta 100 veces.
 *   A partir de ahi hay que borrar el sector entero y volver a grabar
 *
 * - La ventaja es que puedo ver todas las variables y comprobar valores
 */

//uint8_t longParam[] = {1,1,1,1,1,1,2,2,4,8,50};

char defTipoVar[][11]={"0 (error)","No/Si","No/si/Auto","Baud","ModoPozo","uint8","int8","uint16_t","int16","int32","str50"};

uint8_t printPlcDeFlash(BaseSequentialStream *tty, uint16_t sector)
{
  uint16_t varCheck, page, longPLC, i;
  uint8_t  pageAddress, byte;
  if (sector>1)
  {
      chprintf(tty,"Error: sector %d no puede tener datos PLC\n\r", sector);
      return 1;
  }
  page = 16*sector;
  varCheck = W25Q16_read_u16(page, 0);
  if (varCheck==0xFFFF)
  {
      chprintf(tty,"Sector PLC #%d sin grabar\n\r", sector);
      return 0;
  }
  if (varCheck!=0x1234)
  {
      chprintf(tty,"Error: sector PLC #%d no es valida\n\r", sector);
      return 3;
  }
  longPLC = W25Q16_read_u16(page, 2);
  chprintf(tty,"PLC#%d.  #bytes:%d\n\r", sector, longPLC);
  if (longPLC==0)
      return 0;
  pageAddress = 4;
  W25Q16_initStreamRead(page, pageAddress);
  i = 0;
  while (i<longPLC)
  {
      byte = W25Q16_streamRead(&page, &pageAddress);
      if (byte==0)
          chprintf(tty,"\n\r");
      else
          chprintf(tty,"%c",byte);
      i++;
  };
  W25Q16_closeStreamRead();
  chprintf(tty,"\n\r",byte);
  return 0;
}

uint8_t printVar(BaseSequentialStream *tty, uint16_t sector)
{
    uint8_t i, byte, pageAddress, nombVar[LONGNAME], error, pageAddressByteFat, byteFat;
    uint16_t page, chkSum, chkSumStr, tipoVariable, posEnFAT, pageByteFat;
    if (sector<SECTORINICIALVARS)
    {
        chprintf(tty,"Error: sector %d reservado a PLC\n\r", sector);
        return 1;
    }
    page = 16*sector;
    tipoVariable = W25Q16_read(page, 0);
    if (tipoVariable==0xFF)
    {
      return 2;
    }
    chkSum = W25Q16_read_u16(page, 1);
    posEnFAT = leePosEnFAT(sector, &error, &pageByteFat, &pageAddressByteFat, &byteFat);
    chprintf(tty,"sector: %d, tipo: %d (%s), chkSum: %X, pos: %d", sector,tipoVariable,defTipoVar[tipoVariable], chkSum, posEnFAT);
    chkSumStr = 0;
    // lee nombre de la variable
    pageAddress = 3;
    W25Q16_initStreamRead(page, pageAddress);
    i = 0;
    do
    {
        byte = W25Q16_streamRead(&page, &pageAddress);
        nombVar[i] = byte;
        if (byte==0)
            break;
        chkSumStr += byte;
        if (++i==(LONGNAME-1))
        {
            nombVar[i] = 0;
            break;
        }
    } while (TRUE);
    W25Q16_closeStreamRead();
    chprintf(tty,", nombre:%s\n\r",nombVar);
    if (chkSumStr!=chkSum)
    {
        chprintf(tty,"\n\rNo coincide checkSum de flash (%4x) con el del nombre /%4X)\n\r",chkSumStr,chkSum);
        return 3;
    }
    return 0;
}

void volcarFlash(BaseSequentialStream *tty)
{
    for (uint8_t sectorPlc=0;sectorPlc<=1;sectorPlc++)
        printPlcDeFlash(tty, sectorPlc);
    for (uint16_t sectorVar=SECTORINICIALVARS;sectorVar<512;sectorVar++)
        printVar(tty, sectorVar);
}

void dumpFlash(BaseSequentialStream *tty, uint16_t page, uint8_t pageAddress, uint16_t longitud)
{
  uint8_t byte;
  uint16_t i, contFila;
  uint32_t direccion;
  chprintf(tty,"\n\rVolcado pagina %d, pageStart:%d, long:%d\n\r", page, pageAddress,longitud);
  contFila = 999;
  W25Q16_initStreamRead(page, pageAddress);
  for (i=0;i<longitud;i++)
  {
    direccion = (page<<8)+pageAddress;
    if (contFila>15)
    {
      chprintf(tty,"\n\r%6X: ",direccion);
      contFila = 0;
    }
    byte = W25Q16_streamRead(&page, &pageAddress);
    chprintf(tty, "%2X",byte);
    if (byte>=0x20 && byte<=0x7F)
        chprintf(tty, "%c",byte);
    else
        chprintf(tty, " ");
    contFila++;
  }
  W25Q16_closeStreamRead();
  chprintf(tty, "\n\r");
}



