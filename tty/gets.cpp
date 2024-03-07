/*
 * gets.c
 *
 *  Created on: 26/03/2013
 *      Author: joaquin
 */


#include "ch.hpp"
#include "hal.h"
#include "tty.h"
#include "string.h"
#include <stdlib.h>
#include "chprintf.h"


using namespace chibios_rt;

uint8_t chgetch(BaseChannel *tty)
{
	uint8_t carReceived;
	carReceived = streamGet(tty);
	return carReceived;
}

void chgets(BaseChannel *tty, uint8_t *buffer, uint8_t size)
{
    uint8_t ch,ch1,ch2;
    int pos=0;
    while (pos<size)
      {
      ch = chgetch(tty);
      if (ch=='\n' || ch=='\r')
         {
            streamPut(tty, (uint8_t)ch);
         break;
         }
      if (ch==8 && pos>0)
         {
            streamPut(tty, (uint8_t)ch);
         pos--;
         continue;
         }
      if (ch>=0x20)
         {
        streamPut(tty, (uint8_t)ch);
         buffer[pos++] = ch;
         continue;
         }
      if (ch==0x1b) // escape
         {
         ch1 = chgetch(tty);
         if (ch1 != 0x5b) continue; // no es codigo de direccion
         ch2 = chgetch(tty);
         if (ch2 ==0x44 && pos>0)
            {
             streamPut(tty, 8);
            pos--; //flecha izquierda
            }
         continue;
         }
      }
    buffer[pos]=0;
}

uint8_t chgetchTimeOut(BaseChannel *SD, systime_t timeout, uint8_t *huboTimeout)
{
    msg_t msgReceived;
    *huboTimeout = 0;
    msgReceived = chnGetTimeout(SD, timeout);
    if (msgReceived==Q_TIMEOUT)
    {
      *huboTimeout = 1;
      return 0;
    }
    return (uint8_t) msgReceived;
}

void limpiaBuffer(BaseChannel *pSD)
{
    uint8_t huboTimeout;
    while (1==1)
      {
      chgetchTimeOut(pSD, TIME_MS2I(10), &huboTimeout); // 10 ms entre caracteres
      if (huboTimeout)
         break;
      }
}

//void chgetsNoEchoTimeOut(BaseChannel   *pSD, uint8_t *buffer, uint16_t bufferSize,systime_t timeout, uint8_t *huboTimeout)
//{
//    uint8_t ch;
//    uint16_t pos=0;
//    *huboTimeout = 0;
//    ch = chgetchTimeOut(pSD, timeout, huboTimeout);
//    if (*huboTimeout) // timeout en el primer caracter
//    while (1==1)
//      {
//      ch = chgetchTimeOut(pSD, TIME_MS2I(10), huboTimeout); // 10 ms entre caracteres
//      if (*huboTimeout) // timeout
//         break;
//      if (ch=='\r')
//         continue;
//      if (ch=='\n')
//         break;
//      if (pos<bufferSize)
//        buffer[pos++] = ch;
//      }
//    buffer[pos]=0;
//}
//void chgetsNoEchoTimeOut(BaseChannel   *pSD, uint8_t *buffer, uint16_t bufferSize,systime_t timeout, uint8_t *huboTimeout)
//{
//    uint8_t ch;
//    uint16_t pos=0;
//    *huboTimeout = 0;
//    while (1==1)
//      {
//      ch = chgetchTimeOut(pSD, timeout, huboTimeout);
//      if (*huboTimeout) // timeout
//         break;
//      if (ch=='\r')
//         continue;
//      if (ch=='\n')
//         break;
//      if (pos<bufferSize)
//        buffer[pos++] = ch;
//      }
//    buffer[pos]=0;
//}

void chgetsNoEchoTimeOut(BaseChannel   *tty, uint8_t *buffer, uint16_t size,systime_t timeout, uint8_t *huboTimeout)
{
    uint8_t ch,ch1,ch2;
    int pos=0;
    uint16_t numCarsReceived = 0;
    while (pos<size)
      {
      ch = chgetchTimeOut(tty, timeout, huboTimeout);
      if (*huboTimeout) // timeout
         break;
      if (++numCarsReceived>size)
      {
          *huboTimeout = 1;
          break;
      }
      if (ch=='\n' || ch=='\r')
         {
            streamPut(tty, (uint8_t)ch);
         break;
         }
      if (ch==8 && pos>0)
         {
            streamPut(tty, (uint8_t)ch);
         pos--;
         continue;
         }
      if (ch>=0x20)
         {
        streamPut(tty, (uint8_t)ch);
         buffer[pos++] = ch;
         continue;
         }
      if (ch==0x1b) // escape
         {
         ch1 = chgetch(tty);
         if (ch1 != 0x5b) continue; // no es codigo de direccion
         ch2 = chgetch(tty);
         if (ch2 ==0x44 && pos>0)
            {
             streamPut(tty, 8);
            pos--; //flecha izquierda
            }
         continue;
         }
      }
    buffer[pos]=0;
}


// mensajes desde nextion. Empiezan en @ y terminan en 0
void chgetNextionNoEchoTimeOut(BaseChannel   *pSD, uint8_t *buffer, uint16_t bufferSize,systime_t timeout, uint16_t *numBytes, uint8_t *huboTimeout)
{
    uint8_t ch;
    *huboTimeout = 0;
    *numBytes = 0;
    while (1==1)
      {
      ch = chgetchTimeOut(pSD, timeout, huboTimeout);
      if (*huboTimeout) // timeout
         break;
      if (*numBytes<bufferSize)
        buffer[(*numBytes)++] = ch;
      if (*numBytes>1 && buffer[0]=='@' && ch==0) // si es un mesaje adhoc mio
          break;                                 // no esperes al timeout
      }
}

/**
  * @brief  Convert a string to an integer
  * @param  inputstr: The string to be converted
  * @param  intnum: The intger value
  * @retval 0: Correct
  *         -1: Error
  */

/**
  * @brief  Convert a string to an integer
  * @param  inputstr: The string to be converted
  * @param  intnum: The intger value
  * @retval 0: Correct
  *         -1: Error
  */
int32_t HexStr2Int(uint8_t *inputstr, uint32_t *intnum)
{
    uint32_t i = 0, res = 0;
    uint32_t val = 0, enEspacioInicial;

    enEspacioInicial = 1;
    if (inputstr[0] == '\0')
        return 0;
    for (i = 0; i < 11; i++)
    {
        if (inputstr[i] == '\0')
            {
            *intnum = val;
            /* return 1; */
            res = 0;
            break;
            }
        if (enEspacioInicial && inputstr[i]==' ')
            continue;
        enEspacioInicial = 0;
        if (ISVALIDHEX(inputstr[i]))
        {
            val = (val << 4) + CONVERTHEX(inputstr[i]);
        }
        else
        {
            /* return 0, Invalid input */
            res = -1;
            break;
        }
    }
    /* over 8 digit hex --invalid */
    if (i >= 11)
    {
      res = -2;
    }
    return res;
}


int32_t HexStrN2Int(uint8_t *inputstr, uint32_t numDigits, uint32_t *intnum)
{
    uint32_t i = 0, res = 0;
    uint32_t val = 0, enEspacioInicial;
    if (inputstr[0] == '\0')
        return 0;
    enEspacioInicial = 1;
    for (i = 0; i < numDigits; i++)
    {
        if (inputstr[i] == '\0' || i > numDigits)
            {
            *intnum = val;
            res = 0;
            break;
            }
        if (enEspacioInicial && inputstr[i]==' ')
            continue;
        enEspacioInicial = 0;
        if (ISVALIDHEX(inputstr[i]))
        {
            val = (val << 4) + CONVERTHEX(inputstr[i]);
        }
        else
        {
            /* return 0, Invalid input */
            res = -1;
            break;
        }
    }
    /* over 8 digit hex --invalid */
    if (i >= 11)
    {
        res = -2;
    }
    *intnum = val;
    return res;
}

int16_t Str2Int(uint8_t *inputstr, uint32_t *intnum)
{
    uint32_t i = 0, res = 0;
    uint32_t val = 0, enEspacioInicial;

    enEspacioInicial = 1;
    for (i = 0;i < 11;i++)
    {
        if (inputstr[i] == '\0')
        {
            *intnum = val;
            /* return 1 */
            res = 0;
            break;
        }
        if (enEspacioInicial && inputstr[i]==' ')
            continue;
        enEspacioInicial = 0;
        if (ISVALIDDEC(inputstr[i]))
        {
            val = val * 10 + CONVERTDEC(inputstr[i]);
        }
        else
        {
            /* return 0, Invalid input */
            res = -1;
            break;
        }
    }
    /* Over 10 digit decimal --invalid */
    if (i >= 11)
    {
      res = -2;
    }
    return res;
}


void int2str(uint8_t valor, char *string)
{
    uint8_t i;
    for (i=0;i<8;i++)
    {
        if (valor & (1<<i))
            string[7-i] = '1';
        else
            string[7-i] = '0';
    }
    string[8] = 0;
}

int16_t preguntaNumero(BaseChannel *ttyBC, const char *msg, int16_t *numeroPtr, int16_t valorMin, int16_t valorMax)
{
    uint8_t buffer[50];
    int32_t resultado;
    uint8_t huboTimeout;
    BaseSequentialStream *ttyget = (BaseSequentialStream *)ttyBC;
    chprintf((BaseSequentialStream *)ttyget,msg);
    chprintf((BaseSequentialStream *)ttyget," [%d...%d]:", valorMin, valorMax);
    chgetsNoEchoTimeOut(ttyBC, buffer,sizeof(buffer), TIME_MS2I(20000), &huboTimeout);
    chprintf((BaseSequentialStream *)ttyget,"\r");
    if (huboTimeout==1)
      return 2;
    if (!strncmp("",(char *)buffer,10))     // en blanco, acepto defecto
    {
        return 1;
    }
    resultado = atoi((char *)buffer);
    if (resultado>valorMax)
    {
        chprintf((BaseSequentialStream *)ttyget,"Demasiado alto!!\r");
        return 3;
    }
    if (resultado<valorMin)
    {
        chprintf((BaseSequentialStream *)ttyget,"Demasiado bajo!!\r");
        return 4;
    }
    *numeroPtr = resultado;
    return 0;
}

void parseStr(char *cadena,char **parametros, const char *tokens,uint16_t *numParam)
{
    char *puntOut,*puntStr;
    *numParam=0;
    puntOut = cadena;
    do
    {
        puntStr = strsep(&puntOut,tokens);
        if (puntStr)
        {
            parametros[*numParam] = puntStr;
            (*numParam)++;
        }
    } while (puntStr);
}
