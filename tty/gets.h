/*
 * gets.h
 *
 *  Created on: 22/5/2016
 *      Author: joaquin
 */

#ifndef GETS_H_
#define GETS_H_

void chgets(BaseChannel *tty, uint8_t *buffer, uint8_t size);
int32_t HexStr2Int(uint8_t *inputstr, uint32_t *intnum);
int32_t HexStrN2Int(uint8_t *inputstr, uint32_t numDigits, uint32_t *intnum);
int16_t Str2Int(uint8_t *inputstr, uint32_t *intnum);
int32_t leeNumeroMsg(int32_t valorInicial, uint8_t numFilaMsg, uint8_t numFilaInput, char *msg);
uint32_t leeNumero(char *mensaje, uint32_t minValue, uint32_t maxValue);
int16_t preguntaNumero(BaseChannel  *tty, const char *msg, int16_t *numeroPtr, int16_t valorMin, int16_t valorMax);
void int2str(uint8_t valor, char *string);
uint8_t chgetchTimeOut(BaseChannel  *SD, systime_t timeout, uint8_t *huboTimeout);
void limpiaBuffer(BaseChannel *pSD);
void chgetsNoEchoTimeOut(BaseChannel  *pSD, uint8_t *buffer, uint16_t bufferSize,systime_t timeout, uint8_t *huboTimeout);
void chgetNextionNoEchoTimeOut(BaseChannel  *pSD, uint8_t *buffer, uint16_t bufferSize,systime_t timeout, uint16_t *numBytes, uint8_t *huboTimeout);
void parseStr(char *cadena,char **parametros, const char *tokens,uint16_t *numParam);

#endif /* GETS_H_ */
