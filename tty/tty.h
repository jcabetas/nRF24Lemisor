/*
 * tty.h
 *
 *  Created on: 26/03/2013
 *      Author: joaquin
 */

#ifndef TTY_H_
#define TTY_H_

#include <stdarg.h>
//
//int chprintf(const char *format, ...);
int chsprintf(char *out, const char *format, ...);
//int chsnprintf( char *buf, unsigned int count, const char *format, ... );
int choutbyte(int c);
uint8_t chgetch(BaseChannel  *tty);
//uint8_t *chgets(BaseChannel  *tty);

#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

int32_t HexStr2Int(uint8_t *inputstr, uint32_t *intnum);
int32_t HexStrN2Int(uint8_t *inputstr, uint32_t numDigits, uint32_t *intnum);
int16_t Str2Int(uint8_t *inputstr, uint32_t *intnum);

#endif /* TTY_H_ */

