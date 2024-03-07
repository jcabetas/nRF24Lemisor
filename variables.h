/*
 * variables.h
 *
 *  Created on: 06/03/2014
 *      Author: joaquin
 */

#ifndef VARS_H_
#define VARS_H_

#define POS_ADDAMANECER     2
#define POS_ADDATARDECER    4
#define POS_AUTOPUERTA      6
#define POS_DSADDPORDIA     8
#define POS_POSABIERTO     10
#define POS_POSCERRADO     12
#define POS_INCR_AD_PORMIL 14


uint8_t initW25q16(void);
void sleepW25q16(void);
uint8_t leeVariables(void);
void escribeVariables(void);

#endif /* VARS_H_ */
