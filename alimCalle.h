/*
 * alimCalle.h
 *
 *  Created on: 20 sept. 2022
 *      Author: joaquin
 */

#ifndef ALIMCALLE_H_
#define ALIMCALLE_H_

void initServo(void);
void mueveServoAncho(uint16_t ancho);
void mueveServoPos(uint8_t porcPosicion);
uint8_t sleep_for_x_sec(uint32_t nb_sec);
uint8_t stop(uint16_t nb_sec);
uint8_t standby0(uint16_t nb_sec);
void ports_set_lowpower(void);
void ports_set_normal(void);
void initW25q16(void);
void leeHora(void);
void estadoDeseadoPuerta(uint8_t *estDes, uint16_t *sec2change);

void initSerial(void);
void leeGPS(void);
void estadoDeseadoPuertaC(uint8_t *estDes, uint32_t *sec2change);
uint8_t leeVariablesC(void);
void printFechaC(char *buff, uint16_t longBuff);
void addDsC(int16_t dsAdd);
void printSerial(char *msg);
void cierraPuertaC(void);
void abrePuertaC(void);
void opciones(void);
void abrePuertaHastaA1C(void);
void calibraConGPS(void);
void ajustaCALMP_C(int16_t dsDia);

#endif /* ALIMCALLE_H_ */
