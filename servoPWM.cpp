/*
 * pwmInts.c
 *
 *  Created on: 03/06/2012, adaptado 15/11/2015 a Flip32,
 *  adaptado y muy simplificado para BlackPill 20/9/2022
 *      Author: joaquin
 *  Salida servo 1  (PWM1, PA1, TIM2_CH2, AF02)
 */

#include "hal.h"
#include "chprintf.h"

extern "C" {
    void mueveServoPosC(uint16_t porcPosicion);
    void cierraPuertaC(void);
    void abrePuertaC(void);
    uint8_t abrePuertaHastaA1C(void);
}
void printSerialCPP(const char *msg);

extern uint16_t posAbierto;
extern uint16_t posCerrado;
uint16_t porcPosAnterior = 50;

static PWMConfig pwmcfg = {
  3200000, /* 48 MHz PWM clock frequency antes 3125000 */
    64000,   /* PWM period 20 millisecond antes 62500 */
  NULL,  /* No callback */
  /* Channel 2 enabled */
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
  },
  0,
  0,
  0
};

bool tensionCritica(void);


void initServo(void)
{
/*
 *   Salida servo: TIM2CH2 (PA1)
 */
    palSetLineMode(LINE_ONSERVO, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(LINE_PWM, PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_HIGHEST);
    palSetLine(LINE_ONSERVO);
    pwmStart(&PWMD2, &pwmcfg);
}

void closeServo(void)
{
    palSetLineMode(LINE_PWM,PAL_MODE_INPUT_ANALOG );
    palClearLine(LINE_ONSERVO);
    pwmStop(&PWMD2);
}

void mueveServoAncho(uint16_t ancho, uint16_t ms)
{
  // minimo 3200, maximo 6400, medio 4800
  if (ancho<2900) ancho=2900;
  if (ancho>6700) ancho=6700;
  pwmEnableChannel(&PWMD2, 1, ancho);
  chThdSleepMilliseconds(ms);
}

void mueveServoPos(uint16_t porcPosicion)
{
  uint16_t ancho;
  // arranco servo
  initServo();
  palSetLine(LINE_ONSERVO);
  if (PWMD2.state==PWM_STOP)
      pwmStart(&PWMD2, &pwmcfg);
  if (porcPosicion>100) porcPosicion=100;
  // si esta en la posicion, recordar durante 1500ms
  if (porcPosicion==porcPosAnterior)
  {
      ancho = (uint16_t) (2700.0f+3800.0f*porcPosicion/100.0f);
      mueveServoAncho(ancho, 1500);
      closeServo();
      return;
  }
  if (porcPosicion>porcPosAnterior)
  {
      for (int16_t pos=porcPosAnterior;pos<=porcPosicion;pos++)
      {
          ancho = (uint16_t) (2900.0f+3800.0f*pos/100.0f);
          mueveServoAncho(ancho, 15);
      }
      mueveServoAncho(ancho, 500);
      porcPosAnterior = porcPosicion;
      closeServo();
      return;

  }
  if (porcPosicion<porcPosAnterior)
  {
      for (int16_t pos=porcPosAnterior;pos>=porcPosicion;pos--)
      {
          ancho = (uint16_t) (2700.0f+3800.0f*pos/100.0f);
          mueveServoAncho(ancho, 15);
      }
      mueveServoAncho(ancho, 500);
      porcPosAnterior = porcPosicion;
      closeServo();
      return;
  }
}

void mueveServoPosC(uint16_t porcPosicion)
{
    mueveServoPos(porcPosicion);
}

void cierraPuertaC(void)
{
    if (tensionCritica())
        return;
    if (posCerrado==porcPosAnterior)
    {
        // recuerda posicion
        mueveServoPos(posCerrado);
        return;
    }
    uint16_t posIntermedia = posAbierto + ((posCerrado - posAbierto)>>2);
    mueveServoPos(posIntermedia);
    chThdSleepMilliseconds(1500);
    mueveServoPos(posCerrado);
}

void abrePuertaC(void)
{
    mueveServoPos(posAbierto);
}

void cierraPuerta(void)
{
    cierraPuertaC();
}


uint8_t abrePuertaHastaA1C(void)
{
    printSerialCPP("Cerrado sensor\n\r");
    chThdSleepMilliseconds(50);  // compruebo que no es esporadico (p.e. vibracion en sensor)
    mueveServoPos(posAbierto);
    return 1;
}
