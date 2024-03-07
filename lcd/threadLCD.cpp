/**
 * @file stm32/LCD/threadLCD.c
 * @brief proceso LCD
 * @addtogroup LCD
 * @{
 */

#include "ch.hpp"
#include "hal.h"

extern "C" {
  void initLCD(void);
}
#include "string.h"
#include "chprintf.h"
#include "../w25q16/varsFlash.h"
#include "lcd.h"
//#include "colas.h"

using namespace chibios_rt;

//extern struct queu_t colaMsgLcd;

//static const I2CConfig i2ccfg = {
//  OPMODE_I2C,
//  400000,
//  FAST_DUTY_CYCLE_2,
//};

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  125000,
  FAST_DUTY_CYCLE_2,
};

void initLCD(void)
{
//    initColaMsgLcd();
//    palSetPadMode(GPIOB, GPIOB_I2C1SCL,PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);    /* I2C1SCL*/
//    palSetPadMode(GPIOB, GPIOB_I2C1SDA,PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);    /* I2C1SDA*/
    i2cStart(&I2CD1, &i2ccfg); // LCD
    lcd_I2Cinit();
    lcd_CustomChars();
    lcd_backlight();
    lcd_clear();
}


/** @} */

