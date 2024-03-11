#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include "chprintf.h"
#include "ssd1306.h"
#include "stdio.h"
#include <string.h>



static const SSD1306Config ssd1306cfg = {
  &I2CD1,
  SSD1306_SAD_0X78,
};

static SSD1306Driver SSD1306D1;

static void __attribute__((unused)) delayUs(uint32_t val) {
  (void)val;
}

static void __attribute__((unused)) delayMs(uint32_t val) {
  chThdSleepMilliseconds(val);
}


static THD_WORKING_AREA(waOledDisplay, 512);
static __attribute__((noreturn)) THD_FUNCTION(OledDisplay, arg) {
  (void)arg;

  char snum0[15], snum1[20],ionnum[25];
  chRegSetThreadName("OledDisplay");

  ssd1306ObjectInit(&SSD1306D1);
  ssd1306Start(&SSD1306D1, &ssd1306cfg);

  ssd1306FillScreen(&SSD1306D1, 0x00);

  while (TRUE) {
//  ssd1306FillScreen(&SSD1306D1, 0x00);
    ssd1306GotoXy(&SSD1306D1, 0, 1);

    sprintf(snum0, "PA6 13.6");
    ssd1306Puts(&SSD1306D1, snum0, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
    ssd1306UpdateScreen(&SSD1306D1);

    chThdSleepMilliseconds(100);
//  chsnprintf(&snum1[0],sizeof(snum1), "%02f", ad0in);
    sprintf(snum1, "PA7 24.2");

    ssd1306GotoXy(&SSD1306D1, 0, 19);
    ssd1306Puts(&SSD1306D1, snum1, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
    ssd1306UpdateScreen(&SSD1306D1);


    chThdSleepMilliseconds(100);
    sprintf(ionnum, "PA7 23,5, hola");

    ssd1306GotoXy(&SSD1306D1, 0, 39);
    ssd1306Puts(&SSD1306D1, ionnum, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
    ssd1306UpdateScreen(&SSD1306D1);


    chThdSleepMilliseconds(100);
  }

  ssd1306Stop(&SSD1306D1);
}


/*
 * Application entry point.
 */
void testssd1306(void) {
  chThdCreateStatic(waOledDisplay, sizeof(waOledDisplay), NORMALPRIO, OledDisplay, NULL);
  chThdSleepMilliseconds(50000);
}

