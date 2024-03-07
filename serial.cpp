/*
 * serial.cpp
 *
 *  Created on: 9 dic. 2022
 *      Author: joaquin
 */

#include "ch.hpp"
#include "hal.h"
using namespace chibios_rt;

#include "tty.h"
#include "gets.h"
#include "string.h"
#include <stdlib.h>
#include "chprintf.h"
#include "calendarUTC.h"
#include <w25q16/w25q16.h>
#include "variables.h"
#include "version.h"
#include "flashVars.h"

extern "C" {
    void initSerial(void);
    void closeSerial(void);
    void opciones(void);
    void printSerial(const char *msg);
}


extern "C" {
    void cierraPuertaC(void);
    void abrePuertaC(void);
}

bool sdPinOpen = false;

static const SerialConfig ser_cfg = {9600, 0, 0, 0, };//{115200, 0, 0, 0, };

void initSerialLPSD1(void) {
    if (sdPinOpen)
        return;
    palClearLine(LINE_TX1);
    palSetLine(LINE_TX1);
    palSetLineMode(LINE_TX1, PAL_MODE_ALTERNATE(4));
    palSetLineMode(LINE_RX1, PAL_MODE_ALTERNATE(4));
    sdStart(&SD1, &ser_cfg);
    sdPinOpen = true;
}

void closeSerialLPSD1(void) {
    palSetLineMode(LINE_TX1, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_RX1, PAL_MODE_INPUT_ANALOG);
    if (!sdPinOpen)
        return;
    sdStop(&SD1);
    sdPinOpen = false;
}

void printSerial(const char *msg)
{
//    palSetLineMode(LINEONSERIAL, PAL_MODE_OUTPUT_PUSHPULL);
//    ACTIVAPAD(LINEONSERIAL);
    initSerialLPSD1();
    chThdSleepMilliseconds(10);
    chprintf((BaseSequentialStream *) ttyPin,msg);
    chThdSleepMilliseconds(20+strlen(msg));
    closeSerialLPSD1();
//    QUITAPAD(LINEONSERIAL);
}
