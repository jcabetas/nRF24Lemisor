/*
 * serialUSB.c
 *
 *  Created on: 29/9/2019
 *      Author: jcabe
 */

#include "ch.hpp"
#include "hal.h"
extern "C" {
  void initSerialUSB(void);
}
using namespace chibios_rt;

#include <stdio.h>
#include <string.h>
#include "portab.h"
#include "usbcfg.h"



void initSerialUSB(void)
{
  /*
   * Target-dependent setup code.
   */
  portab_setup();

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&PORTAB_SDU1);
  sduStart(&PORTAB_SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

//  while (true) {
//    if (PORTAB_SDU1.config->usbp->state == USB_ACTIVE)
//          break;
//    }
}

