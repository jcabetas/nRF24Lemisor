/*
 ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */
/*
    PLAY Embedded demos - Copyright (C) 2014...2019 Rocco Marco Guglielmi

    This file is part of PLAY Embedded demos.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * Funcionalidad:
 * - El receptor se queda esperando a ordenes del emisor
 * - El emisor periodicamente pide estado al receptor
 *   * el receptor envía estado del servo, bateria y nivel de señal
 *   * el emisor enseña datos propios y del receptor en display
 * - Cuando se cambia palanca, se envia orden de cierre
 *   * el receptor activa el servo y envia nuevo estado al emisor
 *
 * Necesario:
 * - Salida I2C display
 * - Entrada palanca
 * - Puerto serie, para programar (emisor/receptor, clave, posAbierto, posCerrado)
 * - Salida PWM
 */

#include "ch.h"
#include "hal.h"

#include "rf.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

#define  TRANSMITTER

#define  FRAME_LEN   5

void leeVariablesC(void);
void leeTension(float *vBat);
void testssd1306(void);
void ssd1306Init(void);
void printfFilaSSD1306(uint8_t fila,char *msg);

uint8_t estadoPuerta;  // abierto:0 cerrado:1 desconocido:2
float vBatJaula;
uint8_t porcBatJaula;
bool commOk = false;

float vBat, porcBat;
uint16_t paqSent = 0;
uint16_t paqRecd = 0;
bool cambioPuerta, cambioBatJaula, cambioComm;

char estadoStr[3][10] = {"Abierta", "Cerrado", "Puerta:?"};

static const SPIConfig spicfg = {
  false,
  false,
  NULL,
  NULL,
  GPIOA,
  GPIOA_CS,
  SPI_CR1_BR_1 | SPI_CR1_BR_0,
  0
};


static RFConfig nrf24l01_cfg = {
  .line_ce          = LINE_CE,
  .line_irq         = LINE_IRQ,
  .spip             = &SPID1,
  .spicfg           = &spicfg,
  .auto_retr_count  = NRF24L01_ARC_15_times,
  .auto_retr_delay  = NRF24L01_ARD_4000us,
  .address_width    = NRF24L01_AW_5_bytes,
  .channel_freq     = 60,                       // antes 120
  .data_rate        = NRF24L01_ADR_250kbps,
  .out_pwr          = NRF24L01_PWR_0dBm,
  .lna              = NRF24L01_LNA_enabled,
  .en_dpl           = NRF24L01_DPL_enabled,
  .en_ack_pay       = NRF24L01_ACK_PAY_disabled,
  .en_dyn_ack       = NRF24L01_DYN_ACK_disabled
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/


void parpadear(uint8_t numVeces, uint16_t ms) {
    palSetLineMode(LINE_LED, PAL_MODE_OUTPUT_PUSHPULL);
    for (uint8_t i = 0; i < numVeces; i++) {
        palClearLine(LINE_LED); // enciende led placa
        chThdSleepMilliseconds(10);
        palSetLine(LINE_LED); // apaga led placa
        chThdSleepMilliseconds(ms);
    }
    palSetLineMode(LINE_LED, PAL_MODE_INPUT_ANALOG);
}

/*
 * Timing values are taken from the RM except the PRESC set to 9 because
 * the input clock is 72MHz. Con 16 MHz, debe ser dividir entre 2=>PRESC=1
 * The timings are critical, please always refer to the STM32 Reference
 * Manual before attempting changes.
 */
static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(1U) |
  STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
  STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(21U),
  0,
  0
};


void initI2C1(void)
{
    palSetLineMode(LINE_I2C1SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_I2C1SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(LINE_ONDISPLAY, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLine(LINE_ONDISPLAY);
    i2cStart(&I2CD1, &i2cconfig); // LCD
}

//            01234567890
// respuesta "1  731  80"
//            P   V   V%
/*

uint8_t estadoPuerta;  // abierto:0 cerrado:1 desconocido:2
float vBatJaula;
uint8_t porcBatJaula;
bool commOk = false;
 *const char estadoStr[3][10] = {"Abierta", "Cerrado", "Puerta:?"};
 */
void decodeStatus(char *buffer, uint8_t sizeofBuffer)
{
    uint8_t estadoPuertaOld = estadoPuerta;
    if (sizeofBuffer<15)
        return;
    // estado puerta
    char estadoChar = buffer[0];
    if (estadoChar=='0')
        estadoPuerta = 0;
    else if (estadoChar=='1')
        estadoPuerta = 1;
    else
        estadoPuerta = 2;
    if (estadoPuertaOld != estadoPuerta)
        cambioPuerta = true;
    // tension %
    uint8_t porcBatJaulaOld = porcBatJaula;
    porcBatJaula = atoi(&buffer[7]);
    if (porcBatJaula>100)
        porcBatJaula = 0;
    if (porcBatJaulaOld != porcBatJaula)
        cambioBatJaula = true;
    buffer[7] = 0;
    vBatJaula = 0.01f*atoi(&buffer[1]);
}

// bool cambioPuerta, cambioBatJaula, cambioComm;
void ponStatus(void)
{
    char buff2[15];    //    snprintf(buff2, sizeof(buff2),"RX:%.2fV%3d",vBatJaula,porcBatJaula);
    if (cambioComm)
    {
        snprintf(buff2, sizeof(buff2),"S:%d %d",paqSent, paqSent-paqRecd);
        printfFilaSSD1306(1,buff2);
        cambioComm = false;
    }
    if (cambioPuerta)
    {
        printfFilaSSD1306(2,estadoStr[estadoPuerta]);
        cambioPuerta = false;
    }
}

/*
 * Application entry point.
 */
int main(void) {
    char buffer[15], string[15];
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  initI2C1();
  ssd1306Init();

  parpadear(3,200);
  leeVariablesC();
  leeTension(&vBat);

  snprintf(buffer, sizeof(buffer),"TX:%.2fV",vBat);
  printfFilaSSD1306(0,buffer);

  /*
   * SPID1 I/O pins setup.(It bypasses board.h configurations)
   */
  palSetLineMode(LINE_SPI1SCK, PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_SPI1MISO, PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_SPI1MOSI, PAL_MODE_ALTERNATE(0) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  /*
   * CE and IRQ pins setup.
   */
  palSetLineMode(LINE_CE, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_IRQ, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);

  /* Starting Serial Driver 2 with default configurations. */
  sdStart(&SD2, NULL);

  /* RF Driver Object constructor. */
  rfInit();

  /* Starting RF driver. */
  rfStart(&RFD1, &nrf24l01_cfg);

  estadoPuerta = 2;
  commOk = false;
  uint16_t msSinEstado = 20000;
  // bool cambioPuerta, cambioBatJaula, cambioComm;
  cambioPuerta = true;
  cambioBatJaula = true;
  cambioComm = true;
  ponStatus();

  while (TRUE) {
#ifdef TRANSMITTER
    rf_msg_t msgRx, msgTx;
    while(TRUE){
        uint8_t estadoPuertaDeseada = palReadLine(LINE_SENSOR);
        if (estadoPuertaDeseada != estadoPuerta)
        {
            snprintf(buffer, sizeof(buffer),"PUERTA:%d",estadoPuertaDeseada);
            msgTx = rfTransmitString(&RFD1, buffer, "RXadd", TIME_MS2I(75));
            paqSent++;
            cambioComm = true;
            if(msgTx == RF_ERROR)
            {
                ponStatus();
                continue;
            }
            msgRx = rfReceiveString(&RFD1, string, "RXadd", TIME_MS2I(100));
            if(msgRx == RF_OK) {
                commOk = true;
                paqRecd++;
                decodeStatus(string,sizeof(string));
            }
            else
                commOk = false;
            ponStatus();
        }
        else
        {
            if (msSinEstado>20)
            {
                msgTx = rfTransmitString(&RFD1, "ESTADO", "RXadd", TIME_MS2I(75));
                paqSent++;
                cambioComm = true;
                if(msgTx == RF_ERROR)
                {
                    ponStatus();
                    continue;
                }
                msgRx = rfReceiveString(&RFD1, string, "RXadd", TIME_MS2I(100));
                if(msgRx == RF_OK) {
                    msSinEstado = 0;
                    commOk = true;
                    paqRecd++;
                    decodeStatus(string,sizeof(string));
                }
            }
            else
                msSinEstado += 10;
            ponStatus();
        }
        chThdSleepMilliseconds(10);
    }
#else
    string[0] = '\0';
    rf_msg_t msg = rfReceiveString(&RFD1, string, "RXadd", TIME_MS2I(4000));
    if (msg == RF_OK)
        parpadear(5,150);
    else
        parpadear(1,150);
#endif
  }
  rfStop(&RFD1);
}
