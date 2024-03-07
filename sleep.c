/*
 * sleep.c
 *
 *  Created on: 17 sept. 2022
 *      Author: joaquin
 */
#include "ch.h"
#include "hal.h"
#include "alimCalle.h"


#define PWR_CR_LPDS_Pos           (16U)
#define PWR_CR_LPDS_Msk           (0x1UL << PWR_CR_LPDS_Pos)                 /*!< 0x00004000 */
#define PWR_CR_LPDS               PWR_CR_LPDS_Msk                            /*!< Low power run mode */

/*
 * Ver http://forum.chibios.org/viewtopic.php?t=3381
 *
 */
uint8_t GL_Flag_External_WakeUp, GL_Sleep_Requested;

void cb_external_input_wake_up(void *arg)
{
  (void)arg;
  if (!palReadLine(LINE_WKUP))
  {
      GL_Flag_External_WakeUp = 1;
      return;
  }
  if (!palReadLine(LINE_SENSOR))
  {
      GL_Flag_External_WakeUp = 2;
      return;
  }
}

void cb_external_input_wake_upA1(void *arg)
{
  (void)arg;
  if (!palReadLine(LINE_WKUP))
  {
      GL_Flag_External_WakeUp = 1;
      return;
  }
  if (!palReadLine(LINE_SENSOR))
  {
      GL_Flag_External_WakeUp = 2;
      return;
  }
}

void ports_set_normal(void)
{
  palSetLineMode(LINE_WKUP,PAL_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);
//  palSetLineMode(LINE_SENSOR,PAL_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);
  palSetLineMode(LINE_LED, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_PWM,PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_HIGHEST);    /* PWM*/
  palSetLineMode(LINE_SWDIO, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLUP);
  palSetLineMode(LINE_SWDCLK, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLDOWN);
}

void ports_set_lowpower(void)
{
  /* Set all I/O pins to Analog inputs */
  for(uint8_t i = 0; i < 16; i++ )
   {
       //if (i!=GPIOA_SWDIO && i!=GPIOA_SWCLK )   // && i!=GPIOA_TX2 && i!=GPIOA_RX2)
       palSetPadMode( GPIOA, i,PAL_MODE_INPUT_ANALOG );
       palSetPadMode( GPIOB, i,PAL_MODE_INPUT_ANALOG );
       palSetPadMode( GPIOC, i,PAL_MODE_INPUT_ANALOG );
       palSetPadMode( GPIOD, i,PAL_MODE_INPUT_ANALOG );
       palSetPadMode( GPIOE, i,PAL_MODE_INPUT_ANALOG );
       palSetPadMode( GPIOH, ( i % 2 ),PAL_MODE_INPUT_ANALOG );
   }
  palSetLineMode(LINE_WKUP,PAL_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);
  palSetLineMode(LINE_SENSOR,PAL_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);

//  palSetPad(GPIOA, GPIOA_W25Q16_CS);
//  palSetPadMode(GPIOA, GPIOA_W25Q16_CS, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_ONSERVO);
  palSetLineMode(LINE_ONSERVO, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_ONAD);
  palSetLineMode(LINE_ONAD, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_ONGPS);
  palSetLineMode(LINE_ONGPS, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LINE_ONHM10);
  palSetLineMode(LINE_ONHM10, PAL_MODE_OUTPUT_PUSHPULL);

  palSetLineMode(LINE_SWDIO, PAL_MODE_ALTERNATE(0) | PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_SWDCLK, PAL_MODE_ALTERNATE(0) | PAL_MODE_INPUT_ANALOG);

//  palSetPadMode(GPIOB, GPIOB_PWMSERVO,PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_HIGHEST);    /* PWM*/
//  palSetLineMode(LINE_GPIOA_SWDIO, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLUP);
//  palSetLineMode(LINE_GPIOA_SWCLK, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLDOWN);
}


//http://forum.chibios.org/viewtopic.php?t=3381




// http://forum.chibios.org/viewtopic.php?f=16&t=5444
/** ****************************************************************************
 * \fn     void sleep_for_x_sec(uint16_t nb_sec)
 * \brief  Put the STM32 in the Stop2 mode
 *         Must be called only from main()  To be tested (seen some comments on forums)
 * \param[in]  (u16) nb_of sec to wait in sleep mode: 1-65535
 * \return     (u8)  wake source:  WAKE_SOURCE_TIMER, WAKE_SOURCE_EXTERNAL, WAKE_SOURCE_LPUART
 */
// funciona con consumo de 1.9mA (no cambia al quitar ADC ni puesrto serie)

uint8_t sleep_for_x_sec(uint32_t nb_sec)
{
    static RTCWakeup wakeupspec;
    static uint8_t wakeup_source;

    GL_Sleep_Requested = 0;                               // Reset Flag Sleep_Requested

    // prepara interrupcion por boton KEY
    GL_Flag_External_WakeUp = 0;                          // Reset flag that will be set by the CB
    palEnableLineEvent(LINE_WKUP, PAL_EVENT_MODE_FALLING_EDGE);     // Falling edge creates event
    palSetLineCallback(LINE_WKUP, cb_external_input_wake_up, NULL); // Active callback

    // desconozco si se comportara bien en el entorno de 0x10000
    if (nb_sec>0xFFFF && nb_sec<0x10005)
        nb_sec = 0xFFFF;
    // prepara despertar por timer
    if (nb_sec<=0x10000)
    {
        wakeupspec.wutr = ( (uint32_t)4 ) << 16; //antes 4    // bits 16-18 = WUCKSel : Select 1 Hz clk
        wakeupspec.wutr |= nb_sec - 1;                        // bits 0-15  = WUT : Period = x+1 sec
    }
    else
    {
        //WUT = Wakeup unit counter value. WUT = (0x0000 to 0xFFFF) + 0x10000 added when WUCKSEL[2:1 = 11].
        wakeupspec.wutr = ( (uint32_t)6 ) << 16;              // bits 16-18 = WUCKSel : Select 1 Hz clk
        wakeupspec.wutr |= (nb_sec - 0x10000 - 1);             // bits 0-15  = WUT : Period = x+1 sec
    }
    rtcSTM32SetPeriodicWakeup(&RTCD1, &wakeupspec);       // Set RTC wake-up config

    //  ports_set_lowpower();                                 // Set ports for low power
    //  palSetLineMode(LINE_GPIOA_SWDIO, PAL_MODE_INPUT_ANALOG);
    //  palSetLineMode(LINE_GPIOA_SWCLK, PAL_MODE_INPUT_ANALOG);

    chSysDisable();                               // No effect
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;   // No effect        // Disable TickInt Request
    chSysEnable();                                // No effect

    chSysDisable();
    __disable_irq();
    chSysEnable();

    // adormir
    __WFI();

    palSetLineMode(LINE_SWDIO, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(LINE_SWDCLK, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLDOWN);

    PWR->CSR |= PWR_CSR_SBF;                             // Clear standby flag

    __enable_irq();                               // No effect

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    palDisableLineEvent(LINE_WKUP);

    return wakeup_source;
}


// http://forum.chibios.org/viewtopic.php?f=16&t=5444
/** ****************************************************************************
 * \fn     void sleep_for_x_sec(uint16_t nb_sec)
 * \brief  Put the STM32 in the Stop2 mode
 *         Must be called only from main()  To be tested (seen some comments on forums)
 * \param[in]  (u16) nb_of sec to wait in sleep mode: 1-65535
 * \return     (u8)  wake source:  WAKE_SOURCE_TIMER, WAKE_SOURCE_EXTERNAL, WAKE_SOURCE_LPUART
 */
// consumo 26uA, necesito hacer halInit() para que funcione PWM y sleep al volver

/*
 * Atencion: como en stop hay que reiniciar hal, se queda descojonado los eventos, ya que no quedan
 * bien reseteados las mascaras de interrupciones. Hay que modificar comentando en hal_pal_lld.c la linea 241
 *      osalDbgAssert(crport == portidx, "channel mapped on different port");
 */
uint8_t stop(uint16_t nb_sec)
{
    static RTCWakeup wakeupspec;
    static uint8_t wakeup_source;

    GL_Flag_External_WakeUp = 0;
    if (nb_sec==0)
        nb_sec = 1;
    GL_Sleep_Requested = 0;                               // Reset Flag Sleep_Requested

    // prepara interrupcion por boton KEY
    GL_Flag_External_WakeUp = 0;                          // Reset flag that will be set by the CB
    palEnableLineEvent(LINE_WKUP, PAL_EVENT_MODE_FALLING_EDGE);     // Falling edge creates event
    palSetLineCallback(LINE_WKUP, cb_external_input_wake_up, NULL); // Active callback

    palEnableLineEvent(LINE_SENSOR, PAL_EVENT_MODE_FALLING_EDGE);     // Falling edge creates event
    palSetLineCallback(LINE_SENSOR, cb_external_input_wake_upA1, NULL); // Active callback

    // prepara despertar por timer
    wakeupspec.wutr = ( (uint32_t)4 ) << 16; //antes 4             // bits 16-18 = WUCKSel : Select 1 Hz clk
    wakeupspec.wutr |= nb_sec - 1;                        // bits 0-15  = WUT : Period = x+1 sec

    RTCD1.rtc->WPR = 0xCA;       // Disable write protection
    RTCD1.rtc->WPR = 0x53;

    rtcSTM32SetPeriodicWakeup(&RTCD1, &wakeupspec);       // Set RTC wake-up config

    RTCD1.rtc->WPR = 0xFF;

    chSysDisable();                               // No effect
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;   // No effect        // Disable TickInt Request
    chSysEnable();                                // No effect

    chSysDisable();
    __disable_irq();
    chSysEnable();

    DBGMCU->CR = 0;

/*    // lo que pone en http://forum.chibios.org/viewtopic.php?t=2315
     clear PDDS and LPDS bits 176 uA
    PWR->CR &= ~(PWR_CR_LPSDSR | PWR_CR_LPDS);
     set LPDS and clear
    PWR->CR |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
    PWR->CR |= PWR_CR_LPSDSR;*/

// Aqui lo que pone en https://forum.chibios.org/viewtopic.php?t=2383 para un STM32L1
/*
    PWR->CR |= (PWR_CR_LPSDSR | PWR_CR_CSBF | PWR_CR_CWUF);
    PWR->CR &= ~PWR_CR_PDDS;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
*/

    // Lo que esta en RM (pag. 156)
    // SLEEPDEEP bit is set in Cortex®-M0+ System Control register
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    // PDDS bit = 0 in Power Control register (PWR_CR) => modo stop
    PWR->CR &= ~PWR_CR_PDDS;
    // De mi cosecha
    PWR->CR |= (PWR_CR_LPSDSR | PWR_CR_CSBF | PWR_CR_CWUF);
    // WUF bit = 0 in Power Control/Status register (PWR_CSR)
    PWR->CSR &= ~PWR_CSR_WUF;
    // MSI or HSI16 RC oscillator are selected as system clock for Stop mode exit by configuring the STOPWUCK bit in the RCC_CFGR register.
    __WFI();


//    /* Setup the deepsleep mask */
//    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//
//    __WFI();

    __enable_irq();

    palSetLineMode(LINE_SWDIO, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLUP);
    palSetLineMode(LINE_SWDCLK, PAL_MODE_ALTERNATE(0) | PAL_STM32_PUPDR_PULLDOWN);

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;                   // Clear deep sleep mask

    PWR->CSR |= PWR_CSR_SBF;                             // Clear standby flag

    stm32_clock_init();                           // Si lo pongo, en sleep se queda pillado        // Re-init RCC and power, Important
    halInit();

    __enable_irq();                               // No effect

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;    // No effect          // Enable TickInt Request

    palDisableLineEvent(LINE_SENSOR);
    palDisableLineEvent(LINE_WKUP);

    //ports_set_normal();
    return wakeup_source;
}

