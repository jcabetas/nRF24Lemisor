#include "ch.hpp"
#include "hal.h"
#include "string.h"
#include "chprintf.h"


using namespace chibios_rt;

extern "C"
{
    void pruebaADC(void);
    void initAdc(void);
    float hallaCapBatC(void);
}

//                       0%      5%    10%    15%    20%    25%    30%    35%   40%     45%    50%
float lipoVoltCharge[]={3.3f,  3.50f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.8f, 3.82f, 3.84f,\
                        3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f };
//                       55%    60%    65%    70%    75%    80%    85%    90%    95%    100%

/*
 * La tension se lee en PB0 (ADC1-8), a traves de divisor 220/(91+220)=con factor 0,707395
 *   El fondo de escala es 4096 corresponde a 3.3 V
 *   Por tanto la tension es ADC*3.3/(0,707395*4096) = ADC*0,00113891682
 */

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      4
void printSerialCPP(const char *msg);
// Create buffer to store ADC results. This is
// one-dimensional interleaved array

extern int16_t incAdPormil;

adcsample_t samples_buf[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH]; // results array

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 1 channel, SW triggered.
 * Channels:    B0 == AN8.
 * Channels:    B1 == AN9.
 */

// ver os/common/ext/ST/STM32L0xx/stm32l071xx.h
//typedef struct
//{
//  __IO uint32_t ISR;          /*!< ADC Interrupt and Status register,                          Address offset:0x00 */
//  __IO uint32_t IER;          /*!< ADC Interrupt Enable register,                              Address offset:0x04 */
//  __IO uint32_t CR;           /*!< ADC Control register,                                       Address offset:0x08 */
//  __IO uint32_t CFGR1;        /*!< ADC Configuration register 1,                               Address offset:0x0C */
//  __IO uint32_t CFGR2;        /*!< ADC Configuration register 2,                               Address offset:0x10 */
//  __IO uint32_t SMPR;         /*!< ADC Sampling time register,                                 Address offset:0x14 */
//  uint32_t   RESERVED1;       /*!< Reserved,                                                                  0x18 */
//  uint32_t   RESERVED2;       /*!< Reserved,                                                                  0x1C */
//  __IO uint32_t TR;           /*!< ADC watchdog threshold register,                            Address offset:0x20 */
//  uint32_t   RESERVED3;       /*!< Reserved,                                                                  0x24 */
//  __IO uint32_t CHSELR;       /*!< ADC channel selection register,                             Address offset:0x28 */
//  uint32_t   RESERVED4[5];    /*!< Reserved,                                                                  0x2C */
//  __IO uint32_t DR;           /*!< ADC data register,                                          Address offset:0x40 */
//  uint32_t   RESERVED5[28];   /*!< Reserved,                                                           0x44 - 0xB0 */
//  __IO uint32_t CALFACT;      /*!< ADC data register,                                          Address offset:0xB4 */
//} ADC_TypeDef;

// B0 = ADC-IN8
//static const ADCConversionGroup adcgrpcfg1 = {
//  FALSE,
//  ADC_GRP1_NUM_CHANNELS,
//  NULL,
//  adcerrorcallback,
//  ADC_CR_ADSTART,           /* CR */
//  0,                        /* CFGR1 */
//  0,                        /* CFGR2 */
//  ADC_SMPR_SMP_160P5,       /* SMPR */
//  0,                        /* TR */
//  ADC_CHSELR_CHSEL8,        /* CHSELR */
//  0,                        /* DR */
//  0,                        /* CALFACT */
//};
//  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_160P5,                                 /* SMPR */
  ADC_CHSELR_CHSEL8                                /* CHSELR */
};



//static const ADCConversionGroup adcgrpcfg1 = {
//  FALSE,
//  ADC_GRP1_NUM_CHANNELS,
//  NULL,
//  adcerrorcallback,
//  0,                        /* CFGR1 */
//  0,                        /* TR */
//  ADC_SMPR_SMP_160P5,       /* SMPR */
//  ADC_CHSELR_CHSEL8,        /* CHSELR */
//};

extern int16_t incAdPormil;

/* Lee tension */
void leeTension(float *vBat)
{
    palSetLineMode(GPIOA_VINAD, PAL_MODE_INPUT_ANALOG); // B0 medida de bateria
    chThdSleepMilliseconds(10);
    adcStart(&ADCD1,NULL);
    adcConvert(&ADCD1, &adcgrpcfg1, &samples_buf[0], ADC_GRP1_BUF_DEPTH);
    adcStop(&ADCD1);
    *vBat = samples_buf[0]*0.00113919414f*(1.0f+incAdPormil/1000.0f);
//    QUITAPAD(LINE_ONAD);
    chThdSleepMilliseconds(1);
//    palSetPadMode(GPIOB, GPIOB_ONAD,PAL_MODE_INPUT_ANALOG);
}


bool tensionCritica(void)
{
    float vBat;
    leeTension(&vBat);
    if (vBat<3.60f)
        return true;
    else
        return false;
}

float hallaCapBat(float *vBat)
{
    if (*vBat<=lipoVoltCharge[0])
        return 0.0f;
    if (*vBat>=lipoVoltCharge[20])
        return 100.0f;
    // miro en que rango se encuentra
    for (uint8_t n=1;n<sizeof(lipoVoltCharge);n++)
        if (*vBat<=lipoVoltCharge[n])
        {
            // se encuentra entre n-1 y n. Los escalones de carga son del 5%
            float result = (n-1)*5.0f+5.0f*(*vBat - lipoVoltCharge[n-1])/(lipoVoltCharge[n]-lipoVoltCharge[n-1]);
            return result;
        }
    return 999.9f; //
}

float hallaCapBatC(void)
{
    float vBat;
    leeTension(&vBat);
    return hallaCapBat(&vBat);
}

void pruebaADC(void)
{
    char buff[30];
    float vBat;
    leeTension(&vBat);
    float porcBat = hallaCapBat(&vBat);
    chsnprintf(buff,sizeof(buff),"Vbat:%.3fV (%d%%)\r\n",vBat,(int16_t) porcBat);
    printSerialCPP(buff);
}
