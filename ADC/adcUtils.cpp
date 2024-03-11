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
    void hallaFactorADC(void);
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

extern "C"
{
    void leeTension(float *vBat);
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 1 channel, SW triggered.
 * Channels:    B0 == AN8.
 * Channels:    B1 == AN9.
 */


static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_160P5,                                 /* SMPR */
  ADC_CHSELR_CHSEL5                                   /* CHSELR */
};


static const ADCConversionGroup adccfgVref = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_160P5,                                 /* SMPR */
  ADC_CHSELR_CHSEL17                                   /* CHSELR */
};

extern int16_t incAdPormil;

/* ADC internal channels related definitions */
/* Internal voltage reference VrefInt */
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t)0x1FF80078U))
/* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VREF ((uint16_t) 3000U)

bool factorCalculado = false;
float factorADC = 0.00113919494f;
void hallaFactorADC(void)
{
    uint16_t vrefcal = *VREFINT_CAL_ADDR;
    // activa VREFEN
    adcStart(&ADCD1,NULL);
    ADC->CCR |= ADC_CCR_VREFEN;
    chThdSleepMilliseconds(1);
    adcConvert(&ADCD1, &adccfgVref, &samples_buf[0], ADC_GRP1_BUF_DEPTH);
    adcStop(&ADCD1);
    factorADC = 0.00113919494f*((float) samples_buf[1])/((float) vrefcal)*(1.0f+incAdPormil/1000.0f);
}


/* Lee tension */
void leeTension(float *vBat)
{
    if (!factorCalculado)
    {
        hallaFactorADC();
        factorCalculado = true;
    }
    palSetLineMode(GPIOA_VINAD, PAL_MODE_INPUT_ANALOG);
    chThdSleepMilliseconds(5);
    adcStart(&ADCD1,NULL);
    adcConvert(&ADCD1, &adcgrpcfg1, &samples_buf[0], ADC_GRP1_BUF_DEPTH);
    adcStop(&ADCD1);
    *vBat = samples_buf[0]*factorADC;
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
