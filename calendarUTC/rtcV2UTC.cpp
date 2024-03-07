#include "ch.hpp"
#include "hal.h"
using namespace chibios_rt;
#include "string.h"
#include "../calendarUTC/calendarUTC.h"
#include "gets.h"
#include "stdlib.h"
#include "chprintf.h"
/*
  La hora UTC en España:
  - Durante los siete meses que dura el horario de primavera-verano, España está en la zona UTC +2
  - Durante los cinco meses que dura el horario de otoño-invierno, España está en la zona UTC +1

  - El cambio de invierno a verano se hace a las 02:00 del ultimo domingo de marzo
  - El cambio de verano a invierno se hace a las 03:00 del ult. domingo de octubre

  Como lo implementamos:
  - El reloj interno siempre estará en hora UTC
  - no se usa dstflag del RTC
  - El paso de invierno a verano se hace a manubrio, con fechas preestablecidas (struct tm)

 Estructuras de datos:

   uint32_t RTCDateTime
     uint32_t      year: 8;            // Years since 1980.
     uint32_t      month: 4;           // Months 1..12.
     uint32_t      dstflag: 1;         // DST correction flag.
     uint32_t      dayofweek: 3;       // Day of week 1..7.
     uint32_t      day: 5;             // Day of the month 1..31.
     uint32_t      millisecond: 27;    // Milliseconds since midnight.

  struct tm
    int tm_sec       seconds after minute [0-61] (61 allows for 2 leap-seconds)
    int tm_min       minutes after hour [0-59]
    int tm_hour      hours after midnight [0-23]
    int tm_mday      day of the month [1-31]
    int tm_mon       month of year [0-11]
    int tm_year      current year-1900
    int tm_wday      days since Sunday [0-6]
    int tm_yday      days since January 1st [0-365]
    int tm_isdst     daylight savings indicator (1 = yes, 0 = no, -1 = unknown)

 time_t
    num. de segundos desde 1/1/1970

 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define RTC_TR_PM_OFFSET                    22
#define RTC_TR_HT_OFFSET                    20
#define RTC_TR_HU_OFFSET                    16
#define RTC_TR_MNT_OFFSET                   12
#define RTC_TR_MNU_OFFSET                   8
#define RTC_TR_ST_OFFSET                    4
#define RTC_TR_SU_OFFSET                    0

#define RTC_DR_YT_OFFSET                    20
#define RTC_DR_YU_OFFSET                    16
#define RTC_DR_WDU_OFFSET                   13
#define RTC_DR_MT_OFFSET                    12
#define RTC_DR_MU_OFFSET                    8
#define RTC_DR_DT_OFFSET                    4
#define RTC_DR_DU_OFFSET                    0

#define RTC_CR_BKP_OFFSET                   18

extern "C" {
    void checkRTC(void);
    void ajustaCALMP_C(int16_t dsDia);
}
void printSerialCPP(const char *msg);


/*
 * Lookup table with months' length
 */
const uint8_t month_len[12] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};



static void rtc_enter_init(RTCDriver *RTCD1) {
  RTCD1->rtc->ISR |= RTC_ISR_INIT;
  while ((RTCD1->rtc->ISR & RTC_ISR_INITF) == 0)
    ;
}

static inline void rtc_exit_init(RTCDriver *RTCD1) {
  RTCD1->rtc->ISR &= ~RTC_ISR_INIT;
}



/* 1 <= m <= 12,  y > 1752 (in the U.K.) */
/* 0=Sunday */
uint8_t dayofweek(uint16_t y, uint16_t m, uint16_t d)
{
    static uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if( m < 3 )
        y -= 1;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

/*
 *  tm_sec   int seconds after the minute   0-61*
    tm_min  int minutes after the hour      0-59
    tm_hour int hours since midnight        0-23
 */
static void rtc_tr2tmt(const uint32_t tr, struct tm *tim) {
  tim->tm_hour = 10*((tr >> RTC_TR_HT_OFFSET) & 3) + ((tr >> RTC_TR_HU_OFFSET) & 15);
  if (tim->tm_hour>23)
      tim->tm_hour = 23;
  tim->tm_min = 10*((tr >> RTC_TR_MNT_OFFSET) & 7) + ((tr >> RTC_TR_MNU_OFFSET) & 15);
  if (tim->tm_min>59)
      tim->tm_min = 59;
  tim->tm_sec = 10*((tr >> RTC_TR_ST_OFFSET) & 7) + ((tr >> RTC_TR_SU_OFFSET) & 15);
  if (tim->tm_sec>61)
      tim->tm_min = 61;
}



/*
 *  tm_mday int day of the month            1-31
    tm_mon  int months since January        0-11
    tm_year int years since 1900
    tm_wday int days since Sunday           0-6
    tm_yday int days since January 1        0-365
    tm_isdst    int Daylight Saving Time flag
 *
 */
static void rtc_dr2tmd(const uint32_t dr, struct tm *tim) {
    tim->tm_year = 100+10*((dr >> RTC_DR_YT_OFFSET) & 15) + ((dr >> RTC_DR_YU_OFFSET) & 15);
    tim->tm_mon = 10*((dr >> RTC_TR_MNT_OFFSET) & 1) + ((dr >> RTC_TR_MNU_OFFSET) & 15) - 1;
    if (tim->tm_mon>11)
        tim->tm_mon = 11;
    tim->tm_mday = 10*((dr >> RTC_DR_DT_OFFSET) & 3) + ((dr >> RTC_DR_DU_OFFSET) & 15);
    if (tim->tm_mday>31)
        tim->tm_mday = 31;
    tim->tm_wday = (dr >> RTC_DR_WDU_OFFSET) & 7;
    calendar::completeYdayWday(tim);
}

/*
 * @brief   Converts time from struct tm to TR register encoding.
 */
static uint32_t rtc_tmt2tr(const struct tm *tim) {
  uint32_t n, tr = 0;

  /* Seconds conversion.*/
  n = tim->tm_sec;
  tr = tr | ((n % 10) << RTC_TR_SU_OFFSET);
  n /= 10;
  tr = tr | ((n % 6) << RTC_TR_ST_OFFSET);

  /* Minutes conversion.*/
  n = tim->tm_min;
  tr = tr | ((n % 10) << RTC_TR_MNU_OFFSET);
  n /= 10;
  tr = tr | ((n % 6) << RTC_TR_MNT_OFFSET);

  /* Hours conversion.*/
  n = tim->tm_hour;
  tr = tr | ((n % 10) << RTC_TR_HU_OFFSET);
  n /= 10;
  tr = tr | (n << RTC_TR_HT_OFFSET);

  return tr;
}

/**
 * @brief   Converts a date from struct tm to DR register encoding.
 *
 */
static uint32_t rtc_tmd2dr(const struct tm *tim) {
  uint32_t n, dr = 0;

  /* Year conversion. Note, only years last two digits are considered.*/
  n = tim->tm_year + 1900;
  dr = dr | ((n % 10) << RTC_DR_YU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_YT_OFFSET);

  /* Months conversion.*/
  n = tim->tm_mon+1;
  dr = dr | ((n % 10) << RTC_DR_MU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_MT_OFFSET);

  /* Days conversion.*/
  n = tim->tm_mday;
  dr = dr | ((n % 10) << RTC_DR_DU_OFFSET);
  n /= 10;
  dr = dr | ((n % 10) << RTC_DR_DT_OFFSET);

  /* Days of week conversion.*/
  dr = dr | (tim->tm_wday << RTC_DR_WDU_OFFSET);

  return dr;
}



/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to set it on STM32 platform.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtcSetTM(RTCDriver *rtcp, struct tm *tim, uint16_t ds)  {
  uint32_t dr, tr;
  syssts_t sts;

  tr = rtc_tmt2tr(tim);
  dr = rtc_tmd2dr(tim);

  // ajuste verano/invierno => no se usa
  tim->tm_isdst = 0;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Writing the registers.*/
  RTCD1.rtc->WPR = 0xCA;
  RTCD1.rtc->WPR = 0x53;

  rtc_enter_init(&RTCD1);
  rtcp->rtc->TR = tr;
  rtcp->rtc->DR = dr;
  rtcp->rtc->SSR = ds;
  rtcp->rtc->CR = (rtcp->rtc->CR & ~(1U << RTC_CR_BKP_OFFSET)) |
                  (tim->tm_isdst << RTC_CR_BKP_OFFSET);

  rtc_exit_init(&RTCD1);
  RTCD1.rtc->WPR = 0xFF;
  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);
}


/**
 * @brief   Get current time.
 * @note    The function can be called from any context.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[out] timespec pointer to a @p RTCDateTime structure
 *
 * @notapi
 */
void rtcGetTM(RTCDriver *rtcp, struct tm *tim, uint16_t *ds) {
  uint32_t dr, tr, cr;

#if STM32_RTC_HAS_SUBSECONDS
  uint32_t ssr;
#endif /* STM32_RTC_HAS_SUBSECONDS */
  syssts_t sts;

  /* Entering a reentrant critical zone.*/
  sts = osalSysGetStatusAndLockX();

  /* Synchronization with the RTC and reading the registers, note
     DR must be read last.*/
  while ((rtcp->rtc->ISR & RTC_ISR_RSF) == 0)
    ;
#if STM32_RTC_HAS_SUBSECONDS
  ssr = rtcp->rtc->SSR;
#endif /* STM32_RTC_HAS_SUBSECONDS */
  tr  = rtcp->rtc->TR;
  dr  = rtcp->rtc->DR;
  cr  = rtcp->rtc->CR;
  rtcp->rtc->ISR &= ~RTC_ISR_RSF;

  /* Leaving a reentrant critical zone.*/
  osalSysRestoreStatusX(sts);


  /* Decoding day time, this starts the atomic read sequence, see "Reading
     the calendar" in the RTC documentation.*/
  rtc_tr2tmt(tr, tim);

  /* If the RTC is capable of sub-second counting then the value is
     normalized in milliseconds and added to the time.*/
#if STM32_RTC_HAS_SUBSECONDS
  *ds = (10*((STM32_RTC_PRESS_VALUE - 1U) - ssr)) / STM32_RTC_PRESS_VALUE;
#else
  *ds = 0;
#endif /* STM32_RTC_HAS_SUBSECONDS */

  /* Decoding date, this concludes the atomic read sequence.*/
  rtc_dr2tmd(dr, tim);

  /* Retrieving the DST bit.*/
  tim->tm_isdst = (cr >> RTC_CR_BKP_OFFSET) & 1;
}


//// devuelve fecha/hora UTC en formato tm
//void GetTimeTM(struct tm *tim) {
//  rtcGetTime(&RTCD1, &timespec); // lee del RTC fecha y hora
//  rtcConvertDateTimeToStructTm(&timespec, tim, NULL); // convierte a time_t de unix
//}

//time_t GetTimeUnixSecC(void)
//{
//    return calendar::GetTimeUnixSec();
//}

//// devuelve fecha/hora UTC en formato tm, y las decimas de segundo
//void GetTimeTm(struct tm *timp, uint16_t *ds) {
//  getTimeTMC(timp, ds);
////  uint32_t dsec;
////  void getTimeTMC(struct tm *tim, uint16_t ds)
////  //rtcGetTime(&RTCD1, &timespec);
////  //rtcConvertDateTimeToStructTm(&timespec, timp, NULL);
////  dsec = (int)timespec.millisecond / 100;
////  dsec %= 36000;
////  dsec %= 10;
////  *ds = dsec;
//}


void ajustaCALMP(int16_t dsDia)
{
    // hay que calcular cuantos pulsos hay que enmascarar dsDia/864000*32768*32
    if (dsDia>400 || dsDia<-400)
        return;
    int16_t pulsos2mask = (int16_t)((dsDia*4096L)/3375L);
    RTCD1.rtc->WPR = 0xCA;       // Disable write protection
    RTCD1.rtc->WPR = 0x53;
    if (dsDia>0) // hay que anyadir pulsos, CALP es 1
    {
        RTCD1.rtc->CALR = (512 - pulsos2mask);
        RTCD1.rtc->CALR |= RTC_CALR_CALP;
    }
    else
    {
        RTCD1.rtc->CALR = -pulsos2mask;
    }
    RTCD1.rtc->WPR = 0xBB;       // enable write protection
}

void ajustaCALMP_C(int16_t dsDia)
{
    ajustaCALMP(dsDia);
}

void checkRTC(void)
{
    // check
    struct tm tim;
    uint16_t ds;
    rtcGetTM(&RTCD1, &tim, &ds);
    tim.tm_year = 2021 - 1900;
    tim.tm_mon = 3;
    tim.tm_mday = 4;
    tim.tm_sec = 0;
    tim.tm_min = 30;
    tim.tm_hour = 9;
    rtcSetTM(&RTCD1,&tim, 0);
    rtcGetTM(&RTCD1, &tim, &ds);
}


//uint32_t msEntreFechas(RTCDateTime *fechaNew, RTCDateTime *fechaOld)
//{
//    struct tm tm_fechaNew, tm_fechaOld;
//    rtcConvertDateTimeToStructTm(fechaNew, &tm_fechaNew, NULL);
//    rtcConvertDateTimeToStructTm(fechaOld, &tm_fechaOld, NULL);
//    uint32_t sDiff = mktime(&tm_fechaNew)- mktime(&tm_fechaOld);
//    return 1000*sDiff + fechaNew->millisecond - fechaOld->millisecond;
//}




uint8_t esValle(void)
{
    struct tm ahora;
    uint16_t ds;
    rtcGetTM(&RTCD1, &ahora, &ds);
    //tm_wday   int days since Sunday   0-6
    if (ahora.tm_wday==0 || ahora.tm_wday==6)
        return 1;
    // tm_hour  int hours since midnight    0-23
    if (ahora.tm_hour<8)
        return 1;
    return 0;
}


