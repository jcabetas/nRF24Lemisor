#include "ch.hpp"
#include "hal.h"
using namespace chibios_rt;
#include "string.h"
#include "chprintf.h"
#include "../calendarUTC/calendarUTC.h"
#include "gets.h"
#include "stdlib.h"
#include "math.h"

uint8_t dayofweek(uint16_t y, uint16_t m, uint16_t d);
void printSerial(const char *msg);
void printSerialCPP(const char *msg);

extern int16_t addAmanecer;
extern int16_t addAtardecer;
extern uint16_t autoPuerta;  // 0:cerrada, 1:abierta, 2: automatico, 3: autoConMargen
uint32_t secAdaptacion;
uint16_t diaAdaptado;

extern struct queu_t colaMsgTxCan;
extern event_source_t sendMsgCAN_source;

static const uint16_t accu_month_len[12] = {
  0, 31, 59,  90, 120, 151, 181, 212, 243, 273, 304, 334
};

uint16_t calendar::minAmanecer = 370;
uint16_t calendar::minAnochecer = 1194;
uint16_t calendar::diaCalculado = 9999;
float calendar::longitudRad = -0.0697766927f; //  -3.99791
float calendar::latitudRad = 0.7058975433f;   //  40.44495
struct fechaHora calendar::fechaHoraNow = {0,0};
struct tm calendar::fechaNow = {0,0,0,0,0,0,0,0,0};

extern "C"
{
    void leeHora(void);
    void printFechaC(char *buff, uint16_t longBuff);
    void actualizaAmanAnoch(void);
    void estadoDeseadoPuertaC(uint8_t *estDes, uint32_t *sec2change);
    void iniciaSecAdaptacionC(void);
}


/*
 ***************************************************
 * Funciones necesarias para leer y ajustar fechas *
 ***************************************************
 */

/*
 * complete day of year, and day of the week
 */
void calendar::completeYdayWday(struct tm *tim)
{
    uint16_t year;
    uint8_t isLeapYear;
    /* compute day of year, even for leap year */
    year = tim->tm_year + 1900;
    tim->tm_yday = tim->tm_mday - 1;
    tim->tm_yday += accu_month_len[tim->tm_mon];
    isLeapYear = (year%4 == 0 && year%100 != 0) || year%400 == 0;
    if (isLeapYear && tim->tm_mon>1)
        tim->tm_yday++;
    /* compute day of the week */
    tim->tm_wday = dayofweek(year, tim->tm_mon+1, tim->tm_mday);
}

// segundos Unix en UTC
time_t calendar::getSecUnix(struct tm *tm)
{
    // Month-to-day offset for non-leap-years.
    static const int month_day[12] =
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

    // Most of the calculation is easy; leap years are the main difficulty.
    int16_t month = tm->tm_mon % 12;
    uint16_t year = tm->tm_year + tm->tm_mon / 12;
    if (month < 0) {   // Negative values % 12 are still negative.
        month += 12;
        --year;
    }

    // This is the number of Februaries since 1900.
    const uint16_t year_for_leap = (month > 1) ? year + 1 : year;

    time_t rt = tm->tm_sec                          // Seconds
        + 60 * (tm->tm_min                          // Minute = 60 seconds
        + 60 * (tm->tm_hour                         // Hour = 60 minutes
        + 24 * (month_day[month] + tm->tm_mday - 1  // Day = 24 hours
        + 365 * (year - 70)                         // Year = 365 days
        + (year_for_leap - 69) / 4                  // Every 4 years is     leap...
        - (year_for_leap - 1) / 100                 // Except centuries...
        + (year_for_leap + 299) / 400)));           // Except 400s.
    return rt < 0 ? -1 : rt;
}


void calendar::setLatLong(float latitudRadNoche, float longitudRadNoche)
{
    latitudRad = latitudRadNoche;
    longitudRad = longitudRadNoche;
}

/*
 * ver https://www.esrl.noaa.gov/gmd/grad/solcalc/solareqns.PDF
 * Altura solar ajustada a 93 grados (en 96 es demasiado oscuro)
 */
void calendar::ajustaHorasLuz(void)
{
    float diaAno, fraccAno;
    float decl, ha;

    if (diaCalculado == fechaNow.tm_yday)
        return; // ya estaba calculado
    diaAno = (float) (fechaNow.tm_yday);
    fraccAno = 2.0*M_PI/365.0*(diaAno-0.5f);
//    eqTime = +229.18f*(0.000075f+0.001868f*cos(fraccAno)-0.032077f*sin(fraccAno)-0.014615f*cos(2.0f*fraccAno)-0.040849f*sin(2.0f*fraccAno));
    decl = 0.006918f-0.399912f*cosf(fraccAno) + 0.070257f*sinf(fraccAno)-0.006758f*cosf(2.0f*fraccAno)+0.000907f*sinf(2.0f*fraccAno)-0.002697f*cosf(3.0f*fraccAno) + 0.00148f*sinf(3.0f*fraccAno);
    ha = -acosf(cosf(93.0f*M_PI/180.0f)/cosf(latitudRad)/cosf(decl)-tanf(latitudRad)*tan(decl));
    minAmanecer = 60.0f*(12.0f+ha*12.0f/M_PI-longitudRad*12.0f/M_PI);
    minAnochecer = 60.0*(12.0f-ha*12.0f/M_PI-longitudRad*12.0f/M_PI);
    diaCalculado = fechaNow.tm_yday;
}



/****************************************
 * Funciones para ajustar y leer fechas *
 ****************************************
 */

void calendar::rtcGetFecha(void)
{
    struct tm tim;
    uint16_t ds;
    rtcGetTM(&RTCD1, &tim, &ds);
    completeYdayWday(&tim);
    memcpy(&fechaNow, &tim, sizeof(fechaNow));
    fechaHoraNow.secsUnix = getSecUnix(&tim);
    fechaHoraNow.dsUnix = ds;
    ajustaHorasLuz();
}


void leeHora(void)
{
    calendar::rtcGetFecha();
}

void calendar::rtcSetFecha(struct tm *fecha, uint16_t ds)
{
    fechaHoraNow.secsUnix = calendar::getSecUnix(fecha);
    fechaHoraNow.dsUnix = ds;
    rtcSetTM(&RTCD1, fecha, ds);
    rtcGetFecha();
}

void calendar::cambiaFecha(uint16_t *anyo, uint8_t *mes, uint8_t *dia, uint8_t *hora, uint8_t *min, uint8_t *seg, uint8_t *dsPar)
{
    struct tm fechaUTC;
    uint16_t ds;

    rtcGetTM(&RTCD1, &fechaUTC, &ds);         // leo hora local
    if (anyo!=NULL && *anyo>2020 && *anyo<3000) // actualizo datos con lo que hayan pasado
        fechaUTC.tm_year = *anyo-1900;
    if (mes!=NULL && *mes>=1 && *mes<=12)
        fechaUTC.tm_mon = *mes-1;
    if (dia!=NULL && *dia>=1 && *dia<=31)
        fechaUTC.tm_mday = *dia;
    if (hora!=NULL && *hora<=23)
        fechaUTC.tm_hour = *hora;
    if (min!=NULL && *min<=59)
        fechaUTC.tm_min = *min;
    if (seg!=NULL && *seg<=59)
        fechaUTC.tm_sec = *seg;
    if (dsPar!=NULL && *dsPar<=9)
        ds = *dsPar;
    rtcSetFecha(&fechaUTC,ds);
}

void calendar::cambiaFechaTM(uint8_t anyo, uint8_t mes, uint8_t dia, uint8_t hora, uint8_t min, uint8_t seg, uint8_t dsPar)
{
    struct tm fechaUTC;
    uint16_t anyoReal, ds;

    rtcGetTM(&RTCD1, &fechaUTC, &ds);         // leo hora local
    anyoReal = 1900 + anyo;
    mes += 1;
    if (anyoReal>2020 && anyoReal<3000) // actualizo datos con lo que hayan pasado
        fechaUTC.tm_year = anyo;
    if (mes>=1 && mes<=12)
        fechaUTC.tm_mon = mes-1;
    if (dia>=1 && dia<=31)
        fechaUTC.tm_mday = dia;
    if (hora<=23)
        fechaUTC.tm_hour = hora;
    if (min<=59)
        fechaUTC.tm_min = min;
    if (seg<=59)
        fechaUTC.tm_sec = seg;
    if (dsPar<=9)
        ds = dsPar;
    rtcSetFecha(&fechaUTC,ds);
}

/*
 * Funciones que suponen haber leido hora antes con rtcGetFecha()
 */

time_t calendar::getSecUnix(void) {
    return fechaHoraNow.secsUnix;
}

void calendar::gettm(struct tm *fecha)
{
    memcpy(fecha, &fechaNow, sizeof(fechaNow));
}

void calendar::getFechaHora(struct fechaHora *fechHora)
{
    fechHora->secsUnix = fechaHoraNow.secsUnix;
    fechHora->dsUnix = fechaHoraNow.dsUnix;
}

uint8_t calendar::getDOW(void)
{
    return fechaNow.tm_wday;
}



/*
 ************************
 * Funciones auxiliares *
 ************************
 */
uint32_t calendar::dsDiff(struct fechaHora *fechHoraOld)
{
    if (fechaHoraNow.secsUnix < fechHoraOld->secsUnix)
    {
        // se ha debido cambiar la hora, machacamos la hora antigua con la actual
        fechHoraOld->dsUnix =  fechaHoraNow.dsUnix;
        fechHoraOld->secsUnix =  fechaHoraNow.secsUnix;
    }
    uint32_t ds = 10*(fechaHoraNow.secsUnix - fechHoraOld->secsUnix) + fechaHoraNow.dsUnix - fechHoraOld->dsUnix;
    return ds;
}

uint32_t calendar::sDiff(time_t *timetOld)
{
    if (fechaHoraNow.secsUnix < *timetOld)
    {
        // se ha debido cambiar la hora, machacamos la hora antigua con la actual
        *timetOld = fechaHoraNow.secsUnix;
    }
    return (fechaHoraNow.secsUnix - *timetOld);
}

uint32_t calendar::sDiff(struct fechaHora *fechHoraOld)
{
    if (fechaHoraNow.secsUnix < fechHoraOld->secsUnix)
    {
        // se ha debido cambiar la hora, machacamos la hora antigua con la actual
        fechHoraOld->dsUnix =  fechaHoraNow.dsUnix;
        fechHoraOld->secsUnix =  fechaHoraNow.secsUnix;
    }
    return (uint32_t) (fechaHoraNow.secsUnix - fechHoraOld->secsUnix);
}


uint8_t calendar::esDeNoche(void)
{
    uint16_t minutosNow;
    minutosNow = 60*fechaNow.tm_hour + fechaNow.tm_min;
    if (minutosNow<minAmanecer || minutosNow>minAnochecer)
    	return 1;
    else
    	return 0;
}

void calendar::printHoras(char *buff, uint16_t longBuff)
{
    uint8_t hAma,minAma,hNoche,minNoche;
    hAma = minAmanecer/60;
    minAma = minAmanecer - 60*hAma;
    hNoche = minAnochecer/60;
    minNoche = minAnochecer - 60*hNoche;
    chsnprintf(buff,longBuff,"%d:%02d-%d:%02d",hAma,minAma,hNoche,minNoche);
}


void calendar::printFecha(char *buff, uint16_t longBuff)
{
    chsnprintf(buff,longBuff,"%d/%d/%d %d:%d:%d",fechaNow.tm_mday,fechaNow.tm_mon+1,fechaNow.tm_year-100,fechaNow.tm_hour,fechaNow.tm_min,fechaNow.tm_sec);
}


void calendar::vuelcaFecha(void)
{
  char buff[80];
  printFecha(buff,sizeof(buff));
  printSerialCPP(buff);
}


void printFechaC(char *buff, uint16_t longBuff)
{
  calendar::printFecha(buff, longBuff);
}

void calendar::diSecAmanecerAnochecer(uint32_t *secActual, uint32_t *secAmanecer, uint32_t *secAnochecer)
{
    *secActual = 3600L*fechaNow.tm_hour+60L*fechaNow.tm_min + fechaNow.tm_sec;
    *secAmanecer = 60*minAmanecer;
    *secAnochecer = 60*minAnochecer;
}

void calendar::iniciaSecAdaptacion(void)
{
    uint32_t secActual, secAmanecer, secAnochecer;
    calendar::diSecAmanecerAnochecer(&secActual, &secAmanecer, &secAnochecer);
    secAdaptacion = (secAnochecer + 60*addAtardecer - secAmanecer - 60*addAmanecer)/2 - 1800;
    diaAdaptado = fechaNow.tm_yday;
}

void iniciaSecAdaptacionC(void)
{
    calendar::iniciaSecAdaptacion();
}

// Las horas de despertar son (considerando addAmanecer, addAtardecer):
// - Amanecer: secAmanecer + 60*addAmanecer
// - Atardecer: secAnochecer + 60*addAtardecer
// Si hay adaptacion, los secAdaptacion es el tiempo adicional que permanece abierta:
// - Se calcula tiempo que estaria normalmente cerrada (secAnochecer + 60*addAtardecer - secAmanecer - 60*addAmanecer)
// - La mitad del tiempo (-1800) se anyade al amanecer y la otra mitad se descuenta del atardecer
// - Ese es el maximo secAdaptacion, que ira disminuyendo 1800s cada dia
//
void calendar::estadoDeseadoPuerta(uint8_t *estDes, uint32_t *sec2change)
{
    uint32_t secActual, secAmanecer, secAnochecer, secsProxCambio;
    uint8_t momentoDia = 0; // 1 antesAmanecer, 2 despues de amanecer, 3 despues de anochecer
    calendar::diSecAmanecerAnochecer(&secActual, &secAmanecer, &secAnochecer);
    if (secAdaptacion>0 && diaAdaptado != fechaNow.tm_yday)
    {
        secAdaptacion -= 1800;
        if (secAdaptacion < 1800)
            secAdaptacion = 0;
        diaAdaptado = fechaNow.tm_yday;
    }
    if (secActual<secAmanecer + 60*addAmanecer + secAdaptacion)  // antes de amanecer
    {
        secsProxCambio = secAmanecer + 60*addAmanecer + secAdaptacion - secActual + 45;
        momentoDia = 1;
    }
    else if (secActual<secAnochecer + 60*addAtardecer - secAdaptacion) // de dia
    {
        secsProxCambio = secAnochecer + 60*addAtardecer - secAdaptacion - secActual + 45;
        momentoDia = 2;
    }
    else // ya de noche, prox cambio al amanecer del dia siguiente
    {
        secsProxCambio = (86400L-secActual + secAmanecer + 60*addAmanecer + secAdaptacion+ 45L);
        momentoDia = 3;
    }
    *sec2change = secsProxCambio;
    // ponemos estado de puerta
    if (autoPuerta==0)
        *estDes = 0;
    else if (autoPuerta==1)
        *estDes = 1;
    else if (autoPuerta==2 || autoPuerta==3)
    {
        if (momentoDia == 2)
            *estDes = 0;
        else
            *estDes = 1;
    }
}


void estadoDeseadoPuertaC(uint8_t *estDes, uint32_t *sec2change)
{
    calendar::estadoDeseadoPuerta(estDes, sec2change);
}

