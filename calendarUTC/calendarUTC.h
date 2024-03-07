#ifndef _CALENDAR_H_
#define _CALENDAR_H_

#include <ch.h>

#define SECONDS_IN_DAY 86400
#define DEFAULT_YEAR 2008
#define LEAP 1
#define NOT_LEAP 0

void completeYdayWday(struct tm *tim);
void rtcSetTM(RTCDriver *rtcp, struct tm *tim, uint16_t ds);
void rtcGetTM(RTCDriver *rtcp, struct tm *tim, uint16_t *ds);

enum getPeriodoTarifa { punta, llano, valle };

struct fechaHora
{
    time_t secsUnix;
    uint16_t dsUnix;
};

class calendar
{
private:
    static uint16_t minAmanecer, minAnochecer;
    static struct fechaHora fechaHoraNow;
    static struct tm fechaNow;
    static float latitudRad, longitudRad;
    static uint16_t diaCalculado;
public:
    // funciones necesarias para leer y ajustar fechas
    static void completeYdayWday(struct tm *tim);
    static time_t getSecUnix(struct tm *fecha);
    static void setLatLong(float latitudRad, float longitudRad);
    static void ajustaHorasLuz(void);
    // funciones de leey y ajustar fechas
    static void rtcGetFecha(void);
    static void rtcSetFecha(struct tm *fecha, uint16_t ds);
    static void cambiaFecha(uint16_t *anyo, uint8_t *mes, uint8_t *dia, uint8_t *hora, uint8_t *min, uint8_t *seg, uint8_t *ds);
    static void cambiaFechaTM(uint8_t anyo, uint8_t mes, uint8_t dia, uint8_t hora, uint8_t min, uint8_t seg, uint8_t dsPar);
    // estas funciones requieren haber hecho rtcGetFecha() previamente
    static time_t getSecUnix(void);
    static void gettm(struct tm *fecha);
    static void getFechaHora(struct fechaHora *fechaHora);
    static uint8_t getDOW(void);
    // funciones auxiliares
    static void iniciaSecAdaptacion(void);
    static uint32_t dsDiff(struct fechaHora *fechHoraOld);
    static uint32_t sDiff(time_t *timetOld);
    static uint32_t sDiff(struct fechaHora *fechaHora);
    static uint8_t esDeNoche(void);
    static void addDs(int16_t dsAdd);
    static void printHoras(char *buff, uint16_t longBuff);
    static void printFecha(char *buff, uint16_t longBuff);
    static void vuelcaFecha(void);
    static void diSecAmanecerAnochecer(uint32_t *secActual, uint32_t *secAmanecer, uint32_t *secAnochecer);
    static void estadoDeseadoPuerta(uint8_t *estDes, uint32_t *sec2change);
    static void updateUnixTime(void);
    static void enviaFechayHoraPorCAN(struct tm *fechaLocal, uint16_t ds);
    static void trataOrdenNextion(char *vars[], uint16_t numPars);
};


//void RTC_Configuration(void);
//uint16_t WeekDay(struct tm *d);
//uint8_t diasEnMes(uint8_t uint8_t_Month,uint16_t uint16_t_Year);
//uint32_t segundosEnAno(uint16_t uint16_t_Year);
//uint32_t gday(struct tm *d);
//uint8_t fechaLegal(struct tm *d);
//uint8_t esDeNoche(void);
//void d2f(uint32_t secs,struct tm *d);
//void today(struct tm *fecha);
//void incrementaFechaLocal1sec(void);
//void hallaSecsCambHorario(struct tm *fecha, uint32_t *secsInv2Ver, uint32_t *secsVer2Inv);
//void fechaUTM2Local(struct tm *fechaUTM, struct tm *fechaLocal);
//void leeFecha(void);
//void ponFechaLCD(void);
//void ponHoraLCD(void);
//void initCalendar(void);
//void arrancaCalendario(void);
//void actualizaDatosDiarios(void);
#endif
