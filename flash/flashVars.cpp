/*
 * flashVars.c
 *
 *  Created on: 27/03/2013
 *      Author: joaquin
 */

#include "ch.h"
#include "hal.h"
#undef TRUE
#undef FALSE
#include "flash.h"
#include "flashVars.h"
#include "tty.h"
#include "hal.h"
#include "chprintf.h"

#define tty1 (BaseSequentialStream *)&SD2

// prototipos locales
void reestructuraVars(void);
uint16_t *buscaPosVariable(uint16_t tipoVariable, int numPosicion, int *error);
uint16_t *direccionLibre(int *error, int numHWLibres);

/*
 * Las variables se estructuran de la siguiente forma:
 * - 2 bytes indicando:
 *   tipo de variable en los tres bits superiores
 *      000 => no hay nada, 001 => entero 2 bytes, 010 => entero 4 bytes, 011 => string (terminado en cero)
 *      100 => uint16_t, 101 => int16_t
 *      111 => fin de datos
 *   numero de variable en los 14 bits mas bajos (0 si no hay nada)
 * - Valor de la variable
 */

void muestraMemoria(void)
{
    int bytesEnLinea,dato;
    uint16_t *direccion,*direccionFinal;
    direccion = (uint16_t *) StartAddrVar;
    direccionFinal = direccion + 0x80;//(uint16_t *) EndAddrVar;
    bytesEnLinea = 0;
    chprintf(tty1,"Position of memory variables: 0x%X..0x%X:\r\n",direccion,direccionFinal);
    while (1==1)
    {
        if (bytesEnLinea == 0) chprintf(tty1,"0x%08X: ",direccion);
        dato = (int) (*direccion);
        chprintf(tty1,"%04X ",dato);
        direccion++;
        if (++bytesEnLinea >= 10)
        {
        	chprintf(tty1,"\r\n");
            chThdSleepMilliseconds(10);
            bytesEnLinea = 0;
            if (direccion > direccionFinal || dato == 0xFFFF) break;
        }
    }
}

uint16_t *buscaDirVariable(uint16_t tipoVariable,int numPosicion, int *error)
{
    // devuelve el Half-word con la cabecera encontrada
    // si tipoVariable>0, exige que coincida tipo de variable
    uint16_t *direccion,*direccionFinal, cabVar;
    uint16_t dato, posicion;
    if (numPosicion > 0x1FF)  // solo entran 511 variables en los 16 bits inferiores de la cabecera
    {
        *error = 3;
        return 0;
    }
    direccion = (uint16_t *) StartAddrVar;
    direccionFinal = (uint16_t *) EndAddrVar;
    posicion = (uint16_t) numPosicion;
    while (direccion < direccionFinal)
    {
        cabVar  = ((*direccion) >> 13) & 0x7;
        if (cabVar == 0x7) // fin de datos sin encontrarlo
        {
            *error = 1;
            return direccion;
        }
        if (cabVar == 0x0) // trozo en blanco (esta borrado)
        {
            direccion++;  // salto el valor
            continue;
        }
        if (cabVar == 0x1 || cabVar == 0b100 || cabVar == 0b101) // es un entero de dos bytes, salto la cabecera y el dato
        {
            if ((tipoVariable==0 || tipoVariable == cabVar) && posicion == ((*direccion) & 0x1FF))
            {
                *error = 0;
                return direccion;
            }
            direccion++;
            direccion++;
        }
        if (cabVar == 0x2) // es un entero de 4 bytes, salto la cabecera y el dato
        {
            if (tipoVariable == 0x2 && posicion == ((*direccion) & 0x1FF))
            {
                *error = 0;
                return direccion;
            }
            direccion++;  // salto la cabecera
            direccion++;  // salto el primer half-word del dato
            direccion++;  // salto el segundo
            continue;
        }
        if (cabVar == 0x3) // es un string
        {
            if (tipoVariable == 0x3 && posicion == ((*direccion) & 0x1FF))
            {
                *error = 0;
                return direccion;
            }
            direccion++;  // salto la cabecera
            while (direccion < direccionFinal)  // idem datos
            {
                dato = *direccion;
                direccion++;
                if ((dato & 0xF0) == 0 || ((dato & 0xF) == 0)) break;
            }
            continue;
        }
    }
    // he llegado al final de memoria sin encontrar nada, y ya no hay espacio libre
    *error = 3;
    return direccion;
}


void rastreaVariables(void)
{
    // devuelve el Half-word con la cabecera encontrada
    uint16_t *direccion,*direccionFinal, cabVar;
    uint16_t dato, posicion;
    char ch;
    int valor;
    direccion = (uint16_t *) StartAddrVar;
    direccionFinal = (uint16_t *) EndAddrVar;
    chprintf(tty1,"variable lookup:\r\n");
    while (direccion < direccionFinal)
    {
        cabVar  = ((*direccion) >> 13) & 0x7;
        if (cabVar == 0x7) // fin de datos sin encontrarlo
        {
        	chprintf(tty1,"- End of variables found\r\n");
            return;
        }
        if (cabVar == 0x0) // trozo en blanco
        {
            direccion++;  // salto a la siguiente posicion
            continue;
        }
        if (cabVar == 0x1) // es un entero de dos bytes, salto la cabecera y el dato
        {
            posicion = ((*direccion) & 0x1FF);
            direccion++;
            valor = (int) *direccion;
            direccion++;
            chprintf(tty1,"- Variable type '2 bytes', at position %d, value %d\r\n",posicion,valor);
            continue;
        }
        //100 => uint16_t, 101 => int16_t
        if (cabVar == 0b100) // es uint16_t
        {
            posicion = ((*direccion) & 0x1FF);
            direccion++;
            uint16_t valoru16_t = (uint16_t) *direccion;
            direccion++;
            chprintf(tty1,"- Variable type uint16_t', at position %d, value %d\r\n",posicion,valoru16_t);
            continue;
        }
        if (cabVar == 0b101) // es int16_t
        {
            posicion = ((*direccion) & 0x1FF);
            direccion++;
            int16_t valori16_t = (int16_t) *direccion;
            direccion++;
            chprintf(tty1,"- Variable type int16_t', at position %d, value %d\r\n",posicion,valori16_t);
            continue;
        }
        if (cabVar == 0x2) // es un entero de 4 bytes, salto la cabecera y el dato
        {
            posicion = ((*direccion) & 0x1FF);
            direccion++;
            valor = (((int) *direccion) << 16);  // primero upper
            direccion++;
            valor += ((int) *direccion); // ahora lower
            direccion++;
            chprintf(tty1,"- Variable type '4 bytes', at position %d, value 0x%X\r\n",posicion,valor);
            continue;
        }
        if (cabVar == 0x3) // es un string
        {
            posicion = ((*direccion) & 0x1FF);
            direccion++;  // salto la cabecera
            chprintf(tty1,"- Variable type 'string', at position %d, value \"",posicion);
            while (direccion < direccionFinal)  // idem datos
            {
                dato = *direccion;
                direccion++;
                ch = (dato & 0xFF00)>>8;
                if (ch!=0)
                {
                	chprintf(tty1,"%c",ch);
                    ch = dato & 0xFF;
                    if (ch!=0)
                    {
                    	chprintf(tty1,"%c",ch);
                        continue;
                    }
                }
                if ((dato & 0xFF00) == 0 || ((dato & 0xFF) == 0)) break;
            }
            chprintf(tty1,"\"\r\n");
            continue;
        }
    }
    // he llegado al final de memoria sin encontrar nada, y ya no hay espacio libre
}

// W25Q16_write_i16(0, POS_ADDAMANECER, 30);
// W25Q16_read_i16(0, POS_ADDAMANECER);

int16_t uC_read_i16(int numPosicion, int16_t valorDefecto, uint16_t *error)
{
    uint16_t *direccion;
    int errorBusca;
    //  100 => uint16_t, 101 => int16_t
    direccion = buscaDirVariable(0b101, numPosicion, &errorBusca);
    direccion++;
    if (errorBusca == 0)
        return (int16_t) *direccion;
    *error = 1;
    return valorDefecto;
}

uint16_t uC_read_u16(int numPosicion, uint16_t valorDefecto, uint16_t *error)
{
    uint16_t *direccion;
    int errorBusca;
    direccion = buscaDirVariable(0b100, numPosicion, &errorBusca);
    direccion++;
    if (errorBusca == 0)
        return (uint16_t) *direccion;
    *error = 1;
    return valorDefecto;
}

//int leeVariable2bytes(int numPosicion, int valorDefecto, int *error)
//{
//    uint16_t *direccion;
//    direccion = buscaPosVariable(0x1, numPosicion, error);
//    direccion++;
//    if (*error == 0)
//        return (int) *direccion;
//    return valorDefecto;
//}

int leeVariable4bytes(int numPosicion, int valorDefecto, int *error)
{
    uint16_t *direccion;
    int valorDevuelto;
    direccion = buscaPosVariable(0x2, numPosicion, error);
    direccion++;
    if (*error == 0)
    {
        valorDevuelto = (((int) *direccion) << 16);  // primero upper
        direccion++;
        valorDevuelto += ((int) *direccion); // ahora lower
        return valorDevuelto;
    }
    return valorDefecto;
}

void leeVariableString(char *buffer, int bufferSize, int numPosicion, char *valorDefecto, int *error)
{
    uint16_t *direccion,*direccionFinal,dato;
    int numCharEscritos;
    char caracter;

//    chprintf(tty1,"Estoy en leeVariableString, numPos:%d\n\r",numPosicion);
//    chThdSleepMilliseconds(200);
    direccion = buscaPosVariable(0x3, numPosicion, error);
//    chprintf(tty1,"Direccion:%d error:%d\n\r",direccion,*error);
//    chThdSleepMilliseconds(200);
    direccion++;
    numCharEscritos = 0;
    if (*error == 0)
    {
        direccionFinal = (uint16_t *) EndAddrVar;
        while (direccion<direccionFinal)
        {
            if (++numCharEscritos > bufferSize) return;
            dato = *direccion;
            caracter = (char) ((dato & 0xFF00) >> 8);
            *buffer = caracter;
            if (!caracter) return;
            if (++numCharEscritos > bufferSize) return;
            buffer++;
            caracter = (char) (dato & 0xFF);
            *buffer = caracter;
            if (!caracter) return;
            buffer++;
            direccion++;
        }
        *error = 4; // fin de memoria alcanzada
        return;
    }
    // escribo valor de defecto
    while (numCharEscritos < bufferSize)
    {
        *buffer = *valorDefecto;
        if (!(*valorDefecto)) return;
        buffer++;
        valorDefecto++;
        numCharEscritos++;
    }
}


uint16_t *direccionLibre(int *error, int numHWLibres)
{
    uint16_t *direccion,*direccionFinal;
    int heReorganizado = 0, necesitoReorganizar;
    direccion = (uint16_t *) StartAddrVar;
    direccionFinal = (uint16_t *) EndAddrVar;
    while (1==1)
    {
        if (*direccion == 0xFFFF && (direccion+numHWLibres < (uint16_t *) EndAddrVar))
        {
            *error = 0;
            return direccion;
        }
        if (heReorganizado && (direccion+numHWLibres >= (uint16_t *) EndAddrVar))
        {
            chprintf(tty1,"Critical error: variable memory with no space available after optimizing\r\n");
            *error = 5;
            return direccion;
        }
        necesitoReorganizar = 0;
        if (*direccion == 0xFFFF && (direccion+numHWLibres >= (uint16_t *) EndAddrVar))
            necesitoReorganizar = 1;
        direccion++;
        if (direccion >= direccionFinal)
            necesitoReorganizar = 1;
        if (necesitoReorganizar && heReorganizado)
        {
        	chprintf(tty1,"Critical error: variable memory with no space available\r\n");
            *error = 6;
            return direccion;
        }
        if (necesitoReorganizar)
        {
        	chprintf(tty1,"variable memory with no space available, optimizing..\r\n");
            reestructuraVars();
            heReorganizado = 1;
            direccion = (uint16_t *) StartAddrVar;  // vuelvo a intentarlo
        }
    }
}

void escribeVariable2bytes(int numPosicion, int valor, int *error)
{
    uint16_t *direccion,dato;
    int errorDL;
    direccion = buscaPosVariable(0x1, numPosicion, error);
    if (*error == 2) return;  // ni se ha encontrado ni hay posiciones libres (en el futuro, limpiar)
    if (*error == 0)  // variable encontrada
    {
        direccion++;
        if (((int) *direccion) == valor)  // si ya tiene el valor deseado, vuelvo
        {
            *error = 0;
            return;
        }
        direccion--;
        // borro valor antiguo, poniendoloa cero
        programaHalfWord(direccion++,(uint16_t) 0);  // tanto la cabecera
        programaHalfWord(direccion,(uint16_t) 0);    // como el propio dato
    }
    direccion = direccionLibre(&errorDL,2);   // busco posicion valida
    *error = errorDL;
    if (errorDL != 0)
        return;
    dato = (0x1<<13) | numPosicion;
    programaHalfWord(direccion++,dato);         // programa nueva cabecera
    programaHalfWord(direccion,(uint16_t) (valor & 0xFFFF));  // programa valor de la variable
}

//  W25Q16_write_i16(0, POS_ADDAMANECER, addAmanecer);
// *      100 => uint16_t, 101 => int16_t
void uC_write_u16(int numPosicion, uint16_t valor, int *error)
{
    uint16_t *direccion,dato;
    int errorDL;
    direccion = buscaDirVariable(0b100, numPosicion, error);
    if (*error == 2) return;  // ni se ha encontrado ni hay posiciones libres (en el futuro, limpiar)
    if (*error == 0)  // variable encontrada
    {
        direccion++;
        if (((uint16_t) *direccion) == valor)  // si ya tiene el valor deseado, vuelvo
        {
            *error = 0;
            return;
        }
        direccion--;
        // borro valor antiguo, poniendoloa cero
        programaHalfWord(direccion++,(uint16_t) 0);  // tanto la cabecera
        programaHalfWord(direccion,(uint16_t) 0);    // como el propio dato
    }
    direccion = direccionLibre(&errorDL,2);   // busco posicion valida
    *error = errorDL;
    if (errorDL != 0)
        return;
    dato = (0b100<<13) | numPosicion;
    programaHalfWord(direccion++,dato);         // programa nueva cabecera
    programaHalfWord(direccion,(uint16_t) (valor & 0xFFFF));  // programa valor de la variable
}

// *      100 => uint16_t, 101 => int16_t
void uC_write_i16(int numPosicion, uint16_t valor, int *error)
{
    uint16_t *direccion,dato;
    int errorDL;
    direccion = buscaDirVariable(0b101, numPosicion, error);
    if (*error == 2) return;  // ni se ha encontrado ni hay posiciones libres (en el futuro, limpiar)
    if (*error == 0)  // variable encontrada
    {
        direccion++;
        if (((int16_t) *direccion) == valor)  // si ya tiene el valor deseado, vuelvo
        {
            *error = 0;
            return;
        }
        direccion--;
        // borro valor antiguo, poniendoloa cero
        programaHalfWord(direccion++,(uint16_t) 0);  // tanto la cabecera
        programaHalfWord(direccion,(uint16_t) 0);    // como el propio dato
    }
    direccion = direccionLibre(&errorDL,2);   // busco posicion valida
    *error = errorDL;
    if (errorDL != 0)
        return;
    dato = (0b101<<13) | numPosicion;
    programaHalfWord(direccion++,dato);         // programa nueva cabecera
    programaHalfWord(direccion,(uint16_t) valor);  // programa valor de la variable
}


void escribeVariable4bytes(int numPosicion, int valor, int *error)
{
    uint16_t *direccion,dato;
    int errorDL,valorEnFlash;
    direccion = buscaPosVariable(0x2, numPosicion, error);
    if (*error == 2) return;  // ni se ha encontrado ni hay posiciones libres (en el futuro, limpiar)
    if (*error == 0)  // variable encontrada
    {
        direccion++;
        valorEnFlash = (((int) *direccion) << 16);  // primero upper
        direccion++;
        valorEnFlash += ((int) *direccion); // ahora lower
        if (valorEnFlash == valor)  // si ya tiene el valor deseado, vuelvo
        {
            *error = 0;
            return;
        }
        direccion--;
        direccion--;
        // borro valor antiguo, poniendola a cero
        programaHalfWord(direccion++,(uint16_t) 0);  // tanto la cabecera
        programaHalfWord(direccion++,(uint16_t) 0);  // como los datos
        programaHalfWord(direccion,(uint16_t) 0);
    }
    direccion = direccionLibre(&errorDL,3);   // busco posicion valida
    *error = errorDL;
    if (errorDL != 0)
        return;
    dato = (0x2<<13) | numPosicion;
    programaHalfWord(direccion++,dato);         // programa nueva cabecera
    programaHalfWord(direccion++,(uint16_t) ((valor & 0xFFFF0000)>>16));  // programa valor de la variable upper
    programaHalfWord(direccion,(uint16_t) (valor & 0xFFFF));        // programa valor de la variable lower
}


void escribeVariableString(int numPosicion, char *buffer, int bufferSize, int *error)
{
    uint16_t *direccion,*direccionBk,*direccionFinal;
    int errorDL,posCharBuff,dato;
    char ch,ch1,ch2;
    direccion = buscaPosVariable(0x3, numPosicion, error);
//    chprintf(tty1,"en escribeVarStr, direccion:%X error:%d\r\n",direccion,*error);
    direccionBk = direccion;
    if (*error == 2) return;  // ni se ha encontrado ni hay posiciones libres (en el futuro, limpiar)
    posCharBuff = 0;
    if (*error == 0)  // variable encontrada
    {
        direccion++;  // salto cabecera
        while (1==1)
        {
            if (posCharBuff >= bufferSize)
            {
                *error = 0;   // un poco raro, pues le falta el 0 final
                return;
            }
            ch = buffer[posCharBuff++];
            ch1 = (*direccion & 0xFF00) >>8;
            if (ch != ch1)
                break;
            if (ch==0)
            {
                *error = 0;
                return;
            }
            if (posCharBuff >= bufferSize)
            {
                *error = 0;   // un poco raro, pues le falta el 0 final
                return;
            }
            ch = buffer[posCharBuff++];
            ch2 = *direccion & 0xFF;
            if (ch != ch2)
                break;
            if (ch==0)
            {
                *error = 0;
                return;
            }
            direccion++;
        }
        // son diferentes, hay que reescribir la variable
        // borro variable antigua
        direccion = direccionBk;
        programaHalfWord(direccion++,(uint16_t) 0);  // borro la cabecera
        direccionFinal = (uint16_t *) EndAddrVar;
        while (direccion<direccionFinal)
        {
            dato = *direccion;
            programaHalfWord(direccion++,(uint16_t) 0);  // borro este halfWord, despues de leerlo
            ch = (char) ((dato & 0xFF00) >> 8);
            if (!ch) break;
            ch = (char) (dato & 0xFF);
            if (!ch) break;
        }
    }
    // escribo variable nueva
    direccion = direccionLibre(&errorDL,2+posCharBuff/2);   // busco posicion valida
    *error = errorDL;
    if (errorDL != 0)
        return;
    dato = (0x3<<13) | numPosicion;
    programaHalfWord(direccion++,dato);         // programa nueva cabecera
    posCharBuff = 0;
    while (1)
    {
        if (posCharBuff>=bufferSize)
        {
            *error = 8;
            return;
        }
        dato = buffer[posCharBuff++]<<8;
        if (dato==0)
        {
            programaHalfWord(direccion++,dato);         // programa ultimo dato
            *error = 0;
            return;
        }
        if (posCharBuff>=bufferSize)
        {
            *error = 9;
            return;
        }
        ch2 = buffer[posCharBuff++];
        dato |= ch2;
        programaHalfWord(direccion++,dato);         // programa dato
        if (ch2==0)
        {
            *error = 0;
            return;
        }
    }
}


void reestructuraVars(void)
{
    uint16_t dato,datos[PageSize];
    int numDatos,i,HWinPage;
    uint16_t *direccion,*direccionPagInicial,*direccionFinal;
    direccionPagInicial = (uint16_t *) StartAddrVar;
    direccionFinal = (uint16_t *) EndAddrVar;
    HWinPage = PageSize>>1;
    chprintf(tty1,"Optimizing variables\r\n");
    numDatos = 0;
    for (direccion=direccionPagInicial;direccion<=direccionFinal;direccion++)
    {
        dato = (uint16_t) (*direccion);
        if (dato == 0) continue;
        datos[numDatos] = dato;
        numDatos++;
        if (numDatos == HWinPage)  // 2 bytes por HalfWord
        {
            flashErase((flashaddr_t)direccionPagInicial, PageSize);
            for (i=0;i<numDatos;i++)
                programaHalfWord((uint16_t *)(direccionPagInicial+i),datos[i]);
            direccionPagInicial += HWinPage;
            numDatos = 0;
        }
    }
    // borra páginas pendientes de borrar
    direccion = direccionPagInicial;
    while (1==1)
    {
        if (((uint32_t) direccion) >= EndAddrVar)
            break;
        flashErase((flashaddr_t)direccionPagInicial, PageSize);
        direccion += (PageSize>>1);
    }
    // ¿quedan variables por grabar?
    if (numDatos > 0)
    {
        for (i=0;i<numDatos;i++)
            programaHalfWord((uint16_t *)(direccionPagInicial+i),datos[i]);
    }

}

void borroMemoriaVars(void)
{
    uint32_t direccionPagInicial;
    direccionPagInicial = StartAddrVar;
    while (1==1)
    {
        if (direccionPagInicial >= EndAddrVar)
            break;
        flashErase((flashaddr_t)direccionPagInicial, PageSize);
        direccionPagInicial += PageSize;
    }
}

