/*
 * menuVars.c
 *
 *  Created on: 27/03/2013
 *      Author: joaquin
 */

#include <ch.h>
#include <hal.h>

#include "board.h"
#include "tty.h"
#include "gets.h"
#include "flash.h"
#include "flashVars.h"
#include "chprintf.h"

//#define (BaseSequentialStream *)ttyHM10 (BaseSequentialStream *)tty

void menuVars(void)
{
    int error;
    int16_t opcion,varI16,numVariable;
    while (1==1)
    {
        chprintf((BaseSequentialStream *)ttyHM10,"Variables:\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"1: Show memory\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"2: Dump variables\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"3: Write a variable u16\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"4: Write a variable i16\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"5: Read a variable u16/i16\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"6: Erase memory\r\n");
        chprintf((BaseSequentialStream *)ttyHM10,"0: Return\r\n");
        preguntaNumero((BaseChannel *) ttyHM10, "Opcion:", &opcion, 0, 6);
        error = 0;
        if (opcion == 1)
            muestraMemoria();
        if (opcion == 2)
            rastreaVariables();
        if (opcion == 3)   // Write a variable u16
        {
            preguntaNumero((BaseChannel *) ttyHM10, "#variable", &numVariable, 0, 99);
            preguntaNumero((BaseChannel *) ttyHM10, "Valor", &varI16, 0, 6700);
            uC_write_u16(numVariable, (uint16_t) varI16, &error);
            if (error)
            	chprintf((BaseSequentialStream *)ttyHM10,"Error %d while writing\r\n",error);
            else
            	chprintf((BaseSequentialStream *)ttyHM10,"\r\n");
        }
        if (opcion == 4)   // Write a variable i16
        {
            preguntaNumero((BaseChannel *) ttyHM10, "#variable", &numVariable, 0, 99);
            preguntaNumero((BaseChannel *) ttyHM10, "Valor", &varI16, -6000, 6600);
            uC_write_i16(numVariable, (int16_t) varI16, &error);
            if (error)
                chprintf((BaseSequentialStream *)ttyHM10,"Error %d while writing\r\n",error);
            else
                chprintf((BaseSequentialStream *)ttyHM10,"\r\n");
        }
        if (opcion == 5)  // Read a variable u16/i16
        {
            preguntaNumero((BaseChannel *) ttyHM10, "#variable", &numVariable, 0, 99);
            uint16_t *direccion = buscaDirVariable(0, numVariable, &error);
            if (error)
            {
                chprintf((BaseSequentialStream *)ttyHM10,"Error %d while reading variable\r\n");
                break;
            }
            uint8_t cabVar  = ((*direccion) >> 13) & 0x7;
            direccion++;
            if (cabVar==0b100)
                chprintf((BaseSequentialStream *)ttyHM10,"uint16_t: %d\r\n",(uint16_t) (*direccion));
            else if (cabVar==0b101)
                chprintf((BaseSequentialStream *)ttyHM10,"int16_t: %d\r\n",(int16_t) (*direccion));
            else
                chprintf((BaseSequentialStream *)ttyHM10,"Error, tipo de variable: %d\r\n",cabVar);
        }
        if (opcion == 6)
            borroMemoriaVars();
        if (opcion == 0) break;
        if (opcion < 0 || opcion>5)
        	chprintf((BaseSequentialStream *)ttyHM10,"Illegal option\r\n");
    }
}
