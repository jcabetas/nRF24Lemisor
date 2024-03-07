/*
 * flashVars.h
 *
 *  Created on: 28 oct 2023
 *      Author: joaquin
 */

#ifndef FLASH_FLASHVARS_H_
#define FLASH_FLASHVARS_H_


    #define StartAddrVar  ((uint32_t) 0x08020000)
    #define EndAddrVar    ((uint32_t) 0x0803FFFF)
    #define PageSize   ((uint32_t)0x1FFFF)

    void leeVariablesSistemas(void);
    int leeVariable2bytes(int numPosicion, int valorDefecto, int *error);
    int16_t uC_read_i16(int numPosicion, int16_t valorDefecto, uint16_t *error);
    uint16_t uC_read_u16(int numPosicion, uint16_t valorDefecto, uint16_t *error);
    void uC_write_u16(int numPosicion, uint16_t valor, int *error);
    void uC_write_i16(int numPosicion, uint16_t valor, int *error);
    uint16_t *buscaDirVariable(uint16_t tipoVariable,int numPosicion, int *error);
    void menuVars(void);
    void initFrioVariablesSistemas(void);
    void muestraMemoria(void);
    void rastreaVariables(void);
    void escribeVariable2bytes(int numPosicion, int valor, int *error);
    void borroMemoriaVars(void);
    void ajustaRadio(void);
    void leeAjustesDeFlash(void);


#endif /* FLASH_FLASHVARS_H_ */
