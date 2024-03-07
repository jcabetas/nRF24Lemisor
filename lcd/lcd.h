/*
 * LCD.h
 *
 *  Created on: 11/4/2016
 *      Author: joaquin
 */

#ifndef LCD_H_
#define LCD_H_

#include <inttypes.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

#define LCD_ADDR 0x3F //0x27 // para el PCF8574AT ser�a 3F (0111111)
#define LCD_COLS 20
#define LCD_ROWS  4

void lcd_I2Cinit(void);
void lcd_begin(uint8_t dotsize, uint8_t _displayfunction);
void lcd_clear(void);
void lcd_home(void);
void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_scrollDisplayLeft(void);
void lcd_scrollDisplayRight(void);
void lcd_printLeft(void);
void lcd_printRight(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_shiftIncrement(void);
void lcd_shiftDecrement(void);
void lcd_noBacklight(void);
void lcd_backlight(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t);
void lcd_command(uint8_t);
//inline size_t lcd_write(uint8_t value);
void lcd_init(void);

////compatibility API function aliases
void lcd_blink_on(void);						// alias for blink(void)
void lcd_blink_off(void);       					// alias for noBlink(void)
void lcd_cursor_on(void);      	 					// alias for cursor(void)
void lcd_cursor_off(void);      					// alias for noCursor(void)
void lcd_setBacklight(uint8_t new_val);				// alias for backlight(void) and nobacklight(void)
void lcd_load_custom_character(uint8_t char_num, uint8_t *rows);	// alias for createChar(void)
void ponEnLCD(uint8_t fila, char const msg[]);

void arrancaIddleLcd(void);
void paraIddleLcd(void);
void paraLCD(void);

void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
uint8_t lcd_expanderWrite(uint8_t);
void lcd_pulseEnable(uint8_t);
void lcd_write8bits(uint8_t rs, uint8_t backlight, uint8_t data4Upperbits);
int16_t leeStringLCD(char *msg, uint8_t soloNumeros, uint8_t *bufferPar, uint16_t bufferParSize);
int chLcdprintf(const char *fmt, ...);
int chLcdprintfFila(uint8_t fila, const char *fmt, ...);
void updateCarLcd(uint8_t fila, uint8_t col, uint8_t nuevoValor);
void updateFilaLcd(uint8_t fila, uint8_t *nuevosValores);
void updateClearafterLenFilaLcd(uint8_t fila, uint8_t colMax, uint8_t *nuevosValores);
void lcd_CustomChars(void);
#endif /* LCD_H_ */
