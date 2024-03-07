/*
 * LCD.c
 *
 *  Created on: 11/4/2016
 *      Author: joaquin
 */

// LETARTARE for LCD 20x4 Sainsmart

#include "ch.hpp"
#include "hal.h"

extern "C" {
  #include "string.h"
  #include "chprintf.h"
  #include "memstreams.h"
  void ponEnLCDC(uint8_t fila, char const msg[]);
  int chLcdprintfFilaC(uint8_t fila, const char *fmt, ...);
}


using namespace chibios_rt;

#include "lcd.h"

// global variables
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _backlightval;

  uint8_t imagenLcd[LCD_ROWS][LCD_COLS];
  uint8_t lcdCol, lcdRow;

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).


void updateCarLcd(uint8_t fila, uint8_t col, uint8_t nuevoValor)
{
  if (imagenLcd[fila][col] != nuevoValor)
  {
    imagenLcd[fila][col] = nuevoValor;
    lcd_setCursor(col,fila);
    lcd_send(imagenLcd[fila][col],1);
  }
  lcdRow = fila;
  lcdCol = col;
  lcd_setCursor(col,fila);
}

void updateFilaLcd(uint8_t fila, uint8_t *nuevosValores)
{
  int8_t i, col, posDiferente, posUltDif;
  posDiferente = -1;
  posUltDif = -1;
  for (col=0;col<LCD_COLS;col++)
  {
    if (imagenLcd[fila][col]!=nuevosValores[col])
    {
      if (posDiferente < 0) posDiferente = col;
      posUltDif = col;
      imagenLcd[fila][col] = nuevosValores[col];
    }
  }
  if (posDiferente<0) // si no hay cambios
    return;
  lcd_setCursor(posDiferente,fila);
  for (i=posDiferente;i<=posUltDif;i++)
    lcd_send(imagenLcd[fila][i],1);
}

void updateClearafterLenFilaLcd(uint8_t fila, uint8_t colMax, uint8_t *nuevosValores)
{
  int8_t i, col, posDiferente, carDeseado;
  posDiferente = -1;
  for (col=0;col<LCD_COLS;col++)
  {
    if (col<colMax)
      carDeseado =  nuevosValores[col];
    else
      carDeseado = ' ';
    if (imagenLcd[fila][col]!=carDeseado)
    {
      if (posDiferente < 0) posDiferente = col;
      imagenLcd[fila][col] = carDeseado;
    }
  }
  if (posDiferente<0) // si no hay cambios
    return;
  lcd_setCursor(posDiferente,fila); // desde donde hay diferencias
  for (i=posDiferente;i<LCD_COLS;i++)
    lcd_send(imagenLcd[fila][i],1);
}


void ponEnLCD(uint8_t fila, char const msg[])
{
	uint8_t i, buffer[20];
	lcd_setCursor(0,fila);
	for (i=0;msg[i]!=0&&i<LCD_COLS;i++)
		buffer[i] = msg[i];
	for (;i<LCD_COLS;i++)
		buffer[i] = ' ';
	updateFilaLcd(fila,buffer);
//	for (i=0;i<LCD_COLS;i++)
//	{
//		lcd_send(buffer[i],1);
//	}
}
void ponEnLCDC(uint8_t fila, char const msg[])
{
    ponEnLCD(fila, msg);
}

void lcd_begin(uint8_t dotsize, uint8_t _displayfunction) {
    uint8_t f,c;
	if (LCD_ROWS > 1) {
		_displayfunction |= LCD_2LINE;
	}

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (LCD_ROWS == 1)) {
		_displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending lcd_commands. Arduino can turn on way befer 4.5V so we'll wait 50
	osalThreadSleepMilliseconds(50);

	// Now we pull both RS and R/W low to begin lcd_commands
	lcd_expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	osalThreadSleepMilliseconds(1);//delay(1000);

  	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	// we start in 8bit mode, try to set 4 bit mode
	lcd_write8bits(0,0,0x03);
	osalThreadSleepMilliseconds(5); // wait min 4.1ms
	// second try
	lcd_write8bits(0,0,0x03);
	osalThreadSleepMilliseconds(5); // wait min 4.1ms
	// third go!
	lcd_write8bits(0,0,0x03);
	osalThreadSleepMilliseconds(5); // wait min 4.1ms
	// finally, set to 4-bit interface
	lcd_write8bits(0,0,0x02);
	osalThreadSleepMilliseconds(5); // delayMicroseconds(150);

	// set # lines, font size, etc.
	lcd_command(LCD_FUNCTIONSET | _displayfunction);
	//osalThreadSleepMilliseconds(5);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_display();
	osalThreadSleepMilliseconds(5);

	// clear it off
	lcd_clear();

	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	lcd_command(LCD_ENTRYMODESET | _displaymode);
	//osalThreadSleepMilliseconds(5);

	lcd_home();

	lcdCol = 0;
	lcdRow = 0;
	for (f=0;f<LCD_ROWS;f++)
	  for (c=0;c<LCD_COLS;c++)
	    imagenLcd[f][c] = ' ';
	//osalThreadSleepMilliseconds(5);
}


void lcd_I2Cinit(void)
{
  uint8_t displayfunction;
  _backlightval = LCD_NOBACKLIGHT;
  displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  lcd_begin(LCD_5x8DOTS, displayfunction);
}

/********** high level lcd_commands, for the user! */
void lcd_clear(void){
  uint8_t f,c;
	lcd_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
    lcdCol = 0;
    lcdRow = 0;
    for (f=0;f<LCD_ROWS;f++)
      for (c=0;c<LCD_COLS;c++)
        imagenLcd[f][c] = ' ';
	osalThreadSleepMilliseconds(4);//delayMicroseconds(2000);  // this lcd_command takes a long time!
}

void lcd_home(void){
	lcd_command(LCD_RETURNHOME);  // set cursor position to zero
    lcdCol = 0;
    lcdRow = 0;
	osalThreadSleepMilliseconds(4);//delayMicroseconds(2000);  // this lcd_command takes a long time!
}

void lcd_setCursor(uint8_t col, uint8_t row){
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    // si el cursor esta activo, lo actualizo en LCD
//    if (_displaycontrol & LCD_CURSORON)
//    {
      if ( row >= LCD_ROWS ) {
          row = LCD_ROWS-1;    // we count rows starting w/0
      }
      lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
//    }
	lcdCol = col;
	lcdRow = row;
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
	_displaycontrol &= ~LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display(void) {
	_displaycontrol |= LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
	_displaycontrol &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor(void) {
	_displaycontrol |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
	_displaycontrol &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_blink(void) {
	_displaycontrol |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These lcd_commands scroll the display without changing the RAM
void lcd_scrollDisplayLeft(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scrollDisplayRight(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

//// Allows us to fill the first 8 CGRAM locations
//// with custom characters
//void lcd_createChar(uint8_t location, uint8_t charmap[]) {
//	uint8_t i;
//	location &= 0x7; // we only have 8 locations 0-7
//	lcd_command(LCD_SETCGRAMADDR | (location << 3));
//	for (i=0; i<8; i++) {
//		lcd_write(charmap[i]);
//	}
//}

// Turn the (optional) backlight off/on
void lcd_noBacklight(void) {
	_backlightval=LCD_NOBACKLIGHT;
	lcd_expanderWrite(0);
}

void lcd_backlight(void) {
	_backlightval=LCD_BACKLIGHT;
	lcd_expanderWrite(0);
}



/*********** mid level lcd_commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
	lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
	lcd_send(value, Rs);
	return 0;
}



/************ low level data pushing lcd_commands **********/

// write either lcd_command or data
void lcd_send(uint8_t value, uint8_t mode) {
/// LETARTARE
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value<<4)&0xf0;
    ///    uint8_t highnib=value>>4;
    ///    uint8_t lownib=value & 0x0F;
/// <--
    lcd_write4bits((highnib)|mode);
	lcd_write4bits((lownib)|mode);
}

void lcd_write4bits(uint8_t value) {
	lcd_expanderWrite(value);
	lcd_pulseEnable(value);
}

uint8_t lcd_expanderWrite(uint8_t _data){
	uint8_t tx_data[2], rx_data[2];
	msg_t status;
	tx_data[0] = _data | _backlightval;
	systime_t tmo = TIME_MS2I(500); // TIME_INFINITE; //
	i2cAcquireBus(&I2CD1);
	status = i2cMasterTransmitTimeout(&I2CD1, LCD_ADDR, tx_data, 1 , rx_data, 0, tmo);
	i2cReleaseBus(&I2CD1);
	return status;
}

void lcd_pulseEnable(uint8_t _data){
	lcd_expanderWrite(_data | En);	// En high
	lcd_expanderWrite(_data & ~En);	// En low
}

void lcd_write8bits(uint8_t rs, uint8_t backlight, uint8_t data4bits){
	uint8_t data = rs | (backlight<<3) | (data4bits<<4);
	lcd_expanderWrite(data);
	lcd_expanderWrite(data | En);	// En high
	lcd_expanderWrite(data);
}


// Alias functions

void lcd_cursor_on(void){
	lcd_cursor();
}

void lcd_cursor_off(void){
	lcd_noCursor();
}

void lcd_blink_on(void){
	lcd_blink();
}

void lcd_blink_off(void){
	lcd_noBlink();
}

void lcd_load_custom_character(uint8_t char_num, uint8_t *rows){
	lcd_createChar(char_num, rows);
}

void lcd_setBacklight(uint8_t new_val){
	if(new_val){
		lcd_backlight();		// turn backlight on
	}else{
		lcd_noBacklight();		// turn backlight off
	}
}




int chLcdprintf(const char *fmt, ...) {
  va_list ap;
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;
  char buffer[25],*ptrBuffer;
  int retval;

  if (sizeof(buffer) > 0)
    size_wo_nul = sizeof(buffer) - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  msObjectInit(&ms, (uint8_t *)buffer, size_wo_nul, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)(void *)&ms;
  va_start(ap, fmt);
  retval = chvprintf(chp, fmt, ap);
  va_end(ap);

  /* Terminate with a zero, unless size==0.*/
  if (ms.eos < sizeof(buffer))
      buffer[ms.eos] = 0;


  ptrBuffer = buffer;
  while (*ptrBuffer)
  {
    updateCarLcd(lcdRow, lcdCol, *ptrBuffer);
    lcdCol++;
    ptrBuffer++;
  }

  /* Return number of bytes that would have been written.*/
  return retval;
}


int chLcdprintfFila(uint8_t fila, const char *fmt, ...) {
  va_list ap;
  MemoryStream ms;
  BaseSequentialStream *chp;
  size_t size_wo_nul;
  uint8_t buffer[25];
  int retval,i;

  if (sizeof(buffer) > 0)
    size_wo_nul = sizeof(buffer) - 1;
  else
    size_wo_nul = 0;

  /* Memory stream object to be used as a string writer, reserving one
     byte for the final zero.*/
  msObjectInit(&ms, buffer, size_wo_nul, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)(void *)&ms;
  va_start(ap, fmt);
  retval = chvprintf(chp, fmt, ap);
  va_end(ap);

  /* limpiar hasta final de fila */
  for (i=ms.eos;i<LCD_COLS;i++)
      buffer[i] = ' ';
//  lcd_setCursor(0,fila);
//  for (i=0;i<LCD_COLS;i++)
//      lcd_send(buffer[i],1);
  updateFilaLcd(fila, buffer);

  /* Return number of bytes that would have been written.*/
  return retval;
}

int chLcdprintfFilaC(uint8_t fila, const char *fmt, ...)
{
    va_list ap;
    MemoryStream ms;
    BaseSequentialStream *chp;
    size_t size_wo_nul;
    uint8_t buffer[25];
    int retval,i;

    if (sizeof(buffer) > 0)
      size_wo_nul = sizeof(buffer) - 1;
    else
      size_wo_nul = 0;

    /* Memory stream object to be used as a string writer, reserving one
       byte for the final zero.*/
    msObjectInit(&ms, buffer, size_wo_nul, 0);

    /* Performing the print operation using the common code.*/
    chp = (BaseSequentialStream *)(void *)&ms;
    va_start(ap, fmt);
    retval = chvprintf(chp, fmt, ap);
    va_end(ap);

    /* limpiar hasta final de fila */
    for (i=ms.eos;i<LCD_COLS;i++)
        buffer[i] = ' ';
  //  lcd_setCursor(0,fila);
  //  for (i=0;i<LCD_COLS;i++)
  //      lcd_send(buffer[i],1);
    updateFilaLcd(fila, buffer);

    /* Return number of bytes that would have been written.*/
    return retval;
}

void lcd_CustomChars(void)
{
    lcd_command(0x40); // Set pointer to CGRAM
    lcd_send(0b00001,1); // simbolo ok
    lcd_send(0b00001,1);
    lcd_send(0b00010,1);
    lcd_send(0b00010,1);
    lcd_send(0b00100,1);
    lcd_send(0b10100,1);
    lcd_send(0b10100,1);
    lcd_send(0b01000,1);

    lcd_send(0b00000,1);    // segundo caracter: insertar caracter
    lcd_send(0b00000,1);
    lcd_send(0b00000,1);
    lcd_send(0b00000,1);
    lcd_send(0b00000,1);
    lcd_send(0b00000,1);
    lcd_send(0b10001,1);
    lcd_send(0b11111,1);

    lcd_send(0b00010,1);    // tercer caracter: suprimir caracter
    lcd_send(0b00100,1);
    lcd_send(0b01000,1);
    lcd_send(0b11111,1);
    lcd_send(0b11111,1);
    lcd_send(0b01000,1);
    lcd_send(0b00100,1);
    lcd_send(0b00010,1);

    lcd_send(0b00100,1);    // cuarto caracter: antena
    lcd_send(0b00100,1);
    lcd_send(0b00100,1);
    lcd_send(0b00100,1);
    lcd_send(0b00100,1);
    lcd_send(0b00100,1);
    lcd_send(0b00100,1);
    lcd_send(0b11111,1);
}
