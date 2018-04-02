/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "opentx.h"

#if defined(SSD1306)

//The address (0x3C) is 011 1100. When we shift left for writing in I2C, it becomes 0111 1000
#define SSD1306_ADDRESS 0x78

void I2C_init()
{
  PORTD |= (1 << SDA); //Internal pullup resistors for SDA and SCL
  PORTD |= (1 << SCL);
  TWSR = 0x00; //Prescaler 0
  TWBR = 0x0C; //400kHz
  TWCR = (1<<TWEN); //Enabled
}

//Send start signal
void I2C_start()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0); //Wait for transaction to complete
}

//Send stop signal
void I2C_stop()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

//Write this byte to the I2C line
void I2C_write(uint8_t data)
{
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
}
#endif

void lcdSendCtl(uint8_t val)
{
#if defined(SSD1306)
  I2C_start();
  I2C_write(SSD1306_ADDRESS); 
  I2C_write(0x00); //Co 0; D/C 0
  I2C_write(val);
  I2C_stop();
#else
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_CS1);
  #if defined(LCD_MULTIPLEX)
    DDRA = 0xFF; //Set LCD_DAT pins to output
  #endif
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_A0);
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RnW);
  PORTA_LCD_DAT = val;
  PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_E);
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_E);
  PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
  #if defined(LCD_MULTIPLEX)
    DDRA = 0x00; //Set LCD_DAT pins to input
  #endif
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_CS1);
#endif
}

#if defined(PCBSTD) && defined(VOICE)
  volatile uint8_t LcdLock ;
  #define LCD_LOCK() LcdLock = 1
  #define LCD_UNLOCK() LcdLock = 0
#else
  #define LCD_LOCK()
  #define LCD_UNLOCK()
#endif

const static pm_uchar lcdInitSequence[] PROGMEM =
{
//ST7565 eq. : KS0713, SED1565, S6B1713, SPLC501C, NT7532 /34 /38, TL03245
#if defined(LCD_ST7565R)
   0xE2, //Initialize the internal functions
   0xAE, //DON = 0: display OFF
   0xA0, //ADC = 0: normal direction (SEG132->SEG1)
   0xA6, //REV = 0: non-reverse display
   0xA4, //EON = 0: normal display. non-entire
   0xA2, //Select LCD bias
   0xC8, //SHL = 1: reverse direction (COM64->COM1)
   0x2F, //Control power circuit operation VC=VR=VF=1
   0x25, //Select int resistance ratio R2 R1 R0 =5
   0x81, //Set reference voltage Mode
   0x22, //24 SV5 SV4 SV3 SV2 SV1 SV0 = 0x18
   0xAF, //DON = 1: display ON
   0x60  //Set the display start line to zero
#elif defined(LCD_ERC12864FSF)
   0xE2, //Initialize the internal functions
   0xAE, //DON = 0: display OFF
   0xA1, //ADC = 1: reverse direction (SEG132->SEG1)
   0xA6, //REV = 0: non-reverse display
   0xA4, //EON = 0: normal display. non-entire
   0xA3, //Select LCD bias
   0xC0, //SHL = 0: normal direction (COM1->COM64)
   0x2F, //Control power circuit operation VC=VR=VF=1
   0x27, //Select int resistance ratio R2 R1 R0
   0x81, //Set reference voltage Mode
   0x2D, //24 SV5 SV4 SV3 SV2 SV1 SV0
   0xAF  //DON = 1: display ON
#elif defined(LCD_ST7920)
   0x30, //Set 8-bit interface
   0x36, //Repeat with graphics bit set to ON
   0x0C, //Display ON, cursor and blink OFF
   0x01, //Clear display, reset address
   0x06  //Display ON, no cursor
#elif defined(SSD1306)
  0xAE,       //Display off
  0xD5, 0x80, //Set internal oscillator to default
  0xA8, 0x3F, //Set multiplex mode to 63 (i.e., 64 lines)
  0xD3, 0x00, //Display offset 0
  0x8D, 0x14, //Enable internal DC/DC converter 
  0x20, 0x00, //Set page addressing mode
  0xDA, 0x12, //Set COM pins to match OLED panel
  0xD9, 0xF1, //Precharge for 15 clocks, discharge 1
  0xDB, 0x40, //Set VCOMH to 0x4 - undocumented
  #if SSD1306_INVERT
    0xA0,     //0 -> 127
    0xC0,     //0 -> 63
  #else
    0xA1,     //127 -> 0
    0xC8,     //63  -> 0
  #endif
  0x40,       //Start on line 0
  0x81, 0x7F, //Full contrast
  0xA4,       //Read display from RAM (rather than 0xA5 - all on)
  0xA6,       //Normal display polarity
  0x2E,       //Scrolling off
  0xAF        //Display on
#else    //ST7565P (default 9x LCD)
   0xE2, //Initialize the internal functions
   0xAE, //DON = 0: display OFF
   0xA1, //ADC = 1: reverse direction(SEG132->SEG1)
   0xA6, //REV = 0: non-reverse display
   0xA4, //EON = 0: normal display. non-entire
   0xA2, //Select LCD bias=0
   0xC0, //SHL = 0: normal direction (COM1->COM64)
   0x2F, //Control power circuit operation VC=VR=VF=1
   0x25, //Select int resistance ratio R2 R1 R0 =5
   0x81, //Set reference voltage Mode
   0x22, //24 SV5 SV4 SV3 SV2 SV1 SV0 = 0x18
   0xAF  //DON = 1: display ON
#endif
};

inline void lcdInit()
{
  LCD_LOCK();
  #if defined(SSD1306)
    I2C_init();
  #else
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RES);  //LCD reset
    _delay_us(2);
    PORTC_LCD_CTRL |= (1<<OUT_C_LCD_RES);  //LCD normal operation
    #if defined(LCD_ST7920)
      _delay_ms(40);
    #else
      _delay_us(1500);
    #endif
  #endif
  for (uint8_t i=0; i<DIM(lcdInitSequence); i++) {
    lcdSendCtl(pgm_read_byte(&lcdInitSequence[i]));
    #if defined(LCD_ST7920)
      _delay_us(80);
    #endif
  }
#if defined(LCD_ERC12864FSF)
  g_eeGeneral.contrast = 0x2D;
#else
  g_eeGeneral.contrast = 0x22;
#endif
  LCD_UNLOCK();
}

void lcdSetRefVolt(uint8_t val)
{
#if !defined(LCD_ST7920) // No contrast setting for ST7920
  LCD_LOCK();
  lcdSendCtl(0x81);
  #if defined(SSD1306)
    lcdSendCtl((val << 2) + 3); // 3 to 255
  #else
    lcdSendCtl(val); // 0 to 63
  #endif
  LCD_UNLOCK();
#endif
}

#if defined(LCD_ST7920)
void lcdRefresh(){
  lcdRefresh_ST7920(1);
}

uint8_t lcdRefresh_ST7920(uint8_t full)
{
#else
void lcdRefresh()
{
#endif
  LCD_LOCK();
#if defined(LCD_ST7920)
  static uint8_t state;
  uint8_t yst,yend;
  uint8_t x_addr = 0;
  uint8_t y_addr = 0;
  uint16_t line_offset = 0;
  uint8_t col_offset = 0;
  uint8_t bit_count = 0;
  uint8_t result;
  uint8_t *p;
  if(full!=0){
    yst=0;
    yend=64;
    state=0;
  } 
  else{
    switch (state){//Since writing to ST7920 is too slow we need to split it to five bands.
     default:
     case 0:
       yst=0;
       yend=13;
       state=1;
       break;
     case 1:
       yst=13;
       yend=26;
       state=2;
       break;
     case 2:
       yst=26;
       yend=39;
       state=3;
       break;
     case 3:
       yst=39;
       yend=52;
       state=4;
       break;
     case 4:
       yst=52;
       yend=64;
       state=0;
       break;
    }
  }
  
  for (uint8_t y=yst; y<yend; y++) {
    x_addr = 0;
    //Convert coordinates to weirdly-arranged 128x64 screen (the ST7920 is mapped for 256x32 displays)
    if (y > 31) {
      y_addr = y - 32;    //Because there are only 31 addressable lines in the ST7920
      x_addr += 8;        //so we overflow x (7 visible bytes per line) to reach the bottom half
    }
    else {
      y_addr = y;
    }
    lcdSendCtl( 0x80 | y_addr );  //Set Vertical Address
    _delay_us(49);
    lcdSendCtl( 0x80 | x_addr );  //Set Horizontal Address
    _delay_us(49);
    PORTC_LCD_CTRL |= (1<<OUT_C_LCD_A0);    //HIGH RS and LOW RW will put the LCD to
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RnW);  //Write data register mode
    bit_count = y & 0x07; //Count from 0 bis 7 -> 0=0, 1=1..7=7, 8=0, 9=1...
    col_offset = 1 << bit_count; //Build a value for a AND operation with the vorrect bitposition
    line_offset = ( y / 8 ) * 128; //On the ST7565 there are 8 lines with each 128 bytes width
    for (coord_t x=0; x<16; x++) { //Walk through 16 bytes form left to right (128 Pixel)
      p=displayBuf + line_offset + ( x * 8 ); //Calculate the position of the first byte im array
      // adressing the bytes sequential and set the bits at the correct position merging them with an OR operation to get all bits in one byte
      // the position of the LSB is the right-most position of the byte to the ST7920
    result = ((*p++ & col_offset)!=0?0x80:0); 
    result|= ((*p++ & col_offset)!=0?0x40:0);
    result|= ((*p++  & col_offset)!=0?0x20:0);
    result|= ((*p++  & col_offset)!=0?0x10:0);
    result|= ((*p++  & col_offset)!=0?0x08:0);
    result|= ((*p++  & col_offset)!=0?0x04:0);
    result|= ((*p++  & col_offset) !=0?0x02:0);
    result|= ((*p++  & col_offset)!=0?0x01:0);
    PORTA_LCD_DAT = result;
      PORTC_LCD_CTRL |= (1<<OUT_C_LCD_E);
      _delay_us(8);
      PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_E);
      _delay_us(49);
    }
  }
#elif defined(SSD1306)
  uint8_t *p = displayBuf;
  for (uint8_t y = 0xB0; y< 0xB8; y++)
  {
    lcdSendCtl(y);    // 0xB0..B7 sets the page to which we write
    
    #if defined(SH1106)
    lcdSendCtl(0x02);  
    #else
    lcdSendCtl(0x00);
    #endif
    lcdSendCtl(0x10); //Set column address to 0x00 or 0x02
    I2C_start();
    I2C_write(SSD1306_ADDRESS);
    I2C_write(0x40); //Co 0; D/C 1 so we can write a stream
    for (char x=64; x>0; x--)
    {
      I2C_write(*p++);
    }
    I2C_stop();

    #if defined(SH1106)
    lcdSendCtl(0x02); 
    #else
    lcdSendCtl(0x00);
    #endif
    lcdSendCtl(0x14); // 0x40 or 0x42
    I2C_start();
    I2C_write(SSD1306_ADDRESS);
    I2C_write(0x40); //Co 0; D/C 1 so we can write a stream
    for (char x=64; x>0; x--)
    {
      I2C_write(*p++);
    }
    I2C_stop();
  }
#else  //All other LCD
  uint8_t * p = displayBuf;
  for (uint8_t y=0; y < 8; y++) {
#if defined(LCD_ST7565R)
    lcdSendCtl(0x01);
#else
    lcdSendCtl(0x04);
#endif
    lcdSendCtl(0x10); // Column addr 0
    lcdSendCtl( y | 0xB0); //Page addr y
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_CS1);
#if defined(LCD_MULTIPLEX)
    DDRA = 0xFF; // Set LCD_DAT pins to output
#endif
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RnW);
    for (coord_t x=LCD_W; x>0; --x) {
      PORTA_LCD_DAT = *p++;
      PORTC_LCD_CTRL |= (1<<OUT_C_LCD_E);
      PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_E);
    }
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_CS1);
  }
#endif  
  LCD_UNLOCK();
#if defined(LCD_ST7920)
  return state;
#endif  

}
