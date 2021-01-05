/**************************************************************************/
/*! 
    @file     ST7735.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    Copyright (c) 2021, INgo.Rah@gmx.net
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef __ST7735_H__
#define __ST7735_H__

#define false 0
#define true 1
 
// Basic Color definitions
#define	COLOR_BLACK                         (uint16_t)(0x0000)
#define	COLOR_BLUE                          (uint16_t)(0x001F)
#define	COLOR_RED                           (uint16_t)(0xF800)
#define	COLOR_GREEN                         (uint16_t)(0x07E0)
#define COLOR_CYAN                          (uint16_t)(0x07FF)
#define COLOR_MAGENTA                       (uint16_t)(0xF81F)
#define COLOR_YELLOW                        (uint16_t)(0xFFE0)
#define COLOR_WHITE                         (uint16_t)(0xFFFF)

// Grayscale Values
#define COLOR_GRAY_15                       (uint16_t)(0x0861)    //  15  15  15
#define COLOR_GRAY_30                       (uint16_t)(0x18E3)    //  30  30  30
#define COLOR_GRAY_50                       (uint16_t)(0x3186)    //  50  50  50
#define COLOR_GRAY_80                       (uint16_t)(0x528A)    //  80  80  80
#define COLOR_GRAY_128                      (uint16_t)(0x8410)    // 128 128 128
#define COLOR_GRAY_200                      (uint16_t)(0xCE59)    // 200 200 200
#define COLOR_GRAY_225                      (uint16_t)(0xE71C)    // 225 225 225

uint16_t colorsRGB24toRGB565  ( uint8_t r, uint8_t g, uint8_t b );
uint32_t colorsRGB565toBGRA32 ( uint16_t color );
uint16_t colorsBGR2RGB        ( uint16_t color );
uint16_t colorsDim            ( uint16_t color, uint8_t intensity );
uint16_t colorsAlphaBlend     ( uint16_t bgColor, uint16_t foreColor, uint8_t fadePercent );

// Any LCD needs to implement these common methods, which allow the low-level
// initialisation and pixel-setting details to be abstracted away from the
// higher level drawing and graphics code.

typedef enum 
{
  LCD_ORIENTATION_PORTRAIT = 0,
  LCD_ORIENTATION_LANDSCAPE = 1
} lcdOrientation_t;

// This struct is used to indicate the capabilities of different LCDs
typedef struct
{
  uint16_t width;         // LCD width in pixels (default orientation)
  uint16_t height;        // LCD height in pixels (default orientation)
  uint8_t     touchscreen;   // Whether the LCD has a touch screen
  uint8_t     orientation;   // Whether the LCD orientation can be modified
  uint8_t     hwscrolling;   // Whether the LCD support HW scrolling
  uint8_t     fastHLine;     // Whether the driver contains an accelerated horizontal line function
  uint8_t     fastVLine;     // Whether the driver contains an accelerated vertical line function
} lcdProperties_t;

extern void     lcdInit(void);
extern void     lcdTest(void);
extern uint16_t lcdGetPixel(uint16_t x, uint16_t y);
extern void     lcdFillRGB(uint16_t data);
extern void     lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color);
extern void     lcdDrawPixels(uint16_t x, uint16_t y, uint16_t *data, uint32_t len);
extern void     lcdDrawHLine(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color);
extern void     lcdDrawVLine(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color);
extern void     drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint16_t bg, uint8_t size_x,
                            uint8_t size_y);
extern void     lcdBacklight(uint8_t state);
extern void     lcdScroll(int16_t pixels, uint16_t fillColor);
extern uint16_t lcdGetWidth(void);
extern uint16_t lcdGetHeight(void);
extern void     lcdSetOrientation(lcdOrientation_t orientation);
extern uint16_t lcdGetControllerID(void);
extern lcdOrientation_t lcdGetOrientation(void);
extern lcdProperties_t lcdGetProperties(void);

extern void st7735WriteCmd(uint8_t command);

/**************************************************************************
    ST7735 CONNECTOR
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   NC
      2   GND
      3   LED K/-
      4   LED A/+         3.0V
      5   GND
      6   RESET
      7   RS
      8   SDA             Serial Data
      9   SCL             Serial Clock
     10   VCC             2.8-3.4V
     11   VCC             2.8-3.4V
     12   CS
     13   GND
     14   NC

 **************************************************************************/

// Control pins
#define ST7735_RS_PIN          (PA2) /* data command pin */
#define ST7735_SDA_PIN         (PA6) /* serial data */
#define ST7735_SCL_PIN         (PA4) /* serial clock */
#define ST7735_CS_PIN          (PA3) /* chip select */
#define ST7735_RES_PIN         (PA5) 
//#define ST7735_BL_PIN          (7)

// Macros for control line state
#define CLR_RS      do { PORTA &= ~(_BV(ST7735_RS_PIN)); } while(0)
#define SET_RS      do { PORTA |= (_BV(ST7735_RS_PIN)); } while(0)
#define CLR_SDA     do { PORTA &= ~(_BV(ST7735_SDA_PIN )); } while(0)
#define SET_SDA     do { PORTA |= (_BV(ST7735_SDA_PIN)); } while(0)
#define CLR_SCL     do { PORTA &= ~(_BV(ST7735_SCL_PIN )); } while(0)
#define SET_SCL     do { PORTA |= (_BV(ST7735_SCL_PIN)); } while(0)
#define CLR_CS      do { PORTA &= ~(_BV(ST7735_CS_PIN )); } while(0)
#define SET_CS      do { PORTA |= (_BV(ST7735_CS_PIN)); } while(0)
#define CLR_RES     do { PORTA &= ~(_BV(ST7735_RES_PIN )); } while(0)
#define SET_RES     do { PORTA |= (_BV(ST7735_RES_PIN)); } while(0)
#ifdef ST7735_BL_PIN
#define CLR_BL      do { PORTA &= ~(_BV(ST7735_BL_PIN )); } while(0)
#define SET_BL      do { PORTA |= (_BV(ST7735_BL_PIN)); } while(0)
#else
#define CLR_BL      do { } while(0)
#define SET_BL      do { } while(0)
#endif

#define ST7735_NOP      (0x0)
#define ST7735_SWRESET  (0x01)
#define ST7735_SLPIN    (0x10)
#define ST7735_SLPOUT   (0x11)
#define ST7735_PTLON    (0x12)
#define ST7735_NORON    (0x13)
#define ST7735_INVOFF   (0x20)
#define ST7735_INVON    (0x21)
#define ST7735_DISPON   (0x29)
#define ST7735_CASET    (0x2A)
#define ST7735_RASET    (0x2B)
#define ST7735_RAMWR    (0x2C)
#define ST7735_COLMOD   (0x3A)
#define ST7735_MADCTL   (0x36)
#define ST7735_FRMCTR1  (0xB1)
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR   (0xB4)
#define ST7735_DISSET5  (0xB6)
#define ST7735_PWCTR1   (0xC0)
#define ST7735_PWCTR2   (0xC1)
#define ST7735_PWCTR3   (0xC2)
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1   (0xC5)
#define ST7735_PWCTR6   (0xFC)
#define ST7735_GMCTRP1  (0xE0)
#define ST7735_GMCTRN1  (0xE1)

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_SLPOUT 0x11
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29

#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#endif