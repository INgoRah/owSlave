#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifdef __cplusplus
extern "C"{
#endif

void init(void);
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
/*
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);
*/
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
  
#ifdef __cplusplus
} // extern "C"
#endif

#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
#include "ow_attiny25.h"
#endif
#ifdef __AVR_ATmega644__
#include "ow_atmega644.h"
#endif
#ifdef __AVR_ATmega16__
#include "ow_atmega16.h"
#endif

#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
#include "ow_atmega88.h"
#endif

#endif

