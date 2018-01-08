/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "Arduino.h"

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L ) // = 8
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define NOT_A_PORT 0
#define PB 1
#define PC 2
#define PD 3
#define NOT_A_PIN 0

#if !defined(__AVR_ATtiny25__) && !defined(__AVR_ATtiny85__)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PC	, // PC 0 ** D37	
	PC	, // PC 1 ** D36	
	PC	, // PC 2 ** D35	
	PC	, // PC 3 ** D34	
	PC	, // PC 4 ** D33	
	PC	, // PC 5 ** D32	
	PB	, // PB 0	
	PB	, // PB 1
	PB	, // PB 2 ** SPI_SS
	PB	, // PB 3 ** SPI_MOSI
	PB	, // PB 4 ** SPI_MISO	
	PB	, // PB 5 ** SPI_SCK
	// PDs
	PD	, // PD 7 ** D38	
	PD	, // PD 3 ** USART1_TX	
	PD	, // PD 2 ** USART1_RX	
	PD	, // PD 1 ** I2C_SDA	
	PD	, // PD 0 ** I2C_SCL	
};
#endif

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny85__)
	_BV( PB4 )	, // PB 4 ** LAMP	#0
	_BV( PB3 )	, // PC 1 ** LED	#1
	_BV( PB1 )	, // IR sender		#2
	NOT_A_PIN	, //			#3
	NOT_A_PIN	, //			#4
	NOT_A_PIN	, //			#5
	_BV( PB0 )	, // BTN		#6
#else
	_BV( 2 )	, // PC 2 ** LAMP	#0
	_BV( 1 )	, // PC 1 ** LED	#1
	_BV( 0 )	, // PC 0 ** D37	#2
	_BV( 3 )	, // PC 3 ** D34	#3
	_BV( 4 )	, // PC 4 ** D33	#4
	_BV( 5 )	, // PC 5 ** D32	# 5
	_BV( 0 )	, // PB 0 ** BTN	# 6
	_BV( 1 )	, // PB 1 ** 	 	# 7
	_BV( 2 )	, // PB 2 ** SPI_SS	# 8
	_BV( 3 )	, // PB 3 ** SPI_MOSI	# 9
	_BV( 4 )	, // PB 4 ** SPI_MISO 	# 10
	_BV( 5 )	, // PB 5 ** SPI_SCK	# 11
	// pds
	_BV( 7 )	, // PD 7 ** D38	
	_BV( 3 )	, // PD 3 ** USART1_TX	
	_BV( 2 )	, // PD 2 ** USART1_RX	
	_BV( 1 )	, // PD 1 ** I2C_SDA	
	_BV( 0 )	, // PD 0 ** I2C_SCL
#endif
};

#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

// the prescaler is set so that timer1 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER1_OVERFLOW (clockCyclesToMicroseconds(64 * 256)) // = 2048

// the whole number of milliseconds per timer1 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER1_OVERFLOW / 1000) // = 2

// the fractional number of milliseconds per timer1 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER1_OVERFLOW % 1000) >> 3) // = 6
#define FRACT_MAX (1000 >> 3) // = 125

volatile unsigned long timer1_overflow_count = 0;
volatile unsigned long timer1_millis = 0;
static unsigned char timer1_fract = 0;

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny85__)
ISR(TIM1_OVF_vect)
#else
ISR(TIMER1_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer1_millis;
	unsigned char f = timer1_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer1_fract = f;
	timer1_millis = m;
	timer1_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer1_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer1_millis)
	cli();
	m = timer1_millis;
	SREG = oldSREG;

	return m;
}

unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer1_overflow_count;
#if defined(TCNT1L)
	t = TCNT1;
#elif defined(TCNT1)
	t = TCNT1;
#else
	#error timer 1 not defined
#endif

#ifdef TIFR1
	if ((TIFR1 & _BV(TOV1)) && (t < 255))
		m++;
#else
	if ((TIFR & _BV(TOV1)) && (t < 255))
		m++;
#endif

	SREG = oldSREG;
	
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay(unsigned long ms)
{
	uint32_t start = micros();

	while (ms > 0) {
		while ( ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

/* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
void delayMicroseconds(unsigned int us)
{
	// call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/4 of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2; // x4 us, = 4 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 19 (21) cycles above, remove 5, (5*4=20)
	// us is at least 8 so we can substract 5
	us -= 5; // = 2 cycles,

#elif F_CPU >= 12000000L
	// for the 12 MHz clock if somebody is working with USB

	// for a 1 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1.5us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/3 of a microsecond (4 cycles)
	// per iteration, so execute it three times for each microsecond of
	// delay requested.
	us = (us << 1) + us; // x3 us, = 5 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 20 (22) cycles above, remove 5, (5*4=20)
	// us is at least 6 so we can substract 5
	us -= 5; //2 cycles

#elif F_CPU >= 8000000L
	// for the 8 MHz internal clock
	
	// for a 1 and 2 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 2us
	if (us <= 2) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/2 of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1; //x2 us, = 2 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 17 (19) cycles above, remove 4, (4*4=16)
	// us is at least 6 so we can substract 4
	us -= 4; // = 2 cycles
#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
	// return = 4 cycles
}

int digitalRead(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
	if (PINB & _BV(bit))
#else
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN)
		return LOW;

	if (*portInputRegister(port) & bit)
#endif		
		return HIGH;
	return LOW;
}

void pinMode(uint8_t pin, uint8_t mode)
{
	uint8_t bit;
	volatile uint8_t *reg, *out;
	uint8_t oldSREG;
	
	bit = digitalPinToBitMask(pin);
#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
	reg = &DDRB;
	out = &PORTB;
#else	
	uint8_t port;
	
	port = digitalPinToPort(pin);
	reg = portModeRegister(port);
	out = portOutputRegister(port);
#endif
	oldSREG = SREG;
	cli();
	if (mode == INPUT) { 
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		*out &= ~bit;
		*out |= bit;
	} else {
		*reg |= bit;
	}
	SREG = oldSREG;
}

void digitalWrite(uint8_t pin, uint8_t val)
{
	//uint8_t oldSREG = SREG;
	uint8_t bit;
	volatile uint8_t *out;
	
	bit = digitalPinToBitMask(pin);
#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
	out = &PORTB;
#else
	uint8_t port = digitalPinToPort(pin);
	if (port == NOT_A_PIN) return;
	out = portOutputRegister(port);
#endif
	//cli();
	if (val == LOW)
		*out &= ~bit;
	else
		*out |= bit;
	//SREG = oldSREG;
}

static const uint8_t analog_reference = 0;
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int analogRead(uint8_t pin)
{
	uint8_t low, high;

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif
	// without a delay, we seem to read from the wrong channel
	//delay(1);
#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;
#else
	// we dont have an ADC, return 0
	low  = 0;
	high = 0;
#endif

	// combine the two bytes
	return (high << 8) | low;
}

void init()
{
	// this needs to be called before setup() or some functions won't
	// work there
	sei();
	
#if defined(TCCR1A) && defined(WGM11)
	// this combination is for the standard 88/168/328/1280/2560
	// set TOV on TOP=0xFF, 8 bit fast PWM mode
	sbi(TCCR1B, WGM12); // for timer 0 force 8 bit
	sbi(TCCR1A, WGM10);
#endif
	// set timer 1 prescale factor to 64
#if defined(TCCR1) && defined(CS11) && defined(CS10)
	// this combination is for the standard atmega8 / attiny85
	sbi(TCCR1, CS11);
	sbi(TCCR1, CS10);
#elif defined(TCCR1B) && defined(CS11) && defined(CS10)
	// this combination is for the standard 88/168/328/1280/2560
	sbi(TCCR1B, CS11);
	sbi(TCCR1B, CS10);
#else
	#error Timer 1 prescale factor 64 not set correctly
#endif
	// enable timer 1 overflow interrupt
#if defined(TIMSK) && defined(TOIE1)
	sbi(TIMSK, TOIE1);
#elif defined(TIMSK1) && defined(TOIE1)
	sbi(TIMSK1, TOIE1);
#else
	#error	Timer 1 overflow interrupt not set correctly
#endif
}
