// Copyright (c) 2024, INgo Rah INgo.Rah@gmx.net
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//  * All advertising materials mentioning features or use of this
//    software must display the following acknowledgement: This product
//    includes software developed by tm3d.de and its contributors.
//  * Neither the name of tm3d.de nor the names of its contributors may
//    be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#ifdef HAVE_UART
#include "uart.h"
#include "printf.h"
#else
#define printf(...)
#endif
#include "wiring.h"

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
#define ATMEGA
#endif

/* @param mux - ADC channel
  0x22 / B00100010 for internal temperature on ATiny84 */
uint16_t analogRead(uint8_t mux)
{
	uint16_t res;

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
	PRR &= ~(1 << PRADC);
	_delay_us (20);
	ADCSRB = 0;
#if defined(__AVR_ATtiny85__)
	ADMUX = _BV(REFS1) | 0xf; // ADC4 B1111
#endif
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	// ADC8: temperature
	ADMUX = _BV(REFS1) | (mux & 0x37);
#endif
#ifdef ATMEGA
	ADMUX = _BV(REFS0) | (mux & 0x07);
#endif
	/* prescaler of 128 */
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
	_delay_us (30);
#if 0
	ADCSRA |=  _BV(ADIF);  // enable ADC, turn off any pending interrupt
	cli ();
	set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
	sleep_enable();
#endif
	// start the conversion
	ADCSRA |= _BV(ADSC) /* | _BV(ADIE) */;
#if 0
	sei ();
	sleep_cpu ();
	sleep_disable ();
#endif
	wdt_reset();
	/* awake again, reading should be done, but better make sure
	   maybe the timer interrupt fired */
	while (bit_is_set (ADCSRA, ADSC));
	res = ADC;

	ADCSRA = 0;
	/* Switch off adc for save Power */
	PRR |= (1<<PRADC);

	return res;
}

/** 
 * pin - real pin mask
 */
void analogWrite(uint8_t pin, int val)
{
	if (val) {
		/* PWM Phase correct, 8 bit, clk/256 */
#if defined(__AVR_ATtiny85__)
		/* PWM on PB4 only supported. PIO0 == PB4 */
		/* wake up timer */
		PRR &= ~_BV(PRTIM1);
		_delay_us(10);
		// PWM-Mode, OC1B (PB4) cleared on compare match, set when TCNT1 = OCR1C
		GTCCR = _BV(PWM1B) | _BV(COM1B0);
		/* timer on, 
		timer cleared on compare match with OCR1C */
		TCCR1 = _BV(CTC1) | _BV(CS13);
		OCR1B = val;
#endif
#if defined(__AVR_ATtiny84__) || defined(ATMEGA) || defined(__AVR_ATtiny84A__)
		/* wake up timer */
		PRR &= ~_BV(PRTIM1);
		_delay_us(10);
		/* Phase correct dual slope PWM
		   Clear OC1A/OC1B on Compare Match when up-
		   counting. Set OC1A/OC1B on Compare Match when
		   downcounting. TOV on bottom
		*/
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny84A__)
		TCCR1A = _BV(COM1B1) | _BV(WGM10);
		if (pin == 7)
			OCR0B = val;
		else if (pin == 5)
			OCR1B = val;
#else
		if (pin == _BV(PB2)) {
			TCCR1A = _BV(COM1B1) | _BV(WGM10);
			OCR1B = val;
		} else if (pin == _BV(PB1)) {
			TCCR1A = _BV(COM1A1) | _BV(WGM10);
			OCR1A = val;
		}
#endif
		else {
			TCCR1A = 0;
			return;
		}
		/* enable the timer */
		TCCR1B = _BV(CS12);
#endif
	} else {
		/* switch off */
#if defined(__AVR_ATtiny85__)
		/* stop timer and disconnect output, shut down timer to save power */
		TCCR1 = 0;
		GTCCR = 0;
		_delay_us(1);
		PRR |= _BV(PRTIM1);
#endif
#if defined(__AVR_ATtiny84__) || defined(ATMEGA) || defined(__AVR_ATtiny84A__)
		TCCR1A = 0;
		TCCR1B = 0;
		_delay_us(1);
		PRR |= _BV(PRTIM1);
#endif
	}
}