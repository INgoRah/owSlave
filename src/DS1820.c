// Copyright (c) 2021, INgo Rah INgo.Rah@gmx.net
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
#if defined(DS1820_SUPPORT)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "Arduino.h"

#include "owSlave_tools.h"
#include "DS2408.h"
#include "DS1820.h"
#ifdef BMP280_SUPPORT
#include "bmx280.h"
#endif
#ifdef DHT22_SUPPORT
#include "dht22.h"
#endif

/* must be protected by interrupts */
#define wdt_tim_enable() do { \
		wdt_reset(); \
		MCUSR &= ~(_BV(WDRF)); \
		WDTCSR = (1 << WDCE) | (1 << WDE); \
		WDTCSR = (1 << WDP3) | (1 << WDP0); \
		WDTCSR |= _BV(WDIE); \
	}while(0);

/* for status update */
extern pack_t pack;
extern uint8_t int_signal;
extern uint8_t config_info2[26];
extern unsigned long _ms;

volatile packt_t packt;
volatile static uint8_t do_temp;
int16_t last_temp;
#ifdef BMP280_SUPPORT
static uint8_t bmp280_found = 0;
#else
uint16_t adc = 0;
uint8_t cnt = 0;
#endif

#ifdef WDT_vect
ISR(WDT_vect)
#else
ISR(WATCHDOG_vect)
#endif
{
	do_temp++;
}

// when ADC completed, take an interrupt
EMPTY_INTERRUPT (ADC_vect);

void temp_setup()
{
	WDTCSR = _BV(WDIE) | _BV(WDCE);
	wdt_tim_enable();
	packt.TH = 22;
	packt.TL = 18;
	packt.rrFF = 0xff;
	packt.rr00 = 0x0;
	packt.rr10 = 0x10;
	/* 12 bit resolution */
	packt.config = 0x60;
	alarmflag2 = 0;
	packt.temp = 0;
	last_temp = 0;
	/* force full update */
	do_temp = 4;
#ifdef BMP280_SUPPORT
	bmp280_found = bmp280_init();
	if (bmp280_found == 0)
		pack.Status |= 0x08;
#endif
}

/* stores the read and calculated temperature in temp */
void temp_read()
{
	int off = config_info2[1] | config_info2[0] << 8;
	int k1 = (config_info2[3] | config_info2[2] << 8);

#if defined(BMP280_SUPPORT) || defined(DHT22_SUPPORT)
	int16_t t;

#ifdef BMP280_SUPPORT
	t = bmp280_compensate_T16(bmp280_readT());
#else
	if (dht22_read() != 0)
		return;
	packt.rrFF = dht22_readH();
	t = dht22_readT();
#endif /* BMP280_SUPPORT */
	t = (int16_t) ((float)t * k1);
	t -= off;
	packt.temp = (3 * packt.temp + t) / 4;
	pack.Status |= 0x02;
	_ms += 5;
#else /* defined(BMP280_SUPPORT) || defined(DHT22_SUPPORT) */
	PRR &= ~(1<<PRADC);
	_delay_us (20);
	ADCSRB &= ~(1<<ACME);
#if defined(__AVR_ATtiny85__)
	ADMUX = _BV(REFS1) | 0xf; // ADC4 B1111
#endif
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	ADMUX = _BV(REFS1) | 0x22; // B00100010
#endif
	/* prescaler of 128 */
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
	_delay_us (30);

	ADCSRA |=  _BV(ADIF);  // enable ADC, turn off any pending interrupt
	cli ();
	set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
	sleep_enable();
	// start the conversion
	ADCSRA |= _BV(ADSC) | _BV(ADIE);
	sei ();
	sleep_cpu ();
	sleep_disable ();
	wdt_reset();
	/* awake again, reading should be done, but better make sure
	   maybe the timer interrupt fired */
	while (bit_is_set (ADCSRA, ADSC));
	/* Calculate the temperature in C by averaging over 4 measurements */
	if (cnt < 4) {
		adc += ADC;
		cnt++;
		// still doing updates till 4 values can be averaged
		if (cnt == 4)
			adc = adc >> 2;
		else
			do_temp = 4;
	} else 	{
		int16_t temp;
		float k, t;
		k = k1 / 100 * 16;
		k = k / 100;
		off = (off >> 1);
		adc = ((adc * 3) + ADC) >> 2;
		t = (adc - off) * k;
		temp = (int16_t)t;
		if (config_info2[1] & 0x1)
			temp -= 8;

		config_info2[8] = (uint8_t)((adc & 0xff00)) >> 8;
		config_info2[9] = (uint8_t)(adc & 0xff);
		packt.temp = temp;
	}

	ADCSRA = 0;
	PRR |= (1<<PRADC);  /*Switch off adc for save Power */
	_ms++;
#endif
}

void temp_update()
{
	if (last_temp == 0 && packt.temp != 0)
		last_temp = packt.temp;
	if (last_temp == packt.temp)
		return;

	uint8_t t = (uint8_t)(packt.temp >> 4);

	pack.Status |= 0x01;
	// signal a temperature change
	if (config_info2[4] & 0x01)
		alarmflag2 = 1;
	last_temp = packt.temp;
	if (config_info2[5] < 9) {
		uint8_t pin_mask = config_info2[5];

		/* first two pios (1 & 2) are same as bitmasks,
			no bit manipulation needed */
		if (pin_mask > 2)
				pin_mask = 1 << (pin_mask - 1);
		if (config_info2[6] != 0xff)
			if (t < config_info2[6]) {
				pack.PIO_Output_Latch_State &= ~(pin_mask);
				latch_out(pin_mask);
			}
		/* auto switching (heating control) */
		if (config_info2[7] != 0xff)
			if (t > config_info2[7]) {
				pack.PIO_Output_Latch_State |= pin_mask;
				latch_out(pin_mask);
			}
		/* if switching was done, alarm it */
		if (pack.PIO_Activity_Latch_State && !alarmflag) {
			alarmflag = 1;
			if (int_signal == SIG_ARM)
				int_signal = SIG_ACT;
		}
	}
	if (t < packt.TL || t > packt.TH)
		alarmflag2 = 1;
}

void temp_loop()
{
	static uint8_t upd = 0;
	if (gcontrol & 0x20) {
		gcontrol &= ~0x20;
		wdt_reset();
		/* got a start conversion command, force update */
		do_temp = 1;
		upd = 2;
	}
	if (do_temp > 0) {
		wdt_reset();
		WDTCSR |= _BV(WDIE);
		do_temp = 0;
#ifdef BMP280_SUPPORT
		if (!bmp280_found)
			return;
#endif
		/* not too often, every 4th watchdog wake up (4 * 8 s) is enough */
		if (upd++ < 2 && last_temp != 0)
			return;
		upd = 0;
		temp_read();
		temp_update();
	}
}
#endif
