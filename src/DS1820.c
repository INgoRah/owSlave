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
#include <i2cmaster.h>
#endif
#ifdef HAVE_UART
#include "uart.h"
#include "printf.h"
#else
#define printf(...)
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

#ifdef BMP280_SUPPORT
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

int16_t bmp280_compensate_T16(int32_t adc_T)
{
	double var1, var2;
	int32_t t_fine;
	int16_t T;

	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	/* original it was double without *16 ...
	   This converts to the 1820 value */
	T = (int16_t) (t_fine / 5120.0 * 16);

	return T;
}

uint16_t bmp280_read16(uint8_t adr)
{
	uint8_t lsb, msb;

	i2c_start(0xEC + I2C_WRITE);
	i2c_write(adr);
	i2c_rep_start(0xEC + I2C_READ);
	lsb = i2c_read(ACK);
	msb = i2c_read(NACK);
	i2c_stop();

	return msb << 8 | lsb;
}

void bmp280_calib()
{
	dig_T1 = bmp280_read16(0x88);
	dig_T2 = bmp280_read16(0x8A);
	dig_T3 = bmp280_read16(0x8C);
	printf ("T1 = %x T2 = %x T3 = %x\n", dig_T1, dig_T2, dig_T3);
}

void bmp280_init()
{
	uint8_t ret;

	i2c_init();
	// set device address (EC 8 bit = 76 7 bit) and write mode
	ret = i2c_start(0xEC + I2C_WRITE);
	if (ret) {
		printf ("no ack on adr (%i)\n", ret);
		i2c_stop();
		return;
	}
	ret = i2c_write(0xD0);
	if (ret) {
		printf ("no ack on write (%i)\n", ret);
		i2c_stop();
		return;
	}
	i2c_stop();
	i2c_start (0xEC + I2C_READ);
	ret = i2c_read(NACK);
	config_info2[8] = ret;

	if (ret != 0x58)
		return;
	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xF4);
	/* write mode to forced */
	i2c_write(0x45);
	i2c_stop();
	bmp280_calib();
	bmp280_found = 1;
}

int32_t bmp280_readT()
{
	uint8_t msb, lsb, c;

	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xF4);
	/* write mode to forced */
	i2c_write(0x45);
	i2c_stop();
	delay(1);
	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xFA);
	i2c_rep_start(0xEC + I2C_READ);
	msb = i2c_read(ACK);
	lsb = i2c_read(ACK);
	c = i2c_read(NACK);
	i2c_stop();
	return (int32_t)msb << 12 | ((int16_t)lsb << 4) | (c >> 4);
}
#endif

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
	bmp280_found = 0;
	bmp280_init();
	if (bmp280_found == 0)
		pack.Status |= 0x08;
#endif
}

/* stores the read and calculated temperature in temp */
void temp_read()
{
#ifdef BMP280_SUPPORT
	int32_t data = bmp280_readT();
	packt.temp = bmp280_compensate_T16(data);
	pack.Status |= 0x02;
	_ms += 4;
#else
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
		int off = config_info2[1] | config_info2[0] << 8;
		int k1 = (config_info2[3] | config_info2[2] << 8);
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
