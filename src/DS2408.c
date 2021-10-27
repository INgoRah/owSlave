// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
// Copyright (c) 2020, INgo Rah INgo.Rah@gmx.net
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#if defined(WDT_ENABLED) || defined(DS1820_SUPPORT)
#include <avr/wdt.h>
#define wdr wdt_reset
#else
#define wdr() do { ; } while(0)
#endif
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "owSlave_tools.h"
#include "Arduino.h"
#include "pins.h"
#ifdef HAVE_UART
#include "uart.h"
#include "printf.h"
#else
#define printf(...)
#endif

#include "DS2408.h"
#ifdef DS1820_SUPPORT
#include "DS1820.h"
#endif

#define wdt_tim_enable() do { \
		cli(); \
		wdt_reset(); \
		MCUSR &= ~(_BV(WDRF)); \
		WDTCSR = (1 << WDCE) | (1 << WDE); \
		WDTCSR = (1 << WDP3) | (1 << WDP0); \
		WDTCSR |= _BV(WDIE); \
		sei(); \
	}while(0);

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);
extern uint8_t stat_to_sample;
static void owResetSignal(void);
void reg_init(void);
void var_init(void);
static uint8_t auto_toggle(uint8_t i);
void statusPrint();

/* bit number in PIN register to IO mapping */
const uint8_t pio_map [MAX_BTN] = {
	PIN_PIO0,
	PIN_PIO1,
	PIN_PIO2,
	PIN_PIO3,
#ifdef PIN_PIO4
	PIN_PIO4,
#endif
#ifdef PIN_PIO5
	PIN_PIO5,
#endif
#ifdef PIN_PIO6
	PIN_PIO6,
#endif
#ifdef PIN_PIO7
	PIN_PIO7
#endif
};

/* last byte will be calculated, program like
avrdude -C <users path>\.platformio\packages\tool-avrdude\avrdude.conf -c stk500v2 -P COM13 -p attiny85 -U eeprom:w:0x29,0x11,0x04,0x01,0x2,0x66,0x77:m
 */
#ifdef DUAL_ROM
uint8_t owid1[8];
uint8_t owid2[8];
#define owid owid1
#else
uint8_t owid[8];
#endif

#ifndef ATMEGA
static const uint8_t pwm_tbl[16] = { 1, 2, 3, 5, 8, 13, 21, 30, 35, 40, 45, 50, 55, 60, 70, 80 };
#endif

volatile pack_t pack;

#ifdef DUAL_ROM
uint8_t config_info1[26];
#define config_info config_info1
uint8_t config_info2[26];
#else
#endif

/*
 * config from EEPROM
 */
uint8_t config_info[26];
uint8_t signal_cfg;
unsigned long _ms;

struct pinState btn[MAX_BTN];
uint8_t values[CHAN_VALUES];
uint8_t ap = 0;
uint8_t int_signal;
volatile uint8_t btn_active = 0;
#ifdef WITH_LEDS
static uint8_t led2 = 0;
#endif

#ifdef WITH_LEDS
static void led_flash(void)
{
#ifndef AVRSIM // defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	int i, j;

	for (i = 4; i > 1; i--) {
		LED_ON();
		for (j = 0; j < i; j++)
			_delay_ms(25);
		LED_OFF();
		_delay_ms(50);
	}
#endif
}
#endif

static uint8_t crc(uint8_t* input, uint8_t len) {
	unsigned char inbyte, crc = 0;
	unsigned char i, mix;
	uint8_t* addr = input;

	while (len--) {
		inbyte = *addr++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;
			inbyte >>= 1;
		}
	}

	return crc;
}

unsigned long millis()
{
	return _ms;
}

void delay(unsigned long ms)
{
	while (ms > 0) {
#ifndef AVRSIM
		_delay_ms(1);
#endif
		_ms++;
		ms--;
	}
}

static inline void act_latch(uint8_t p, uint8_t mask)
{
	wdr();
	if ((mask & pack.Conditional_Search_Channel_Selection_Mask) != 0)
		pack.PIO_Activity_Latch_State |= mask;
	if (p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
}

static inline void pin_sync_ls(uint8_t pins, uint8_t p, uint8_t mask)
{
	if (pins & p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
}

static inline void latch_state(uint8_t mask)
{
	if ((mask & pack.Conditional_Search_Channel_Selection_Mask) == 0)
		return;
	pack.PIO_Activity_Latch_State |= mask;
}

/* Function is / must be protected by interrupts */
static void sync_pins()
{
	uint8_t pins;

	pins = PIN_REG;
	pin_sync_ls(pins, PIN_PIO0, 0x1);
	pin_sync_ls(pins, PIN_PIO1, 0x2);
	pin_sync_ls(pins, PIN_PIO2, 0x4);
	pin_sync_ls(pins, PIN_PIO3, 0x8);
#ifdef PIN_PIO4
	pin_sync_ls(pins, PIN_PIO4, 0x10);
#endif
#ifdef PIN_PIO5
	pin_sync_ls(pins, PIN_PIO5, 0x20);
#endif
#ifdef PIN_PIO6
	pin_sync_ls(pins, PIN_PIO6, 0x40);
#endif
#ifdef PIN_PIO7
	pin_sync_ls(pins, PIN_PIO7, 0x80);
#endif
}

ISR(PCINT0_vect) {
	btn_active = 1;
#if defined(GIFR) && defined(PCIF0)
	GIFR = _BV(PCIF0);
#endif
#if defined(GIFR) && defined(PCIF)
	GIFR = _BV(PCIF);
#endif
}

/* sets a pin, but does not change any interrupt on that pin.
  If the pin was configured as input, the interupt is active. */
void pin_set(uint8_t p, uint8_t id, uint8_t bb)
{
	if (pack.PIO_Output_Latch_State & bb) {
		/* According to spec:
		 * set 1 / non-conducting (off)
		 * set to input or inactive
		 */
		pack.PIO_Logic_State |= bb;
		if (config_info[CFG_CFG_ID + id] == CFG_OUT_PWM ||
			config_info[CFG_CFG_ID + id] == CFG_OUT_HIGH) {
			/* no pull up */
			PIN_DDR |= p;
			PORT_REG &= ~p;
		} else {
			/* TODO: only if configured...*/
			/* input and enable pull up - DS2408 standard */
			PIN_DDR &= ~(p);
			PORT_REG |= (p);
		}
	} else {
		/* set 0 / 0 = conducting (on) */
		pack.PIO_Logic_State &= ~bb;
		if (config_info[CFG_CFG_ID + id] == CFG_OUT_HIGH) {
			/* set output and high on 0 */
			PIN_DDR |= p;
			PORT_REG |= (p);
		} else {
			/* set output and low on 0 - DS2408 standard */
			PIN_DDR |= p;
			PORT_REG &= ~p;
		}
	}
}

/* Checks for a change on a signal output latch and if, applies it */
void latch_out(uint8_t bb)
{
	uint8_t p = 0, id;

	/* code saver, but maybe slower */
#if 0
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map) / sizeof(pio_map[0]); i++) {
		if (bb == mask) {
			p = pio_map[i];
			id = i;
			break;
		}
		mask = mask << 1;
	}
#else
	switch (bb)
	{
		case 0x1:
			p = PIN_PIO0;
			id = 0;
			break;
		case 0x2:
			p = PIN_PIO1;
			id = 1;
			break;
		case 0x4:
			p = PIN_PIO2;
			id = 2;
			break;
		case 0x8:
			p = PIN_PIO3;
			id = 3;
			break;
#ifdef PIN_PIO4
		case 0x10:
			p = PIN_PIO4;
			id = 4;
			break;
#endif
#ifdef PIN_PIO5
		case 0x20:
			p = PIN_PIO5;
			id = 5;
			break;
#endif
#ifdef PIN_PIO6
		case 0x40:
			p = PIN_PIO6;
			id = 6;
			break;
#endif
#ifdef PIN_PIO7
		case 0x80:
			p = PIN_PIO7;
			id = 7;
			break;
#endif
		default:
			return;
	}
#endif
	cli();
	/* logic state = real state, output_latch state inverted! */
	if ((pack.PIO_Logic_State & bb) !=
		(pack.PIO_Output_Latch_State & bb)) {
		/* pin change by write and set
		   PIO_Logic_State accordingly */
		pin_set(p, id, bb);
		latch_state(bb);
	}
#if defined(GIFR) && defined(PCIF0)
	GIFR = _BV(PCIF0);
#endif
#if defined(GIFR) && defined(PCIF)
	GIFR = _BV(PCIF);
#endif
	sei();
}

static void owResetSignal(void)
{
	int to = 10;

	wdr();
	while (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0)) {
		delay(1);
		if (to-- == 0)
			/* try again later */
			return;
	};
	wdr();
	cli();
	sbi (OW_DDR, OW_PINN);
	_delay_us (1060);
	cbi (OW_DDR, OW_PINN);
	_ms++;
	/* wait for presence detect pulse from other clients, but after
	   960 there should be nothing more */
	to = 10;
	while ((OW_PIN & OW_PINN) == 0) {
		_delay_us (100);
		if (to-- == 0) {
			_ms++;
			break;
		}
	}
	int_signal = SIG_NO;
	sei();
	wdr();
#ifdef HAVE_UART
	serial_write('>');
#endif
}

/** Variables init */
void var_init(void)
{
	int i;

	/* all off */
	pack.PIO_Logic_State = 0xff;
	pack.PIO_Output_Latch_State = 0xff;
	pack.FF1 = 0xFF;
	pack.FF2 = 0xFF;
	alarmflag = 0;
	btn_active = 0;
	for (i = 0;i < MAX_BTN;i++)
		initBtn(1, &btn[i]);
	pack.PIO_Activity_Latch_State = 0;
	/* enable alarm state reporting */
	pack.Conditional_Search_Channel_Selection_Mask = 0xFF;
	signal_cfg = 0xff;

	int_signal = SIG_ARM;
}

void reg_init(void)
{
	uint8_t p;
	uint8_t mask;

#if defined(PCICR) && defined(PCIE0)
	PCICR |= _BV(PCIE0);
#endif
#if defined(GIMSK) && defined(PCIE0)
	/* HW_INIT takes care */
	GIMSK |= _BV(PCIE0);
#endif
	for (int i = 0; i < MAX_BTN; i++) {
		p = pio_map[i];
		if (p == 0)
			/* there is an error in the map, for all till MAX_BTN there should be an entry */
			continue;
		/* check config */
		switch (config_info[CFG_CFG_ID + i])
		{
		/*
		 * inputs
		 */
		case CFG_BTN:
		case CFG_SW:
		case CFG_PASS_PU:
		case CFG_PASS_INV_PU:
			/* pull up */
			PORT_REG |= p;
			PCMSK |= p; // enable interrupt
			break;
		case CFG_ACT_LOW:
		case CFG_ACT_HIGH:
		case CFG_PASS:
		case CFG_PASS_INV:
			/* no pull up */
			PORT_REG &= ~(p);
			PCMSK |= p; // enable interrupt
			break;
		/*
		 * outputs
		 */
		case CFG_OUT_HIGH:
			/* output initially active low
			 * PIO_Logic_State (inverted) is high cause it
			 * represents transistor output
			 */
			PCMSK &= ~(p); // disable interrupt
			PIN_DDR |= (p);
			PORT_REG &= ~(p);
			break;
			/* fall-through */
		case CFG_OUT_LOW:
			/* default as DS2408, active low, not setting to output yet
			 * this will be done in activation of latch
			 */
			PORT_REG |= p;
			PCMSK &= ~(p);
			break;
		case CFG_OUT_PWM:
			PCMSK &= ~(p);
			PIN_DDR |= (p);
			break;
		case CFG_DEFAULT:
		default:
			/* disable */
			PIN_DDR &= ~(p);
			PORT_REG &= ~(p);
			PCMSK &= ~(p);
			break;
		}
	}
#if defined(__AVR_ATtiny85__)
	/* can be removed because done in the loop above */
	if (config_info[CFG_CFG_ID] == CFG_OUT_PWM) {
		PCMSK &= ~(_BV(PB4));
		PIN_DDR |= _BV(PB4);
		/* no pull up */
		PORT_REG &= ~_BV(PB4);
#endif
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	if (config_info[CFG_CFG_ID + 5] == CFG_OUT_PWM) {
		PIN_DDR |= _BV(PA5);
		PCMSK &= ~(0x20);
#endif
		TCNT1 = 0xf;
#ifndef ATMEGA
	} else {
		/* timer 1 not needed */
		PRR |= _BV(PRTIM1);
	}
#endif
	sync_pins();
	mask = 1;
	for (int i = 0; i < MAX_BTN; i++) {
		/* check config and perform initial auto_switch */
		if (config_info[CFG_CFG_ID + i] >= CFG_PASS &&
			config_info[CFG_CFG_ID + i] <= CFG_PASS_PU) {
			auto_switch(i, pack.PIO_Logic_State & mask);
		}
		mask = mask << 1;
	}
}

void cfg_init(void)
{
#ifndef AVRSIM
	int to = 100;

	/* let power stabalize */
	/*_delay_ms(20); */
	while (!eeprom_is_ready() && to--)
		_delay_ms(1);

	eeprom_read_block((void*)owid, (const void*)0, 7);
#endif
	if (owid[0] != 0x29 || (owid[3] != ~owid[1])) {
		owid[0] = 0x29;
		owid[3] = ~owid[1];
		owid[4] = ~owid[2];
		owid[5] = 0x66;
		owid[6] = 0x77;
		owid[7] = crc(owid, 7);
	}
#ifdef DUAL_ROM
	for (int i = 1; i < 7; i++)
		owid2[i] =  owid[i];
	owid2[0] = 0x28;
	owid2[7] = crc(owid2, 7);
#endif
#ifndef AVRSIM
	eeprom_read_block((void*)&config_info, (const void*)7, CFG_TYPE_ID);
#ifdef DUAL_ROM
	eeprom_read_block((void*)&config_info2, (const void*)(8 + CFG_TYPE_ID), 22);
	config_info2[8] = 0xff;
	config_info2[9] = 0xff;
#endif
#endif
	config_info[CFG_VERS_ID] = MAJ_VERSION;
	config_info[CFG_VERS_ID + 1] = MIN_VERSION;
#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	config_info[CFG_TYPE_ID] = 4;
#endif
#if defined(__AVR_ATtiny85__)
	config_info[CFG_TYPE_ID] = 2;
#endif
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	if (config_info[CFG_TYPE_ID] == 0xff)
#ifdef DS1820_SUPPORT
		config_info[CFG_TYPE_ID] = 11;
#else
		/* default new version */
		config_info[CFG_TYPE_ID] = 1;
#endif
#endif
}

void setup()
{
#ifdef WDT_ENABLED
	uint8_t mcusr_old;

	/* the watchdog timer remains active even after a system reset (except a
	 * power-on condition), using the fastest prescaler value.
	 * It is therefore required to turn off the watchdog early
	 * during program startup */
	mcusr_old = MCUSR;
	MCUSR = 0;
	wdt_disable();
	if (mcusr_old & _BV(WDRF)) {
		/* Watchdog occured */
		pack.Status = 0x88;
		LED2_ON();
	} else
		pack.Status = 0x80;
#endif
#ifdef DS1820_SUPPORT
	pack.Status = 0x80;
#endif
#ifdef HAVE_UART
	serial_init();
#endif

	cfg_init();
	var_init();
	OWST_INIT_ALL_OFF;
	OWST_EN_PULLUP
	/* pull ups on all pins set by OWST_INIT_ALL_OFF */
	OWINIT();
	reg_init();
	sei();
#ifdef DS1820_SUPPORT
	temp_setup();
#endif
#ifndef AVRSIM
	_delay_ms(50);
#endif
#ifdef WITH_LEDS
	led_flash();
#endif
}

void ow_loop()
{

	if (reset_indicator) {
		ap = 0;
		// stat_to_sample=0;
		reset_indicator=0;
		// TODO: only then enable the alarm signal (arm)
		int_signal = SIG_ARM;
	}
	if (gcontrol == 0)
		return;
	if (gcontrol & 1) {
		/* write, data in PIO_Output_Latch_State */
		uint8_t i;
		if (config_info[CFG_CFG_ID] == CFG_OUT_PWM) {
#if defined(__AVR_ATtiny85__)
			/* PWM on PB4 only supported. PIO0 == PB4 */
#if PIN_PIO0 !=_BV(PB4)
#error Not supported!
#endif
			if (pack.PIO_Output_Latch_State & 0x01) {
				/* wake up timer */
				PRR &= ~_BV(PRTIM1);
				_delay_us(10);
				TCNT1 = pwm_tbl[15] + 1; // maybe not needed ...
				// timer on, timer cleared on compare match with OCR1C, prescaler 1/128 => F_TIMER = 31kHz
				TCCR1 = _BV(CTC1) | _BV(CS13);
				OCR1C = pwm_tbl[15] + 1;
				// PWM-Mode, OC1B (PB4) cleared on compare match, set when TCNT1 = OCR1C
				GTCCR = _BV(PWM1B) | _BV(COM1B0);
				OCR1B = pwm_tbl[(pack.PIO_Output_Latch_State & 0xf0) >> 4];
			}
			else {
				/* stop timer and disconnect output, shut down timer to save power */
				TCCR1 = 0;
				GTCCR = 0;
				_delay_us(1);
				PRR |= _BV(PRTIM1);
			}
#endif
		} else if (config_info[CFG_CFG_ID + 5] == CFG_OUT_PWM) {
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (pack.PIO_Output_Latch_State > 0) {
				/* PWM on PA5 / PIO5 */
				TCCR1A = _BV(COM1B1) /*| _BV(COM1B0)*/ | _BV(WGM00);
				TCCR1B = /*_BV(WGM13) | _BV(WGM12) | */ _BV(CS12);
				OCR1B = pwm_tbl[(pack.PIO_Output_Latch_State & 0xf0) >> 4];
			} else {
				TCCR1A = 0;
				TCCR1B = 0;
			}
#endif
		} else {
			for (i = 0; i < MAX_BTN; i++)
				if (config_info[CFG_CFG_ID + i] & CFG_OUT_MASK)
					latch_out(1 << i);
		}
		/* avoid signal generation and let the
		 latch reset activate it again
		 */
		int_signal = SIG_NO;
		gcontrol &= ~0x01;
	}
	if (gcontrol & 2) {
		/* OW_RESET_ACTIVITY, pack data and alarm is reset in asm */
		gcontrol &= ~0x02;
		int_signal = SIG_ARM;
	}
	if (gcontrol & 0x4) {
		cli();
		stat_to_sample=values[ap];
		if (ap++ > CHAN_VALUES-1)
			ap = 0;
		gcontrol &= ~0x04;
		sei();
	}
	if (gcontrol & 0x8) {
		/* read channel data */
		cli();
		stat_to_sample=values[0];
		ap = 1;
		gcontrol &= ~0x08;
		sei();
	}
	if (gcontrol & 0x10) {
		cli();
		gcontrol &= ~0x10;
		/* store crc and once matches the sent one, store eeprom */
		if (config_info[CFG_VERS_ID] == 0x55) {
			config_info[CFG_VERS_ID] = MAJ_VERSION;
			config_info[CFG_VERS_ID + 1] = MIN_VERSION;
			// saving...
			eeprom_write_block((const void*)config_info, (void*)7, CFG_VERS_ID - 1);
#ifdef DUAL_ROM
			eeprom_write_block((const void*)config_info2, (void*)(8 + CFG_TYPE_ID), 22);
#endif
		}
		reg_init();
		sei();
		statusPrint();
	}
}

void statusPrint()
{
#ifdef HAVE_UART
	struct pinState *p;
	int i;

	printf ("\n%d %d %d %d\n", TIMSK & (1<<TOIE0), mode, int_signal, btn_active);
	for (i = 0; i < MAX_BTN; i++) {
		p = &btn[i];
		if (PIN_DDR & pio_map[i]) {
			printf ("#%i out\n", i+1);
		} else {
			printf ("#%i %d %d\n", i+1, p->state, p->press);
		}
	}
#endif
}

static uint8_t auto_toggle(uint8_t i)
{
	uint8_t out;
	out = config_info[CFG_SW_ID + i];

	if (out > 8)
		return 0;

	/* return if not valid, not configured or deactivated */
	out = 1 << (out - 1);
	// toggle output
	if (pack.PIO_Output_Latch_State & out)
		pack.PIO_Output_Latch_State &= ~out;
	else
		pack.PIO_Output_Latch_State |= out;
	latch_out(out);
	// signal not the input, but only the out change
	return 1;
}

/* check fo a defined auto switch with "PASS" config and
   switching the configured pin
   returns 0 if no switch configured otherwise 1
*/
uint8_t auto_switch(uint8_t i, uint8_t val)
{
	uint8_t out;
	out = config_info[CFG_SW_ID + i];

	/* return if not valid, not configured or deactivated */
	if (out > 8)
		return 0;

	out = 1 << (out - 1);
	switch (config_info[CFG_CFG_ID + i]) {
		case CFG_PASS:
			if (val)
				pack.PIO_Output_Latch_State &= ~out;
			else
				pack.PIO_Output_Latch_State |= out;
			break;
		case CFG_PASS_INV:
		case CFG_PASS_INV_PU:
			if (val)
				pack.PIO_Output_Latch_State |= out;
			else
				pack.PIO_Output_Latch_State &= ~out;
			break;
		default:
			return 0;
	}
	latch_out(out);

	// signal not the input, but only the out change
	return 1;
}

uint8_t btn_loop(uint8_t st, uint8_t mask, uint8_t i, struct pinState *p)
{
	uint8_t act_btns = 0;

	switch (st) {
	case BTN_PRESSED_LONG:
		/* active low but for longer time, signal start of pressing long */
		pack.FF1 = 0;
		// get the next edge
		act_btns |= mask;
		act_latch(1, mask);
		break;
	case BTN_PRESSED:
		/* short pressed and done */
		wdr();
		pack.FF1 = p->press / 32;
		if (auto_toggle(i) == 0)
			act_latch(1, mask);
		break;
	case BTN_RELEASED:
		wdr();
		/* long pressed done for push buttons or
			toggled the switch */
		pack.FF1 = p->press / 32;
		act_latch(1, mask);
		break;
	case BTN_PRESS_LOW:
		/* intermediate end state, wait for release
		 * for push buttons.
		 * Switch should go to sleep
		 */
		if (config_info[CFG_CFG_ID + i] == CFG_SW) {
			wdr();
			act_latch(0, mask);
		} else
			act_btns |= mask;
		break;
	case BTN_LOW:
		/* waiting for release for push buttons.
			In case of non push button we are done here */
		if (config_info[CFG_CFG_ID + i] == CFG_BTN)
			act_btns |= mask;
		break;
	/* Internal and intermediate state are ongoing evaluations
	 * BTN_PRESS_LOW BTN_INVALID BTN_UNSTABLE BTN_TIMER_HIGH BTN_TIMER_LOW
	 */
	case BTN_HIGH:
		/* means no change */
		// TODO: break here?? we are done!
		break;
	default:
		act_btns |= mask;
		break;
	}

	return act_btns;
}

void pin_change_loop()
{
	int i;
	uint8_t pins, in, act_btns, mask = 1;
#ifdef DEBUG
	static int stPrints = 50;
#endif

	pins = PIN_REG;
	/* go over all buttons and see whether still validation needs to be done,
	 * otherwise we can go to sleep again (if act_btns = 0) */
	act_btns = 0;
	/* mark as handled for now, if another int occurs, we will see it later.
	 * This is set again after the loop for any outstandng validation or
	 * overridden by int */
	btn_active = 0;
	for (i = 0; i < MAX_BTN; i++) {

		/* ignore outputs and skip unconfigured ones */
		if ((config_info[CFG_CFG_ID + i] & CFG_OUT_MASK) != 0 ||
			 config_info[CFG_CFG_ID + i] == 0) {
			mask = mask << 1;
			continue;
		}
		/* get current pin and set to 0 or 1 */
		in = !!(pins & pio_map[i]);
		/* check config */
		if ((config_info[CFG_CFG_ID + i] & CFG_BTN_MASK) != 0) {
			/* push button handling */
			int st;
			struct pinState *p = &btn[i];

			st = checkBtn(in, p);
			act_btns |= btn_loop(st, mask, i, p);
		} else {
			/* pin change handling without validation */
			switch (config_info[CFG_CFG_ID + i]) {
				case CFG_ACT_HIGH:
					/* alarm only on rising edge */
					if (in == 1) {
						pack.FF1 = 0xff;
						if ((pack.PIO_Logic_State & mask) == 0)
							act_latch(1, mask);
					} else
						pack.PIO_Logic_State &= ~mask;
					break;
				case CFG_ACT_LOW:
					/* alarm only on falling edge, not yet verified */
					pack.FF1 = 0xff;
					if (in == 0)
						act_latch(0, mask);
					else
						pack.PIO_Logic_State |= mask;
					break;
				case CFG_PASS:
				case CFG_PASS_INV:
				case CFG_PASS_INV_PU:
					// latch only on change!
					if ((in && (pack.PIO_Logic_State & mask) == 0) ||
					    (!in && (pack.PIO_Logic_State & mask) != 0)) {
						pack.FF1 = 0xff;
						act_latch(in, mask);
						if (auto_switch(i, in) != 0) {
							/* in case of auto_switch, logic state must be set */
							if (in)
								pack.PIO_Logic_State |= mask;
							else
								pack.PIO_Logic_State &= ~mask;
						}
					}
					break;
				default:
					/* not a button and not configured */
					break;
			}
		}
		mask = mask << 1;
	}
	if (pack.PIO_Activity_Latch_State && !alarmflag) {
		alarmflag = 1;
		pack.Status |= 0x10;
		/* TODO: check whether to alarm or not */
		if ((pack.PIO_Activity_Latch_State & signal_cfg) && int_signal == SIG_ARM)
			int_signal = SIG_ACT;
	}
	if (act_btns != 0)
		btn_active = act_btns;
#ifdef DEBUG
	if (stPrints-- == 0) {
		statusPrint();
	}
#endif
}

void loop()
{
	ow_loop();

	/* ongoing OW access (in interrupt), serve data with high prio and skip others */
	if (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0))
		return;
#ifdef DS1820_SUPPORT
	temp_loop();
#endif
	if (btn_active)
		pin_change_loop();
	if (int_signal == SIG_ACT)
		owResetSignal();
	cli();
	if (btn_active || int_signal == SIG_ACT || TCCR1 != 0) {
		sei();
		if (TCCR1 != 0)
			wdr();
		delay(1);
	}
	else {
#if defined(WDT_ENABLED) && !defined(DS1820_SUPPORT)
		wdt_disable();
#endif
		LED_OFF();
		OWST_MAIN_END
		if (mode == 0) {
#ifndef AVRSIM
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
#endif
		}
		sei();
#if defined(WDT_ENABLED) && !defined(DS1820_SUPPORT)
		wdt_enable(WDTO_8S);
#endif
		LED_ON();
	}
}

#if defined(UNIT_TEST) || defined(AVRSIM)
int main_func(void)
#else
int main(void)
#endif
{
	setup();

	while (1) {
		loop();
	}

	return 0;
}
