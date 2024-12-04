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
#include "wiring.h"
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
#ifdef DS2450_SUPPORT
#include "DS2450.h"
#endif

#define CFG_CUSTOM1 39 // (7 + CFG_TYPE_ID + 9)

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

void setup();
void loop();
static void ow_loop();
static uint8_t btn_loop(uint8_t st, uint8_t mask, uint8_t i, struct pinState *p);
static void pin_change_loop();

static void owResetSignal(void);
static void reg_init(void);
static void var_init(void);
static void cfg_init(void);
static void auto_toggle(uint8_t i);
#ifdef TIMER_SUPPORT
static void timed_switch(uint8_t type, uint8_t tim);
#endif /* TIMER_SUPPORT */
void statusPrint();
static void pin_set(uint8_t p, uint8_t id, uint8_t bb);

/* bit number in PIN register to IO mapping */
const uint8_t pio_map[MAX_BTN] = {
	PIN_PIO0,
	PIN_PIO1,
#ifdef PIN_PIO2
	PIN_PIO2,
#endif
#ifdef PIN_PIO3
	PIN_PIO3,
#endif
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

volatile pack_t pack;

#ifdef DUAL_ROM
uint8_t config_info1[26];
#define config_info config_info1
uint8_t config_info2[8];
#else
#endif
#ifdef TIMER_SUPPORT
volatile pin_t pin_tmr;
uint8_t cfg_custom1[22];
uint8_t cfg_type;

struct {
	unsigned long tmr;
	uint8_t pin;
	uint8_t lvl;
	uint8_t type;
}tmr[MAX_TIMER];
#endif /* TIMER_SUPPORT */

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
/** Activity state bitmask
 * ACT_BUTTON (1): button handling active
 * x: timer running
 */
volatile uint8_t active = 0;
#ifdef WITH_LEDS
static uint8_t led2 = 0;
#endif

#ifdef WITH_LEDS_FLASH
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

// TODO use assembly function
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
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map); i++) {
		uint8_t p = pio_map[i];
		if (p != 0) {
			switch (config_info[CFG_CFG_ID + i])
			{
				default:
					if (pins & p)
						pack.PIO_Logic_State |= mask;
					else
						pack.PIO_Logic_State &= ~mask;
					break;
				case CFG_OUT_HIGH:
					if (pins & p)
						pack.PIO_Logic_State &= ~mask;
					else
						pack.PIO_Logic_State |= mask;
					break;
			}
		}
		mask = mask << 1;
	}
}

ISR(PCINT0_vect) {
	active |= ACT_BUTTON;
#if defined(GIFR) && defined(PCIF0)
	GIFR = _BV(PCIF0);
#endif
#if defined(GIFR) && defined(PCIF)
	GIFR = _BV(PCIF);
#endif
}

/* sets a pin, but does not change any interrupt on that pin.
  If the pin was configured as input, the interupt is active. */
static inline void pin_set(uint8_t p, uint8_t id, uint8_t bb)
{
	if (pack.PIO_Output_Latch_State & bb) {
		/* According to spec:
		 * set 1 / non-conducting (off)
		 * set to input or inactive
		 */
		pack.PIO_Logic_State |= bb;
		if (config_info[CFG_CFG_ID + id] == CFG_OUT_PWM ||
			config_info[CFG_CFG_ID + id] == CFG_OUT_HIGH) {
			/* output low */
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

#if 0
	/* code saver, but maybe slower
	 * saves 100 bytes of code
	 */
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map) / sizeof(pio_map[0]); i++) {
		if (bb == mask) {
			p = pio_map[i];
			id = i;
		}
		mask = mask << 1;
	}
#else
	switch (bb)
	{
		case 0x1:
			id = 0;
			break;
		case 0x2:
			id = 1;
			break;
		case 0x4:
			id = 2;
			break;
		case 0x8:
			id = 3;
			break;
#ifdef PIN_PIO4
		case 0x10:
			id = 4;
			break;
#endif
#ifdef PIN_PIO5
		case 0x20:
			id = 5;
			break;
#endif
#ifdef PIN_PIO6
		case 0x40:
			id = 6;
			break;
#endif
#ifdef PIN_PIO7
		case 0x80:
			id = 7;
			break;
#endif
		default:
			return;
	}
#endif
	p = pio_map[id];
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
static void var_init(void)
{
	int i;

	/* all off */
	pack.PIO_Logic_State = 0xff;
	pack.PIO_Output_Latch_State = 0xff;
	pack.FF1 = 0xFF;
	pack.FF2 = 0xFF;
	alarmflag = 0;
	active = 0;
	for (i = 0;i < MAX_BTN;i++)
		initBtn(1, &btn[i]);
#ifdef TIMER_SUPPORT
	tmr[0].lvl = 0;
#endif
	pack.PIO_Activity_Latch_State = 0;
	/* enable alarm state reporting */
	pack.Conditional_Search_Channel_Selection_Mask = 0xFF;
	signal_cfg = 0xff;

	int_signal = SIG_ARM;
}

static void reg_init(void)
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
		case CFG_OUT_PWM:
			/* output initially active low
			 * PIO_Logic_State (inverted) is high cause it
			 * represents transistor output
			 */
			PCMSK &= ~(p); // disable interrupt
			PIN_DDR |= (p);
			PORT_REG &= ~(p);
#if defined(__AVR_ATtiny85__)
			TCNT1 = 0xff;
			OCR1C = 0xff;
#endif
			break;
			/* fall-through */
		case CFG_OUT_LOW:
			/* default as DS2408, active low, not setting to output yet
			 * this will be done in activation of latch
			 */
			PORT_REG |= p;
			PCMSK &= ~(p);
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
	/* timer 1 not yet needed */
	PRR |= _BV(PRTIM1);

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

/* EEPROM
  0  | 1 | 2 | 3   4   5   6   7   8   9  10  11  12 13 14 15 16 17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
   CFG                         0  | 1 |2 |3   4   5  6  7  8  9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27
   PINCFG                                 0 | 1  |2 |3  4  5  6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  
| 29  ID  bus  ~   ~   66  77 |CRC| RES  |SW  1   2  3  4  5  6   7  |CFG 1   2   3   4   5   6   7  |FEA|R  |MAJ|MIN|TYP| OFF  | FACT
*/
static void cfg_init(void)
{
	uint8_t crc_calc;
#ifndef AVRSIM
	int to = 100;

	while (!eeprom_is_ready() && to--)
		_delay_ms(2);

	eeprom_read_block((void*)owid, (const void*)0, 8);
#endif /* AVRSIM */
	crc_calc = crc(owid, 7);
	if (owid[0] != 0x29 || crc_calc != owid[7]) {
		owid[0] = 0x29;
		owid[1] = DEFAULT_OWID_ADR;
		owid[2] = DEFAULT_OWID_BUS;
		owid[3] = ~owid[1];
		owid[4] = ~owid[2];
		owid[5] = 0x66;
		owid[6] = 0x77;
		owid[7] = crc(owid, 7);
	}
#ifdef DUAL_ROM
	for (int i = 1; i < 7; i++)
		owid2[i] =  owid[i];
#ifdef DS1820_SUPPORT
	owid2[0] = 0x28;
#endif
#ifdef DS2450_SUPPORT
	owid2[0] = 0x20;
#endif
	owid2[7] = crc(owid2, 7);
#endif
#ifndef AVRSIM
	eeprom_read_block((void*)&config_info, (const void*)7, CFG_TYPE_ID);
#ifdef DUAL_ROM
	eeprom_read_block((void*)&config_info2, 
		(const void*)(8 + CFG_TYPE_ID), 
		sizeof(config_info2));
#endif
#ifdef TIMER_SUPPORT
	eeprom_read_block((void*)&cfg_custom1, (const void*)CFG_CUSTOM1, sizeof(cfg_custom1));
#endif /* TIMEOUT_SUPPORT */
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
	if (config_info[CFG_TYPE_ID] == 0xff) {
#ifdef DS1820_SUPPORT
		config_info[CFG_TYPE_ID] = 11;
#else
		/* default new version */
		config_info[CFG_TYPE_ID] = 1;
#endif
	}
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
#ifndef AVRSIM
	/* let the power supply get stable */
	_delay_ms(50);
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
#ifdef WITH_LEDS_FLASH
	led_flash();
#endif
	if (pack.Status == 0x88) {
		/* alarm! */
		int_signal = SIG_ACT;
		alarmflag = 1;
	}
}

static void ow_loop()
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
		uint8_t i;
		// TODO check standard IO handling with PWM outputs
		// started via pin timer
		// TODO setting off via latch must stop timer?
		for (i = 0; i < MAX_BTN; i++)
			if (config_info[CFG_CFG_ID + i] & CFG_OUT_MASK)
				latch_out(1 << i);
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
#if 0
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
#endif
	if (gcontrol & 0x10) {
		/* this is set when all bytes (22) were received */
		cli();
		gcontrol &= ~0x10;
		/* store crc and once matches the sent one, store eeprom */
		if (config_info[CFG_VERS_ID] == 0x55) {
			config_info[CFG_VERS_ID] = MAJ_VERSION;
			config_info[CFG_VERS_ID + 1] = MIN_VERSION;
			// saving...
			eeprom_update_block((const void*)config_info, (void*)7, CFG_VERS_ID - 1);
#ifdef DUAL_ROM
			eeprom_write_block((const void*)config_info2, (void*)(8 + CFG_TYPE_ID), 22);
#endif
		}
		reg_init();
		sei();
		statusPrint();
	}
	if (gcontrol & 0x40) {
		cli();
		gcontrol &= ~0x40;
		sei();
#ifdef TIMER_SUPPORT
		timed_switch(pin_tmr.type, pin_tmr.val1);
#endif /* TIMER_SUPPORT */
	}
}

void statusPrint()
{
#ifdef HAVE_UART

	printf ("\n%d %d %d %d\n", TIMSK & (1<<TOIE0), mode, int_signal, active);
#if 0
	struct pinState *p;
	for (int i = 0; i < MAX_BTN; i++) {
		p = &btn[i];
		if (PIN_DDR & pio_map[i]) {
			printf ("#%i out %d\n", i+1, ((PORT_REG & pio_map[i]) == pio_map[i]));
		} else {
			printf ("#%i in %d %d\n", i+1, p->state, p->press);
		}
		mask = mask << 1;
	}
#endif
	printf("ACT %X LOGIC %X OUT %X DDR %X PORT %X\n", pack.PIO_Activity_Latch_State,
	       pack.PIO_Logic_State,
	       pack.PIO_Output_Latch_State, PIN_DDR, PORT_REG);
#endif
}

/**
 * press 0 for short, 1 for long
 */
static void auto_toggle(uint8_t i)
{
	uint8_t out;
	uint8_t cfg = config_info[CFG_SW_ID + i];
#if defined(TIMER_SUPPORT) || defined(LONGPRESS_AUTO_SUPPORT)
	uint8_t cfgx = cfg_custom1[CFG_CUSTOM1_SWA0 + i];
#endif /* TIMER_SUPPORT */

	/* output 0 or 0xff not configured. */
	if (cfg == 0xff || cfg == 0)
		return;
#ifdef TIMER_SUPPORT
	/* switch space at 5 */
	if (cfg & 0x20 && cfgx != 0xff) {
		/* extended config */
		// out 0 .. 7
		out = (cfgx & 0x7) - 1;
		if (out >= 0 && out < 8) {
			pin_tmr.pin = out;
			pin_tmr.val2 = 0xFE;
			timed_switch(cfgx & 0xf0, cfg_custom1[CFG_CUSTOM1_TM1/* + tmr_nr */]);
		}
	}
#endif /* TIMER_SUPPORT */
#ifdef LONGPRESS_AUTO_SUPPORT
	if (config_info[CFG_SW_ID + i] & 0x40 &&
	    cfg_custom1[CFG_CUSTOM1_SWB0 + i] != 0xff ) {
	) {
	}
#endif /* LONGPRESS_AUTO_SUPPORT*/
	/* output pin value 1 .. 7, 0 or 0xff not configured.
	 * For the sake of saving config space auto switching
	 * pin#8 is not supported */
	out = cfg & 0x1f;
	if (out == 0 || out > 8)
		return;
	/* return if not valid, not configured or deactivated */
	out = 1 << (out - 1);
	// toggle output
	if (pack.PIO_Output_Latch_State & out)
		pack.PIO_Output_Latch_State &= ~out;
	else
		pack.PIO_Output_Latch_State |= out;
	latch_out(out);
}

/* check for a defined auto switch with "PASS" config and
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

#ifdef TIMER_SUPPORT
static void timed_switch_stopping();
static void timed_switch_stop();

/* Start a timer or do related configs
 * Timer start needs pin_tmr.pin (0..7) and pin_tmr.val2 (level)
 */
static void timed_switch(uint8_t type, uint8_t tim)
{
	uint8_t out;
#if MAX_TIMER==1
	const uint8_t tmr_nr = 0;
#else
	// TDOD add more timers
	// look for available timer
	uint8_t tmr_nr;
#endif
	static uint8_t brightness = 0x80;

	if (type == TMR_TYPE_BRIGHTNESS) {
		brightness = pin_tmr.val1;
		return;
	}
	if (type == TMR_TYPE_THRESHOLD) {
		cfg_custom1[0] = 1;
		cfg_custom1[CFG_CUSTOM1_THR] = pin_tmr.val1;
		// save in eeprom
		eeprom_update_block((void*)cfg_custom1, (void*)CFG_CUSTOM1, sizeof(cfg_custom1));
		return;
	}
	if (type == TMR_TYPE_STOP_DIM) {
		timed_switch_stopping();
		return;
	}
	if (type == TMR_TYPE_STOP) {
		timed_switch_stop();
		return;
	}
	/* anything else != 0 will trigger a timer */
	if (type == 0)
		return;
	out = 1 << (pin_tmr.pin);
	/* check if pin is already on without timer.
		* In that case don't start the timer.
		* If a timer is running retrigger */
	if ((pack.PIO_Output_Latch_State & out) == 0 &&
#if MAX_TIMER==1
		(active & ACT_TIMER1) == 0)
#endif
		return;
	/* if ((type == TMR_TYPE_TRG_DIM_DARK) || (type == TMR_TYPE_TRG_DIM) |
	    (type == TMR_TYPE_START_DIM_DARK) || (type == TMR_TYPE_TRG_DARK)) */
	if ((type >= TMR_TYPE_TRG_DARK) && (type <= TMR_TYPE_START_DIM_DARK)) {
		/* check if this is supported by FEAT otherwise 
			use external setting TMR_TYPE_BRIGHTNESS */
		if ((config_info[CFG_CFG_FEAT] & FEAT_ADC_LIGHT) == 0) {
			int a = analogRead(0);
			brightness = (a >> 2) & 0xFE;
		}
		if (brightness < cfg_custom1[CFG_CUSTOM1_THR]) {
			/* day time */
			return;
		}
	}
	// Button toggle not yet implemented!
	pack.PIO_Output_Latch_State &= ~out;
	LED2_ON();
	if (config_info[CFG_CFG_ID + pin_tmr.pin] == CFG_OUT_PWM) {
		analogWrite(pio_map[pin_tmr.pin], pin_tmr.val2);
		pack.PIO_Logic_State &= ~out;
		latch_state(out);
		alarmflag = 1;
		if (int_signal == SIG_ARM)
			int_signal = SIG_ACT;
	} else
		latch_out(out);

	tmr[tmr_nr].lvl = pin_tmr.val2;
	tmr[tmr_nr].type = type;
	if(type == TMR_TYPE_ON)
		return;
#if MAX_TIMER==1
	active |= ACT_TIMER1;
#else
	//TODO find free timer if more supported
#endif
	pack.Status |= 0x20;
	tmr[tmr_nr].pin = pin_tmr.pin;
	// TODO not working on tiny???
	//uint16_t t = tim * cfg_custom1[CFG_CUSTOM1_DIF];
	tmr[tmr_nr].tmr = millis() + tim * 1000 * 10;
	statusPrint();
}

static void timed_switch_stop()
{
	uint8_t out;
#if MAX_TIMER==1
	const uint8_t tmr_nr = 0;

	active &= ~(ACT_TIMER1 | ACT_DIM_DN1);
#endif /* MAX_TIMER */
	tmr[tmr_nr].lvl = 0;
	out = 1 << (tmr[tmr_nr].pin);
	pack.PIO_Output_Latch_State |= out;
	pack.Status &= ~0x60;
	if (config_info[CFG_CFG_ID + tmr[tmr_nr].pin] == CFG_OUT_PWM) {
		analogWrite(pio_map[tmr[tmr_nr].pin], 0);
		pack.PIO_Logic_State &= ~out;
		latch_state(out);
		if (int_signal == SIG_ARM && !alarmflag) {
			int_signal = SIG_ACT;
			alarmflag = 1;
		}
	} else
		latch_out(out);
	statusPrint();
}

/* Dimming down. Check pin config or stored type for support before */
static void timed_switch_stopping()
{
#if MAX_TIMER==1
	const uint8_t tmr_nr = 0;
#endif /* MAX_TIMER */

	/* now off */
	LED2_OFF();
	if (tmr[tmr_nr].lvl == 0)
		return;
	active &= ~ACT_TIMER1;
	active |= ACT_DIM_DN1;
	tmr[tmr_nr].tmr = millis() + cfg_custom1[CFG_CUSTOM1_DIMD];
	tmr[tmr_nr].lvl--;
	analogWrite(pio_map[pin_tmr.pin], tmr[tmr_nr].lvl);
	pack.Status &= ~0x20;
	pack.Status |= 0x40;
	if (int_signal == SIG_ARM && !alarmflag) {
		int_signal = SIG_ACT;
	 	alarmflag = 1;
	}
}

/* for a running timer this function gets called every 1 ms 
 */
static void timed_switch_loop()
{
#if MAX_TIMER==1
	const uint8_t tmr_nr = 0;
#endif /* MAX_TIMER */

	/* timer check */
	if ((tmr[tmr_nr].tmr == 0 || millis() < tmr[tmr_nr].tmr))
		return;
	/* expired */
	if (active & ACT_TIMER1) {
		if (tmr[tmr_nr].type == TMR_TYPE_TRG_DIM_DARK ||
		 tmr[tmr_nr].type == TMR_TYPE_TRG_DIM /*||
		 tmr[tmr_nr].type == TMR_TYPE_START_DIM_DARK || 
		 tmr[tmr_nr].type == TMR_TYPE_START_DIM*/) {
			timed_switch_stopping();
		 } else
			timed_switch_stop();
	}
	if (active & ACT_DIM_DN1) {
		tmr[tmr_nr].tmr = millis() + cfg_custom1[CFG_CUSTOM1_DIMD];
		if (tmr[tmr_nr].lvl-- == 0) {
			timed_switch_stop();
			return;
		}
		analogWrite(pio_map[tmr[tmr_nr].pin], tmr[tmr_nr].lvl);
	}
}
#endif /* TIMER_SUPPORT */

static uint8_t btn_loop(uint8_t st, uint8_t mask, uint8_t i, struct pinState *p)
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
		// TODO if long press auto switch supported add press type
		act_latch(1, mask);
		auto_toggle(i);
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
		/* means no change for btn, but signal for switch */
		if (config_info[CFG_CFG_ID + i] == CFG_SW) {
			wdr();
			act_latch(0, mask);
		}
		break;
	default:
		act_btns |= mask;
		break;
	}

	return act_btns;
}

static void pin_change_loop()
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
						auto_toggle(i);
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
					/* filter out disturber */
					_delay_ms(5);
					in = !!(PIN_REG & pio_map[i]);
					/* latch only on change! */
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
		/* TODO: check whether to alarm or not */
		if ((pack.PIO_Activity_Latch_State & signal_cfg) && int_signal == SIG_ARM)
			int_signal = SIG_ACT;
	}
	if (act_btns != 0)
		active |= ACT_BUTTON;
	else
		active &= ~ACT_BUTTON;
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
	if (((TIMSK & (1 << TOIE0)) != 0) || (mode != 0))

		return;
#ifdef DS1820_SUPPORT
	temp_loop();
#endif
#ifdef DS2450_SUPPORT
	adc_loop();
#endif
	if (active & ACT_BUTTON)
		pin_change_loop();
	if (int_signal == SIG_ACT)
		owResetSignal();
	cli();
	if (active || int_signal == SIG_ACT || TCCR1 != 0) {
		sei();
		wdr();
		delay(1);
#ifdef TIMER_SUPPORT
		if (active & ACT_TIMER)
			timed_switch_loop();
#endif /* TIMER_SUPPORT */
	} else {
#if defined(WDT_ENABLED) && !defined(DS1820_SUPPORT)
		wdt_disable();
#endif
		LED_OFF();
		OWST_MAIN_END;
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

#if !defined(UNIT_TEST) && !defined(AVRSIM)
int main(void)
{
	setup();
#ifdef DEBUG
	statusPrint();
#endif
	while (1) {
		loop();
	}

	return 0;
}
#endif
