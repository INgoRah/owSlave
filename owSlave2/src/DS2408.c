
// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
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
#ifdef WDT_ENABLED
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

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);
extern uint8_t stat_to_sample;
static void owResetSignal(void);
void reg_init(void);
void var_init(void);

OWST_EXTERN_VARS

void statusPrint();

/* last byte will be calculated, program like
C:\Users\rah\.platformio\packages\tool-avrdude
avrdude -C C:\Users\rah\.platformio\packages\tool-avrdude\avrdude.conf -c stk500v2 -P COM13 -p attiny85 -U eeprom:w:0x29,0x11,0x04,0x01,0x2,0x66,0x77:m
		 */
uint8_t owid[8];

/* bit number in PIN register to IO mapping */
const uint8_t pio_map [] = {
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
	PIN_PIO7,
#endif
	0
};

volatile pack_t pack;

/*
 * config from EEPROM
 * | btn | pin | pol | sw0 |  sw1 |  sw2 | sw3 | sw4 | sw5 | sw6 | sw7 |
 *   cfg0 | cfg1 | cfg2 | cfg3 |cfg4 | cfg5| cfg6 | cfg7 | feature | x |
 *   maj_vers | min_vers | type
 */
uint8_t config_info[26];
uint8_t signal_cfg;
unsigned long _ms;

struct pinState btn[MAX_BTN];
uint8_t values[CHAN_VALUES];
uint8_t ap = 0;
volatile uint8_t int_signal;
volatile uint8_t btn_active = 0;
volatile uint8_t pin_state;
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

static int getADC()
{
  while((ADCSRA & _BV(ADSC)));    // Wait until conversion is finished
  return ADC;
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
#if 0
	why was this??
	if ((config_info[CFG_POL_ID] & p) == 0)
		/* ignore logic state here cause inverted and set before */
		return;
#endif
	if (pins & p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
}

static inline void latch_state(uint8_t pins, uint8_t p, uint8_t mask)
{
#if 0
	if (pins & p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
#endif
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
	pin_state = pins & (~PIN_DDR);
}

uint8_t wdcounter=0;

#ifdef WDT_vect
ISR(WDT_vect) 
#else
ISR(WATCHDOG_vect) 
#endif
{
	wdcounter++;
	if (reset_indicator==1)
		reset_indicator++;
	else if (reset_indicator == 2)
		mode=0;
	reg_init();
	var_init();
	LED2_ON();
}

ISR(PCINT0_vect) {
	uint8_t pins;

	pins = PIN_REG;
	// remove LED and output
	pins &= (~PIN_DDR);

	if (pin_state != pins) {
		btn_active = 1;
		pin_state = pins;
#ifdef DEBUG
		printf("%02X ", pins);
#endif
	}
#if defined(GIFR) && defined(PCIF0)
	GIFR = _BV(PCIF0);
#endif
#if defined(GIFR) && defined(PCIF)
	GIFR = _BV(PCIF);
#endif
}

// when ADC completed, take an interrupt 
EMPTY_INTERRUPT (ADC_vect);

void pin_set(uint8_t p, uint8_t id, uint8_t bb)
{
	if (pack.PIO_Output_Latch_State & bb) {
		/* According to spec:
		 * set 1 / non-conducting (off)
		 * set to input or inactive
		 */
		if (config_info[CFG_CFG_ID + id] == CFG_PIN_PWM ||
			(config_info[CFG_POL_ID] & p) == 0) {
			/* no pull up */
			PIN_DDR |= p;
			PORT_REG &= ~p;
			pack.PIO_Logic_State |= bb;
			if (config_info[CFG_CFG_ID + id] == 1)
#if defined(__AVR_ATtiny85__)
				GTCCR &= ~(1<<PWM1B);
#else
				;
#endif
		} else {
			/* enable pull up */
			PIN_DDR &= ~(p);
			PORT_REG |= (p);
			PCMSK |= p;
		}
	} else {
		/* set 0 / 0 = conducting (on) */
		PCMSK &= ~(p);
		if ((config_info[CFG_POL_ID] & p) == 0) {
			/* set output and high on 0 */
			PIN_DDR |= p;
			PORT_REG |= (p);
			pack.PIO_Logic_State &= ~bb;
		} else {
			/* set output and low on 0 */
			PIN_DDR |= p;
			PORT_REG &= ~p;
		}
		if (config_info[CFG_CFG_ID + id] != 0xff) {
#if defined(__AVR_ATtiny85__)
			OCR1B = (pack.PIO_Output_Latch_State & 0xf0) >> 4;
			// PWM-Mode, !OC1B set on compare match, Cleared when TCNT1 = 0
			GTCCR = (1<<PWM1B) | (1<<COM1B0);
#else
				;
#endif
		}
	}
}

void latch_out(uint8_t bb)
{
	uint8_t p = 0, id;

	/* 
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map) / sizeof(pio_map[0]); i++) {
		if (bb == mask) {
			p = pio_map[i];
			id = i;
			break;
		}
		mask = mask << 1;
	}
	*/
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
	cli();
	/* logic state = real state, output_latch state inverted! */
	if ((pack.PIO_Logic_State & bb) !=
		(pack.PIO_Output_Latch_State & bb)) {
		/* pin change by write */
		printf ("set %02X to %d\n", bb, pack.PIO_Output_Latch_State & bb);
		pin_set(p, id, bb);
		sync_pins();
		latch_state(PIN_REG, p, bb);
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

	OWST_INIT_ALL_OFF;
	OWST_EN_PULLUP
	/* pull ups on all pins set by OWST_INIT_ALL_OFF */
	OWINIT();
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__) || defined(ATMEGA)
	/* ignoring changes on output */
	PCMSK0 = (0xFF - PIN_PIO0 - PIN_PIO1);
#endif
#if defined(__AVR_ATtiny85__)
	/* PIN_DDR |= PIN_PIO0 | PIN_PIO1; */
	PCMSK = (_BV(PCINT0) | _BV(PCINT1) /* | _BV(PCINT3) */);
#endif
#if defined(PCICR) && defined(PCIE0)
	PCICR = _BV(PCIE0);
#endif
	for (int i = 0; i < MAX_BTN; i++) {
		p = pio_map[i];
		/* check config */
		if (config_info[CFG_CFG_ID + i] != CFG_DEFAULT)
			/* no pull up */
			PORT_REG &= ~(p);
		if ((config_info[CFG_POL_ID] & p) == 0) {
			PORT_REG &= ~(p);
			if (config_info[CFG_PIN_ID] & p) {
				/* output initially active low */
				PIN_DDR |= (p);
				PCMSK &= ~(p);
				/* PIO_Logic_State (inverted) is high cause it
				represents transistor output */
			}
		}
	}
#if defined(__AVR_ATtiny85__)
	if (config_info[CFG_CFG_ID] == CFG_ACT_PWM) {
		/* PWM on PB4 */
		PCMSK &= ~(_BV(PB4));
#endif
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	if (config_info[CFG_CFG_ID + 5] == CFG_ACT_PWM) {
		PIN_DDR |= _BV(PA5);
		PCMSK &= ~(0x20);
#endif
		TCNT1 = 0xf;
#ifndef ATMEGA
	} else {
		/* timer 1 not needed */
		PRR |= PRTIM1;
	}
#endif
	sync_pins();
}

void cfg_init(void)
{
#ifdef _CHANGEABLE_ID_
	uint8_t crc_check;
#endif
#ifndef AVRSIM
	int to = 100;

	/* let power stabalize */
	/*_delay_ms(20); */
	while (!eeprom_is_ready() && to--)
		_delay_ms(1);

	eeprom_read_block((void*)owid, (const void*)0, 7);
#endif
	if (owid[3] != ~owid[1]) {
		owid[3] = ~owid[1];
		owid[4] = ~owid[2];
		owid[7] = crc(owid, 7);
	}
#ifdef _CHANGEABLE_ID_
	crc_check = eeprom_read_byte((const uint8_t*)E2END);
	if (crc_check != 0xff)
		eeprom_write_block((const void*)owid, (void*)E2END - 7, 8);
#endif
#ifndef AVRSIM
	eeprom_read_block((void*)&config_info, (const void*)7, CFG_TYPE_ID);
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
		/* default new version */
		config_info[CFG_TYPE_ID] = 1;
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
	}
	else
#endif
		pack.Status = 0x80;

#ifdef HAVE_UART
	serial_init();
#endif
	cfg_init();
	var_init();
	reg_init();
	sei();
#ifndef AVRSIM
	_delay_ms(100);
#endif
#ifdef WITH_LEDS
	led_flash();
#endif
#ifdef HAVE_UART
	printf ("%02x...%02x %02x - %02X %02X | %02X %02X\n", owid[1], owid[2], owid[7],
		config_info[CFG_BTN_ID], config_info[CFG_PIN_ID], config_info[CFG_SW_ID],
		config_info[CFG_SW_ID + 1]);
	statusPrint();
#endif
}

void temp_read()
{
	uint16_t temp;

	PRR &= ~(1<<PRADC);
	_delay_us (10);
	ADCSRB &= ~(1<<ACME);
	ADMUX = _BV(REFS1) | 0x22; // B00100010
	/* prescaler of 128 */
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
	_delay_us (20);

	/* low noise not working */
	ADCSRA |=  _BV(ADIF);  // enable ADC, turn off any pending interrupt
	cli ();
	set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
	sleep_enable();  
	// start the conversion
	ADCSRA |= _BV(ADSC) | _BV(ADIE);
	sei ();
	sleep_cpu ();     
	sleep_disable ();
	/* awake again, reading should be done, but better make sure
	   maybe the timer interrupt fired */
	while (bit_is_set (ADCSRA, ADSC));
	/* Calculate the temperature in C */
	temp = getADC() - 275 + (int)config_info[CFG_OFFSET];
	pack.FF1 = (uint8_t)(temp & 0xff);
	
	ADCSRA = 0;
	PRR |= (1<<PRADC);  /*Switch off adc for save Power */
	pack.Status |= 0x40;
}

void ow_loop()
{
	static uint8_t out_latch = 0xff;
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

		if (pack.PIO_Output_Latch_State == 0x44 &&
			(config_info[CFG_CFG_FEAT] & FEAT_TEMP) == 0) {
			temp_read();
			pack.PIO_Output_Latch_State = out_latch;
		}
		else if (config_info[CFG_CFG_ID] == CFG_ACT_PWM) {
#if defined(__AVR_ATtiny85__)
			if (pack.PIO_Output_Latch_State > 0) {
				TCCR1 = (1<<CTC1) | (1<<CS13);
				OCR1C = 0xf;
				GTCCR = (1<<PWM1B) | (1<<COM1B0);
				OCR1B = (pack.PIO_Output_Latch_State & 0xf0) >> 4;
			}
			else {
				TCCR1 = 0;
				GTCCR = 0;
			}
#endif
		} else if (config_info[CFG_CFG_ID + 5] == CFG_ACT_PWM) {
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (pack.PIO_Output_Latch_State > 0) {
				/* PWM on PA5 / PIO5 */
				TCCR1A = _BV(COM1B1) /*| _BV(COM1B0)*/ | _BV(WGM00);
				TCCR1B = /*_BV(WGM13) | _BV(WGM12) | */ _BV(CS12);
				OCR1B = pack.PIO_Output_Latch_State;
			} else {
				TCCR1A = 0;
				TCCR1B = 0;
			}
#endif
		} else {
			out_latch = pack.PIO_Output_Latch_State;
			printf ("LogicState=%02X OutLatch=%02X\n", pack.PIO_Logic_State, pack.PIO_Output_Latch_State);
			for (i = 1; i < 0x80; i = i * 2)
				latch_out(i);
		}
		// avoid signal generation
		int_signal = SIG_NO;
		gcontrol &= ~0x01;
	}
	if (gcontrol & 2) {
		/* OW_RESET_ACTIVITY */
		cli();
		pack.PIO_Activity_Latch_State = 0;
		pack.Status &= 0x3F;
		pack.FF1 = 0xff;
		gcontrol =  0;
		alarmflag = 0;
		int_signal = SIG_ARM;
		sei();
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
		printf ("%02x %02x %02x %02X %02X %02X\n", config_info[CFG_BTN_ID],
		config_info[CFG_PIN_ID], config_info[CFG_SW_ID], config_info[CFG_SW_ID + 1],
		config_info[CFG_SW_ID + 2], config_info[CFG_SW_ID + 3]);
		cli();
		gcontrol &= ~0x10;
		/* store crc and once matches the sent one, store eeprom */
		if (config_info[CFG_VERS_ID] == 0x55) {
			config_info[CFG_VERS_ID] = MAJ_VERSION;
			PIN_DDR |= 0x2;
			PORT_REG &= ~0x02;
			// saving...
			eeprom_write_block((const void*)config_info, (void*)7, CFG_VERS_ID - 1);
		}
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

static uint8_t auto_switch(uint8_t i)
{
	uint8_t out;
	out = config_info[CFG_SW_ID + i];

	if (out == 0xff)
		return 0;

	printf ("auto sw %d -> %d (%02X)\n", i, out, pack.PIO_Output_Latch_State);
	// toggle output
	if (pack.PIO_Output_Latch_State & out)
		pack.PIO_Output_Latch_State &= ~out;
	else
		pack.PIO_Output_Latch_State |= out;
	latch_out(out);
	// signal not the input, but only the out change
	return 1;
}

uint8_t btn_loop(uint8_t st, uint8_t mask, uint8_t i, struct pinState *p)
{
	uint8_t act_btns = 0, out;

	switch (st) {
	case BTN_PRESSED_LONG:
		/* active low but for longer time */
		pack.FF1 = p->press / 8;
		// get the next edge
		act_btns |= mask;
		statusPrint();
		break;
	case BTN_PRESSED:
		/* short pressed and done */
		wdr();
		pack.FF1 = p->press / 8;
		if (auto_switch(i) == 0)
			act_latch(1, mask);
		printf ("#%i press\n", i);
#ifdef DEBUG
		if (p->cnt > 10) {
			p->cnt = 0;
			statusPrint();
		}
#endif
		break;
	case BTN_RELEASED:
		wdr();
		/* long pressed done for push buttons or
			toggled the switch */
		pack.FF1 = p->press / 8;
		act_latch(1, mask);
		printf ("#%i release\n", i);
		break;
	case BTN_PRESS_LOW:
		/* intermediate end state, wait for release
			* for push buttons.
			* Switch should go to sleep
			**/
		act_btns |= mask;
		if (config_info[CFG_BTN_ID] & mask) {
			wdr();
			act_latch(0, mask);
		}
		break;
	case BTN_LOW:
		/* waiting for release for push buttons. 
			In case of non push button we are done here */
		if (!(config_info[CFG_BTN_ID] & mask))
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
	act_btns = 0;
	/* mark as handled for now, if another int occurs, we will see it later.
	 * This is set again after the loop for any outstandng validation or
	 * overridden by int */
	btn_active = 0;
	for (i = 0; i < MAX_BTN; i++) {

		/* ignore outputs */
		if ((config_info[CFG_PIN_ID] & pio_map[i]) != 0) {
			mask = mask << 1;
			continue;
		}
		in = !!(pins & pio_map[i]);
		/* check config */
		if ((config_info[CFG_BTN_ID] & pio_map[i]) == 0) {
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
						if ((pack.PIO_Logic_State & mask) == 0) {
							if (auto_switch(i) == 0)
								act_latch(1, mask);
							pack.PIO_Logic_State |= mask;
						}
					}
					else
						pack.PIO_Logic_State &= ~mask;
					break;
				case CFG_ACT_LOW:
					/* alarm only on falling edge, not yet verified */
					if (in == 0)
						act_latch(0, mask);
					else
						pack.PIO_Logic_State |= mask;
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
		btn_active = (act_btns << 1);
#ifdef DEBUG
	if (stPrints-- == 0) {
		statusPrint();
	}
#endif
}

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define TCCR1 TCCR1B
#endif
#if defined(ATMEGA)
#define TCCR1 TCCR1B
#endif

void loop()
{
	ow_loop();

	/* ongoing OW access (in interrupt), serve data with high prio and skip others */
	if (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0))
		return;
	if (btn_active)
		pin_change_loop();
	if (int_signal == SIG_ACT)
		owResetSignal();
	cli();
	if (btn_active || int_signal == SIG_ACT || TCCR1 != 0) {
		sei();
		if (TCCR1 != 0)
			wdr();
#ifndef AVRSIM
		_delay_ms(1);
#endif
		_ms++;
	}
	else {
#ifdef WDT_ENABLED
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
#ifdef WDT_ENABLED
		wdt_enable(WDTO_4S);
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
