
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

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
#define ATMEGA
#endif
#if defined(ATMEGA)
#define PCINT_VECTOR PCINT0_vect
#define PCMSK PCMSK0
#define TIMSK TIMSK0

#define OW_PIN PIND //1 Wire Pin as number
#define OW_PINN PORTD2
#define OW_DDR DDRD  //pin direction register

#define PORT_REG PORTC
#define PIN_REG PINC
#define PIN_DDR DDRC

#define PIN_PIO0 (1<<PINB0) /* BTNIN */
#define PIN_PIO1 (1<<PINB1) /* RINGIN */

#define PIN_PIO2 (1<<PINC1)
#define PIN_PIO3 (1<<PINC2)
#define PIN_PIO4 _BV(PC3)
#define PIN_PIO5 (1<<PINC4)
#define LED2 _BV(PC3)
#define LED _BV(PC1)

#define	LED_ON() do { DDRC |= LED;PORTC &= ~LED; }while(0)
#define	LED_OFF() do {DDRC &= ~LED;PORTC |= LED; }while(0)
#define	LED2_ON() do { DDRC |= LED2;PORTC &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRC &= ~LED2;PORTC |= LED2; led2=0; }while(0)

#define DOOR 1 /* PC1 */
#define RING 0 /* PC2 */
#define RINGIN 7 /* PB1 */
#define BTNIN 6 /* PB0 */

#define MAX_BTN 2
#endif

/* All tinies */
#if defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || \
	defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || \
	defined(__AVR_ATtiny84A__) || defined(__AVR_ATtiny85__)
#define PCINT_VECTOR PCINT0_vect

#define OW_PIN PINB //1 Wire Pin as number
#define OW_PINN PORTB2
#define OW_DDR DDRB  //pin direction register
#endif

#if defined(__AVR_ATtiny85__)
#define ACTIVE_LOW
#define PORT_REG PORTB
#define PIN_REG PINB
#define PIN_DDR DDRB

#define PIN_PIO0 _BV(PB4) /* light : ouput */
#define PIN_PIO1 _BV(PB3) /* LED */
#define PIN_PIO2 _BV(PB1)
#define PIN_PIO3 _BV(PB0) /* light switch: input */
#define LED _BV(PB3)
#define	LED_ON()  do { DDRB |= LED;  PORTB &= ~LED; } while(0)
#define	LED_OFF() do { DDRB &= ~LED; PORTB |= LED;  } while(0)
#define	LED2_ON() do {  }while(0)
#define	LED2_OFF() do { }while(0)

#define MAX_BTN 2
#endif

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define TIMSK TIMSK0
/* means: 0 is output low, 1 input high (seen from owfs) */
#define ACTIVE_LOW
#define PIN_REG PINA
#define PORT_REG PORTA
#define PIN_DDR DDRA

#define PIN_PIO0 (1<<PINA1)	// predefined output 1
#define PIN_PIO1 (1<<PINA0)	// predefined output 2
#define PIN_PIO2 (1<<PINA2)
#define PIN_PIO3 (1<<PINA3)
#define PIN_PIO4 (1<<PINA4)
#define PIN_PIO5 (1<<PINA5)
#define PIN_PIO6 (1<<PINA6)
#define LED _BV(PB1)
#define LED2 _BV(PB0)

#define PCMSK PCMSK0
#define	LED_ON() do { DDRB |= LED;PORTB &= ~LED; }while(0)
#define	LED_OFF() do {DDRB &= ~LED;PORTB |= LED; }while(0)
#define	LED2_ON() do { DDRB |= LED2;PORTB &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRB &= ~LED2;PORTB |= LED2; led2=0; }while(0)

#define MAX_BTN 7
#endif

#define CHAN_VALUES 16

#define SIG_NO 0
#define SIG_ACT 1
#define SIG_ARM 2

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);
extern uint8_t stat_to_sample;
static void owResetSignal(void);

OWST_EXTERN_VARS

OWST_WDT_ISR

typedef union {
	volatile uint8_t bytes[0x20];
	struct {
		/* Actual states of the IOs read via command F0 or F5 / reg adr 88 */
		uint8_t PIO_Logic_State;
		/* The data in this register represents the latest data
		 * written to the PIO through the Channel-access Write
		 * command 5A / reg adr 89
		 */
		uint8_t PIO_Output_Latch_State;
		/*
		 * The data in this register represents the current state of
		 * the PIO activity latches
		 */
		uint8_t PIO_Activity_Latch_State;
		uint8_t Conditional_Search_Channel_Selection_Mask;
		uint8_t Conditional_Search_Channel_Polarity_Selection;
		uint8_t Status; //008D
		uint8_t FF1; //008E
		uint8_t FF2; //008F

	};
} pack_t;
volatile pack_t pack;

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

void statusPrint();

/* last byte will be calculated, program like
C:\Users\rah\.platformio\packages\tool-avrdude
avrdude -C C:\Users\rah\.platformio\packages\tool-avrdude\avrdude.conf -c stk500v2 -P COM13 -p attiny85 -U eeprom:w:0x29,0x11,0x04,0x01,0x2,0x66,0x77:m
 */
uint8_t owid[8];

#define VERSION 1

#define CFG_BTN_ID 0
#define CFG_PIN_ID 1
#define CFG_POL_ID 2
#define CFG_SW_ID 3
#define CFG_VERS_ID 11

/*
 * config from EEPROM
 * | btn | pin | pol | sw0 |  sw1 |  sw2 | sw3 | sw4 | sw5 | sw6 | sw7 | vers |
 * btn: 0 means a push button (based on real port pin mask)
 *      1 represents a simple input with bouncing, no press button
 * pin: a 1 represents input (with change detection), 0: output
 *      (based on real port pin mask)
 * pol: a 1 represents normal polarity (set 0 = conducting, on)
 *      (based on real port pin mask)
 * sw*: auto switch the specified ouput (0xff no switching)
 *      * - pio map index
 *          entry pin number based on pio map
 *      No latch will be set on input, but only on output switch
 * vers: Version
 */
uint8_t config_info[26] = {0x0};
uint8_t signal_cfg;
unsigned long _ms;

struct pinState btn[MAX_BTN];
uint8_t values[CHAN_VALUES];
uint8_t ap = 0;
volatile uint8_t int_signal;
volatile uint8_t btn_active = 0;
volatile uint8_t pin_state;
static uint8_t led2 = 0;

#if 1 // defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) 
static void led_flash(void)
{
	int i, j;

	for (i = 4; i > 1; i--) {
		LED_ON();
		for (j = 0; j < i; j++)
			_delay_ms(25);
		LED_OFF();
		_delay_ms(50);
	}
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
		_delay_ms(1);
		_ms++;
		ms--;
	}
}

static inline void act_latch(uint8_t p, uint8_t mask)
{
#ifdef WDT_ENABLED
	wdt_reset();
#endif
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

static inline void pin_change(uint8_t pins, uint8_t p, uint8_t mask)
{
	if (pins & p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
	if ((mask & pack.Conditional_Search_Channel_Selection_Mask) == 0)
		return;
	pack.PIO_Activity_Latch_State |= mask;
}

static void sync_pins()
{
	uint8_t pins;
#if  defined(ATMEGA)
	pins = PINB;
#else
	pins = PIN_REG;
#endif
	pin_sync_ls(pins, PIN_PIO0, 0x1);
	pin_sync_ls(pins, PIN_PIO1, 0x2);
#if  defined(ATMEGA)
	pins = PIN_REG;
#endif
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
	pin_sync_ls(pins, PIN_PIO6, 0x40);
#endif
#if  defined(ATMEGA)
	pins = PINB;
#endif
	pin_state = pins & (~PIN_DDR);
}

ISR(PCINT0_vect) {
	uint8_t pins;
#if defined(ATMEGA)
	pins = PINB;
#else
	pins = PIN_REG;
	// remove LED and output
	pins &= (~PIN_DDR);
#endif

	if (pin_state != pins) {
#ifdef WDT_ENABLED
		wdt_enable(WDTO_8S);
#endif
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

void pin_set(uint8_t bb)
{
	uint8_t p = 0;

	/* 
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map) / sizeof(pio_map[0]); i++) {
		if (bb == mask) {
			p = pio_map[i];
			break;
		}
		mask = mask << 1;
	}
	*/
	switch (bb)
	{
		case 0x1:
			p = PIN_PIO0;
			break;
		case 0x2:
			p = PIN_PIO1;
			break;
		case 0x4:
			p = PIN_PIO2;
			break;
		case 0x8:
			p = PIN_PIO3;
			break;
#ifdef PIN_PIO4
		case 0x10:
			p = PIN_PIO4;
			break;
#endif
#ifdef PIN_PIO5
		case 0x20:
			p = PIN_PIO5;
			break;
#endif
#ifdef PIN_PIO6
		case 0x40:
			p = PIN_PIO6;
			break;
#endif
#ifdef PIN_PIO7
		case 0x80:
			p = PIN_PIO7;
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
		if (pack.PIO_Output_Latch_State & bb) {
			/* According to spec:
			 * set 1 / non-conducting (off)
			 * set to input
			 */
			PIN_DDR &= ~(p);
#ifdef ACTIVE_LOW
			/* enable pull up */
			PORT_REG |= (p);
#else
			/* no pull up */
			PORT_REG &= ~p;
#endif /* ACTIVE_LOW */
			PCMSK |= p;
			// clear interrupt flag?
			signal_cfg |= bb;
		} else {
			/* set 0 / 0 = conducting (on) */
			PCMSK &= ~(p);
			/* set output and low on 0 */
			PIN_DDR |= p;
			PORT_REG &= ~p;
			signal_cfg &= ~bb;
		}
		pin_change(PIN_REG, p, bb);
	}
#if defined(GIFR) && defined(PCIF0)
	GIFR = _BV(PCIF0);
#endif
#if defined(GIFR) && defined(PCIF)
	GIFR = _BV(PCIF);
#endif
#if  defined(ATMEGA)
	pin_state = PINB;
#else
	pin_state = PIN_REG & (~PIN_DDR);
#endif
	sei();
}

static void owResetSignal(void)
{
	int to = 10;
	
	while (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0)) {
		delay(1);
		if (to-- == 0)
			/* try again later */
			return;
	};
	cli();
	sbi (OW_DDR, OW_PINN);
	_delay_us (960);
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
#ifdef HAVE_UART
	serial_write('>');
#endif
}

void setup()
{
	int i;
	uint8_t id[9];
	int to = 100;

	/* all off */
	pack.PIO_Logic_State = 0xff;
	pack.PIO_Output_Latch_State = 0xff;
	pack.FF1 = 0xDE;
	pack.FF2 = 0xAD;
	pack.Status |= 0x80;
	OWST_INIT_ALL_OFF;
	OWST_EN_PULLUP
	LED_ON();
#ifdef HAVE_UART
	serial_init();
#endif
	while (!eeprom_is_ready() && to--)
		_delay_ms(1);
	eeprom_read_block((void*)&id, (const void*)0, 9);
	if (id[0] == 0x29 && id[1] != 0xFF) {
		for (i = 0; i < 7; i++)
			owid[i] = id[i];
	}
	owid[7] = crc(owid, 7);
	eeprom_read_block((void*)&config_info, (const void*)7, 24);
	config_info[CFG_VERS_ID] = VERSION;
	/* pull ups on all pins set by OWST_INIT_ALL_OFF */
	OWINIT();

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	/* ignoring changes on output */
	PCMSK0 = (0xFF - PIN_PIO0 - PIN_PIO1);
#endif
#if defined(__AVR_ATtiny85__)
	/* PIN_DDR |= PIN_PIO0 | PIN_PIO1; */
	PCMSK = (_BV(PCINT0) | _BV(PCINT1) /* | _BV(PCINT3) | _BV(PCINT4)*/);
#endif
#if defined(ATMEGA)
	/* output on relais pins */
	PIN_DDR |= _BV(PC0) | _BV(PC1);
	/* set inputs with pull up */
	PORTB = _BV(PB0) | _BV(PB1);
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1);
	PCICR = _BV(PCIE0); /* enable port B ints */
#endif
	sync_pins();
	for (i = 0;i < MAX_BTN;i++)
		initBtn(1, &btn[i]);
	pack.PIO_Activity_Latch_State = 0;
	/* enable alarm state reporting */
	pack.Conditional_Search_Channel_Selection_Mask = 0xFF;
	signal_cfg = 0xff;

	int_signal = SIG_ARM;
	sei();
	delay(200);
	LED_OFF();
	delay(200);
	led_flash();
	printf ("%02x...%02x %02x - %02X %02X | %02X %02X\n", owid[1], owid[2], owid[7],
		config_info[CFG_BTN_ID], config_info[CFG_PIN_ID], config_info[CFG_SW_ID],
		config_info[CFG_SW_ID + 1]);
	statusPrint();
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
	if (gcontrol & 0xF) {
		pack.Status &= ~0x80;
	}
	if (gcontrol & 1) {
		/* write, data in PIO_Output_Latch_State */
		uint8_t i;
		printf ("LogicState=%02X OutLatch=%02X\n", pack.PIO_Logic_State, pack.PIO_Output_Latch_State);
		for (i = 1; i < 0x80; i = i * 2) {
			pin_set(i);
		}
		// avoid signal generation
		int_signal = SIG_NO;
		gcontrol &= ~0x01;
	}
	if (gcontrol & 2) {
		/* OW_RESET_ACTIVITY */
		cli();
		pack.PIO_Activity_Latch_State = 0;
		gcontrol &=  ~0x02;
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
		cli();
		gcontrol &= ~0x10;
		printf ("%02x %02x %02x %02X %02X %02X\n", config_info[CFG_BTN_ID],
		config_info[CFG_PIN_ID], config_info[CFG_SW_ID], config_info[CFG_SW_ID + 1],
		config_info[CFG_SW_ID + 2], config_info[CFG_SW_ID + 3]);
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
#ifdef ATMEGA
		if (DDRB & pio_map[i]) {
#else			
		if (PIN_DDR & pio_map[i]) {
#endif		
			printf ("#%i out\n", i+1);
		} else 
		{
			printf ("#%i %d %d\n", i+1, p->state, p->press);
		}
	}
#endif
}

void btn_loop()
{
	int i;
	uint8_t pins, in, act_btns, mask = 1, out;
	static int stPrints = 50;

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	pins = PINB;
#else
	pins = PIN_REG;
#endif
	act_btns = 0;
	for (i = 0; i < MAX_BTN; i++) {
		int st;
		struct pinState *p;
#ifdef DEBUG
		int prev;
#endif
#ifdef ATMEGA
		if (DDRB & pio_map[i]) {
#else
		if (PIN_DDR & pio_map[i]) {
#endif
			mask = mask << 1;
			continue;
		}
		p = &btn[i];
		in = !!(pins & pio_map[i]);
#ifdef DEBUG
		prev = p->state;
#endif		
		st = checkBtn(in, p);
		switch (st) {
		case BTN_PRESSED_LONG:
			/* active low but for longer time */
			LED2_OFF();
			pack.Status |= mask;
			pack.FF1 = p->press / 8;
			// get the next edge
			act_btns |= mask;
			statusPrint();
			break;
		case BTN_PRESSED:
			/* short pressed and done 
			if (led2) {
				LED2_OFF();
			} else {
				LED2_ON();
			}*/
			pack.FF1 = p->press / 8;
			pack.Status &= ~mask;
			out = config_info[CFG_SW_ID + i];
			if (out != 0xff) {
				printf ("auto sw %d -> %d (%02X)\n", i, out, pack.PIO_Output_Latch_State);
				// toggle output
				if (pack.PIO_Output_Latch_State & out)
					pack.PIO_Output_Latch_State &= ~out;
				else
					pack.PIO_Output_Latch_State |= out;
				pin_set(out);
				// signal not the input, but only the out change
			} else
				act_latch(1, mask);
			printf ("#%i press\n", i);
			if (p->cnt > 10) {
				p->cnt = 0;
				statusPrint();
			}
			break;
		case BTN_RELEASED:
			/* long pressed done for push buttons or
			    toggled the switch */
			pack.Status |= mask;
			pack.FF1 = p->press / 8;
			act_latch(1, mask);
			printf ("#%i release\n", i);
			break;
		case BTN_PRESS_LOW:
			/* intermediate end state, wait for release
			   for push butotns */
			// for push button / switch should go to sleep
			act_btns |= mask;
			if (config_info[CFG_BTN_ID] & mask) {
				act_latch(0, mask);
			}
			pack.FF1 = 0xFF;
			//printf ("btn low\n");
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
		default:
			act_btns |= mask;
			break;
		}
#ifdef NO_DEBUG
		if (prev != st)
			printf("%i: %i > %i [%02X]\n", i, prev, st, act_btns);
#endif
		mask = mask << 1;
	}
	if (pack.PIO_Activity_Latch_State && !alarmflag) {
		alarmflag = 1;
		/* TODO: check whether to alarm or not */ 
		if ((pack.PIO_Activity_Latch_State & signal_cfg) && int_signal == SIG_ARM)
			int_signal = SIG_ACT;
	}
	pack.FF2 = act_btns;
	if (int_signal == SIG_ACT)
		pack.FF2 |= 0x80;
	values[5] = act_btns;
	if (act_btns == 0) {
		btn_active = 0;
		if (btn_active) {
			printf ("btn done\n");
			statusPrint();
		}
	} else
		btn_active = (act_btns << 1);
	if (stPrints-- == 0) {
		led_flash();
		statusPrint();
	}
}

void loop()
{
	ow_loop();

	/* ongoing OW access (in interrupt), serve data with high prio and skip others */
	if (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0))
		return;
	if (btn_active)
		btn_loop();
	if (int_signal == SIG_ACT)
		owResetSignal();
	sync_pins();
#if 0
	int i;
	values[6] = btn_active;
	values[7] = 0xAA;
	// D9 0 3 CA FE 7 E AA 0 0 1 0 1 55 0 CF 0 
	//                     #2   #3 
	for (i = 1; i < MAX_BTN; i++) {
		struct pinState *p = &btn[i];
		values[8+ (2*i)] = p->state;
		values[8+ (2*i) + 1] = p->press / 8;
		if ((8 + (2*i) + 1) >= CHAN_VALUES)
			break;
	}
	values[CHAN_VALUES-2] = 0x55;
	values[CHAN_VALUES-1] = crc(values, CHAN_VALUES);
#endif
}

#ifdef UNIT_TEST
int main_func(void)
#else
int main(void)
#endif
{
	setup();
	while (1) {
		loop();
		if (btn_active || int_signal == SIG_ACT)
			delay(1);
		else {
			LED_OFF();
#ifdef WDT_ENABLED
			wdt_disable();
#endif
			OWST_MAIN_END
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_enable();
			sleep_cpu();
			LED_ON();
		}
	}
}
