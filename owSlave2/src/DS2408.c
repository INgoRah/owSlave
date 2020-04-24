
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
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "owSlave_tools.h"
#include "Arduino.h"
#include "pins.h"

#if  defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) 
#define PCINT_VECTOR PCINT0_vect
#define PORT_REG PORTC
#define PIN_REG PINC
#define PIN_DDR DDRC

#define PIN_PIO0 (1<<PINB0) /* BTNIN */
#define PIN_PIO1 (1<<PINB1) /* RINGIN */

#define PIN_PIO2 (1<<PINC1)
#define PIN_PIO3 (1<<PINC2)
#define PIN_PIO4 _BV(PC3)
#define PIN_PIO5 (1<<PINC4)
#define LED _BV(PC3)

#define DOOR 1 /* PC1 */
#define RING 0 /* PC2 */
#define RINGIN 7 /* PB1 */
#define BTNIN 6 /* PB0 */
#endif

/* All tinies */
#if defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || \
	defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || \
	defined(__AVR_ATtiny84A__) || defined(__AVR_ATtiny85__)
#define OW_PORT PORTB //1 Wire Port
#define OW_PIN PINB //1 Wire Pin as number
#define OW_PINN PORTB2
#define OW_DDR DDRB  //pin direction register
#endif

#if defined(__AVR_ATtiny85__)
#define ACTIVE_LOW
#define PCINT_VECTOR PCINT0_vect
#define PORT_REG PORTB
#define PIN_REG PINB
#define PIN_DDR DDRB

#define PIN_PIO0 _BV(PB4) /* light : ouput */
#define PIN_PIO1 _BV(PB3)
#define PIN_PIO2 _BV(PB1)
#define PIN_PIO3 _BV(PB0) /* light switch: input */
#define LED _BV(PB3)
#define	LED_ON()  do { DDRB |= LED;  PORTB &= ~LED; } while(0)
#define	LED_OFF() do { DDRB &= ~LED; PORTB |= LED;  } while(0)
#define	LED2_ON() do {  }while(0)
#define	LED2_OFF() do { }while(0)
#endif

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define TIMSK TIMSK0
/* means: 0 is output low, 1 input high (seen from owfs) */
#define ACTIVE_LOW
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PORT_REG PORTA
#define PIN_DDR DDRA

#define PIN_PIO0 (1<<PINA1)
#define PIN_PIO1 (1<<PINA2)
#define PIN_PIO2 (1<<PINA3)
#define PIN_PIO3 (1<<PINA4)
#define PIN_PIO4 (1<<PINA5)
#define PIN_PIO5 (1<<PINA6)
#define LED _BV(PB1)
#define LED2 _BV(PB0)

#define PCMSK PCMSK0
#define	LED_ON() do { DDRB |= LED;PORTB &= ~LED; }while(0)
#define	LED_OFF() do {DDRB &= ~LED;PORTB |= LED; }while(0)
#define	LED2_ON() do { DDRB |= LED2;PORTB &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRB &= ~LED2;PORTB |= LED2; led2=0; }while(0)
#endif

#define CHAN_VALUES 6
#define MAX_BTN 4

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);
extern uint8_t stat_to_sample;
static void owResetSignal(void);

/* last byte will be calculated, program like
C:\Users\rah\.platformio\packages\tool-avrdude
avrdude -C C:\Users\rah\.platformio\packages\tool-avrdude\avrdude.conf -c stk500v2 -P COM13 -p attiny85 -U eeprom:w:0x29,0x11,0x04,0x01,0x2,0x66,0x77:m
 */
uint8_t owid[8];/* = {0x29, 0x1, 0x1, 0x4, 0x00, 0x16, 0x11, 0};*/
uint8_t config_info[26] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
		uint8_t FF1;
		uint8_t FF2;

	};
} pack_t;
volatile pack_t pack;

struct pinState btn[MAX_BTN];
uint8_t values[CHAN_VALUES];
uint8_t ap = 1;
volatile uint8_t int_signal = 0;
volatile uint8_t btn_active = 0;
static uint8_t pin_state = 0xFF;
unsigned long _ms;
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

uint8_t crc8(void)
{
	uint8_t lscrc = 0x0;

	for (uint8_t i = 0; i < CHAN_VALUES; i++) {
		uint8_t v = values[i];
		//if (v==0) v=0xFF;
		uint8_t bit = 1;
		uint8_t lb;

		for (uint8_t j = 0; j < 8; j++) {
			if ((v&bit) == bit) lb = 1; else lb = 0;
			if ((lscrc&1) != lb)	lscrc = (lscrc>>1)^0x8c; else	lscrc = (lscrc>>1);
			bit = bit*2;
		}
	}
	return lscrc;
}

static uint8_t crc() {
	unsigned char inbyte, crc = 0, len = 7;
	unsigned char i, mix;
	unsigned char* addr = owid;

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
	if ((mask & pack.Conditional_Search_Channel_Selection_Mask) != 0)
		pack.PIO_Activity_Latch_State |= mask;
}

static inline void pin_change(uint8_t pins, uint8_t p, uint8_t mask)
{
	if (pins & p)
		pack.PIO_Logic_State |= mask;
	else
		pack.PIO_Logic_State &= ~mask;
	if ((mask & pack.Conditional_Search_Channel_Selection_Mask) == 0)
		return;
	if ((pins & p) != (pin_state  & p))
		pack.PIO_Activity_Latch_State |= mask;
}

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

static void sync_pins()
{
	uint8_t pins;
#if  defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	pins = PINB;
#else
	pins = PIN_REG;
#endif
	pin_change(pins, PIN_PIO0, 0x1);
	pin_change(pins, PIN_PIO1, 0x2);
	pin_change(pins, PIN_PIO2, 0x4);
	pin_change(pins, PIN_PIO3, 0x8);
#ifdef PIN_PIO4
	pin_change(pins, PIN_PIO4, 0x10);
#endif			
#ifdef PIN_PIO5
	pin_change(pins, PIN_PIO5, 0x20);
#endif			
#ifdef PIN_PIO6
	pin_change(pins, PIN_PIO6, 0x40);
#endif
#ifdef PIN_PIO7
	pin_change(pins, PIN_PIO6, 0x40);
#endif
	pin_state = pins;
}

ISR(PCINT0_vect) {
	uint8_t pins;
#if  defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	pins = PINB;
#else
	pins = PIN_REG;
#endif
	if (pin_state != pins) {
		btn_active = 1;
		pin_state = pins;
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
			break;
	}
	cli();
	if (p != 0 && (pack.PIO_Logic_State & bb) !=
		(pack.PIO_Output_Latch_State & bb)) {
		/* pin change by write */
		pack.PIO_Activity_Latch_State |= bb;
		/*  set alarmflag ? */
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
		} else {
			/* set 0 / 0 = conducting (on) */
			PCMSK &= ~(p);
			/* set output and low on 0 */
			PIN_DDR |= p;
			PORT_REG &= ~p;
		}
#if  defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
		pin_change(PINB, p, bb);
#else
		pin_change(PIN_REG, p, bb);
#endif
	}
	sei();
}

static void owResetSignal(void)
{
	int to = 10;
	uint8_t oldSREG;
	
	LED_ON();
	while (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0)) {
		delay(1);
		if (to-- == 0)
			/* try again later */
			return;
	};
#if 0
	/* not needed, because no timer and mode == 0 indicates high  */
	do {
		to = 20;
		// check line busy
		while ((OW_PIN & OW_PINN) == 0 && to > 0) {
			delay(1);
			to--;
		};
	} while ((OW_PIN & OW_PINN) == 0 || mode != 0);
#endif
	oldSREG = SREG;
	cli();
	sbi (OW_DDR, OW_PINN);
	_delay_us (960);
	cbi (OW_DDR, OW_PINN);
	int_signal = 0;
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
	SREG = oldSREG;
	LED_OFF();
}

void setup()
{
	int i;
	uint8_t id[8];

	pack.FF1 = 0xDE;
	pack.FF2 = 0xAD;
	pack.Status |= 0x80;
	 //0x0E 0x19 0x48 0x00
	values[0] = 0x00;
	values[1] = 8;
	values[2] = 26;
	values[3] = 0;
	values[4] = 5;
	values[6] = 0x00;
	values[7] = 0x00;
	values[CHAN_VALUES-1] = crc8();

	eeprom_read_block((void*)&id, (const void*)0, 8);
	if (id[0] != 0xFF && id[1] != 0xFF) {
		for (i = 0; i < 8; i++)
			owid[i] = id[i];
		return;
	}
	owid[7] = crc();
	for (i = 0;i < MAX_BTN;i++)
		initBtn(1, &btn[i]);
	/* pull ups on all pins set by OWST_INIT_ALL_OFF */
	OWST_INIT_ALL_OFF;

	OWST_EN_PULLUP
	OWINIT();
	
#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	PCMSK0 = (0xFF - PIN_PIO0);
	// PCMSK1 for LEDS not used
	GIMSK |= _BV(PCIE0);
#endif
#if defined(__AVR_ATtiny85__)
	/* PIN_DDR |= PIN_PIO0 | PIN_PIO1; */
	PCMSK = (_BV(PCINT0) | _BV(PCINT1) /*| _BV(PCINT3) | _BV(PCINT4)*/);
	GIMSK |= (1 << PCIE);
#endif
#if  defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) 
	/* output on relais pins */
	PIN_DDR |= _BV(PC0) | _BV(PC1);
	/* set inputs */
	PORTB = _BV(PB0) | _BV(PB1);
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1);
	PCIFR = _BV(PCIF0);
	PCICR = _BV(PCIE0);
#endif
	LED_ON();
	pack.Conditional_Search_Channel_Selection_Mask = 0;
	sync_pins();
	/* enable alarm state reporting */
	pack.Conditional_Search_Channel_Selection_Mask = 0xFF;
	sei();
#if 1
	_delay_ms(300);
	LED_OFF();
#else
	_delay_ms(10);
#endif
	//led_flash();
}

void ow_loop()
{
	if (reset_indicator) {
		ap = 0;
		// stat_to_sample=0;
		reset_indicator=0;
	}
	if (gcontrol & 1) {
		/* write, data in PIO_Output_Latch_State */
		uint8_t i;
		for (i = 1; i < 0x80; i = i * 2) {
			pin_set(i);
		}
		gcontrol &= ~0x01;
	}
	if (gcontrol & 2) {
		/* OW_RESET_ACTIVITY */
		pack.PIO_Activity_Latch_State = 0;
		gcontrol &=  ~0x02;
		alarmflag = 0;
	}
	if (gcontrol & 0x4) {
		stat_to_sample=values[ap];
		if (ap++ > CHAN_VALUES-1)
			ap = 0;
		gcontrol &= ~0x04;
	}
	if (gcontrol & 0x8) {
		/* read channel data */
		stat_to_sample=values[1];
		ap = 2;
		gcontrol &= ~0x08;
		alarmflag = 0;
	}
}

void btn_loop()
{
	int i;
	uint8_t pins, in, act_btns, mask = 1;

#if defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
	pins = PINB;
#else
	pins = PIN_REG;
#endif
	act_btns = 0;
	for (i = 0; i < MAX_BTN; i++) {
		int st;
		struct pinState *p = &btn[i];

		in = !!(pins & pio_map[i]);
		st = checkBtn(in, p);
		//values[i] = st;
		switch (st) {
		case BTN_PRESSED_LONG:
			/* active low but for longer time */
			LED2_OFF();
			pack.FF1 |= mask;
			act_btns++;
			break;
		case BTN_PRESSED:
			/* pressed and done */
			if (led2) {
				LED2_OFF();
			} else {
				LED2_ON();
			}
			pack.FF1 &= ~mask;
			pack.FF2 = p->press;
			pack.PIO_Logic_State |= mask;
			//values[i+8] = p->press;
			act_latch(1, mask);
			act_btns++;
			break;
		case BTN_RELEASED:
			/* long pressed done */
			pack.PIO_Logic_State |= mask;
			//values[i+8] = p->press;
			act_latch(1, mask);
			act_btns++;
			break;
		case BTN_PRESS_LOW:
			/* intermediate end state, wait for release */
			pack.FF1 &= ~mask;
			pack.PIO_Logic_State &= ~mask;
			// for push button / switch should go to sleep
			act_btns++;
			break;
		case BTN_HIGH:
			/* means no change */
			break;
		case BTN_LOW:
			act_btns++;
			break;
		/* Internal and intermediate state are ongoing evaluations
		 * BTN_PRESS_LOW BTN_INVALID BTN_UNSTABLE BTN_TIMER_HIGH BTN_TIMER_LOW
		 */
		default:
			//if (p->press < 2000)
			act_btns++;
			break;
		}
		mask = mask << 1;
	}
	if (pack.PIO_Activity_Latch_State && !alarmflag) {
		alarmflag = 1;
		int_signal = 1;
	}
	if (act_btns == 0)
		btn_active = 0;
}

void loop()
{
#if defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) 
	if ((PINB & PIN_PIO0) == 0) {
		/* door */
		PORT_REG |= _BV(PC1);
		PORT_REG &= ~(LED);
	}
	else {
		/* door */
		PORT_REG &= ~_BV(PC1);
		PORT_REG |= (LED);
	}
	if ((PINB & PIN_PIO1) == 0)
		led_flash();
#endif
	ow_loop();

	/* ongoing OW access, serve data with high prio and skip others */
	if (((TIMSK & (1<<TOIE0)) != 0) || (mode !=0))
		return;
	if (btn_active)
		btn_loop();
	if (int_signal)
		owResetSignal();
}

int main(void)
{
	setup();
	while (1) {
		//LED_ON();
		loop();
#if 0
		delay(1);
#else
		if (btn_active || int_signal)
			delay(1);
		else {
			LED_OFF();
			/* only sleepp if no action pending */
			OWST_MAIN_END
		}
#endif
	}
}
