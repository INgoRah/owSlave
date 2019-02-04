//owdevice - A small 1-Wire emulator for AVR Microcontroller
//
//Copyright (C) 2012  Tobias Mueller mail (at) tobynet.de
//Copyright (C) 2018  INgo.Rah@gmx.net
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
// any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//http://www.tm3d.de/index.php/1-wire-device-mit-avr
// http://www.mikrocontroller.net/topic/44100#new
// http://www.tm3d.de/index.php/
#include "Arduino.h"
#include "owflexdev.h"
#ifdef HAVE_UART
#include "uart.h"
#include "printf.h"
#else
#define printf(...)
#endif
#include "pins.h"
#include <util/delay.h>

#define DEBUG
#define ENABLE_SLEEP

#define OW_WRITE_SCRATCHPAD 0x4E
#define OW_READ_SCRATCHPAD 0xBE

#if (OW_FAMILY == 0xA3)
extern void ir_sendNEC (unsigned long data, int nbits);
#endif
	
/* set 1-Wire line to low */
#define SET_LOW OW_DDR |= OW_PINN; OW_PORT &= ~OW_PORTN
/* set 1-Wire pin as input */
#define RESET_LOW OW_DDR &= ~OW_PINN

//States / Modes
#define OWM_SLEEP 0		//Waiting for next reset pulse
#define OWM_RESET 1		//Reset pulse received
#define OWM_PRESENCE 2		//sending presence pulse
#define OWM_CHK_RESET 3		//waiting of rising edge from reset pulse
#define OWM_READ_COMMAND 8	//read 8 bit of command
#define OWM_SEARCH_ROM 4	//SEARCH_ROM algorithms
#define OWM_MATCH_ROM 5		//test number
#define OWM_READ_SCRATCHPAD 6
#define OWM_WRITE_SCRATCHPAD 7
#define OWM_TEST 8

//Write a bit after next falling edge from master
//its for sending a zero as soon as possible 
#define OWW_NO_WRITE 2
#define OWW_WRITE_1 1
#define OWW_WRITE_0 0

/*
 * Timings
 * There are still master adaptations needed at least for searching
 */
#define OWT_READLINE 3 // 3 for fast master, 4 for slow master and long lines
#define OWT_MIN_RESET 40
#define OWT_LOWTIME 3 // 3 for fast master, 4 for slow master and long lines 
#define OWT_PRESENCE 20
#define OWT_RESET_PRESENCE 4

#define MAX_DATA 8
#ifndef OW_SERIAL
#define OW_SERIAL 1
#endif	

volatile uint8_t rdata[MAX_DATA+1];
volatile uint8_t wdata[MAX_DATA+1];

uint8_t owCommand(uint8_t cmd);
void prepareScratch();
static uint8_t crc();

volatile uint8_t scrc;		//CRC calculation
/* Input buffer for a command */
volatile uint8_t cbuf;
/* Store interrupt source for read out via scratchpad */
volatile uint8_t IntSrc;

/* defaults:
 * - 2 input and output
 * - default pin #0: lamp, #1 LED
 */
uint8_t owid[8] ={ OW_FAMILY, FD_SERIAL, FD_VERSION, 0x22,
	FD_PIN_LAMP | (FD_PIN_LEDST << 2), 0x00, 0x01, 0 };

uint8_t func;		// function currently selected
volatile uint8_t bitp;		//pointer to current bit
volatile uint8_t bytep;		//pointer to current byte

volatile uint8_t mode;		//state
volatile uint8_t wmode;		//if 0 next bit that send the device is  0
volatile uint8_t actbit;	//current
volatile uint8_t srcount;	//counter for search rom
uint8_t pinMsk;

ISR(INT0_vect) {
	sleep_disable();
	if (wmode == OWW_WRITE_0) {
		SET_LOW;
		wmode = OWW_NO_WRITE;
	}		//if necessary set 0-Bit 
	/* disable interrupt, only in OWM_SLEEP mode it is active */
	DIS_OWINT;

	switch (mode) {
	case OWM_SLEEP:
		TCNT_REG = ~(OWT_MIN_RESET);
		/* todo: introduce a new state with interrupts disabled? */
		SET_FALLING;
		EN_OWINT;	//other edges ?
		break;
		//start of reading with falling edge from master, reading closed in timer isr
	case OWM_MATCH_ROM: //falling edge wait for receive 
	case OWM_WRITE_SCRATCHPAD:
	case OWM_READ_COMMAND:
		TCNT_REG = ~(OWT_READLINE);	//wait a time for reading
		break;
	case OWM_SEARCH_ROM:	//Search algorithm waiting for receive or send
		if (srcount < 2) {
			//this means bit or complement is writing, 
			TCNT_REG = ~(OWT_LOWTIME);
		} else
			//init for read answer of master 
			TCNT_REG = ~(OWT_READLINE);
		break;
	case OWM_READ_SCRATCHPAD:	//a bit is sending 
		TCNT_REG = ~(OWT_LOWTIME);
		break;
	case OWM_CHK_RESET:	//rising edge of reset pulse
		SET_FALLING;
		//waiting for sending presence pulse
		TCNT_REG = ~(OWT_RESET_PRESENCE);
		mode = OWM_RESET;
		break;
	}
	EN_TIMER;
}

ISR(TIMER0_OVF_vect)
{
	uint8_t lwmode = wmode;	//let this variables in registers
	uint8_t lmode = mode;
	uint8_t lbytep = bytep;
	uint8_t lbitp = bitp;
	uint8_t lsrcount = srcount;
	uint8_t lactbit = actbit;

	//Ask input line sate 
	uint8_t p = ((OW_PIN & OW_PINN) == OW_PINN);
	//Interrupt still active ?
	if (CHK_INT_EN) {
		// reset pulse
		if (p == 0) {
			lmode = OWM_CHK_RESET;	//wait for rising edge
			SET_RISING;
		}
		DIS_TIMER;
	} else
		switch (lmode) {
		case OWM_RESET:	
			/* Minimum reset pulse time elapsed 
			 * now go in presence state, signal it
			 */
			lmode = OWM_PRESENCE;
			SET_LOW;
			TCNT_REG = (uint8_t)~(OWT_PRESENCE);
			DIS_OWINT;	//No Pin interrupt necessary only wait for presence is done
			break;
		case OWM_PRESENCE:
			RESET_LOW;	//Presence is done now wait for a command
			lmode = OWM_READ_COMMAND;
			cbuf = 0;
			lbitp = 1;	//Command buffer have to set zero, only set bits will write in
			break;
		case OWM_READ_COMMAND:
			/* Set bit if line high  */
			if (p)
				cbuf |= lbitp;
			lbitp = (lbitp << 1);
			if (!lbitp) {	//8-Bits read
				lbitp = 1;
				/* execute the command now */
				switch (cbuf) {
				case 0x55:
					/* reserved */
					lmode = OWM_MATCH_ROM;
					lbytep = 0;
					break;
				case 0xF0:
					/* reserved, initialize search rom */
					lmode = OWM_SEARCH_ROM;
					lsrcount = 0;
					lbytep = 0;
					//set initial bit
					lactbit = (owid[0] & 1) == 1;
					//prepare for writing when next falling edge
					lwmode = lactbit;
					break;
				case OW_WRITE_SCRATCHPAD:
					lmode = OWM_WRITE_SCRATCHPAD;
					lbytep = 0;
					//initialize writing position in scratch pad
					wdata[0] = 0; 
					break;
				case OW_READ_SCRATCHPAD:
					lmode = OWM_READ_SCRATCHPAD;
					lbytep = 0;
					scrc = 0;	//from first position
					rdata[0] = IntSrc;
					rdata[7] = func;
					/* in case of ADC reset activity bit */
					func &= ~4;
					lactbit = (lbitp & rdata[0]) == lbitp;
					/* prepare for sending first bit */
					lwmode = lactbit;
					break;
				default:
					lmode = owCommand(cbuf);
				}
			}
			break;
		case OWM_SEARCH_ROM:
			/*
			 * Set low also if nothing send 
			 * (branch takes time and memory)
			 */
			RESET_LOW;
			//next search rom mode
			lsrcount++;
			switch (lsrcount) {
			case 1:
				//preparation sending complement
				lwmode = !lactbit;
				break;
			case 3:
				/* check master bit */
				if (p != (lactbit == 1)) {
					// not the same go sleep
					lmode = OWM_SLEEP;
				} else {
					lbitp = (lbitp << 1);
					if (lbitp == 0) {
						//prepare next bit
						lbitp = 1;
						lbytep++;
						if (lbytep >= 8) {
							lmode = OWM_SLEEP;	
							//all bits processed 
							break;
						}
					}
					lsrcount = 0;
					lactbit =
					    (owid[lbytep] & lbitp) == lbitp;
					lwmode = lactbit;
				}
				break;
			}
			break;
		case OWM_MATCH_ROM:
			// Compare with ID Buffer
			if (p == ((owid[lbytep] & lbitp) == lbitp)) {
				lbitp = (lbitp << 1);
				if (!lbitp) {
					lbytep++;
					lbitp = 1;
					if (lbytep >= 8) {
						lmode = OWM_READ_COMMAND;
						//same? get next command
						cbuf = 0;
						break;
					}
				}
			} else
				lmode = OWM_SLEEP;
			break;
		case OWM_WRITE_SCRATCHPAD:
			if (p)
				wdata[lbytep] |= lbitp;
			lbitp = (lbitp << 1);
			if (!lbitp) {
				/* prepare next bit */
				lbitp = 1;
				if (lbytep++ == MAX_DATA) {
					lmode = OWM_SLEEP;
					break;
				}
				wdata[lbytep] = 0;
			}
			break;
		case OWM_READ_SCRATCHPAD:
			{
				uint8_t lscrc = scrc;

				RESET_LOW;
				if ((lscrc & 1) != lactbit)
					lscrc = (lscrc >> 1) ^ 0x8c;
				else
					lscrc >>= 1;
				lbitp = (lbitp << 1);
				if (!lbitp) {
					/* clear interrupt status */
					IntSrc = 0;
					lbitp = 1;
					if (lbytep++ >= MAX_DATA) {
						lmode = OWM_SLEEP;
						rdata[3] = 0;
						rdata[4] = 0;
						break;
					} else if (lbytep == MAX_DATA - 1)
						rdata[MAX_DATA] = lscrc;
				}
				lactbit = (lbitp & rdata[lbytep]) == lbitp;
				lwmode = lactbit;
				scrc = lscrc;
				break;
			}
		}
	if (lmode == OWM_SLEEP) {
		DIS_TIMER;
	}
	if (lmode != OWM_PRESENCE) {
		TCNT_REG = ~(OWT_MIN_RESET - OWT_READLINE);	//OWT_READLINE around OWT_LOWTIME
		EN_OWINT;
	}

	mode = lmode;
	wmode = lwmode;
	bytep = lbytep;
	bitp = lbitp;
	srcount = lsrcount;
	actbit = lactbit;
}

/* finished receiving a command. This function is called from timer context.
 * Depending on the return value to upper state machine handles further data
 * returns new one wire mode
 */
uint8_t owCommand(uint8_t cmd)
{
	uint8_t lmode = OWM_SLEEP;
	int i;
#if (OW_FAMILY == 0xA3)
	union {
		unsigned long l;
		unsigned char b[4];
	} d;
#endif

	switch (cmd) {
	case 0x20:
	case 0x30:
	case 0x21:
	case 0x31:
#if (OW_FAMILY == 0xA8)
		if ((cmd & 0x30) == 0x30) {
			/* on */
			digitalWrite (cmd & 0xf, HIGH);
		}
		if ((cmd & 0x30) == 0x20) {
			/* off */
			digitalWrite (cmd & 0xf, LOW);
		}
		break;
#endif	
	case 0x22:
	case 0x32:
	case 0x23:
	case 0x33:
		/* 
		 * 2xh : switch output of pin 1-4 to low
		 * 3xh : switch output of pin 1-4 to high
		 */
		if ((cmd & 0x30) == 0x30) {
			/* on */
			//pinMode(cmd & 0xf, OUTPUT);
			digitalWrite (cmd & 0xf, LOW);
		}
		if ((cmd & 0x30) == 0x20) {
			/* off */
			//pinMode(cmd & 0xf, INPUT);
			digitalWrite (cmd & 0xf, HIGH);
		}
		break;
	case 0x40:
		func = 0;
		printf("test\n\t");
		break;
#if (OW_FAMILY == 0xA8)
	case 0x41:
		func &= ~0x10;
		break;
	case 0x51:
		func |= 0x10;
		break;
#endif	
	case 0x42:
		func &= ~2;
		break;
	case 0x52:
		func |= 2;
		break;
#ifdef BTN_SUPPORT
	case 0x48:
		func &= ~0x20;
		break;
	case 0x58:
		func |= 0x20;
		break;
#endif
	case 0x44:	//Start Convert
		//func |= 0x4;
		prepareScratch();
		break;
#if (OW_FAMILY == 0xA3)
	case 0x70:
		// send IR
		for (i = 0; i < 4; ++i)
			d.b[i] = wdata[i];
		/*
		 * Tuner:5EA16897  = 1587636375
		 * CD:5EA1A857 = 1587652695
		 * On/off 5EA1F807 = 1587673095
		 * Vol up 5EA158A7 = 1587632295
		 * Vol down 5EA1D827 = 1587664935
		 */
		ir_sendNEC (d.l, 32);
		break;
#endif
	case 0x80:
		for (i = 0; i < 7; ++i) {
			owid[i] = wdata[i];
		}
		owid[7] = crc();
		eeprom_write_block ((const void*)&owid, (void*)0, 8);
		break;
	default:
		//all other commands do nothing
		break;
	}
	
	return lmode;
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

#if defined(PCINT0_vect)
/*
 * PIN B interrupts
 */
ISR(PCINT0_vect)
{
	/* check on falling edge on pin B0 .. B1 */
	if (!(PINB & _BV(PINB0))) {
#if defined(PCMSK0)
		cbi (PCMSK0, PCINT0);
#endif
#if defined(PCMSK)
		cbi (PCMSK, PCINT0);
#endif
		IntSrc |= 0x10;
	}
	if (!(PINB & _BV(PINB1))) {
#if defined(PCMSK0)
		cbi (PCMSK0, PCINT1);
#endif
#if defined(PCMSK)
		cbi (PCMSK, PCINT1);
#endif
		IntSrc |= 0x20;
	}
}
#endif

/* static setup at compile time */
void owidSetup()
{
	uint8_t i, id[8];
	/*
	uint8_t osc;
	osc = eeprom_read_byte((const void*)8);
	if (osc == 0xFF)
		eeprom_write_byte((uint8_t *)8, OSCCAL);
	*/
	eeprom_read_block((void*)&id, (const void*)0, 8);
	if (id[0] != 0xFF && id[1] != 0xFF) {
		for (i = 0; i < 8; i++)
			owid[i] = id[i];
		return;
	}
	/* 4th: max output/input pins | [7..4] output| [3..0] input | */
#if (OW_FAMILY == 0xA8)
	owid[3] = 0x44;
#endif	
#if (OW_FAMILY == 0xA3)
	owid[4] |= (FD_PIN_IRTX << 4);
	owid[5] = (FD_PIN_BTN << 4);
#endif
#if (OW_FAMILY == 0xA8)
	/*
	 * Pin 0-3 setup
	 * #4 PD3 input with external interrupt
	 */
	owid[4] = (FD_PIN_OUT) | (FD_PIN_OUT << 2) | (FD_PIN_UNUSED << 4) | (FD_PIN_OUT << 6);
	/*
	 * Pin 4-7 setup
	 */
	owid[5] = FD_PIN_UNUSED | (FD_PIN_UNUSED << 2) | (FD_PIN_BTN << 4) | (FD_PIN_INT << 6);
#endif
	/* group id */
	owid[7] = crc();
	eeprom_write_block ((const void*)&owid, (void*)0, 8);
}

void pinSetup()
{
	// setup PIN #0, Lamp
	switch (owid[4] & 0x3) {
		case FD_PIN_LAMP:
		case FD_PIN_OUT:
			pinMode(0, OUTPUT);
			break;
	}
	// setup PIN #1, LED
	switch ((owid[4] & 0xc) >> 2) {
		case FD_PIN_LEDST:
		case FD_PIN_OUT:
			pinMode(1, OUTPUT);
			break;
		case FD_PIN_ADC:
			break;
	}
	// setup PIN #2
	switch ((owid[4] & 0x30) >> 4) {
		case FD_PIN_IRTX:
		case FD_PIN_OUT:
			pinMode(2, OUTPUT);
			break;
	}
#if (OW_FAMILY == 0xA8)
	// setup PIN #3, LED
	switch ((owid[4] & 0xc0) >> 6) {
		case FD_PIN_OUT:
			pinMode(3, OUTPUT);
			break;
	}
#endif
	pinMsk = 0; 
#if (OW_FAMILY == 0xA8)
#if 0	
	// setup PIN #4
	switch (owid[5] & 0x03) {
		case FD_PIN_INT:
			pinMsk |= _BV(PCINT12);
	}
	// setup PIN #5
	switch ((owid[5] & 0x0c) >> 2)  {
		case FD_PIN_OUT:
			pinMode (5, OUTPUT);
			break;
		case FD_PIN_INT:
			pinMsk |= _BV(PCINT13);
		case FD_PIN_IN:
			pinMode (5, INPUT);
			break;
	}
#endif
	// setup PIN #6
	switch ((owid[5] & 0x30) >> 4)  {
		case FD_PIN_OUT:
			pinMode (6, OUTPUT);
			break;
		case FD_PIN_BTN:
			pinMsk |= _BV(PCINT0);
			/* fall-through */
		case FD_PIN_IN:
			pinMode (6, INPUT_PULLUP);
			break;
	}
	// setup PIN #7
	switch ((owid[5] & 0xc0) >> 6)  {
		case FD_PIN_OUT:
			pinMode (7, OUTPUT);
			break;
		case FD_PIN_INT:
			pinMsk |= _BV(PCINT1);
			/* fall-through */
		case FD_PIN_IN:
			pinMode (7, INPUT_PULLUP);
			break;
	}
#endif
#if defined PCICR
	// atmega 88/168/...
	sbi (PCICR, PCIE0);
#elif defined(GIMSK) 
	// attiny85....
	sbi (GIMSK, PCIE);
#else
	#error missing PC interrupt mask
#endif
}

static inline void enablePinInts()
{
	uint8_t oldSREG = SREG;
	cli();
#if defined(PCMSK0)
	PCMSK0 = pinMsk;
	PCIFR = _BV(PCIF0);
#elif defined(PCMSK) 
	// attiny85....
	PCMSK = pinMsk;
	sbi(GIFR, PCIF);
#endif
	SREG = oldSREG;
}

void setup() {
#ifdef DEBUG
	int i;
#endif
#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
#ifdef HAVE_UART
	serial_init();
#else
	DDRD |= _BV (PD0) | _BV (PD1);
#endif /* HAVE_UART */
#else
	DDRB |= _BV(PB3) | _BV(PB4);
#endif /* __AVR_ATmegax8__ */
	wmode = OWW_NO_WRITE;
	mode = OWM_SLEEP;
	RESET_LOW;
	SET_FALLING;
	INIT_AVR();
	DIS_TIMER;
	init();
	cli();
	owidSetup();
	pinSetup();
	IntSrc = 0;
	func = 2;
	enablePinInts();
	sei();

#ifdef DEBUG
	printf("\n\rSetup..DDRC=%X PORTC=%X PINC=%X\n\r", DDRC & 0xf, PORTC & 0xf, PINC & 0xf);
	for (i = 5; i > 1; i--) {
		digitalWrite(LED, HIGH);
		delay(25 * i);
		digitalWrite(LED, LOW);
		delay(50);
	}
#endif	
}

static inline void owResetSignal()
{
#if 0
	int to = 20;
	uint8_t oldSREG;
	
	// check line busy
	while ((OW_PIN & OW_PINN) == 0 && to > 0) {
		delay(1);
		to--;
	}
	if (to == 0)
		return;
	oldSREG = SREG;
	cli();
	SET_LOW;
	_delay_us (60);
	//delayMicroseconds(30);
	RESET_LOW;
	while ((OW_PIN & OW_PINN) == 0 && to > 0) {
		delay(1);
		to--;
	}
	SREG = oldSREG;
	printf ("signal!\r\n");
#endif
}

void prepareScratch()
{
	int i;
	/*
	 * setup data to be read
	 */
	rdata[1] = 0;
	rdata[2] = 0;
	for (i = 0; i < (owid[3] & 0xF); i++)
		rdata[1] |= digitalRead(i) << i;
	for (i = 0; i < ((owid[3] & 0xF0) >> 4); i++)
		rdata[2] |= digitalRead(i + 6) << i;
#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
#else	
	rdata[3] = 0;
	rdata[4] = DDRB;
#endif	
	rdata[5] = PORTB;
	rdata[6] = PINB;
}		

void loop() {
	uint8_t oldSREG;
#ifdef HAVE_UART
	static long stTime;
#endif	
#ifdef DEBUG
	int i;	
	static long ledTime;
	static char led;
#endif	
#if (OW_FAMILY == 0xA8)
	static char ring = 0;
	static char bell = 0;
	static unsigned long ringTime;
#endif

#if (OW_FAMILY == 0xA8)
	if (IntSrc & 0x10) {
		IntSrc &= ~0x10;
		/* check button till released */
		func |= 8;
	}
	if (IntSrc & 0x20) {
		IntSrc &= ~0x20;
		func |= 0x10;
	}
	if (func & 0x10) {
		digitalWrite(LED, digitalRead(RINGIN));
		switch (checkBtn(RINGIN, &btn[1])) {
		case BTN_LOW:
			if (ring == 0) {
				ringTime = millis();
				ring = 1;
			}
			break;
		case BTN_PRESSED:
		case BTN_RELEASED:
		case BTN_HIGH:
			/* filter out spike */
			if (ring == 1) {
				printf("Ring! func=%X\t INT=%X Time=%d\n\r", func, IntSrc, ringTime);
				ringTime = millis() - ringTime;
				printf("Ring End! func=%X\t INT=%X dTime=%d\n\r", func, IntSrc, ringTime);
				ring = 0;
				rdata[3] = (ringTime & 0xff00) >> 8;
				rdata[4] = ringTime & 0xff;
				IntSrc |= 4;
				owResetSignal();
			}
			/* fall-through */
		case BTN_INVALID:
			enablePinInts();
			func &= ~(0x10);
			break;
		}
	}
#endif	
#ifdef BTN_SUPPORT
	if (func & 0x20 || func & 0x8) {
		switch (checkBtn(BTNIN, &btn[0])) {
		case BTN_PRESSED_LONG:
#if (OW_FAMILY == 0xA8)
			digitalWrite(DOOR, HIGH);
#endif		
			enablePinInts();
			break;
		case BTN_PRESSED:
			func &= ~(8);
#if (OW_FAMILY == 0xA8)
			if (bell == 1) {
				bell = 0;
				digitalWrite(RING, LOW);
			}
			else {
				bell = 1;
				digitalWrite(RING, HIGH);
			}
#endif			
			IntSrc |= 2;
			owResetSignal();
			enablePinInts();
			break;
		case BTN_RELEASED:
			/* fall-through */
		case BTN_INVALID:
		case BTN_HIGH:
#if (OW_FAMILY == 0xA8)
			digitalWrite(DOOR, LOW);
#endif	
			func &= ~(8);
			enablePinInts();
			break;
		}
	}
#endif	
#ifdef DEBUG
	if (func & (2)) {
		if (led == 1 && millis() - ledTime > 200) {
			digitalWrite(LED, HIGH);
			prepareScratch();
			ledTime = millis();
			led = 0;
		}
		if (led == 0 && millis() - ledTime > 800) {
			digitalWrite(LED, LOW);
			prepareScratch();
			ledTime = millis();
			led = 1;
		}
	}
#endif	
#ifdef ENABLE_SLEEP
	oldSREG = SREG;
	cli();
	if ((mode == OWM_SLEEP || mode == OWM_READ_SCRATCHPAD) &&
		func == 0) {
		/* for wake-up */
		enablePinInts();
		SET_LEVEL;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		SREG = oldSREG;
		sleep_mode();
		delay(1);
		prepareScratch();
#if defined(HAVE_UART) && defined(DEBUG)
	} else {
		SREG = oldSREG;
		if (millis() - stTime > 5000) {
			stTime = millis();
			printf("func=%X\tow=%X INT=%X PCMSK=%X\n\r", func, mode, IntSrc, PCMSK0);
			printf("DDRC=%X PORTC=%X PINC=%X\n\r", DDRC & 0xf, PORTC & 0xf, PINC & 0xf);
			printf("DDRD=%X PORTD=%X PIND=%X\n\r", DDRD & OW_PINN, PORTD & OW_PINN, PIND & OW_PINN);
		}
#endif		
	}
	
#endif /* ENABLE_SLEEP */
}

int main(void)
{
	setup();

	while (1) {
		loop();
	}
}
