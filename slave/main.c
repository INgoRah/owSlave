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
#define OWT_READLINE 5 // 3 for fast master, 4 for slow master and long lines
#define OWT_MIN_RESET 40
#define OWT_LOWTIME 5 // 3 for fast master, 4 for slow master and long lines 
#define OWT_PRESENCE 20
#define OWT_RESET_PRESENCE 4

#define MAX_DATA 10
#ifndef OW_SERIAL
#define OW_SERIAL 1
#endif	

#define MAX_BTN 2

#define	BTN_LOW  0
#define	BTN_HIGH  1
#define	BTN_TIMER_LOW  2
#define	BTN_TIMER_HIGH 3
#define	BTN_UNSTABLE 4
#define BTN_PRESSING 5
#define BTN_PRESS_LOW 6
#define BTN_PRESSED_LONG 7
#define BTN_PRESSED 8
#define BTN_UNKNOWN 9
#define BTN_INVALID 10

struct button {
	unsigned long time;
	unsigned long press;
	int state /*: 4*/;
	int last /*: 4*/;
}btn[MAX_BTN];

volatile uint8_t data[MAX_DATA+1];

uint8_t owCommand(uint8_t cmd);

volatile uint8_t scrc;		//CRC calculation
/* Input buffer for a command */
volatile uint8_t cbuf;
/* Store interrupt source for read out via scratchpad */
volatile uint8_t IntSrc;

uint8_t owid[8] =
#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x04, 0xFA };
#if (OW_FAMILY == 0xA3)
{ 0xA3, 0x01, 0x3, 0x0, 0x00, 0x00, 0x01, 0xCA };
#else
{ 0xA2, 0x01, 0x1, 0x0, 0x00, 0x00, 0x01, 0xC5 };
#endif
#else
{ 0xA8, 0x01, 0x5, 0x0, 0x00, 0x00, 0x01, 0xC5 };
#endif
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x01, 0xC5 };
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x02, 0x27 };
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x03, 0x79 };

uint8_t func;		// function currently selected
volatile uint8_t bitp;		//pointer to current bit
volatile uint8_t bytep;		//pointer to current byte

volatile uint8_t mode;		//state
volatile uint8_t wmode;		//if 0 next bit that send the device is  0
volatile uint8_t actbit;	//current
volatile uint8_t srcount;	//counter for search rom
uint8_t pinMsk;

PIN_INT {
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

TIMER_INT {
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
					data[0] = 0; 
					break;
				case OW_READ_SCRATCHPAD:
					lmode = OWM_READ_SCRATCHPAD;
					lbytep = 0;
					scrc = 0;	//from first position
					/*
					 * setup data to be read
					 */
					data[0] = IntSrc;
					/* clear interrupt status */
					IntSrc = 0;
#if defined(__AVR_ATtiny25__) || defined (__AVR_ATtiny85__)
					data[1] = 0;
					if ((PININ & LED) == 0)
						data[1] |= 2;
					if ((PININ & LAMP) == 0)
						data[1] |= 1;
#else
					data[1] = (PINOUT & 0x1F);
//					data[7] = DDRC;
#endif	
					data[2] = (PININ & 0x1F);
					data[5] = func;
					data[6] = btn[0].state;
					/* in case of ADC reset activity bit */
					func &= ~4;
					lactbit = (lbitp & data[0]) == lbitp;
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
				data[lbytep] |= lbitp;
			lbitp = (lbitp << 1);
			if (!lbitp) {
				/* prepare next bit */
				lbitp = 1;
				if (lbytep++ == MAX_DATA) {
					lmode = OWM_SLEEP;
					break;
				}
				data[lbytep] = 0;
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
					lbitp = 1;
					if (lbytep++ >= MAX_DATA) {
						lmode = OWM_SLEEP;
						break;
					} else if (lbytep == 9)
						data[9] = lscrc;
				}
				lactbit = (lbitp & data[lbytep]) == lbitp;
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
 Depending on the return value to upper state machine handles further data
returns new one wire mode */
uint8_t owCommand(uint8_t cmd)
{
	uint8_t lmode = OWM_SLEEP;
#if (OW_FAMILY == 0xA3)
	int i;
	union {
		unsigned long l;
		unsigned char b[4];
	} d;
#endif

	/* 4xh : switch output of pin 1-4 to high
	 * 5xh : switch output of pin 1-4 to low
	 */
	if ((cmd & 0x30) == 0x30) {
		/* on */
		pinMode(cmd & 0x7, OUTPUT);
		digitalWrite (cmd & 0x7, LOW);
	}
	if ((cmd & 0x30) == 0x20) {
		/* off */
		/* switch to input */
		pinMode(cmd & 0x7, INPUT);
	}
	switch (cmd) {
	case 0x48:
		func &= ~2;
		break;
	case 0x44:	//Start Convert
		func |= 0x4;
		break;
	case 0x64:	//check button state
		//pinMode(1, INPUT);
		func |= 0x8;
		break;
	case 0x65:	//disable check button
		func &= ~0x8;
		break;
	case 0x68:
		func |= 2;
		break;
	case 0x70:
#if (OW_FAMILY == 0xA3)
		// send IR
		for (i = 0; i < 4; ++i)
			d.b[i] = data[i];
		/*
		 * Tuner:5EA16897  = 1587636375
		 * CD:5EA1A857 = 1587652695
		 * On/off 5EA1F807 = 1587673095
		 * Vol up 5EA158A7 = 1587632295
		 * Vol down 5EA1D827 = 1587664935
		 */
		ir_sendNEC (d.l, 32);
#endif
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
ISR(PCINT0_vect)
{
#if defined(PCMSK0)
	cbi (PCMSK0, PCINT0);
#endif
#if defined(PCMSK)
	cbi (PCMSK, PCINT0);
#endif
	/* check button till released */
	func |= 8;
	digitalWrite(3, LOW);
}
#endif

/* PB1 */
#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
#if defined(PCMSK0)
	cbi (PCMSK0, PCINT1);
#endif
#if defined(PCMSK)
	cbi (PCMSK, PCINT1);
#endif
	IntSrc |= 4;
}
#endif

/* static setup at compile time */
void owidSetup()
{
	eeprom_read_block((void*)&owid, (const void*)0, 7);
	if (owid[0] != 0xFF) {
		owid[1] = FD_SERIAL + 1;
		return;
	}
	owid[0] = OW_FAMILY;
	owid[1] = FD_SERIAL;
	owid[2] = FD_VERSION;
	/* 4th: max output/input pins | [7..4] output| [3..0] input | */
#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
	owid[3] = 0x55;
#else
	owid[3] = 0x32;
#endif	
#if (OW_FAMILY == 0xA2)
	owid[4] = FD_PIN_LAMP;
	owid[5] = 0;
#endif
#if (OW_FAMILY == 0xA3)
	owid[4] = FD_PIN_LAMP | (FD_PIN_LEDST << 2) | (FD_PIN_IRTX << 4);
	owid[5] = (FD_PIN_BTN << 4);
#endif
#if (OW_FAMILY == 0xA8)
	/*
	 * Pin 1-4 setup
	 * #4 PD3 input with external interrupt
	 */
	owid[4] = (FD_PIN_LAMP) | (FD_PIN_LEDST << 2) | (FD_PIN_OUT << 6);
	/*
	 * Pin 5-8 setup
	 */
	owid[5] = (FD_PIN_INT << 2) | (FD_PIN_BTN << 4);
#endif
	/* group id */
	owid[6] = 1;
	owid[7] = crc();
	eeprom_write_block ((const void*)&owid, (void*)0, 7);
}

void pinSetup()
{
	//pinMode (5, OUTPUT);
	//pinMode (4, OUTPUT);
	//pinMode (3, OUTPUT);
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
	// setup PIN #3, LED
	switch ((owid[4] & 0xc0) >> 6) {
		case FD_PIN_OUT:
			pinMode(3, OUTPUT);
			break;
	}
	pinMsk = 0; 
	// setup PIN #5
	if (((owid[5] & 0x0c) >> 2) == FD_PIN_INT) {
		pinMode (5, INPUT);
		pinMsk |= PCINT1; 
	}
	// setup PIN #6
	if (((owid[5] & 0x30) >> 4) == FD_PIN_BTN) {
		pinMode (6, INPUT);
		pinMsk |= PCINT0; 
	}
#if defined PCIFR
	// atmega 88/168/...
	sbi (PCICR, PCIE0);
	PCIFR=0xff;
	PCMSK0 = pinMsk;
#elif defined(GIFR) 
	// attiny85....
	sbi (GIMSK, PCIE);
	GIFR = 0xff;
	// attiny85....
	PCMSK = pinMsk;
#else
	#error missing PC interrupt mask
#endif
}

void setup() {
	int i;
#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
#ifdef HAVE_UART
	uart_init((UART_BAUD_SELECT_DOUBLE_SPEED((BAUD), F_CPU)));
#else
	DDRD |= _BV (PD0) | _BV (PD1);
#endif
#endif
	IntSrc = 0;
	wmode = OWW_NO_WRITE;
	func = 0;
	RESET_LOW;
	SET_FALLING;
	INIT_AVR();
	DIS_TIMER;
	mode = OWM_SLEEP;
	data[0] = 0;
	data[1] = 0;
	data[2] = 0xbe;
	data[3] = 0xef;
	data[4] = 0xca;
	data[5] = 0xfe;
	owidSetup();
	pinSetup();
	sei();
	for (i = 5; i > 1; i--) {
		LED_OFF();
		delay(25 * i);
		LED_ON();
		delay(50);
	}
	LED_OFF();
}

int checkBtn (uint8_t pin, struct button *p)
{
	uint8_t in;
	
	in = digitalRead(pin);
	if (p->last != in) {
		p->last = in;
		p->state = BTN_UNSTABLE;
		p->time = 0;
		return BTN_UNSTABLE;
	}
	/* from here we are stable */
	switch (p->state) {
	case BTN_TIMER_HIGH:
		if (millis() - p->time < 100)
			return BTN_UNSTABLE;
		if (in != HIGH)
			return BTN_INVALID;
		p->state = BTN_HIGH;
		/* button released or was high, check press time */
		if (p->press == 0)
			return BTN_INVALID;
		return BTN_PRESSED;
	case BTN_TIMER_LOW:
		if (millis() - p->time < 100)
			return BTN_UNSTABLE;
		if (in != LOW)
			return BTN_INVALID;
		/* stable state */
		/* 0 or 1 = LOW or HIGH */
		p->state = BTN_LOW;
		p->time = millis();
		p->press = millis();
		return BTN_PRESS_LOW;
		break;
	case BTN_LOW:
		if (p->press == 0)
			return BTN_INVALID;
		if (millis() - p->press > 1000) {
			p->press = 0;
			return BTN_PRESSED_LONG;
		}
		return BTN_PRESSING;
	case BTN_HIGH:
		p->press = 0;
		return BTN_HIGH;
	case BTN_UNSTABLE:
		if (in == LOW)
			p->state = BTN_TIMER_LOW;
		else
			p->state = BTN_TIMER_HIGH;
		p->time = millis();
		return BTN_UNSTABLE;
	}
	return BTN_UNKNOWN;
}

static inline void enablePinInts()
{
#if defined(PCMSK0)
	PCMSK0 = pinMsk;
	PCIFR = pinMsk;
#elif defined(PCMSK) 
	// attiny85....
	PCMSK = pinMsk;
	sbi(GIFR, PCIF);
#endif
}

void loop() {
	int i;
	static long ledTime;
	static char led;
	int t;

	if (func & 8) {
		t = checkBtn(6, &btn[0]);
		data[7] = t;
		switch (t) {
		case BTN_PRESS_LOW:
			func |= 1;
			digitalWrite(3, HIGH);
			break;
		case BTN_PRESSED_LONG:
			func &= ~(1 | 2 | 8);
			LED_OFF();
			IntSrc |= 1;
			enablePinInts();
			break;
		case BTN_PRESSED:
			func &= ~(1 | 8);
			if (func & 2)
				func &= ~2;
			else
				func |= 2;
			IntSrc |= 2;
			enablePinInts();
			break;
		case BTN_UNSTABLE:
			/* wait */
			break;
		case BTN_HIGH:
		case BTN_INVALID:
			func &= ~(1 | 8);
			enablePinInts();
			digitalWrite(3, HIGH);
			break;
		}
	}
	if (IntSrc & 4) {
	}
	t = 900;
	if (func & 2)
		t = 400;
	if (func & 1)
		t = 50;
	if (func & ( 2 | 1 | 8)) {
		if (led == 1 && millis() - ledTime > 100) {
			LED_OFF();
			ledTime = millis();
			led = 0;
		}
		if (led == 0 && millis() - ledTime > t) {
			LED_ON();
			ledTime = millis();
			led = 1;
		}
	}
#define SLEEP_ENABLE
	if (mode == OWM_SLEEP && func == 0) {
#ifdef SLEEP_ENABLE
		uint8_t oldSREG;
		for (i = 5; i > 1; i--) {
			LED_OFF();
			delay(200);
			LED_ON();
			delay(100 * i);
			if (func != 0)
				return;
		}
		LED_OFF();
		digitalWrite(3, LOW);
		oldSREG = SREG;
		cli();
		/* for wake-up */
		//if ((owid[5] & 0x03) == FD_PIN_BTN)
		enablePinInts();
		SET_LEVEL;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		SREG = oldSREG;
		sleep_mode();
		digitalWrite(3, HIGH);
#endif			
	}
}

int main(void)
{
	init();
	setup();

	while (1) {
		loop();
	}
}
