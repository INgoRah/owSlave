//owdevice - A small 1-Wire emulator for AVR Microcontroller
//
//Copyright (C) 2012  Tobias Mueller mail (at) tobynet.de
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
//
//VERSION 1.1 1Wire Barometric Pressure Sensor ATMEGA48 
//VERSION 1.2 DS18B20  ATTINY13 (AD input) and ATTINY25 (internal sensor)
//http://www.tm3d.de/index.php/1-wire-device-mit-avr
// http://www.mikrocontroller.net/topic/44100#new
// http://www.tm3d.de/index.php/
#include "Arduino.h"

#define OW_WRITE_SCRATCHPAD 0x4E
#define OW_READ_SCRATCHPAD 0xBE

extern void ir_sendNEC (unsigned long data, int nbits);
	
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
volatile uint8_t data[MAX_DATA+1];

uint8_t owCommand(uint8_t cmd);

volatile uint8_t scrc;		//CRC calculation

volatile uint8_t cbuf;		//Input buffer for a command
uint8_t owid[8] =
{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x04, 0xFA };
//{ 0xA3, 0x0, 0x10, 0x84, 0x00, 0x00, 0x01, 0xCA };
//{0x28, 0xA2, 0xD9, 0x84, 0x00, 0x00, 0x02, 0xEA};  
//{0x28, 0xA2, 0xD9, 0x84, 0x00, 0x00, 0x03, 0xB4};
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x01, 0xC5 };
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x02, 0x27 };
//{ 0xA2, 0xA2, 0x10, 0x84, 0x00, 0x00, 0x03, 0x79 };
//set your own ID http://www.tm3d.de/index.php/tools/14-crc8-berechnung

/*
commands supported:
LED On: 0x67 / 103
LED Off: 0x47 / 71
cmd 1 197 103
cmd 1 197 71
cmd 2 39 ...

Read 0xBE / 190
read 1 197 190
read 1 197 71
read 2 39 ...
returns DE AD <lamp status> <state>
*/

volatile uint8_t func;		// function currently selected

volatile uint8_t bitp;		//pointer to current bit
volatile uint8_t bytep;		//pointer to current byte

volatile uint8_t mode;		//state
volatile uint8_t wmode;		//if 0 next bit that send the device is  0
volatile uint8_t actbit;	//current
volatile uint8_t srcount;	//counter for search rom

void pinMode(uint8_t pin, uint8_t mode)
{
	uint8_t bit = pin;

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		DDRB &= ~bit;
		PORTB &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		PORTB &= ~bit;
		PORTB |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		DDRB |= bit;
		SREG = oldSREG;
	}
}

void digitalWrite(uint8_t bit, uint8_t val)
{
	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW)
		PORTB &= ~bit;
	else
		PORTB |= bit;

	SREG = oldSREG;
}

void delayMicroseconds(unsigned int us)
{
	int i;
	
	for (i=0; i < us; i++)
		_delay_us (1);
}

PIN_INT {

	if (wmode == OWW_WRITE_0) {
		SET_LOW;
		wmode = OWW_NO_WRITE;
	}		//if necessary set 0-Bit 
	/* disable interrupt, only in OWM_SLEEP mode it is active */
	DIS_OWINT;

	switch (mode) {
	case OWM_SLEEP:
		TCNT_REG = ~(OWT_MIN_RESET);
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
	DBG_2OFF();
#ifdef TEST_CODE
	if (mode == OWM_TEST) {
		LED_ON();
		DIS_TIMER;
	}
#endif	
	//Interrupt still active ?
	if (CHK_INT_EN) {
		//maybe reset pulse
		if (p == 0) {
			lmode = OWM_CHK_RESET;	//wait for rising edge
			SET_RISING;
		}
		DIS_TIMER;
	} else
		switch (lmode) {
		case OWM_RESET:	//Reset pulse and time after is finished, now go in presence state
			lmode = OWM_PRESENCE;
			SET_LOW;
			TCNT_REG = (uint8_t)~(OWT_PRESENCE);
			DIS_OWINT;	//No Pin interrupt necessary only wait for presence is done
			LED_OFF();
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
			RESET_LOW;	//Set low also if nothing send (branch takes time and memory)
			lsrcount++;	//next search rom mode
			switch (lsrcount) {
			case 1:
				lwmode = !lactbit;	//preparation sending complement
				break;
			case 3:
				if (p != (lactbit == 1)) {
					//check master bit
					lmode = OWM_SLEEP;	//not the same go sleep
				} else {
					lbitp = (lbitp << 1);
					if (lbitp == 0) {
						//prepare next bit
						lbitp = 1;
						lbytep++;
						if (lbytep >= 8) {
							lmode = OWM_SLEEP;	//all bits processed 
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
						lmode = OWM_READ_COMMAND;	//same? get next command
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
	DBG_2ON();
}

/* finished receiving a command. This function is called from timer context.
 Depending on the return value to upper state machine handles further data
returns new one wire mode */
uint8_t owCommand(uint8_t cmd)
{
	uint8_t lmode = OWM_SLEEP;
	int i;
	union {
		unsigned long l;
		unsigned char b[4];
	} d;

	switch (cmd) {
	case 0x67:
		//LED_ON();
		func = 1;
		DIS_TIMER;
		break;
	case 0x68:
		LAMP_ON();
		DIS_TIMER;
		break;
	case 0x69:
		break;
	case 0x70:
		// send IR
		DIS_TIMER;
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
		break;
	case 0x47:
		LED_OFF();
		func = 0;
		break;
	case 0x48:
		LAMP_OFF();
		func = 0;
		break;
	case 0x44:	//Start Convert
	default:
		//all other commands do nothing
		break;
	}
	
	return lmode;
}

static int lamp = 0;
void check_btn ()
{
	static int t_sample = 0;

	if (T_IN() == 0 && t_sample != 0xff) {
		if (t_sample++ > 150) {
			t_sample = 0xff;
			if (lamp == 0) {
				lamp = 1;
				LED_ON();
				LAMP_ON();
			} else {
				lamp = 0;
				LAMP_OFF();
				LED_OFF();
			}
			//data[2] = lamp;
		}
		_delay_ms(1);
	}
	if (T_IN() && t_sample == 0xff)
		t_sample = 0;
}

static uint8_t crc() {
	unsigned char inbyte, crc = 0, len = 7;
	unsigned char i, max;
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

void setup() {
#if defined __AVR_ATmega48__ || defined __AVR_ATmega88__
#ifdef HAVE_UART
	uart_init((UART_BAUD_SELECT_DOUBLE_SPEED((BAUD), F_CPU)));
#else
	DDRD |= _BV (PD0) | _BV (PD1);
#endif
#endif
	wmode = OWW_NO_WRITE;
	func = 0;
	RESET_LOW;
	SET_FALLING;
	INIT_AVR();
	DIS_TIMER;
	sei();
	_delay_ms(50);
	LED_ON();
	_delay_ms(200);
	LED_OFF();
	mode = OWM_SLEEP;
	data[0] = 0xde;
	data[1] = 0xad;
	data[2] = 0xbe;
	data[3] = 0xef;
	data[4] = 0xca;
	data[5] = 0xfe;
	owid[7] = crc();
}
	
void loop() {
	//check_btn();
#if 1
	if (func == 1 && mode == OWM_SLEEP) {
		LED_ON();
		_delay_ms(200);
		LED_OFF();
		_delay_ms(1000);
	}
#endif
}

int main(void)
{
	setup();

	/*DBG_P("Init Slave\n\r");*/
	while (1) {
		loop();
	}
}
