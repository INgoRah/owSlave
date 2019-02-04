//OW Pin
#define OW_PORT PORTD		//1 Wire Port
#define OW_PIN PIND		//1 Wire Pin as number
#define OW_PORTN (1<<PIND2)	//Pin as bit in registers
#define OW_PINN (1<<PIND2)
#define OW_DDR DDRD		//pin direction register

//Pin interrupt  
#define EN_OWINT {EIMSK|=(1<<INT0);EIFR|=(1<<INTF0);}	//enable interrupt
#define DIS_OWINT  EIMSK&=~(1<<INT0);	//disable interrupt
#define SET_RISING EICRA=(1<<ISC01)|(1<<ISC00);	//set interrupt at rising edge
#define SET_FALLING EICRA=(1<<ISC01);	//set interrupt at falling edge
#define SET_LEVEL EICRA=0; //set interrupt at low level
#define CHK_INT_EN (EIMSK&(1<<INT0))==(1<<INT0)	//test if interrupt enabled

//Timer Interrupt
#define EN_TIMER {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);}	//enable timer interrupt
#define DIS_TIMER TIMSK0 &=~(1<<TOIE0);	// disable timer interrupt
#define TCNT_REG TCNT0		//register of timer-counter

/* 
 * Initializations of AVR
 * CLKPR: 8Mhz
 * GIMSK: set direct GIMSK register 
 * TCCR0B: 8mhz /64 couse 8 bit Timer interrupt every 8us
 * PINs B1-5 input signal, PINs C1..5 output
 * PIN D2 one wire interface
 * #1 PC2 output relais, active low
 * #2 PC1 LED, active low
 * #3 PB1 input with pull-up
 *        or output of PWM for IR_SEnd active high Family A3
 * #4 PD3 input with external interrupt
 * #5 PB0 input switch
 * PIN C3-C5 optional output
 */
#define INIT_AVR() CLKPR=(1<<CLKPCE); \
	CLKPR=0; \
	TIMSK0=0; \
	EIMSK=(1<<INT0); \
	TCCR0B=(1<<CS00)|(1<<CS01);

#define PINOUT PINC

#define LED 3 /* PC3 */
#define DOOR 1 /* PC1 */
#define RING 0 /* PC2 */
#define RINGIN 7 /* PB1 */
#define BTNIN 6 /* PB0 */

#ifdef HAVE_UART
#include "uart.h"
#endif
