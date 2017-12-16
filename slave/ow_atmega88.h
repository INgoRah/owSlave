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
#define CHK_INT_EN (EIMSK&(1<<INT0))==(1<<INT0)	//test if interrupt enabled
#define PIN_INT ISR(INT0_vect)	// the interrupt service routine
//Timer Interrupt
#define EN_TIMER {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);}	//enable timer interrupt
#define DIS_TIMER TIMSK0 &=~(1<<TOIE0);	// disable timer interrupt
#define TCNT_REG TCNT0		//register of timer-counter
#define TIMER_INT ISR(TIMER0_OVF_vect)	//the timer interrupt service routine

//Initializations of AVR
#define INIT_AVR CLKPR=(1<<CLKPCE); \
	CLKPR=0; /*8Mhz*/  \
	TIMSK0=0; \
	EIMSK=(1<<INT0);  /*set direct GIMSK register*/ \
	TCCR0B=(1<<CS00)|(1<<CS01);	/*8mhz /64 couse 8 bit Timer interrupt every 8us */ \
	DDRC = _BV(PC1);

#define LED_OFF()	PORTC |= _BV (PC1)
#define LED_ON()	PORTC &= ~(_BV (PC1))

#define DBG_1ON() PORTD |= _BV (PD0)
#define DBG_1OFF() PORTD &= ~(_BV (PD0))
#define DBG_2ON() PORTD |= _BV (PD1)
#define DBG_2OFF() PORTD &= ~(_BV (PD1))

#ifdef HAVE_UART
#include "uart.h"
#define DBG_C(x) uart_putc(x)
#define DBG_P(x) uart_puts_P(x)
#define DBG_N(x) uart_puthex_nibble(x)
#define DBG_X(x) uart_puthex_byte(x)
#define DBG_Y(x) uart_puthex_word(x)
#define DBG_NL() uart_putc('\n')
#else				/* no UART */
#define DBG_C(x) do { } while(0)
#define DBG_P(x) do { } while(0)
#define DBG_N(x) do { } while(0)
#define DBG_X(x) do { } while(0)
#define DBG_Y(x) do { } while(0)
#define DBG_NL() do { } while(0)
#endif
