// OW_PORT Pin 7  - PB2
// OW Pin
#define OW_PORT PORTB // 1 Wire Port
#define OW_PIN PINB // 1 Wire Pin as number
#define OW_PORTN (1<<PINB2)  //Pin as bit in registers
#define OW_PINN (1<<PINB2)
#define OW_DDR DDRB  //pin direction register

//Pin interrupt	
#define EN_OWINT {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
#define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
#define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
#define SET_LEVEL MCUCR=0; //set interrupt at low level
#define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
#define PIN_INT ISR(INT0_vect)  // the interrupt service routine

//Timer Interrupt
#define EN_TIMER {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
#define DIS_TIMER TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
#define TCNT_REG TCNT0  //register of timer-counter
#define TIMER_INT ISR(TIM0_OVF_vect) //the timer interrupt service routine

/* 
 * Initializations of AVR
 * CLKPR: 8Mhz
 * GIMSK: set direct GIMSK register 
 * TCCR0B: 8mhz /64 couse 8 bit Timer interrupt every 8us
 * PB2 one wire interface
 * #1: PB4 output relais, active low or ADC for Family A4
 * #2: PB3 LED, active low
 * #3: PB1 input with pull-up
 *        or output of PWM for IR_SEnd active high Family A3
 * #4: not supported
 * #5: PB0 input switch
 */ 
#define INIT_AVR() CLKPR=(1<<CLKPCE); \
	CLKPR=0; \
	TIMSK=0; \
	GIMSK=(1<<INT0); \
	TCCR0B=(1<<CS00)|(1<<CS01); \
	DDRB = _BV(PB3) | _BV(PB4) | _BV(PB1); \
	PORTB = _BV(PB3) | _BV(PB4) | _BV (PB0)

#define PINOUT PINB
#define PININ PINB
#define LED _BV (PB3)
#define LAMP _BV (PB4)
#define LED_OFF()	PORTB |= _BV (PB3)
#define LED_ON()	PORTB &= ~(_BV (PB3))
#define LAMP_OFF()	PORTB |= _BV (PB4)
#define LAMP_ON()	PORTB &= ~(_BV (PB4))
