  //OW_PORT Pin 19  - PD5
  //OW Pin
#define OW_PORT PORTD		//1 Wire Port
#define OW_PIN PIND		//1 Wire Pin as number
#define OW_PORTN (1<<PIND5)	//Pin as bit in registers
#define OW_PINN (1<<PIND5)
#define OW_DDR DDRD		//pin direction register
#define SET_LOW OW_DDR|=OW_PINN;OW_PORT&=~OW_PORTN;	//set 1-Wire line to low
#define RESET_LOW {OW_DDR&=~OW_PINN;}	//set 1-Wire pin as input

//Pin interrupt
#define EN_OWINT {PCIFR|=(1<<PCIF3);PCICR|=(1<<PCIE3); PCIFR|=(1<<PCIF3);}	//PCMSK3 |= (1<<PCINT29);}//GICR|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt
#define DIS_OWINT  PCICR &= ~(1<<PCIE3);	//GICR&=~(1<<INT0);  //disable interrupt

uint8_t setRF;
#define SET_RISING setRF = 1;	//MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING setRF = 0;	//MCUCR=(1<<ISC01); //set interrupt at falling edge
#define CHK_INT_EN (PCICR&(1<<PCIE3))==(1<<PCIE3)	//test if interrupt enabled
#define PIN_INT ISR(PCINT3_vect)	// the interrupt service routine
  //Timer Interrupt
#define EN_TIMER {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);}	//enable timer interrupt
#define DIS_TIMER TIMSK0 &=~(1<<TOIE0);	// disable timer interrupt
#define TCNT_REG TCNT0		//register of timer-counter
#define TIMER_INT ISR(TIMER0_OVF_vect)	//the timer interrupt service routine

#if F_CPU == 20000000
#define OWT_MIN_RESET 113	//360
#define OWT_RESET_PRESENCE 10	//32
#define OWT_PRESENCE 50		//160
#define OWT_READLINE 8		//for fast master, 10 for slow master and long lines 24-32
#define OWT_LOWTIME 8		//for fast master, 10 for slow master and long lines 24-32
#elif F_CPU == 8000000
#define OWT_MIN_RESET 45	//360
#define OWT_RESET_PRESENCE 4	//32
#define OWT_PRESENCE 20		//160
#define OWT_READLINE 3		//for fast master, 4 for slow master and long lines 24-32
#define OWT_LOWTIME 3		//for fast master, 4 for slow master and long lines 24-32
#endif
  //Initializations of AVR
#define INIT_AVR TIMSK0=0; \
             PCMSK3 |= (1<<PCINT29); \
             PCICR|=(1<<PCINT3);  /*set direct GIMSK register*/ \
             TCCR0B=(1<<CS00)|(1<<CS01);	/*8mhz /64 couse 8 bit Timer interrupt every 8us */
