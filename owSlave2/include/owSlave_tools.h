#ifndef OWSLAVE_TOOLS_H
#define OWSLAVE_TOOLS_H

#define OWST_EXTERN_VARS
	extern uint8_t mode;\
	extern uint8_t gcontrol;\
	extern uint8_t reset_indicator;\
	extern uint8_t alarmflag; \
	extern void OWINIT(void);\
	extern void EXTERN_SLEEP(void);

#if defined(__AVR_ATtiny85__)
#define OWST_INIT_ALL_OFF \
	PRR|=(1<<PRUSI)|(1<<PRADC);  /*Switch off usi and adc for save Power*/\
	ACSR|=(1<<ACD);  /*Disable Comparator*/\
	ADCSRB|=(1<<ACME); /*Disable Analog multiplexer*/\
	PORTB |= ~(1<<PINB2); /*Make PullUp an all Pins but not OW_PIN*/

#define OWST_EN_PULLUP MCUCR &=~(1<<PUD); /*All Pins Pullup...*/

#define OWST_MAIN_END \
	if (((TIMSK & (1<<TOIE0))==0)&& (mode==0))	  {\
		MCUCR |= (1<<SE)| (1<<SM1);\
		MCUCR &=~(1<<ISC01);\
	} else {\
		MCUCR |= (1<<SE);\
		MCUCR &=~ (1<<SM1);\
	} \
	asm("SLEEP");

#define OWST_WDT_ISR

#define OWST_INIT_USI_ON
#endif /* __AVR_ATtiny85__ */

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define OWST_INIT_ALL_OFF \
	PRR|=(1<<PRUSI)|(1<<PRADC);  /*Switch off usi and adc for save Power*/\
	ACSR|=(1<<ACD);  /*Disable Comparator*/\
	ADCSRB|=(1<<ACME); /*Disable Analog multiplexer*/\
	PORTB |= ~(1<<PINB2); /*Make PullUp an all Pins but not OW_PIN*/\
	PORTA |=0xFF;

#define OWST_INIT_ADC_ON \
PRR|=(1<<PRUSI);  /*Switch off usi and adc for save Power*/\
ACSR|=(1<<ACD);  /*Disable Comparator*/\
PORTB = ~(1<<PINB2); /*Make PullUp an all Pins but not OW_PIN*/\
PORTA = 0xFF;

#define OWST_INIT_USI_ON \
PRR|=(1<<PRADC);  /*Switch off usi and adc for save Power*/\
ACSR|=(1<<ACD);  /*Disable Comparator*/\
ADCSRB|=(1<<ACME); /*Disable Analog multiplexer*/\
PORTB |= ~(1<<PINB2); /*Make PullUp an all Pins but not OW_PIN*/\
PORTA |= 0xFF;

#define OWST_INIT_ALL_ON \
PORTB|=~(1<<PINB2); /*Make PullUp an all Pins but not OW_PIN*/\

#define OWST_EN_PULLUP MCUCR &= ~(1<<PUD); /*All Pins Pullup...*/

#define OWST_WDT_ISR \
uint8_t wdcounter=0;\
ISR(WATCHDOG_vect) {/*	#else ISR(WDT_vect) {		#endif*/\
		wdcounter++;\
		if (reset_indicator==1) reset_indicator++;\
		else if (reset_indicator==2) mode=0;\
}

#define OWST_WDR_CONFIG8\
	WDTCSR |= ((1<<WDCE) );   /* Enable the WD Change Bit//| (1<<WDE)*/\
	WDTCSR |=   (1<<WDIE) |              /* Enable WDT Interrupt*/\
	(1<<WDP3) | (1<<WDP0);   /*Set Timeout to ~8 seconds*/

#define OWST_WDR_CONFIG4\
	WDTCSR |= ((1<<WDCE) );   /* Enable the WD Change Bit//| (1<<WDE)*/\
	WDTCSR |=   (1<<WDIE) |              /* Enable WDT Interrupt*/\
	(1<<WDP3) ;   /*Set Timeout to ~2 seconds*/

#define OWST_WDR_CONFIG2\
	WDTCSR |= ((1<<WDCE) );   /* Enable the WD Change Bit//| (1<<WDE)*/\
	WDTCSR |=   (1<<WDIE) |              /* Enable WDT Interrupt*/\
	(1<<WDP2) | (1<<WDP1)| (1<<WDP0);   /*Set Timeout to ~1 seconds*/

#define OWST_WDR_CONFIG1\
	WDTCSR |= ((1<<WDCE) );   /* Enable the WD Change Bit//| (1<<WDE)*/\
	WDTCSR |=   (1<<WDIE) |              /* Enable WDT Interrupt*/\
	(1<<WDP2) | (1<<WDP1);   /*Set Timeout to ~1 seconds*/


#define OWST_TESTSW \
	int testSW(void) {\
		uint8_t r;\
		DDRB&=~(1<<PORTB0);  /*Eingang*/\
		__asm__ __volatile__ ("nop");\
		PORTB|=(1<<PORTB0); /*Pullup*/\
		__asm__ __volatile__ ("nop");\
		__asm__ __volatile__ ("nop");\
		__asm__ __volatile__ ("nop");\
		__asm__ __volatile__ ("nop");\
		__asm__ __volatile__ ("nop");\
		r=PINB&(1<<PORTB0);\
		__asm__ __volatile__ ("nop");\
		PORTB&=~(1<<PORTB0);\
		__asm__ __volatile__ ("nop");\
		DDRB|=(1<<PORTB0);  /*Eingang*/\
		return (r==0);\
	}\

#define OWST_MAIN_END \
	if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))	  {\
		MCUCR|=(1<<SE)|(1<<SM1);\
		MCUCR&=~(1<<ISC01);\
	} else {\
		MCUCR|=(1<<SE);\
		MCUCR&=~(1<<SM1);\
	}\
	asm("SLEEP");

#endif /* __AVR_ATtinyx4__ */

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__)
#define ATMEGA
#define OWST_INIT_ALL_OFF \
	PRR|=(1<<PRTWI)|(1<<PRSPI)|(1<<PRADC);  /*Switch off SPI/TWI and adc for save Power*/\
	ACSR|=(1<<ACD);  /*Disable Comparator*/\
	ADCSRB|=(1<<ACME); /*Disable Analog multiplexer*/\
	PORTD |= ~(1<<PIND2); /*Make PullUp an all Pins but not OW_PIN*/\

#define OWST_INIT_ALL_ON \
	PORTA|=0xFF;
	
#define OWST_EN_PULLUP MCUCR &=~(1<<PUD); /*All Pins Pullup...*/

#define OWST_WDT_ISR

#define OWST_MAIN_END \
	if (((TIMSK0 & (1<<TOIE0))==0) && (mode==0)) { \
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); \
		EICRA &= ~(1<<ISC01); \
	} else \
		set_sleep_mode(SLEEP_MODE_ADC); \
	} \
	sleep_cpu();

#endif /* __AVR_ATmegax8x__ */

//********************** AD_WANDLER ********************************
//******************************************************************
//clock fuer ADC 50kHz - 200kHz     8M / 128 = 62500   = 4M/64   erste Messng 25 = 0,4ms / zweite Messung 13 rund 0,21ms
#ifdef __4MHZ__
#define OWST_INIT_ADC \
ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
#else
#define OWST_INIT_ADC \
ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
#endif

#define OWST_ADC_CONF ADCSRA|=(1<<ADSC); while ((ADCSRA&(1<<ADSC)));

#define OWST_ADCREF_VCC 0
#define OWST_ADCREF_AREF (1<<REFS0)
#define OWST_ADCREF_INT ((1<<REFS0)|(1<<REFS1))

#define OWST_ADCIN_PA0 0
#define OWST_ADCIN_PA1 1
#define OWST_ADCIN_PA2 2
#define OWST_ADCIN_PA3 3
#define OWST_ADCIN_PA4 4
#define OWST_ADCIN_PA5 5
#define OWST_ADCIN_PA6 6
#define OWST_ADCIN_PA7 7


#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define OWST_ADCIN_REFINT 0b0100001
#endif
#if  defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) ||defined(__AVR_ATmega328__) ||defined(__AVR_ATmega328P__) ||defined(__AVR_ATmega328PB__)
#define OWST_ADCIN_REFINT 0b00001110
#endif
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__)||defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define AD_PORT PORTA
#define AD_DDR DDRA
#endif

#if  defined(__AVR_ATmega48__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) ||defined(__AVR_ATmega328__) ||defined(__AVR_ATmega328P__) ||defined(__AVR_ATmega328PB__)
#define AD_PORT PORTC
#define AD_DDR DDRC
#endif

#define OWST_ADC_CONF16_FUNC \
uint16_t owst_ADC_run() {/*14,5ms*/\
	uint16_t r=0;\
	for(uint8_t i=0;i<64;i++) {	OWST_ADC_CONF 	r+=ADC;	}\
	return r;\
} \
double owst_ADC_runf() {/*16,5ms*/\
	double r=0;\
	for(uint8_t i=0;i<64;i++) {	OWST_ADC_CONF 	r+=ADC;	}\
	r*=0.9993;\
	return r;\
}

#define OWST_ADC_CONF16_OSS_FUNC \
uint16_t owst_ADC_OSS_run() {/*896ms*/\
	uint32_t r=0;\
	for(uint16_t i=0;i<4096;i++) {\
		OWST_ADC_CONF \
		r+=ADC;\
	}\
	return r>>6;\
} \
double owst_ADC_OSS_runf() {/*964ms*/\
	double r=0;\
	for(uint16_t i=0;i<4096;i++) {\
		OWST_ADC_CONF \
		r+=ADC;\
	}\
	r+=5150;\
	r*=0.9993;\
	return r/64.0;\
} 

#endif