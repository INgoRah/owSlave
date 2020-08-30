#ifndef DS2408_H
#define DS2408_H

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

#define PORT_REG PORTB
#define PIN_REG PINB
#define PIN_DDR DDRB

#define PIN_PIO0 (1<<PINB0)
#define PIN_PIO1 (1<<PINB1)
#define PIN_PIO2 (1<<PINB2)
#define PIN_PIO3 (1<<PINB3)
#define PIN_PIO4 _BV(PB4)
#define PIN_PIO5 _BV(PB5)
#define PIN_PIO6 _BV(PB6)
#define LED2 _BV(PC3)
#define LED _BV(PC1)

#define	LED_ON() do { DDRC |= LED;PORTC &= ~LED; }while(0)
#define	LED_OFF() do {DDRC &= ~LED;PORTC |= LED; }while(0)
#define	LED2_ON() do { DDRC |= LED2;PORTC &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRC &= ~LED2;PORTC |= LED2; led2=0; }while(0)

#define MAX_BTN 4
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

#define VERSION 1
#define CFG_BTN_ID 0
#define CFG_PIN_ID 1
#define CFG_POL_ID 2
#define CFG_SW_ID 3
#define CFG_VERS_ID 11

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

#endif /* DS2408_H */