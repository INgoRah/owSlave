#ifndef DS2408_H
#define DS2408_H

#ifndef WITH_LEDS
#define	LED_ON() do { ; } while(0)
#define	LED_OFF() do { ; } while(0)
#define	LED2_ON() do { ; } while(0)
#define	LED2_OFF() do { ; } while(0)
#endif

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
#ifdef AVRSIM
#define PIN_REG PORTC
#else
#define PIN_REG PINB
#endif
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

#ifdef WITH_LEDS
#define	LED_ON() do { DDRC |= LED;PORTC &= ~LED; }while(0)
#define	LED_OFF() do {DDRC &= ~LED;PORTC |= LED; }while(0)
#define	LED2_ON() do { DDRC |= LED2;PORTC &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRC &= ~LED2;PORTC |= LED2; led2=0; }while(0)
#endif

#ifndef MAX_BTN
#define MAX_BTN 4
#endif /* ifndef MAX_BTN */
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
#define PIN_PIO1 _BV(PB3)
#define PIN_PIO2 _BV(PB1)
#define PIN_PIO3 _BV(PB0) /* light switch: input */
#define LED _BV(PB3)
#ifdef WITH_LEDS
#define	LED_ON()  do { DDRB |= LED;  PORTB &= ~LED; } while(0)
#define	LED_OFF() do { DDRB &= ~LED; PORTB |= LED;  } while(0)
#define	LED2_ON() do {  }while(0)
#define	LED2_OFF() do { }while(0)
#endif
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
#ifdef WITH_LEDS
#define	LED_ON() do { DDRB |= LED;PORTB &= ~LED; }while(0)
#define	LED_OFF() do {DDRB &= ~LED;PORTB |= LED; }while(0)
#define	LED2_ON() do { DDRB |= LED2;PORTB &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRB &= ~LED2;PORTB |= LED2; led2=0; }while(0)
#endif
#define MAX_BTN 7
#endif

#define CHAN_VALUES 16

#define SIG_NO 0
#define SIG_ACT 1
#define SIG_ARM 2

#define MIN_VERSION 1
#define MAJ_VERSION 1
/** 0 means a push button (based on real port pin mask).
 * A state change (alarm) is signaled on releasing the button. 
 * 1 represents a simple input with bouncing, no press button
 * A state change (alarm) is signaled on low and high. 
 */
#define CFG_BTN_ID 0
/**  A 1 represents an output pin otherwise input
 *   (based on real port pin mask).
 **/
#define CFG_PIN_ID 1
/** Used for output via transistor.
 * A 1 represents normal polarity (open collector). Based on real port
 * pin mask.
 * Set 0 = conducting, on, pin active low 
 * set 1 / non-conducting (off), input
 * A 0 means reversed polarity for transistor output:
 * Set 0 = on, pin is active high (no input)
 * Set 1 = off, inactive, pin is in high resistive not
 *         driving high, no pullup
 * Logic state represents the transistor output, not pin output
 * For input: a 1 represents normal polarity (push button to ground) with
 *   pull-up resistor. High is inactive, low pushed
 *   A 0 means high active, low inactive. No pull-up enabled.
 */
#define CFG_POL_ID 2
/** Auto switch config for each PIO
 *  0xff = default / off
 * */
#define CFG_SW_ID 3  /* .. 10 */
/** PIO configuration
 *  1 = PWM output
 *  2 = Touch input with short high output. Reacts only on rising edge
 *  3 = Touch input with short high output. Reacts only on rising edge
 *  0xff = default
 * */
#define CFG_CFG_ID 11 /* .. 18 */
#define CFG_DEFAULT 0xff
#define CFG_ACT_PWM 1
#define CFG_ACT_HIGH 2 /** signal is active high, no pull up */
#define CFG_ACT_LOW 3

#define CFG_CFG_FEAT 19
#define FEAT_TEMP 0x01

/* 20 */
#define CFG_VERS_ID 21
/** Type: 0 - ATiny84 v1
 * 		  1 - ATiny84 v2
 *  	  2 - ATiny85 v1
 * 		  3 - ATiny84 custom
 * 		  4 - ATMega v1
 */
#define CFG_TYPE_ID 23

#define CFG_PIN_PWM 1
#define CFG_PIN_DEF 0xff

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