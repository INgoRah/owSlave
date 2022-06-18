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

#define PIN_PIO0 _BV(PB1)
#define PIN_PIO1 _BV(PB0)
#define PIN_PIO2 _BV(PB2)
#define PIN_PIO3 _BV(PB3)
#define PIN_PIO4 _BV(PB4)
#define PIN_PIO5 _BV(PB5)
#define PIN_PIO6 _BV(PB6)
#define PIN_PIO7 _BV(PB7)
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
#endif /* ATMEGA */

/* All tinies */
#if defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || \
	defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || \
	defined(__AVR_ATtiny84A__) || defined(__AVR_ATtiny85__)
#define PCINT_VECTOR PCINT0_vect

#if defined(BMP280_SUPPORT) && defined(WITH_LEDS)
#error conflict with pins
#endif

#define OW_PIN PINB //1 Wire Pin as number
#define OW_PINN PORTB2
#define OW_DDR DDRB  //pin direction register
#endif

#if defined(__AVR_ATtiny85__)
#define ACTIVE_LOW
#define PORT_REG PORTB
#define PIN_REG PINB
#define PIN_DDR DDRB

#define PIN_PIO0 _BV(PB4) /* light : ouput/PWM */
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
#ifndef MAX_BTN
#define MAX_BTN 4
#endif
#endif

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define TIMSK TIMSK0
/* means: 0 is output low, 1 input high (seen from owfs) */
#define ACTIVE_LOW
#define PIN_REG PINA
#define PORT_REG PORTA
#define PIN_DDR DDRA
#define PCMSK PCMSK0

#define PIN_PIO0 (1<<PINA1)	// predefined output 1
#define PIN_PIO1 (1<<PINA0)	// predefined output 2
#define PIN_PIO2 (1<<PINA2)
#define PIN_PIO3 (1<<PINA3)
#define PIN_PIO4 (1<<PINA4)
#define PIN_PIO5 (1<<PINA5)
#define PIN_PIO6 (1<<PINA6)
#define PIN_PIO7 (1<<PINA7)
#ifdef WITH_LEDS
#define LED _BV(PB1)
#define LED2 _BV(PB0)

#define	LED_ON() do { DDRB |= LED;PORTB &= ~LED; }while(0)
#define	LED_OFF() do {DDRB &= ~LED;PORTB |= LED; }while(0)
#define	LED2_ON() do { DDRB |= LED2;PORTB &= ~LED2; led2=1; }while(0)
#define	LED2_OFF() do {DDRB &= ~LED2;PORTB |= LED2; led2=0; }while(0)
#endif
#ifndef MAX_BTN
#define MAX_BTN 8
#endif
#endif

#define CHAN_VALUES 16

#define SIG_NO 0
#define SIG_ACT 1
#define SIG_ARM 2

#define MIN_VERSION 5
#define MAJ_VERSION 1

/** Auto switch config for each PIO
 *  0xff = default / off
 *  range from 1 (=PIO0) .. 8 (=PIO8) otherwise invalid or deactivated
 * */
#define CFG_SW_ID 3  /* .. 10 */
/*
 * PIO configuration
 */
#define CFG_CFG_ID 11 /* .. 18 */
#define CFG_DEFAULT 0xff /* =0x10 / press button */
#define CFG_UNUSED 0
/* input */

/** Signal is active high, no pull up. Touch input with short high output.
 * Reacts only on rising edge and thus does not support auto switch */
#define CFG_ACT_HIGH 2
/** Signal is active low, no pull up. Input with short low output.
 * Reacts only on falling edge and thus does not support auto switch */
#define CFG_ACT_LOW 3
/** Active low with pull up */
#define CFG_ACT_LOW_PU 4
/** Level is passed as is, alarm on change */
#define CFG_PASS 5
/** Level is passed with inverted logic, alarm on change.
 * Level 0 activates the output, level 1 deactivates it
 */
#define CFG_PASS_INV 6
/** Same as CFG_PASS_INV but enables a pull up */
#define CFG_PASS_INV_PU 7
/** Same as CFG_PASS but enables a pull up */
#define CFG_PASS_PU 8
#define CFG_BTN_MASK 0x10 /** button mask */
/** Press button with validation and long press detection.
 * A 1 represents normal polarity (push button to ground) with
 * pull-up resistor. High is inactive, low pushed */
#define CFG_BTN 0x10

#define CFG_SW 0x11 /** switch button */

/* output */
#define CFG_OUT_MASK 0x20

/** Active low output (default DS2408 behaviour).
 * Represents normal polarity (open collector)
 * Set PIO to 0 = conducting, on, pin active low
 * set 1 / non-conducting (off), input
*/
#define CFG_OUT_LOW 0x21

/** Active high output. Used for output via transistor.
 * Set 0 = on, pin is active high (no input)
 * Set 1 = off, inactive, pin is in high resistive not
 *         driving high, no pullup
 * Logic state represents the transistor output, not pin output
 */
#define CFG_OUT_HIGH 0x22
#define CFG_OUT_PWM 0x23

#define CFG_CFG_FEAT 19
#define FEAT_TEMP 0x01

#define CFG_OFFSET 20

#define CFG_VERS_ID 21
/** Type: 0 - ATiny84 v1
 * 		  10 - ATiny84 v1 with DS1820
 * 		  1 - ATiny84 v2
 * 		  11 - ATiny84 v2 with DS1820
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
		 * Active low output (default DS2408 behaviour).
         * Set PIO to 0 = conducting, on, pin active low
         * set 1 / non-conducting (off) / input
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

uint8_t auto_switch(uint8_t i, uint8_t val);
void latch_out(uint8_t bb);

#endif /* DS2408_H */