// Copyright 2009 Ken Shirriff
// For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
// Edited by INgo to optimize for ATtiny without external clock and without Arduino
//
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
// Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
//
#if (OW_FAMILY == 0xA3)
#include "Arduino.h"

#ifdef F_CPU
#	define SYSCLOCK  F_CPU     // main Arduino clock
#else
#	define SYSCLOCK  8000000  // main internal
#endif

// PWM, Phase Correct, TOP OCRA, 
// no prescaler (CS20=1)
#define TIMER_ENABLE_PWM     (TCCR0A |= _BV(COM0B1))
#define TIMER_DISABLE_PWM    (TCCR0A &= ~(_BV(COM0B1)))
#define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR0A = _BV(WGM00); \
  TCCR0B = _BV(WGM02) | _BV(CS00); \
  OCR0A = pwmval; \
  OCR0B = pwmval / 3; \
})

#define TIMER_PWM_PIN        2  /* ATtiny85 */

#define NEC_BITS          32
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_BIT_MARK     560
#define NEC_ONE_SPACE   1690
#define NEC_ZERO_SPACE   560
#define NEC_RPT_SPACE   2250

//+=============================================================================
// Sends an IR mark for the specified number of microseconds.
// The mark output is modulated at the PWM frequency.
void mark (unsigned int time)
{
	TIMER_ENABLE_PWM; // Enable pin 3 PWM output
	switch (time) {
	case NEC_HDR_MARK:
		_delay_ms (9);
		break;
	case NEC_BIT_MARK:     
		_delay_us (560);
		break;
	default:
		break;
	}
}

//+=============================================================================
// Leave pin off for time (given in microseconds)
// Sends an IR space for the specified number of microseconds.
// A space is no output, so the PWM output is disabled.
// Cope with _delay_xs limitation and optimize using fixed values in delay 
// functions (they need to be know at compile time
void  space (unsigned int time)
{
	TIMER_DISABLE_PWM; // Disable pin 3 PWM output
	switch (time) {
	case NEC_HDR_SPACE:
		_delay_ms (4);
		_delay_us (500);
		break;
	case NEC_ONE_SPACE:
		_delay_ms (1);
		_delay_us (300);
		_delay_us (390);
		break;
	case NEC_ZERO_SPACE:
		_delay_us (560);
		break;
	default:
		break;
	}
}

//+=============================================================================
// Enables IR output.  The khz value controls the modulation frequency in kilohertz.
// The IR output will be on pin 3 (OC2B).
// This routine is designed for 36-40KHz; if you use it for other values, it's up to you
// to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
// TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
// controlling the duty cycle.
// There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
// To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
// A few hours staring at the ATmega documentation and this will all make sense.
// See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.
//
void enableIROut (int khz)
{
	// Disable the Timer2 Interrupt (which is used for receiving IR)
	
	pinMode(TIMER_PWM_PIN, OUTPUT);
	digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low
	// COM2A = 00: disconnect OC2A
	// COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
	// WGM2 = 101: phase-correct PWM with OCRA as top
	// CS2  = 000: no prescaling
	// The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
	TCCR0A = _BV(WGM00);
	TCCR0B = _BV(WGM02) | _BV(CS00);
	/* measured
	  38.9 kHz: OCR0A = 106
	  38.2 kHz: OCR0A = 108 */
	OCR0A = 106;
	// original was 1/3 of OCR0A
	OCR0B = 50;
}

union{ unsigned long data; uint8_t bytes[4]; } convert;

void ir_sendNEC (unsigned long data, int nbits)
{
	unsigned long  mask;
	uint8_t oldTCCRB = TCCR0B;
	uint8_t oldTCCRA = TCCR0A;
	// Set IR carrier frequency
	enableIROut(38);
	// Header
	mark(NEC_HDR_MARK);
	space(NEC_HDR_SPACE);

	// Data
	for (mask = 1UL << (nbits - 1);  mask;  mask >>= 1) {
		if (data & mask) {
			mark(NEC_BIT_MARK);
			space(NEC_ONE_SPACE);
		} else {
			mark(NEC_BIT_MARK);
			space(NEC_ZERO_SPACE);
		}
	}

	// Footer
	mark(NEC_BIT_MARK);
	space(0);  // Always end with the LED off
	
	TCCR0A = oldTCCRA;
	TCCR0B = oldTCCRB;
	digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low
}
#endif
