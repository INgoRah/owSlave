/*
 * Written by Adafruit Industries.
 * Modified by INgo.Rah@gmx.net for ATTiny
 *
 *  MIT license, all text above must be included in any redistribution
 */
#include "Arduino.h"

#ifdef DHT22_SUPPORT

#define TIMEOUT 0xff
#define LOW 0
#define HIGH 1

#define PINDHT PB0
uint8_t data[5];
uint8_t cycles[80];

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
#define MAXCYCLES 250
uint8_t expectPulse(uint8_t level)
{
	uint16_t count = 0;
	uint8_t portState = level ? _BV(PINDHT) : 0;

	while ((PINB & _BV(PINDHT)) == portState) {
		if (count++ >= MAXCYCLES)
			// Exceeded timeout, fail.
			return TIMEOUT;
		delayMicroseconds(5);
	}

	return count;
}

/** pull-up time (in microseconds) before DHT reading */
#define PULLTIME 55
int dht22_read()
{
	// Reset 40 bits of received data to zero.
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;

	// Send start signal.  See DHT datasheet for full signal diagram:
	//   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

	// Go into high impedence state to let pull-up raise data line level and
	// start the reading process.
	PORTB |= _BV(PINDHT);
	delay(1);

	// First set data line low for a period according to sensor type
	DDRB |= _BV(PINDHT);
	// output low
	PORTB &= ~_BV(PINDHT);
	delayMicroseconds(1200); // data sheet says "at least 1ms"
	// End the start signal by setting data line high for 40 microseconds.
	// pinMode(_pin, INPUT_PULLUP);
	PORTB |= _BV(PINDHT);
	DDRB &= ~_BV(PINDHT);

	// Delay a moment to let sensor pull data line low.
	delayMicroseconds(PULLTIME);

	// Now start reading the data line to get the value from the DHT sensor.

	// Turn off interrupts temporarily because the next sections
	// are timing critical and we don't want any interruptions.
	cli ();

	// First expect a low signal for ~80 microseconds followed by a high signal
	// for ~80 microseconds again.
	if (expectPulse(LOW) == TIMEOUT) {
	  sei();
	  return -1;
	}
	if (expectPulse(HIGH) == TIMEOUT) {
	  sei();
	  return -1;
	}

	// Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
	// microsecond low pulse followed by a variable length high pulse.  If the
	// high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
	// then it's a 1.  We measure the cycle count of the initial 50us low pulse
	// and use that to compare to the cycle count of the high pulse to determine
	// if the bit is a 0 (high state cycle count < low state cycle count), or a
	// 1 (high state cycle count > low state cycle count). Note that for speed
	// all the pulses are read into a array and then examined in a later step.
	for (int i = 0; i < 80; i += 2) {
	  cycles[i] = expectPulse(LOW);
	  cycles[i + 1] = expectPulse(HIGH);
	}
	sei();
	// Timing critical code is now complete.

	// Inspect pulses and determine which ones are 0 (high state cycle count < low
	// state cycle count), or 1 (high state cycle count > low state cycle count).
	for (int i = 0; i < 40; ++i) {
		uint8_t lowCycles = cycles[2 * i];
		uint8_t highCycles = cycles[2 * i + 1];
		if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
			//_lastresult = false;
			return -1;
		}
		data[i / 8] <<= 1;
		// Now compare the low and high cycle times to see if the bit is a 0 or 1.
		if (highCycles > lowCycles) {
			// High cycles are greater than 50us low cycle count, must be a 1.
			data[i / 8] |= 1;
		}
		// Else high cycles are less than (or equal to, a weird case) the 50us low
		// cycle count so this must be a zero.  Nothing needs to be changed in the
		// stored data.
	}
	// Check we read 40 bits and that the checksum matches.
	if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
		return 0;
	else
		return -1;

	return 0;
}

uint8_t dht22_readH()
{
	float f;

	f = ((uint16_t)data[0]) << 8 | data[1];
	f *= 0.1;

	return (uint8_t)(f);
}

int16_t dht22_readT()
{
	float f;

	f = ((uint16_t)(data[2] & 0x7F)) << 8 | data[3];
	f *= 1.6;
	if (data[2] & 0x80)
		f *= -1;

	return (int16_t)(f);
}
#endif