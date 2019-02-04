#include <Arduino.h>
#include "pins.h"

#ifdef BTN_SUPPORT
int checkBtn (uint8_t pin, struct pinState *p)
{
	uint8_t in;
	
	in = digitalRead(pin);
	if (p->last != in) {
		p->last = !!in;
		p->state = BTN_UNSTABLE;
		return BTN_UNSTABLE;
	}
	/* from here we are stable */
	switch (p->state) {
	case BTN_TIMER_HIGH:
		if (millis() - p->time < 25)
			return BTN_UNSTABLE;
		p->state = BTN_HIGH;
		in = p->hl_state;
		p->hl_state = 0;
		/* button released or was high, check press time */
		switch (in) {
		case BTN_PRESSED_LONG:
			return BTN_RELEASED;
		case BTN_PRESS_LOW:
			return BTN_PRESSED;
		default:
			return BTN_INVALID;
		}
	case BTN_TIMER_LOW:
		if (millis() - p->time < BTN_MIN_TIME)
			return BTN_UNSTABLE;
		/* stable state */
		/* 0 or 1 = LOW or HIGH */
		p->state = BTN_LOW;
		p->press = millis();
		p->hl_state = BTN_PRESS_LOW;
		return BTN_PRESS_LOW;
	case BTN_LOW:
		if (millis() - p->press > BTN_LONG_PRESS_TIME &&
			p->hl_state != BTN_PRESSED_LONG) {
			p->hl_state = BTN_PRESSED_LONG;
			return BTN_PRESSED_LONG;
		}
		return BTN_LOW;
	case BTN_HIGH:
		return BTN_HIGH;
	case BTN_UNSTABLE:
		if (in == LOW)
			p->state = BTN_TIMER_LOW;
		else
			p->state = BTN_TIMER_HIGH;
		p->time = millis();
		return BTN_UNSTABLE;
	}
	return in;
}
#endif

#if 0 //(OW_FAMILY == 0xA8)
int checkPulse(uint8_t pin, struct pinState *p)
{
	uint8_t in;

	in = digitalRead(pin);
	switch (p->state) {
	case PIN_LOW:
		if (in == LOW)
			return PIN_LOW;
		else {
			p->state = PIN_CHECK_HIGH;
			p->time = millis();
		}
		break;
	case PIN_HIGH:
		if (in == LOW) {
			p->state = PIN_CHECK_HIGH;
			p->time = millis();
			p->press = 0;
		} else 
			return PIN_HIGH;
		break;
	case PIN_CHECK_HIGH:
		if (in == HIGH) {
			if (millis() - p->time > 20) {
				p->state = PIN_HIGH;
				/* we had a sufficient low */
				return PIN_LOW;
			}
			p->press++;
			p->state = PIN_CHECK_LOW;
			p->time = millis();
		}
		else if (millis() - p->time > 50) {
			p->state = PIN_CHECK_LOW;
			return PIN_LOW;
		}
		break;
	case PIN_CHECK_LOW:
		if (in == LOW) {
			if (millis() - p->time > 20)
				return PIN_LOW;
			p->press++;
			p->state = PIN_CHECK_HIGH;
			p->time = millis();
		} else if (millis() - p->time > 50)
			p->state = PIN_HIGH;
		break;
	}

	if (p->press > 4)
		return PIN_FLOAT;

	return PIN_UNSTABLE;
}
#endif
