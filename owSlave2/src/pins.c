#include <Arduino.h>
#include "pins.h"

void initBtn (uint8_t in, struct pinState *p)
{
	p->last = !!in;
	if (in)
		p->state = BTN_HIGH;
	else
		p->state = BTN_LOW;
}

int checkPin (uint8_t in, struct pinState *p)
{
	if (p->last != in) {
		p->last = in;
		return BTN_UNSTABLE;
	}
	return in;
}

int checkBtn (uint8_t in, struct pinState *p)
{
	if (p->last != in) {
		p->last = in;
		p->state = BTN_UNSTABLE;
		return BTN_UNSTABLE;
	}
	/* from here we are stable */
	switch (p->state) {
	case BTN_TIMER_HIGH:
		if (millis() - p->time < BTN_MINH_TIME)
			return BTN_UNSTABLE;
		p->state = BTN_HIGH;
		in = p->hl_state;
		p->hl_state = 0;
		/* button released or was high, check press time */
		switch (in) {
		case BTN_PRESSED_LONG:
			p->press = millis() - p->press;
			return BTN_RELEASED;
		case BTN_PRESS_LOW:
			p->press = millis() - p->press;
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
		if (in != 0)
			return BTN_HIGH;
		/* fall-through */
	case BTN_UNSTABLE:
		if (in == 0)
			p->state = BTN_TIMER_LOW;
		else
			p->state = BTN_TIMER_HIGH;
		p->time = millis();
		return BTN_UNSTABLE;
	}
	return in;
}
