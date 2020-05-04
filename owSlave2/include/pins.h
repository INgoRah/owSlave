#ifndef __PINS
#define __PINS

/*
 *  Pin/Button configuration
 */
#define BTN_MIN_TIME 25
#define BTN_MINH_TIME 25
#define BTN_LONG_PRESS_TIME 400

/*
 * Button states
 */
#define	BTN_LOW  0
#define	BTN_HIGH  1
#define	BTN_TIMER_LOW  2
#define	BTN_TIMER_HIGH 3
#define	BTN_UNSTABLE 4
#define BTN_PRESS_LOW 5
#define BTN_PRESSED_LONG 6
#define BTN_PRESSED 7
#define BTN_INVALID 8
#define BTN_RELEASED 9

/*
 * Pin states
 */
#define PIN_LOW 0
#define PIN_HIGH 1

struct pinState {
	unsigned long time;
	unsigned long press;
	uint8_t hl_state;
	uint8_t state;
	uint8_t last;
	/* counter of press for debuggin */
	uint8_t cnt;
};

void initBtn (uint8_t in, struct pinState *p);
int checkBtn (uint8_t in, struct pinState *p);
int checkPin (uint8_t in, struct pinState *p);

#endif /* __PINS */
