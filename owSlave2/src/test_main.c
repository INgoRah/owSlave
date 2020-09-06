#if defined(AVRSIM)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifdef WDT_ENABLED
#include <avr/wdt.h>
#endif
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "owSlave_tools.h"
#include "Arduino.h"
#include "pins.h"
#ifdef HAVE_UART
#include "uart.h"
#include "printf.h"
#else
#define printf(...)
#endif

#include "DS2408.h"

extern struct pinState btn[MAX_BTN];
extern void setup();
extern void loop();
extern uint8_t config_info[26];
extern volatile pack_t pack;
extern void pin_set(uint8_t bb);
extern volatile uint8_t int_signal;
extern volatile uint8_t btn_active;

int ttimer;
uint8_t sigdone;
uint8_t g_testId = 0;

struct tvector {
	long ms;
	int btn;
	int signal;
	const char* remark;
	int id;
};

struct tvector btnState[] = {
{ 0, 1, 0,"#0 unstable", 0 },
#if 0
{ 2, 1, 0 ,"", 0 },
{ 3, 0, 1, "", 0 },
{ 8, 0, 0, "", 0 },
{ 10, 1, 1, "", 0 },
{ 14, 1, 0, "", 0 },
{ 36, 1, 0, "", 0 },
{ 40, 1, 0, "", 0 },
{ 46, 1, 0, "", 0 },
{ 0, 1, 0, "#0.1 too short press", 0 },
{ 4, 0, 1, "", 0 },
{ 6, 0, 0, "", 0 },
{ 50, 1, 1, "", 0 },
{ 52, 1, 0, "", 0 },
{ 55, 1, 0, "", 0 },
{ 60, 1, 0, "", 0 },
{ 90, 1, 0, "", 0 },
{ 120, 1, 0, "", 0 },
{ 0, 0, 1, "", 0 },
#endif
{ 5, 0, 1, "", 0 },
{ 10, 0, 0, "", 0 },
{ 80, 0, 0, "#1 pressing low", 1 },
{ 90, 0, 0, "", 0 },
{ 125, 1, 1, "", 0 },
{ 160, 1, 0, "#2 pressed", 2 },
{ 170, 1, 0, "", 0 },
{ 240, 1, 0, "#2.1 stable", 3 },
{ 250, 1, 0, "", 4 },
{ 255, 1, 0, "", 0 },
#if 0
{ 232, 1, 0, "#2 spike", 0 },
{ 233, 0, 1, "", 0 },
{ 234, 1, 0, "", 0 },
{ 235, 0, 1, "", 0 },
{ 236, 1, 0, "", 0 },
{ 237, 1, 0, "", 0 },
{ 238, 1, 0, "", 0 },
{ 360, 1, 0, "", 0 },
{ 362, 1, 0, "", 0 },
{ 380, 0, 1, "#3 long press", 0 },
{ 381, 1, 0, "", 0 },
{ 390, 0, 1, "", 0 },
{ 400, 0, 0, "", 0 },
{ 530, 0, 0, "", 0 },
{ 560, 0, 0, "", 0 },
{ 960, 0, 0, "", 0 },
{ 962, 1, 1, "", 0 },
{ 964, 1, 1, "", 0 },
{ 1070, 1, 1, "", 0 },
{ 1680, 1, 0, "", 0 },
{ 1700, 1, 0, "#4 floating", 0 },
{ 1702, 0, 1, "", 0 },
{ 1704, 1, 1, "", 0 },
{ 1706, 0, 1, "", 0 },
{ 1708, 1, 1, "", 0 },
{ 1710, 0, 1, "", 0 },
{ 1720, 1, 0, "", 0 },
{ 1722, 0, 0, "", 0 },
{ 1732, 1, 0, "", 0 },
{ 1734, 1, 0, "", 0 },
{ 1744, 1, 0, "", 0 },
{ 1840, 1, 0, "", 0 },
{ 1850, 0, 1, "#5 slow floating", 0 },
{ 1851, 0, 0, "", 0 },
{ 1852, 0, 0, "", 0 },
{ 1853, 0, 0, "", 0 },
{ 1854, 0, 0, "", 0 },
{ 1855, 1, 1, "", 0 },
{ 1856, 1, 0, "", 0 },
{ 1857, 1, 0, "", 0 },
{ 1858, 1, 0, "", 0 },
{ 1859, 1, 0, "", 0 },
{ 1860, 0, 1, "", 0 },
{ 1861, 0, 0, "", 0 },
{ 1862, 0, 0, "", 0 },
{ 1863, 0, 0, "", 0 },
{ 1864, 0, 0, "", 0 },
{ 1865, 0, 0, "", 0 },
{ 1866, 1, 1, "", 0 },
{ 1867, 1, 0, "", 0 },
{ 1868, 1, 0, "", 0 },
{ 1869, 1, 0, "", 0 },
{ 1870, 1, 0, "", 0 },
{ 1871, 0, 1, "", 0 },
{ 1872, 0, 0, "", 0 },
{ 1873, 0, 0, "", 0 },
{ 1874, 0, 0, "", 0 },
{ 1875, 0, 0, "", 0 },
{ 1876, 1, 1, "", 0 },
{ 1877, 1, 0, "", 0 },
{ 1878, 1, 0, "", 0 },
{ 1879, 1, 0, "", 0 },
{ 1979, 1, 0, "", 0 },
{ 1990, 1, 0, "", 0 },
{ 1, 1, 0, "", 0 },
{ 2, 1, 0, "", 0 },
{ 3, 1, 0, "", 0 },
{ 5, 0, 1, "#6 twice pressing low", 0 },
{ 6, 0, 0, "", 0 },
{ 7, 0, 0, "", 0 },
{ 110, 0, 0, "", 0 },
{ 112, 0, 0, "", 0 },
{ 114, 1, 1, "#2 pressed", 0 },
{ 115, 1, 0, "", 0 },
{ 116, 1, 0, "", 0 },
{ 117, 1, 0, "", 0 },
{ 220, 1, 0, "#2.1 stable", 0 },
{ 240, 1, 0, "", 0 },
{ 240, 0, 1, "#2.2 2nd press", 0 },
{ 242, 0, 0, "", 0 },
{ 380, 0, 0, "", 0 },
{ 382, 1, 1, "", 0 },
{ 382, 1, 0, "", 0 },
{ 510, 1, 0, "", 0 },
{ 520, 1, 0, "", 0 },
{ 530, 1, 0, "", 0 },
#endif
};

static int testCheck(int id)
{
	switch (id) {
	case 1:
	case 5:
		// btn press -> latch set?
		printf("pressing st=%d\n", btn[0].state);
		if (btn[0].state != BTN_LOW)
			return -1;
		break;
	case 2:
	case 6:
		// btn press -> latch set?
		printf("#2 pressed st=%d\n", btn[0].state);
		if (btn[0].state != BTN_TIMER_HIGH)
			return -1;
		break;
	case 3:
		printf(" st=%d %02X %d %d %02X %02X\n", btn[0].state, pack.PIO_Activity_Latch_State, alarmflag, sigdone, PORT_REG, PIN_DDR);
		// PIO
		if ((pack.PIO_Activity_Latch_State & 0x4) != 0x04)
			return -1;
		if ((alarmflag) != 1)
			return -1;
		if (sigdone == 0)
			return -1;
		if ((PIN_DDR & 0x4) != 0x4)
			return -1;
		if ((PORT_REG & 0x4) != 0)
			return -1;
		break;
	case 7:
		printf(" st=%d %02X %d %d %02X %02X\n", btn[0].state, pack.PIO_Activity_Latch_State, alarmflag, sigdone, PORT_REG, PIN_DDR);
	}

	return 0;
}

extern unsigned long _ms;
int btnTest()
{
	int ret;

	for (ttimer = 0; ttimer < (sizeof(btnState) / sizeof(struct tvector)); ttimer++) {
		struct tvector* p = &btnState[ttimer];

		if (p->signal)
			btn_active = 1;
		if (p->btn)
			PINB |= 1;
		else
			PINB &= ~1;
		sigdone = 0;
		loop();
		while (_ms < p->ms && btn_active)
			loop();
		if (p->id) {
			g_testId++;
			ret = testCheck(g_testId);
			/*if (ret != 0)
				return -1;*/
		}
	}
	return 0;
}

int test_pinset()
{
	// result in setting  output
	pack.PIO_Output_Latch_State &= ~0x2;
	// activate port
	pin_set(2);
	if ((PIN_DDR & 0x2) != 0x2)
		return -1;
	if ((PORT_REG & 0x2) != 0x0)
		return -1;
	pack.PIO_Output_Latch_State = 0xff;
	// pin 2 clear / input
	pin_set(2);
	if (PIN_DDR != 0)
		return -1;
	if ((PORT_REG & 0x02) != 0x02)
		return -1;
}

// Press button on PIN_PIO0 (1<<PINB0)
void btnPress()
{
	btn[0].state = BTN_TIMER_HIGH;
	btn[0].time = BTN_MINH_TIME + _ms + 1;
	btn[0].hl_state = BTN_PRESS_LOW;
	PORTB |= 1;
	btn_active = 1;
	loop();
}

int switchTest()
{
	PORTB = 0;	
	config_info[CFG_SW_ID] = 0x2; // 2 = PIO1 (1<<PINB1)
	// Button on PIN_PIO0 (1<<PINB0)
	btnPress();
	// check for active output PIO1 (1<<PINB1)
	if ((PIN_DDR & 0x2) != 0x2)
		return -1;
	if ((PORT_REG & 0x2) != 0x0)
		return -1;
	if (pack.PIO_Logic_State & 0x2 == 0)
		return -1;
	if (pack.PIO_Activity_Latch_State & 0x2 != 0x2)
		return -1;
	if (btn_active != 0)
		return -1;
	if (int_signal != SIG_NO) // means it was done
		return -1;
	gcontrol = 2;
	loop();
	if (pack.PIO_Activity_Latch_State != 0)
		return -1;
	btnPress();
	// check for non active output PIO1 (1<<PINB1)
	if ((PIN_DDR & 0x2) != 0)
		return -1;
	if (pack.PIO_Logic_State & 0x2 == 2)
		return -1;
	if (pack.PIO_Activity_Latch_State & 0x2 != 0x2)
		return -1;
	if (btn_active != 0)
		return -1;
	gcontrol = 2;
	loop();

	return 0;
}

int main()
{
	PINB = 0xff;
	setup();
	//btnTest();
	/*| btn | pin | pol | sw1 | sw2 | sw3 | sw4 | sw5 | sw6 | sw7 | sw8 | */
	config_info[CFG_BTN_ID] = 0x0;
	config_info[CFG_SW_ID] = 0x2;
	config_info[4] = 0xff;
	//btnTest();
	switchTest();

	return 0;
}
#endif

#if defined(UNIT_TEST)
int main() {
	setup();
    UNITY_BEGIN();
    TEST_ASSERT_EQUAL(btn[0].state, 1);
    UNITY_END(); // stop unit testing

    while(1){}
}
#endif
