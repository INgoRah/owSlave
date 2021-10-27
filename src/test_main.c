#if defined(AVRSIM)
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifdef WDT_ENABLED
#include <avr/wdt.h>
#endif
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <setjmp.h>

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

#define UNITY_BEGIN() UnityBegin(__FILE__)
#define UNITY_END() UnityEnd()

#define TEST_PROTECT() (setjmp(Unity.AbortFrame) == 0)
#define TEST_ABORT() longjmp(Unity.AbortFrame, 1)

#define RUN_TEST(...) UnityDefaultTestRun(RUN_TEST_FIRST(__VA_ARGS__), RUN_TEST_SECOND(__VA_ARGS__))
#define RUN_TEST_FIRST(...) RUN_TEST_FIRST_HELPER(__VA_ARGS__, throwaway)
#define RUN_TEST_FIRST_HELPER(first, ...) (first), #first
#define RUN_TEST_SECOND(...) RUN_TEST_SECOND_HELPER(__VA_ARGS__, __LINE__, throwaway)
#define RUN_TEST_SECOND_HELPER(first, second, ...) (second)

#define UNITY_FAIL_AND_BAIL   { Unity.CurrentTestFailed  = 1; TEST_ABORT(); }
#define RETURN_IF_FAIL_OR_IGNORE if (Unity.CurrentTestFailed || Unity.CurrentTestIgnored) return

#define UNITY_TEST_ASSERT_EQUAL_INT(expected, actual, line, message)  UnityAssertEqualNumber((int)(expected), (int)(actual), (message), (uint16_t)(line))

#define TEST_ASSERT_EQUAL(expected, actual)         UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, NULL)
#define TEST_ASSERT_NOT_EQUAL(expected, actual)     UNITY_TEST_ASSERT(((expected) != (actual)), __LINE__, " Expected Not-Equal")

const char PROGMEM UnityStrOk[]                            = "OK";
const char PROGMEM UnityStrPass[]                          = "PASS";
const char PROGMEM UnityStrFail[]                          = "FAIL";
const char PROGMEM UnityStrIgnore[]                        = "IGNORE";

extern const uint8_t pio_map [];
extern uint8_t gcontrol;
extern unsigned long _ms;
extern struct pinState btn[MAX_BTN];
extern void setup();
extern void loop();
extern uint8_t config_info[26];
extern volatile pack_t pack;
extern void latch_out(uint8_t bb);
extern volatile uint8_t int_signal;
extern volatile uint8_t btn_active;

jmp_buf env;

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

typedef void (*UnityTestFunction)(void);
#define UNITY_COUNTER_TYPE uint16_t

struct UNITY_STORAGE_T
{
    const char* TestFile;
    const char* CurrentTestName;
    uint16_t CurrentTestLineNumber;
	uint16_t CurrentSubTestLineNumber;
    UNITY_COUNTER_TYPE NumberOfTests;
    UNITY_COUNTER_TYPE TestFailures;
    UNITY_COUNTER_TYPE TestIgnores;
    UNITY_COUNTER_TYPE CurrentTestFailed;
    UNITY_COUNTER_TYPE CurrentTestIgnored;
    jmp_buf AbortFrame;
};

struct UNITY_STORAGE_T Unity;

void UnityBegin(const char* filename)
{
    Unity.TestFile = filename;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestLineNumber = 0;
    Unity.NumberOfTests = 0;
    Unity.TestFailures = 0;
    Unity.TestIgnores = 0;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
}

/*-----------------------------------------------*/
int UnityEnd(void)
{
    printf("%i" , Unity.NumberOfTests);
    printf("%i", Unity.TestFailures);
    if (Unity.TestFailures == 0U)
    {
        printf("%s", UnityStrOk);
    }
    else
    {
        printf("%s", UnityStrFail);
#ifdef UNITY_DIFFERENTIATE_FINAL_FAIL
        UNITY_OUTPUT_CHAR('E'); UNITY_OUTPUT_CHAR('D');
#endif
    }
    return (int)(Unity.TestFailures);
}

/*-----------------------------------------------*/
void UnityConcludeTest(void)
{
    if (Unity.CurrentTestIgnored)
    {
        Unity.TestIgnores++;
    }
    else if (!Unity.CurrentTestFailed)
    {
        //UnityTestResultsBegin(Unity.TestFile, Unity.CurrentTestLineNumber);
        printf("%s", UnityStrPass);
    }
    else
    {
        Unity.TestFailures++;
    }

    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
}

void UnityDefaultTestRun(UnityTestFunction Func, const char* FuncName, const int FuncLineNum)
{
    Unity.CurrentTestName = FuncName;
    Unity.CurrentTestLineNumber = (uint16_t)FuncLineNum;
    Unity.NumberOfTests++;
    if (TEST_PROTECT())
    {
        Func();
    }
    if (TEST_PROTECT())
    {
    }
    UnityConcludeTest();
}

void UnityAssertEqualNumber(const int expected,
                            const int actual,
                            const char* msg,
                            const uint16_t lineNumber)
{
    RETURN_IF_FAIL_OR_IGNORE;

    if (expected != actual)
    {
		printf("%i:", lineNumber);
		printf("%s:", Unity.CurrentTestName);
	    Unity.CurrentSubTestLineNumber = (uint16_t)lineNumber;
/*        UnityTestResultsFailBegin(lineNumber);
        UnityPrint(UnityStrExpected);
        UnityPrintNumberByStyle(expected, style);
        UnityPrint(UnityStrWas);
        UnityPrintNumberByStyle(actual, style);
        UnityAddMsgIfSpecified(msg);*/
        UNITY_FAIL_AND_BAIL;
    }
}

#if 0
uint8_t bitnumber(uint8_t bitmap)
{
	int i, res = 1;

	for (i = 0x1; i <= 0x80; i = i << 1) {
		if (bitmap & i)
			return res;
		res++;
	}
	/* invalid */
	return 0xff;
}

uint8_t mask2Pio(uint8_t bb)
{
	uint8_t i, mask = 1;

	for (i = 0; i < sizeof(pio_map) / sizeof(pio_map[0]); i++) {
		if (bb == mask) {
			return pio_map[i];
		}
		mask = mask << 1;
	}

	return 0;
}
#endif

uint8_t bitCount(uint8_t bitmap)
{
	int i, res = 0;

	for (i = 0x1; i <= 0x80; i = i << 1) {
		if (bitmap & i)
			res++;
	}
	/* none */
	return res;
}

// Press button on PIN pinut
void btnPress(const uint8_t pinut)
{
	btn[pinut].state = BTN_TIMER_HIGH;
	btn[pinut].time = BTN_MINH_TIME + _ms + 1;
	btn[pinut].hl_state = BTN_PRESS_LOW;
	PIN_REG |= pio_map[pinut];
	btn_active = 1;
	loop();
}

static int testCheck(int id, const uint8_t pinut)
{
	switch (id) {
	case 1:
	case 5:
		// btn press -> latch set?
		printf("pressing st=%d\n", btn[pinut].state);
		if (btn[pinut].state != BTN_LOW)
			return -1;
		break;
	case 2:
	case 6:
		// btn press -> latch set?
		printf("#2 pressed st=%d\n", btn[pinut].state);
		if (btn[pinut].state != BTN_TIMER_HIGH)
			return -1;
		break;
	case 3:
		printf(" st=%d %02X %d %d %02X %02X\n", btn[pinut].state, pack.PIO_Activity_Latch_State, alarmflag, sigdone, PORT_REG, PIN_DDR);
		// PIO
		/* not really correct, but by define the same */
		if ((pack.PIO_Activity_Latch_State & (2 ^ pinut)) != (2 ^ pinut))
			return -1;
		if ((alarmflag) != 1)
			return -1;
		if (sigdone == 0)
			return -1;
		if ((PIN_DDR & (2 ^ pinut)) != (2 ^ pinut))
			return -1;
		if ((PORT_REG & (2 ^ pinut)) != 0)
			return -1;
		break;
	case 7:
		printf(" st=%d %02X %d %d %02X %02X\n", btn[pinut].state, pack.PIO_Activity_Latch_State, alarmflag, sigdone, PORT_REG, PIN_DDR);
	}

	return 0;
}

extern unsigned long _ms;
int btnTest()
{
	int ret = 0;
	static const uint8_t pinut = 2; // pin under test

	config_info[CFG_CFG_ID] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 1] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 2] = CFG_BTN;
	PIN_REG = 0xfF;
	PORT_REG = 0;
	setup();

	for (ttimer = 0; ttimer < (sizeof(btnState) / sizeof(struct tvector)); ttimer++) {
		struct tvector* p = &btnState[ttimer];

		if (p->signal)
			btn_active = 1;
		if (p->btn)
			PIN_REG |= pio_map[pinut];
		else
			PIN_REG &= ~(pio_map[pinut]);
		sigdone = 0;
		loop();
		while (_ms < p->ms && btn_active)
			loop();
		if (p->id) {
			g_testId++;
			ret = testCheck(g_testId, pinut);
			if (ret != 0)
				return -1;
		}
	}
	return ret;
}

/* tests setting pins in various conditions */
int test_pinset()
{
	/* pin - 1 under test which is set
	 * 0 = PIO0 = PINA1
	 * 1 = PIO1 = PINA0 ... */
	static const uint8_t pinut_msk = 1;
	static const uint8_t piout_msk = 2;

	// PIO0 reversed pol
	config_info[CFG_CFG_ID] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + 1] = CFG_OUT_LOW;  // our test pin!
	config_info[CFG_CFG_ID + 2] = CFG_BTN;
	config_info[CFG_CFG_ID + 3] = CFG_PASS_INV_PU;
	PIN_REG = 0xfF;
	PORT_REG = 0;
	/*
	 *  checking setup
	 */
	setup();
	/* could be set to output, but should be still inactive */
	if ((PIN_DDR & pinut_msk) != 0)
		return -1;
	/* PIO1 must be set to high */
	if ((PORT_REG & pinut_msk) != 1)
		return -1;
	if ((pack.PIO_Logic_State & piout_msk) != piout_msk)
		// must be inactive, means must be high
		return -1;
	/* active high pin must be active and show low = inactive */
	if ((PIN_DDR & 2) != 2)
		return -1;
	/* PIO0 must be set to low */
	if ((PORT_REG & 2) != 0)
		return -1;

	/*
	 *  checking setting pin (to active low)
	 */
	// result in setting  output
	pack.PIO_Output_Latch_State &= ~(piout_msk);
	// activate port
	latch_out(piout_msk);
	/* active low pin turns active = low */
	if ((PORT_REG & pinut_msk) != 0)
		return -1;
	if ((pack.PIO_Logic_State & piout_msk) != 0)
		// must be active
		return -1;

	/*
	 *  checking clearing pin (to pull up high)
	 */
	pack.PIO_Output_Latch_State |= piout_msk;
	// activate port
	latch_out(piout_msk);
	if ((PORT_REG & pinut_msk) != pinut_msk)
		// port must be high
		return -1;
	if ((PIN_DDR & pinut_msk) != 0)
		/* must be inactive */
		return -1;
	if ((pack.PIO_Logic_State & piout_msk) != piout_msk)
		// must be inactive
		return -1;

	pack.PIO_Output_Latch_State = 0xff;

	return 0;
}

int pinSignal(const uint8_t pinmsk)
{
	struct tvector pinState[] = {
		/* ms, pin, signal, remark, id */
		{ 0, 0, 0,"#0 unstable", 0 },
		{ 2, 0, 0 ,"", 0 },
		{ 5, 1, 1, "#1 pin active", 1 },
		{ 20, 0, 1, "#2 inactive", 2 },
		{ 40, 0, 0, "", 3 },
	};

	for (ttimer = 0; ttimer < (sizeof(pinState) / sizeof(struct tvector)); ttimer++) {
		struct tvector* p = &pinState[ttimer];

		if (p->signal) {
			btn_active = 1;
			if (p->btn) {
				PIN_REG |= (pinmsk);
			}
			else {
				PIN_REG &= ~(pinmsk);
			}
		}
		sigdone = 0;
		loop();
		while (_ms < p->ms && btn_active)
			loop();
		if (p->id) {
			g_testId++;
			switch (g_testId) {
				case 1:
					/* signaled check for correct states */
					if ((pack.PIO_Logic_State & (pinmsk)) != (pinmsk))
						return -1;
					if ((pack.PIO_Activity_Latch_State & (pinmsk)) != (pinmsk))
						return -1;
					if ((alarmflag) != 1)
						return -1;
					if (bitCount(pack.PIO_Activity_Latch_State) > 1)
						return -1;
					pack.PIO_Activity_Latch_State &= ~(pinmsk);
					alarmflag = 0;
					break;
				case 2:
					if ((pack.PIO_Logic_State & (pinmsk)) != 0)
						return -1;
					if ((pack.PIO_Activity_Latch_State & (pinmsk)) == (pinmsk))
						return -1;
					if ((alarmflag) != 0)
						return -1;
				break;
			}
			//ret = testCheck(g_testId);
			/*if (ret != 0)
				return -1;*/
		}
	}

	return 0;
}

/* set all pins to input button, no auto switch and
 * init the port - called before setup()
 */
void basic_init()
{
	memset (&config_info[CFG_CFG_ID], CFG_BTN, 8);
	memset (&config_info[CFG_SW], 0xFF, 8);

	PIN_REG = 0xff;
	PORT_REG = 0xff;
	PIN_DDR = 0x0;
}

void thermo_handle(void)
{
	// pin masks == latch maks (accidently)
	uint8_t th_in_msk = 0x08; // pin mask for thermostat
	uint8_t out_msk = 0x04; // pin mask for connected auto switch

	basic_init();
	config_info[CFG_CFG_ID] = CFG_OUT_LOW; // sw 0
	config_info[CFG_CFG_ID + 2] = CFG_OUT_LOW; // out: sw 2 -> mask b0100
	config_info[CFG_CFG_ID + 3] = CFG_PASS_INV_PU; // in: sw 3 -> mask b1000
	// switch PIO3 (= 0x4) automatically
	config_info[CFG_SW_ID + 3] = 3; // sw 3 to switch out 3 (= starting from 1)
	// our thermostat is not active -> high
	// PIN_REG = 0xff & (~(th_in_msk));
	setup();
	/*
	 * 1) Verify initial settings
	 */
	TEST_ASSERT_EQUAL(PORT_REG & 0x1, 0x1);
	// output not active
	TEST_ASSERT_EQUAL(PORT_REG & out_msk, out_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State, 0xff);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xff);

	/*
	 * 2) Activate thermostat and auto switch ouput
	 */
	PIN_REG &= ~(th_in_msk); // thermo goes to low, means active
	btn_active = 1;
	loop();
	// must be low:
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & th_in_msk, 0);
	TEST_ASSERT_EQUAL(PORT_REG & out_msk, 0x0); // check auto switch
	// and output
	TEST_ASSERT_EQUAL(PIN_DDR & out_msk, out_msk);
	// state should signal low for input and output
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & 0xc, 0);
	// only in is high, output is set (= 0)
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & 0xc, th_in_msk);
	// both pin should signal change
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & 0xc, 0xc);
	/*
	 * 3) Deactivate thermostat
	 */
	PIN_REG |= th_in_msk;
	btn_active = 1;
	loop();

	/*
	 * 4) Activate thermostat and force output to inactive
	 */
	// and output
	//TEST_ASSERT_EQUAL(PIN_DDR & out_msk, out_msk);
}

void thermo_handle_overwrite(void)
{
	// pin masks == latch maks (accidently)
	uint8_t th_in_msk = 0x08;
	uint8_t out_msk = 0x04;

	basic_init();
	config_info[CFG_CFG_ID] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 2] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 3] = CFG_PASS_INV_PU;
	// switch PIO3 (= 0x4) automatically
	config_info[CFG_SW_ID + 3] = 3;
	// our thermostat is not active -> high
	// PIN_REG = 0xff & (~(th_in_msk));
	setup();

	TEST_ASSERT_EQUAL(pack.PIO_Logic_State, 0xff);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xff);
	/*
	 * 1) Activate thermostat
	 */
	// this was verified in previous test
	PIN_REG &= ~(th_in_msk); // thermo goes to low, means active
	btn_active = 1;
	loop();
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & out_msk, 0);
	// state should signal low for input and output
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & (out_msk | th_in_msk), 0);
	pack.PIO_Activity_Latch_State = 0;
	/*
	 * 2) Deactivate output (forced from host)
	 */
	pack.PIO_Output_Latch_State |= (out_msk);
	gcontrol |= 1;
	loop();
	// check logic state changed
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & out_msk, out_msk); // again 1 = inactive
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & th_in_msk, 0); // still low
	pack.PIO_Activity_Latch_State = 0;
	/*
	 * 3) Deactivate thermostat - no change except notification
	 */
	PIN_REG |= th_in_msk;
	btn_active = 1;
	loop();
	// check logic state changed
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & th_in_msk, th_in_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & th_in_msk, th_in_msk);

	/*
	 * 4) Activate thermostat and auto switch ouput. Back to normal
	 */
	PIN_REG &= ~(th_in_msk); // thermo goes to low, means active
	btn_active = 1;
	loop();
	// must be low:
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & th_in_msk, 0);
	TEST_ASSERT_EQUAL(PORT_REG & out_msk, 0x0); // check auto switch
	// and output
	TEST_ASSERT_EQUAL(PIN_DDR & out_msk, out_msk);
	// state should signal low for input and output
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & 0xc, 0);
	// only in is high, output is set (= 0)
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & 0xc, th_in_msk);
	// both pin should signal change
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & 0xc, 0xc);
}

/* Tests setting an output pin via PIO_Logic_State (from 1-wire)
*/
void pinChgTest(void)
{
	static const uint8_t pinut = 7; // pin under test
	static const uint8_t pinmsk = 0x80; // pin mask under test

	/* set up needs it */
	config_info[CFG_CFG_ID] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + 1] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 2] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 3] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 4] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 5] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 6] = CFG_ACT_HIGH;
	config_info[CFG_CFG_ID + 7] = CFG_ACT_HIGH;

	config_info[CFG_CFG_ID + pinut] = CFG_ACT_HIGH;
	/* assume the pin is inactive = low because inverted pol */
	PIN_REG = 0xff & (~(pinmsk));
	PORT_REG = 0x0;
	setup();
	/*
	 * 1) Verify initial settings
	 */
	// must be low:
	TEST_ASSERT_EQUAL(PORT_REG & pinmsk, 0x0);
	// and output
	TEST_ASSERT_EQUAL(DDRB & pinmsk, 0x0);
	// state should signal "not set"
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & 0x1, 0x01);

	/*
	 * mock changing pin...
	 */
	//  TODO: why signaling now?
	pack.PIO_Logic_State &= ~(pinmsk);
	pinSignal(pinmsk);
	// should check?
	/*
	 * setting output via latch
	 */
	pack.PIO_Output_Latch_State &= ~0x2;
	// activate port
	latch_out(2);
	loop();
	pinSignal(pinmsk);
}

int switchTest()
{
	static const uint8_t pinut = 2; // pin under test

	/* set up needs it */
	config_info[CFG_CFG_ID] = CFG_BTN;
	config_info[CFG_CFG_ID + 1] = CFG_BTN;
	config_info[CFG_CFG_ID + 2] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + 3] = CFG_BTN;
	config_info[CFG_CFG_ID + 4] = CFG_BTN;
	config_info[CFG_CFG_ID + 5] = CFG_BTN;
	config_info[CFG_CFG_ID + 6] = CFG_BTN;
	config_info[CFG_CFG_ID + 7] = CFG_ACT_HIGH;
	//  PIO0 / button switches PIO2
	config_info[CFG_SW_ID] = 0x2; // 2 = PIO1 (1<<PINB1)
	setup();

	PIN_REG = 0xfF;
	PORT_REG = 0;
	// Button on PIN_PIO0 (1<<PINB0)
	btnPress(pinut);
	// check for active output PIO1 (1<<PINB1)
	if ((PIN_DDR & (2 ^ pinut)) != (2 ^ pinut))
		return -1;
	if ((PORT_REG & (2 ^ pinut)) != (2 ^ pinut))
		return -1;
	if ((pack.PIO_Logic_State & (2 ^ pinut)) != 0)
		return -1;
	if ((pack.PIO_Activity_Latch_State & (2 ^ pinut)) != (2 ^ pinut))
		return -1;
	if (btn_active != 0)
		return -1;
	if (int_signal != SIG_NO) // means it was done
		return -1;
	gcontrol = 2;
	loop();
	if (pack.PIO_Activity_Latch_State != 0)
		return -1;
	btnPress(pinut);
	// check for non active output PIO1 (1<<PINB1)
	if ((PIN_DDR & (2 ^ pinut)) != 0)
		return -1;
	if ((pack.PIO_Logic_State & (2 ^ pinut)) == 2)
		return -1;
	if ((pack.PIO_Activity_Latch_State & (2 ^ pinut)) != (2 ^ pinut))
		return -1;
	if (btn_active != 0)
		return -1;
	gcontrol = 2;
	loop();

	return 0;
}

int owGlobalTest()
{
	/* set up needs it */
	config_info[CFG_CFG_ID] = CFG_BTN;
	config_info[CFG_CFG_ID + 1] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 2] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + 3] = CFG_BTN;

	/* assume the pin is inactive = low because inverted pol */
	PIN_REG = 0xff;
	PORT_REG = 0x0;
	setup();
	//pack.PIO_Logic_State &= ~(1);
	pack.PIO_Output_Latch_State &= ~0x2;
	// activate port
	gcontrol |= 1;
	loop();
	gcontrol |= 2;
	loop();
	pack.PIO_Activity_Latch_State = 0;

	pack.PIO_Output_Latch_State |= 0x2;
	// deactivate port
	gcontrol |= 1;
	gcontrol |= 2;
	loop();
	pack.PIO_Activity_Latch_State = 0;

	return 0;
}

int main()
{
	int ret = 0;

    UNITY_BEGIN();
	memset (config_info, 0xff, sizeof(config_info));
	RUN_TEST (thermo_handle);
	RUN_TEST (thermo_handle_overwrite);
	RUN_TEST (pinChgTest);

    UNITY_END();
	ret = switchTest();
	if (ret != 0)
		printf ("switchTest issue");
	ret = owGlobalTest();
	if (ret != 0)
		printf ("owGlobalTest issue");
	ret = test_pinset();
	if (ret != 0)
		printf ("test_pinset issue");

	if (ret != 0)
		printf ("issue");

	return ret;
}
#endif
