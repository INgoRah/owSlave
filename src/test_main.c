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
#include <avr_mcu_section.h>
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
#define UNITY_TEST_ASSERT(expected, actual, line, message) UnityAssertNotEqualNumber((int)(expected), (int)(actual), (message), (uint16_t)(line))

#define TEST_ASSERT_EQUAL(expected, actual)         UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, NULL)
#define TEST_ASSERT_NOT_EQUAL(expected, actual)     UNITY_TEST_ASSERT((expected), (actual), __LINE__, " Expected Not-Equal")

const char PROGMEM UnityStrOk[]                            = "OK";
const char PROGMEM UnityStrPass[]                          = "PASS";
const char PROGMEM UnityStrFail[]                          = "FAIL";
const char PROGMEM UnityStrIgnore[]                        = "IGNORE";

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

void UnityBegin(const char *filename);
int UnityEnd(void);
void UnityAssertNotEqualNumber(const int expected,
                            const int actual,
                            const char* msg,
                            const uint16_t lineNumber);
void UnityAssertEqualNumber(const int expected,
                            const int actual,
                            const char* msg,
                            const uint16_t lineNumber);
void UnityConcludeTest(void);
void UnityDefaultTestRun(UnityTestFunction Func, const char* FuncName, const int FuncLineNum);


#define STAT() \
	printf("%d: Stat %X Act %X Logic %X Out %X DDR %X PORT %X PIN %X\n", \
	       __LINE__, pack.Status, pack.PIO_Activity_Latch_State, \
	       pack.PIO_Logic_State, \
	       pack.PIO_Output_Latch_State, PIN_DDR, PORT_REG, PIN_REG)

const struct avr_mmcu_vcd_trace_t _MMCU_ _mytrace[]  = {
	{ AVR_MCU_VCD_SYMBOL("PORTB"), .what = (void*)&PORTB, },
	{ AVR_MCU_VCD_SYMBOL("PINB"), .what = (void*)&PINB, },
	{ AVR_MCU_VCD_SYMBOL("DDRB"), .what = (void*)&DDRB, },
};

void highlevel_btn(void);
int owGlobalTest();
void switch_test();
void timed_pin();
void thermo_handle();
void thermo_handle_overwrite();
void pinChgTest();
int test_pinset();

/* Helpers */
uint8_t bitCount(uint8_t bitmap);
void btnPress(const uint8_t pinut);
void basic_init();
int pinSignal(const uint8_t pinmsk);
int ll_btn_test();


/* DUT functions */
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
extern volatile uint8_t active;
#ifdef TIMER_SUPPORT
extern uint8_t cfg_custom1[22];
#endif

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

void UnityBegin(const char *filename)
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
    printf("%i Tests, " , Unity.NumberOfTests);
    printf("%i fails\n", Unity.TestFailures);
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
    printf("\n");
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
        printf("PASS\n");
    }
    else
    {
        Unity.TestFailures++;
	printf("FAIL\n");
    }
 
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
}

void UnityDefaultTestRun(UnityTestFunction Func, const char* FuncName, const int FuncLineNum)
{
    Unity.CurrentTestName = FuncName;
    printf("%s ...", FuncName);
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
		STAT();
/*        UnityTestResultsFailBegin(lineNumber);
        UnityPrint(UnityStrExpected);
        UnityPrintNumberByStyle(expected, style);
        UnityPrint(UnityStrWas);
        UnityPrintNumberByStyle(actual, style);
        UnityAddMsgIfSpecified(msg);*/
        UNITY_FAIL_AND_BAIL;
    }
}

void UnityAssertNotEqualNumber(const int expected,
                            const int actual,
                            const char* msg,
                            const uint16_t lineNumber)
{
	RETURN_IF_FAIL_OR_IGNORE;

	if (expected == actual) {
		printf("%i:", lineNumber);
		printf("%s:", Unity.CurrentTestName);
		Unity.CurrentSubTestLineNumber = (uint16_t)lineNumber;
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

// Press button on PIN pinut 0..7
void btnPress(const uint8_t pinut)
{
	btn[pinut].state = BTN_TIMER_HIGH;
	btn[pinut].time = BTN_MINH_TIME + _ms + 1;
	btn[pinut].hl_state = BTN_PRESS_LOW;
	PIN_REG |= pio_map[pinut];
	active |= ACT_BUTTON;
	do {
		loop();
	} while (active & ACT_BUTTON);
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
int ll_btn_test()
{
	int ret = 0;
	static const uint8_t pinut = 2; // pin under test

	config_info[CFG_CFG_ID] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 1] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 2] = CFG_BTN;
	PIN_REG = 0xfF;
	//PORT_REG = 0;
	setup();

	for (ttimer = 0; ttimer < (sizeof(btnState) / sizeof(struct tvector)); ttimer++) {
		struct tvector* p = &btnState[ttimer];

		if (p->signal)
			active |= ACT_BUTTON;
		if (p->btn)
			PIN_REG |= pio_map[pinut];
		else
			PIN_REG &= ~(pio_map[pinut]);
		sigdone = 0;
		loop();
		while (_ms < p->ms && active)
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
			active |= ACT_BUTTON;
			if (p->btn) {
				PIN_REG |= (pinmsk);
			}
			else {
				PIN_REG &= ~(pinmsk);
			}
		}
		sigdone = 0;
		loop();
		while (_ms < p->ms && active)
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
	memset(&config_info[CFG_CFG_ID], CFG_OUT_LOW, 2);
	memset(&config_info[CFG_CFG_ID + 2], CFG_BTN, 6);
	memset (&config_info[CFG_SW], 0xFF, 8);
	pack.Conditional_Search_Channel_Selection_Mask = 0xff;
	pack.PIO_Activity_Latch_State = 0;
	PIN_REG = 0xff;
	PORT_REG = 0;
	PIN_DDR = 0x0;
}

void thermo_handle(void)
{
	// pin masks == latch maks (accidently)
	uint8_t th_in_msk = 0x08; // pin mask for thermostat
	const uint8_t out_msk = 0x04; // pin mask for connected auto switch

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
	// IO0 = 1
	TEST_ASSERT_EQUAL(PORT_REG & 0x1, 0x1);
	// output not active
	TEST_ASSERT_NOT_EQUAL(PIN_DDR & out_msk, out_msk);
	TEST_ASSERT_EQUAL(PORT_REG & out_msk, out_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State, 0xFF);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xff);

	/*
	 * 2) Activate thermostat and auto switch ouput
	 */
	PIN_REG &= ~(th_in_msk); // thermo goes to low, means active
	active |= ACT_BUTTON;
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
	active |= ACT_BUTTON;
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
	active |= ACT_BUTTON;
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
	active |= ACT_BUTTON;
	loop();
	// check logic state changed
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & th_in_msk, th_in_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & th_in_msk, th_in_msk);

	/*
	 * 4) Activate thermostat and auto switch ouput. Back to normal
	 */
	PIN_REG &= ~(th_in_msk); // thermo goes to low, means active
	active |= ACT_BUTTON;
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

void timed_pin(void)
{
	uint16_t i;
	/* test pins 0..7 */
	// B2 = PIO2 = OC1B or B1 = PIO0 = OC1A
	/* pins in sense of PIO numbering */
	const uint8_t dut_pin = 0, dut_pinmsk = 1 << (dut_pin);
	/* actual pin mask, PIO0 = 0x2, PIO1 = 0x1, PIO2 = 0x4 .. */
	const uint8_t pinmsk = 0x2;
	const uint8_t tmr_pin = 6, tmr_msk = 1 << (tmr_pin);

	basic_init();
	/* PIO0 = B1 = 0x2! */
	config_info[CFG_CFG_ID + dut_pin] = CFG_OUT_PWM;
	/* PIO1 = B0 */
	config_info[CFG_CFG_ID + 1] = CFG_UNUSED;
	config_info[CFG_CFG_ID + tmr_pin] = CFG_BTN;
	/* switching pin 1 via timer button 4 */
	config_info[CFG_SW_ID + tmr_pin] = 0x20;
	memset(cfg_custom1, 0xff, sizeof(cfg_custom1));
	cfg_custom1[1] = 0x70; // thr
	cfg_custom1[2] = 0x1; // dim
	cfg_custom1[3] = 0x1; // tim1
	cfg_custom1[4] = 0x1; // tim2
	cfg_custom1[5 + tmr_pin] = TMR_TYPE_TRG_DIM | 1; // SWA6
	goto NON_PWM;
	setup();
	pack.Status = 0;
	// PWM pin (OCR1B) must be low:
	TEST_ASSERT_EQUAL(PORT_REG & pinmsk, 0x0);
	// and output
	TEST_ASSERT_EQUAL(DDRB & pinmsk, pinmsk);
	loop();
	/* initial state */
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State, 0xFF);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xff);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State, 0x0);
	/* test group: trigger timer by command */
	/*
	Case 1: check on and timed off with dim down via button auto toggle
	*/
	btnPress(tmr_pin);
	/* auto switched pin active */
	// if PWM not valid?
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & dut_pinmsk, dut_pinmsk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & tmr_msk, tmr_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
	TEST_ASSERT_EQUAL(pack.Status & 0x20, 0x20);
	pack.PIO_Activity_Latch_State = 0;
	// check pack.Status == 0x20
	/* loop for 1ms * 1000 * 10 secs */
	for (i = 0; i <= 1000 * 10 + 1;i++) {
		TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
		if (pack.Status == 0x40) {
			break;
		}
		loop();
	}
	TEST_ASSERT_EQUAL(i, 9998);
	TEST_ASSERT_EQUAL(alarmflag, 1);
	alarmflag = 0;
	// dimming down
	TEST_ASSERT_EQUAL(pack.Status, 0x40);
	/* loop for dimming down 1ms * 1000 * 1 secs (dim)*/
	for (i = 0; i < 1000 * 1 - 1;i++) {
		if ((pack.PIO_Output_Latch_State & dut_pinmsk) != 0)
			break;
		if ((pack.Status) == 0)
			break;
		loop();
	}
	TEST_ASSERT_EQUAL(i, 254);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & dut_pinmsk, dut_pinmsk);
	TEST_ASSERT_EQUAL(pack.Status, 0x0);
	loop();
	/* timed switched pin inactive */
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xFF);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & dut_pinmsk, dut_pinmsk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & tmr_msk, 0);
	/*
	 * Test on non PWM pin
	 */
NON_PWM:
	config_info[CFG_CFG_ID + dut_pin] = CFG_OUT_LOW;
	cfg_custom1[5 + tmr_pin] = TMR_TYPE_TRG | 1; // SWA6 switches pin 1
	setup();
	btnPress(tmr_pin);
	/* auto switched pin active */
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & dut_pinmsk, dut_pinmsk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & tmr_msk, tmr_msk);
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
	TEST_ASSERT_EQUAL(pack.Status & 0x20, 0x20);
	pack.PIO_Activity_Latch_State = 0;
	/* loop for 1ms * 1000 * 10 secs */
	for (i = 0; i <= 1000 * 10 + 1;i++) {
		if (pack.Status == 0x0) {
			break;
		}
		TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State & dut_pinmsk, 0);
		loop();
	}
	TEST_ASSERT_EQUAL(i, 9998);
	TEST_ASSERT_EQUAL(alarmflag, 1);
	alarmflag = 0;
	/* timed switched pin inactive */
	TEST_ASSERT_EQUAL(pack.PIO_Output_Latch_State, 0xFF);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & dut_pinmsk, dut_pinmsk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & tmr_msk, 0);
	pack.PIO_Activity_Latch_State = 0;
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
	config_info[CFG_CFG_ID + 7] = CFG_ACT_LOW;
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
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & pinmsk, 0);

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
	pack.PIO_Output_Latch_State &= ~pinmsk;
	// activate port
	latch_out(pinut + 1);
	loop();
	pinSignal(pinmsk);
}

void highlevel_btn(void)
{
	static const uint8_t pinut = 6; // pin under test 0..7
	static const uint8_t pinmsk = 1 << (pinut);
	// 1 = PIO0 (1<<PINB1) -> 0x02
	
	basic_init();
	/* normaly low and high on active */
	config_info[CFG_CFG_ID] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + 1] = CFG_OUT_LOW;
	config_info[CFG_CFG_ID + pinut] = CFG_BTN;
	setup();
	/* simple button alarm */
	// is input
	TEST_ASSERT_EQUAL(PIN_DDR & pinmsk, 0);
	// is inactive (high)
	TEST_ASSERT_EQUAL(PORT_REG & pinmsk, pinmsk);
	// latch state is inactive (high)
	TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & pinmsk, pinmsk);
	// logic high
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & pinmsk, pinmsk);
	btnPress(pinut);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
	pack.PIO_Activity_Latch_State = 0;

	//  PIO2 / button switches PIO0
	config_info[CFG_SW_ID + pinut] = 1; // 1 = PIO0 (1<<PINB1)
	STAT();
	btnPress(pinut);
	STAT();
	/* note: PIO0 == B1, PIO1 = B0 */
	// is ouput
	TEST_ASSERT_EQUAL(PIN_DDR & 0x2, 0x2);
	// is active (low)
	TEST_ASSERT_EQUAL(PORT_REG & 0x2, 0);
	// latch state is active
        TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & 1, 0);
        // logic low
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & 1, 0);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & 1, 1);
	// still unclear: should btn also alarm??
	//TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
	pack.PIO_Activity_Latch_State = 0;	

	btnPress(pinut);
	// is input
	TEST_ASSERT_EQUAL(PIN_DDR & 0x2, 0);
	// is inactive (high)
	TEST_ASSERT_EQUAL(PORT_REG & 0x2, 0x2);
	// latch state is inactive (high)
	TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & 1, 1);
	// logic high
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & 1, 1);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & 1, 1);
	pack.PIO_Activity_Latch_State = 0;	
}

void switch_test()
{
	static const uint8_t pinut = 2; // pin under test
	static const uint8_t pinmsk = 1 << pinut;
	
	basic_init();
	/* set up needs it */
	config_info[CFG_CFG_ID] = CFG_BTN;
	config_info[CFG_CFG_ID + 1] = CFG_BTN;
	/* normaly low and high on active */
	config_info[CFG_CFG_ID + pinut] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + pinut + 1] = CFG_OUT_HIGH;
	config_info[CFG_CFG_ID + 7] = CFG_ACT_HIGH;
	//  PIO0 / button switches PIO2
	config_info[CFG_SW_ID] = 3; // 3 = PIO2 (1<<PINB1)
	setup();
	// is output
	TEST_ASSERT_EQUAL(PIN_DDR & pinmsk, pinmsk);
	// is low
	TEST_ASSERT_EQUAL(PORT_REG & pinmsk, 0);
        TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & pinmsk, pinmsk);
        // Button on PIN_PIO0 (1<<PINB0)
        btnPress(0);
	// check for active output PIO2
	TEST_ASSERT_EQUAL(PIN_DDR & pinmsk, pinmsk);
        // first time fail: TEST_ASSERT_EQUAL (PORT_REG & pinmsk, pinmsk);
        // TEST_ASSERT_EQUAL (pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
        TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & pinmsk, 0);
        pack.PIO_Activity_Latch_State = 0;
        btnPress (0);
        TEST_ASSERT_EQUAL (PORT_REG & pinmsk, 0);
        TEST_ASSERT_EQUAL (pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
        pack.PIO_Activity_Latch_State = 0;
        btnPress (0);
        TEST_ASSERT_EQUAL(PIN_DDR & pinmsk, pinmsk);
        TEST_ASSERT_EQUAL(PORT_REG & pinmsk, pinmsk);
        TEST_ASSERT_EQUAL(pack.PIO_Logic_State & pinmsk, 0);
        TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
        TEST_ASSERT_EQUAL((active & ACT_BUTTON), 0);
	TEST_ASSERT_EQUAL(int_signal, SIG_NO); // means it was done
	gcontrol = 2;
	loop();
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
}

int owGlobalTest()
{
	static const uint8_t pinut = 1; // pin under test 0..7
	static const uint8_t pinmsk = 1 << (pinut);

	basic_init();
	setup();
	pack.PIO_Output_Latch_State &= ~pinmsk;
	// activate port
	gcontrol |= 1;
	loop();
	// is active (low) / Note: B0 and B1 changed
	TEST_ASSERT_EQUAL(PORT_REG & 0x1, 0);
	TEST_ASSERT_EQUAL(PIN_DDR & 0x1, 0x1);
	// latch state is active
	TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & pinmsk, 0);
	// logic low
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & pinmsk, 0);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);
	pack.PIO_Activity_Latch_State = 0;

	pack.PIO_Output_Latch_State |= pinmsk;
	// deactivate port
	gcontrol |= 1;
	loop();
	TEST_ASSERT_EQUAL(PORT_REG & 0x1, 0x1);
	TEST_ASSERT_EQUAL(PIN_DDR & 0x1, 0);
	// latch state is inactive
	TEST_ASSERT_EQUAL (pack.PIO_Output_Latch_State & pinmsk, pinmsk);
	// logic high
	TEST_ASSERT_EQUAL(pack.PIO_Logic_State & pinmsk, pinmsk);
	TEST_ASSERT_EQUAL(pack.PIO_Activity_Latch_State & pinmsk, pinmsk);

	pack.PIO_Activity_Latch_State = 0;

	return 0;
}

int main()
{
	int ret = 0;

	memset(config_info, 0xff, sizeof(config_info));
	UNITY_BEGIN();
	RUN_TEST(owGlobalTest);
	goto test_end;
	RUN_TEST(highlevel_btn);
	//goto test_end;
	RUN_TEST(timed_pin);
	RUN_TEST(thermo_handle);
	RUN_TEST(thermo_handle_overwrite);
	RUN_TEST(switch_test);
	RUN_TEST(pinChgTest);
test_end:
	UNITY_END ();
#if 1
	ret = test_pinset();
	printf("test_pinset ");
	if (ret == 0)
		printf("PASS\n");
	else
		printf("FAIL\n");


	ret = ll_btn_test();
#endif

	return ret;
}
#endif
