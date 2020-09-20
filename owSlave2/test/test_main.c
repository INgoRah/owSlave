#include <unity.h>
#include "owSlave_tools.h"
#include "Arduino.h"
#include "pins.h"

extern struct pinState btn[2];
extern void setup();

#if defined(UNIT_TEST) || defined(AVRSIM)
int main()
{
	PIN_REG = 0xFF;

	pack.Conditional_Search_Channel_Selection_Mask = 0xff;
	setup();

	PIN_DDR = 0; PORT_REG = 0xff;
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

	pack.PIO_Activity_Latch_State = 0;
	pack.PIO_Output_Latch_State = 0xff;
	alarmflag = 0;
	PIN_DDR = 0; PORT_REG = 0xff;
	btnTest();
	/*| btn | pin | pol | sw1 | sw2 | sw3 | sw4 | sw5 | sw6 | sw7 | sw8 | */
	config_info[3] = 0xff;
	config_info[4] = 0xff;
	btnTest();

	return 0;
}
#endif

int main() {
	setup();
    UNITY_BEGIN();
    TEST_ASSERT_EQUAL(btn[0].state, 1);
    UNITY_END(); // stop unit testing

    while(1){}
}
