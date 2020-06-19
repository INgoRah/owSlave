#include <unity.h>
#include "owSlave_tools.h"
#include "Arduino.h"
#include "pins.h"

extern struct pinState btn[7];
extern void setup();

int main() {
	setup();
    UNITY_BEGIN();
    TEST_ASSERT_EQUAL(btn[0].state, 1);
    UNITY_END(); // stop unit testing

    while(1){}
}
