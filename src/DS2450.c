#ifdef DS2450_SUPPORT
#include "Arduino.h"
#if defined(WDT_ENABLED)
#include <avr/wdt.h>
#define wdr wdt_reset
#else
#define wdr() do { ; } while(0)
#endif
#include "owSlave_tools.h"
#include "wiring.h"
#include "DS2450.h"

volatile packadc_t packadc;

void adc_loop()
{
	uint8_t idx;

	if (gcontrol & 0x20) {
		gcontrol &= ~0x20;
		idx = packadc.convc1 - 1;
		if (idx > 3)
			return;
		wdt_reset();
		/* got a start conversion command, force update */
		packadc.ch[idx] = analogRead(idx);
	}
}
#endif /* DS2450_SUPPORT */
