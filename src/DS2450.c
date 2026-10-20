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
		/* got a start conversion command, force update of all
		   channels in the input select mask: bit 0 = A .. bit 3 = D.
		   The read-out control (convc2) is not needed, the result
		   is there before the master can read */
		for (idx = 0; idx < 4; idx++) {
			if ((packadc.convc1 & (1 << idx)) == 0)
				continue;
		wdt_reset();
			packadc.ch[idx] = analogRead(idx);
	}
	}
}
#endif /* DS2450_SUPPORT */
