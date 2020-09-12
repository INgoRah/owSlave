# owSlave2
Fork from https://git.smho.de/gw/?p=owSlave2.git;a=summary

## Changes
Interrupt generation using the ow line low for 900 us
Timer functions like in Arduino but on polling base within the loop. No wakeup from sleep
Pin and push button validation
Watchdog support:
    After 8 secs of invalid state a reset occurs (e.g. a hang in button validation).
    This also applies to pressing a button for 8 secs (force reset)
Initial OneWire ID is copyied over to the end of EEPROM for reading next time.