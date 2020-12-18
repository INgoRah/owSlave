# owSlave2
Fork from https://git.smho.de/gw/?p=owSlave2.git;a=summary

## Changes

Version 1.3, 2021-01-14
- Split the files for 2408 and 1820
- In case of temperature support, regularly wake up and check alarms thresholds.
  This is using watchdog timer - power optimized.
- Temperature controlled switch (heating control, thresholds configurable
- Temperature change reporting (configurable)
- Read initial state for input switches of types CFG_PASS* and perform auto switch.
    Otherwise a thermostat might be ignored on restart and no heating active where
    it should
- per port pin intialization to reduce power if not connected (pull up not needed)

Version 1.2
- For PWM output (on ATTiny85 PB4 / PIO0 and ATTiny84 PA5) the all output latch bits
  are used for the duty cycle (bightnes), 0 for off
- PWM output level to be applied on change request (not only on switching it on)
- optimize touch sensor handling (active high, no bouncing, just rising edge with validation).
    POL and SW_ID to be implemented

Version 1.1
- Interrupt generation using the ow line low for 900 us
- Timer functions like in Arduino but on polling base within the loop. No wakeup from sleep
- Pin and push button validation
- Watchdog support:
    After 8 secs of invalid state a reset occurs (e.g. a hang in button validation).
    This also applies to pressing a button for 8 secs (force reset)
- Initial OneWire ID is initialized and copyied over to the end of EEPROM on
  first boot for reading next time.
- store new config / custom command or newid command feature

## To Do
[ ] Watchdog fires if switching PIO without polling/resetting alarm
[ ] regression testing signal generation and improvements (bus detection and delay)
[ ] Disarm interrupt on reset pulse and arm again afterwards.
    Needs check wether slave can cope with just a reset without ROM commands.
    Timeout after reset?
[ ] iButton type 2 interrupt (assert reset for long time).
    Needs master improvements detecting it and slave ASM code extending the reset low time

  ## Issues
  [ ] config save does not work
  [ ] once setting PIO,