# owSlave2
Fork from https://git.smho.de/gw/?p=owSlave2.git;a=summary

## Changes
- Interrupt generation using the ow line low for 900 us
- Timer functions like in Arduino but on polling base within the loop. No wakeup from sleep
- Pin and push button validation
- Watchdog support:
    After 8 secs of invalid state a reset occurs (e.g. a hang in button validation).
    This also applies to pressing a button for 8 secs (force reset)
- Initial OneWire ID is initialized and copyied over to the end of EEPROM on
  first boot for reading next time.
- For PWM output (on ATTiny85 PB4 / PIO0 and ATTiny84 PA5) the all output latch bits
  are used for the duty cycle (bightnes), 0 for off
[x] PWM output level to be applied on change request (not only on switching it on)
[x] optimize touch sensor handling (active high, no bouncing, just rising edge with validation).
    POL and SW_ID to be implemented
[x] store new config / custom command or newid command feature

## To Do
[ ] Watchdog fires if switching PIO without polling/resetting alarm
[ ] per port pin intialization to reduce power if not connected (pull up not needed)
[ ] regression testing signal generation and improvements (bus detection and delay)
[ ] Disarm interrupt on reset pulse and arm again afterwards.
    Needs check wether slave can cope with just a reset without ROM commands.
    Timeout after reset?
[ ] iButton type 2 interrupt (assert reset for long time).
    Needs master improvements detecting it and slave ASM code extending the reset low time