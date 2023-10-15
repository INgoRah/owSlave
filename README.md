# owSlave2
Fork from https://git.smho.de/gw/?p=owSlave2.git;a=summary, License

I2C master code from an unofficial github repository for Peter Fleury's I2C library
http://homepage.hispeed.ch/peterfleury/avr-software.html#libs now cloned to
https://github.com/alx741/avr_i2c.git
License GPL v3
## Changes

Version 1.5, 2021-11-28
- Temperature and Humidity support with DHT22 on pin PB0 (compile option)
  Humiditiy is reported in the byte #5 instead of FF

Version 1.4, 2021-10-27
- support BMP280 with dual rom with DS1820 on a soft I2C master
- report any latch change even for auto switch
- remove latch output on configured input pins

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
[ ] Generate alarm for latch even when auto switching
[ ] Support for timer switches using upper auto switching bits [4..7]: 1 - 30 secs, 2 - 1 min, 3 - 2 mins .. 15 - 14 min.
[ ] Watchdog fires if switching PIO without polling/resetting alarm
[ ] regression testing signal generation and improvements (bus detection and delay)
[ ] Disarm interrupt on reset pulse and arm again afterwards.
    Needs check wether slave can cope with just a reset without ROM commands.
    Timeout after reset?
[ ] iButton type 2 interrupt (assert reset for long time).
    Needs master improvements detecting it and slave ASM code extending the reset low time
[ ] Dual 2408 with on PCF8473
[ ] Pin overlay for in and out using all 10 pins (reset and 1-wire not available)

  ## Issues
  [ ] long press no longer working with outstanding changes
  [ ] config save does not work sometimes
  [ ] once setting PIO overides heating override ... fixed by pin change corrections?
  [ ] If bad signals on the bus or power outage OW ID could be damaged. Needs protection

  # Documentation

  ## Configuration

  ### EEPROM Layout
  | OW ID                     |           | Auto Switch Configuration     | PIN configuration                      |    |    | Version      | Temperature
  |Typ| ID|Bus|~ID|~BS| 66| 77|   RES     |SW0|SW1|SW2|SW3|SW4|SW5|SW6|SW7|CFG   1    2    3    4    5    6     7  |FEA |R   |MAJ |MIN |TYP |OFF      | FACT
  | 29|   |   |   |   |   |   |   |   |   |   |

  ### Configuration Definition
  SWx  [3..10]
    Auto switch an ouput on (short) press of this pin. Output starts with 1 (1 switches PIO0...).
    Example: SW4 = 0x01 switches PI0 on latch 4
    0x80 disables it,
    0x10 - 0x20 are reserved for future double and tripple press
    0x40 reserved for timer based switching
  CFGx [11..18]
  FEA [19] - Feature
  Reserved [20]
  Version [21..22]
  Type [23]
