# Slave
[-] clean up pins. Btn + output or remove fixed outputs (from config)
[x] auto switch by configuration
[x] configure hardware (buttons vs. normal pin, active low vs high) and store in eeprom
[x] fix initial input pin state
double press
[x] Owfs iDS2408 - with interrupt, with additional low time measurement or long/short/double press
Reduce crc functions
[x] read ID from eeprom
[ ] store new config / custom command or newid command feature
[?] test mode
[ ] regression testing signal generation and improvements (bus detection and delay)
[x] watchdog for long button press (or no end state after 2 secs), mode != 0 and signal not acknowledged.

## Bugs
[x] Initial boot up not showing in search

# Master

## Arduino

### Bugs
[ ] race condition on alarms?

### Features
[x] Switch matrix as backup
[ ] Timer for PIR
[ ] Long press detection (scenes?)
[ ] Host interface for switching and alarm reporting
[-] separate input/output for slew rate control transistor (2n...)

## OwCtrl (RasPI)
owfs master changes?
- own owfs
- try to fix emulation
- own driver part like ds2482
- combine with addtional alarm handling (multimaster)

## IOBroker
How to handle arlarms in iobroker?
- GPIO for alarm indication (get event in iob?)
- implement message in iob-owfs to poll alarm
- polling I2C IF as watchdog every 500 ms / if not activate switching
