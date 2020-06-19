#Todo

## Slave
[-] clean up pins. Btn + output or remove fixed outputs (from config)
[x] auto switch by configuration
[x] configure hardware (buttons vs. normal pin, active low vs high) and store in eeprom
[x] fix initial input pin state
double press
[x] Owfs iDS2408 - with interrupt, with additional low time measurement or long/short/double press
Reduce crc functions
[x] read ID from eeprom

## Master
separate input/output for slew rate control transistor (2n...)
owfs master changes?
- own owfs
- try to fix emulation
- own driver part like ds2482
- combine with addtional alarm handling (multimaster)
Switch matrix as backup
How to handle arlarms in iobroker?
- GPIO for alarm indication (get event in iob?)
- implement message in iob-owfs to poll alarm
- polling I2C IF as watchdog every 500 ms / if not activate switching