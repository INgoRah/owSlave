rem @set PATH=c:\progra~2\Arduino\hardware\tools\avr\bin;%PATH%
@set PATH=C:\Users\rah\.platformio\packages\tool-avrdude;%PATH%
rem D:\tools\bin\;
rem avrdude -C C:\progra~2\Arduino\hardware\tools\avr\etc\avrdude.conf -c stk500v2 -P COM13 -p attiny85 -U eeprom:r:-:h
rem Dump
rem <fam> <2 bit store | 6 bit id> <4bit btn | 4 bit pins> <version> 0x55 0x66 0x77
rem avrdude -C C:\progra~2\Arduino\hardware\tools\avr\etc\avrdude.conf -c stk500v2 -B 20 -i10 -P COM13 -p attiny85 -U eeprom:w:0x29,0x11,0x04,0x01,0x2,0x66,0x77:m
rem avrdude -c stk500v2 -P COM13 -p attiny84 -U eeprom:w:0x29,0x42,0x48,0x02,0x55,0x66,0x77,0:m
avrdude -c stk500v2 -P COM13 -p attiny84 -U eeprom:w:0x29,0x1,0x48,0x03,0x05,0x66,0x77,0x0,0x3:m
avrdude -c stk500v2 -P COM13 -p attiny84 -U eeprom:r:-:h

@pause
