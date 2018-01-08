# OneWire Interface Slave
## Main Features

## Changes

- Supporting input button with debouncing. Fires and interrupt on OW line after releasing the button
- Sleep support if not button is used and no ongoing conversion and silence on the OW line
- CRC generated on the fly no longer needed in owid initially

## TODO

## Technical Documentation

### OneWire ID
1st - Family:
 *    A2 normal switch, PortB1 serves as Input
 *    A3 IR sender on PortB1 plus switch
 *    A4 ADC and PWM output
 *    A8 Multiport input/output, ATMEGA based
2nd: serial id
3rd: version
4th: max output/input pins | [7..4] output| [3..0] input |
5th: out/input pin 1 - 4 special function, 2 bit per pin
     [0..1] PIN1
     0: unused
     1: Lamp output (supports blinking)
     2: output (no blinking)
     3: res
     [2..3] PIN2
     0: unused
     1: ADC
     2: output
     3: LED status
     [4..5] PIN3
     0: unused
     1: Lamp output (supports blinking)
     2: IR sender
     3: PWM DAC
     [0..1] PIN4
     0: unused
     1: input, Wake-up source with interrupt
     2: ouput
     3: IR input (not yet implemented)
6th: pin 5 - 8 special function
     [6..7] PIN5
     0: unused
     1: Button input (debounced) / Wake-up source
     2: ADC input
     3: input
     [2..3] PIN6
     0: unused
     1: input, Wake-up source with interrupt
     2: ouput
     3: input
     [4..5] PIN3
     0: unused
     1: input, Wake-up source
     2: ouput
     3: input
     [6..7] PIN4
     0: unused
     1: input, Wake-up source
     2: ouput
     3: input
7th: optional group or sequence ID

### Commands
0x44: in case ADC is used, start a conversion
0x70: send IR data from scratchpad data 0 .. 3
0x2x: switch output of pin x (0..4) to input
0x3x: switch output of pin x (0..4) to output and low
0x64: enable button polling
0x65: disable button polling
0x68: enable LED blinking
0x48: disable LED blinking
0xBE: Read Scratchpad
0x4E: Write Scratchpad
0x?? Program ID (from scratchpad)
0x?? Program Config (from scratchpad)

### Scratchpad
0: interrupt status
1: output port pins status - all pins output or input
2: input port pins status
3-4: optional ADC value (call conversion, cmd 0x44 first)
5: current function
6: DDRB
7: DDRC

### PINS
0: Lamp PC1/PB1
1: LED PC1/PB1
2: optional PC0/PB0
3: optional PC2/PB2
4: optional PC3/PB3
5: optional PC4/PB4

