#ifndef __OW_FLEXDEV
#define __OW_FLEXDEV

#define FD_FAMILY_A2 0xA2
#define FD_FAMILY_A3 0xA3
#define FD_FAMILY_A8 0xA8

/* OWID #5,#6 special function defines
5th: out/input pin 0 - 5 special function, 2 bit per pin
     [0..1] PIN0
     0: unused
     1: Lamp output (supports blinking)
     2: output (no blinking)
     3: res
     [2..3] PIN1
     0: unused
     1: ADC
     2: output
     3: LED status
     [4..5] PIN2
     0: unused
     1: IR sender
     2: ouput
     3: PWM DAC
     [6..7] PIN3
     0: unused
     1: input, Wake-up source with interrupt
     2: Lamp output (supports blinking)
     3: IR input (not yet implemented)
6th: pin 6 -  special function
     [6..7] PIN4
     0: unused
     1: input, Wake-up source
     2: ADC input
     3: input
     [2..3] PIN5
     0: unused
     1: input, Wake-up source with interrupt
     2: ouput
     3: input
     [4..5] PIN6
     0: unused
     1: Button input (debounced) / Wake-up source
     2: ouput
     3: input
     [6..7] PIN7
     0: unused
     1: input, Wake-up source
     2: ouput
     3: input
 */
#define FD_PIN_UNUSED 0
#define FD_PIN_LAMP 1
#define FD_PIN_ADC 1
#define FD_PIN_IRTX 1
/* input, Wake-up source with interrupt */
#define FD_PIN_INT 1
/* Button input (debounced) / Wake-up source */
#define FD_PIN_BTN 1
#define FD_PIN_OUT 2
#define FD_PIN_LEDST 3
#define FD_PIN_IRRX 3
/* input without interrupt */
#define FD_PIN_IN 3


#endif /* __OW_FLEXDEV */