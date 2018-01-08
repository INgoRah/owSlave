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
<<<<<<< HEAD
     1: IR sender
     2: ouput
=======
     1: Lamp output (supports blinking)
     2: IR sender
>>>>>>> Version Update with more decent commands and ATMEGA support
     3: PWM DAC
     [6..7] PIN3
     0: unused
     1: input, Wake-up source with interrupt
<<<<<<< HEAD
     2: Lamp output (supports blinking)
=======
     2: ouput
>>>>>>> Version Update with more decent commands and ATMEGA support
     3: IR input (not yet implemented)
6th: pin 6 -  special function
     [6..7] PIN4
     0: unused
<<<<<<< HEAD
     1: input, Wake-up source
=======
     1: Button input (debounced) / Wake-up source
>>>>>>> Version Update with more decent commands and ATMEGA support
     2: ADC input
     3: input
     [2..3] PIN5
     0: unused
     1: input, Wake-up source with interrupt
     2: ouput
     3: input
     [4..5] PIN6
     0: unused
<<<<<<< HEAD
     1: Button input (debounced) / Wake-up source
     2: ouput
     3: input
     [6..7] PIN7
=======
     1: input, Wake-up source
     2: ouput
     3: input
     [6..7] PIN4
>>>>>>> Version Update with more decent commands and ATMEGA support
     0: unused
     1: input, Wake-up source
     2: ouput
     3: input
 */
<<<<<<< HEAD
#define FD_PIN_UNUSED 0
#define FD_PIN_LAMP 1
#define FD_PIN_ADC 1
#define FD_PIN_IRTX 1
=======

#define FD_PIN_UNUSED 0
#define FD_PIN_LAMP 1
#define FD_PIN_LEDST 3
#define FD_PIN_ADC 1
#define FD_PIN_OUT 2
#define FD_PIN_IRTX 2
#define FD_PIN_IRRX 3
>>>>>>> Version Update with more decent commands and ATMEGA support
/* input, Wake-up source with interrupt */
#define FD_PIN_INT 1
/* Button input (debounced) / Wake-up source */
#define FD_PIN_BTN 1
<<<<<<< HEAD
#define FD_PIN_OUT 2
#define FD_PIN_LEDST 3
#define FD_PIN_IRRX 3
/* input without interrupt */
#define FD_PIN_IN 3


=======
/* input without interrupt */
#define FD_PIN_IN 3

/*
     1: Button input (debounced) / Wake-up source
     2: ADC input
     3: level input
     4: level input with wake-up
     5: IR input (not yet implemented)
     6: reserved for special input cases
     7: Lamp output (supports blinking)
     8: Lamp output (no blinking)
     9: LED
     10: LED status
     11: Level output (timed if configured in scratchpad)
     11: PWM for DAC output
     12: IR sender
*/     
     
>>>>>>> Version Update with more decent commands and ATMEGA support
#endif /* __OW_FLEXDEV */