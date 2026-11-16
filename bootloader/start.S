/* Entry of the bootloader at BL_START, the reset vector with BOOTRST
 * programmed. No C runtime (-nostartfiles): r24:r25 have to arrive in
 * bl_main() as the application left them, they carry BL_MAGIC when it
 * asks for an update. bl_main() initializes all it uses itself, there
 * is no .data and .bss is not cleared. */
#include <avr/io.h>

	.section .vectors,"ax",@progbits
	.global __vectors
__vectors:
	cli
	clr	r1
	out	_SFR_IO_ADDR(SREG), r1
	ldi	r28, lo8(RAMEND)
	ldi	r29, hi8(RAMEND)
	out	_SFR_IO_ADDR(SPH), r29
	out	_SFR_IO_ADDR(SPL), r28
	; r24:r25 untouched: the first argument of bl_main()
	rjmp	bl_main
