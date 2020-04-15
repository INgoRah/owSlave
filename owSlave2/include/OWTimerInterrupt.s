// Copyright (c) 2017, Tobias Mueller tm(at)tm3d.de
// All rights reserved. 
// 
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are 
// met: 
// 
//  * Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the 
//    distribution. 
//  * All advertising materials mentioning features or use of this 
//    software must display the following acknowledgement: This product 
//    includes software developed by tm3d.de and its contributors. 
//  * Neither the name of tm3d.de nor the names of its contributors may 
//    be used to endorse or promote products derived from this software 
//    without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 


.global TIMER_INTERRUPT
TIMER_INTERRUPT:
	sdb
	push r_temp
	in r_temp,_SFR_IO_ADDR(SREG)  
	push r_temp
	push r_temp2
	cdb
	sbic OW_PIN,OW_PINN  ; abkuerzung wenn Leitung schon h
	rjmp tint_end ; Leitung 1 kein Reset
	ldi r_temp,0  
	out TCNT_REG,r_temp
	CLEAR_TOV_FLAG
	ldi r_temp2,3 ;3x overrun for timeout
tint_loop_rend:
	JMP_NO_TOV ;ueberspringe wenn kein ueberlauf
	rjmp tint_handle_timeout
	sbis OW_PIN,OW_PINN ;warten bis leitung wieder h 
	rjmp tint_loop_rend
	;JMP_NO_TOV ;ueberspringe wenn kein ueberlauf
	cpi r_temp2,3
	;rjmp tint_overrun
	brne tint_overrun
	in r_temp,TCNT_REG ;schauen ob es lange genug gedauert hat fuer reset
	cpi r_temp,OWT_RESET2 
	brlo tint_end
tint_overrun:
	ldi r_temp,0
	out TCNT_REG,r_temp
	;zwischen Reset und Presets
tint_loop_res_pres:
	in r_temp,TCNT_REG
	cpi r_temp,OWT_RESET_PRESENT
	brlo tint_loop_res_pres ;Warten zwischen reset und presets
	sbi OW_DDR,OW_PINN  ;presents impuls
	;reset impuls
tint_loop_pres:
	in r_temp,TCNT_REG
	cpi r_temp,OWT_PRESENT
	brlo tint_loop_pres
	cbi OW_DDR,OW_PINN  
	ldi r_temp,OW_READ_ROM_COMMAND ; Initialisieren von Rom command
	sts mode,r_temp
	ldi r_temp,1
	sts reset_indicator,r_temp
	sts bcount,r_temp ;bit eins
	clr r_temp
	sts sendflag,r_temp ;empfangen (rom_command)
	;sts wzero,r_temp ;alles 0 
	RESETZEROMARKER
	CLEAR_INTERRUPT_FLAG
tint_end:
	;CLEAR_INTERRUPT_FLAG
	DIS_TIM_INT
	pop r_temp2
	pop r_temp
	out _SFR_IO_ADDR(SREG),r_temp
	pop r_temp
	reti
;;;

tint_handle_timeout:
	dec r_temp2
	breq tint_end
	CLEAR_TOV_FLAG
	rjmp tint_loop_rend