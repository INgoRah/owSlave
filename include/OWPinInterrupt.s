// Copyright (c) 2015, Tobias Mueller tm(at)tm3d.de
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

	
.global PIN_INTERRUPT  
PIN_INTERRUPT:
	;leitung auf Low ziehen
	TESTZEROMARKER ; ueberspringe wenn zeromarker=0   ;sbic und cbi/sbi andern SREG nicht
	sbi OW_DDR,OW_PINN
	push r_temp2                //; wichtig: Benutzte Register und das
    in r_temp2,_SFR_IO_ADDR(SREG)     //; Status-Register (SREG) sichern!
	;Zeromarker loeschen
	RESETZEROMARKER 
	;Weitere Register sichern
    push r_temp2 
	push r_temp
	sdb
	; Wegen Power Down Mode Widerherstellen der fallenden Flanke und disable von POWER DONW 
	SET_FALLING_RESET_SLEEP
	push r_bcount
	push r_rwbyte
	push r_sendflag
	;schauen ob noch ein Bit in der Pipeline 
	lds r_bcount,bcount
	lds r_rwbyte,rwbyte
	lds r_sendflag,sendflag
int_internal_start:
	;Timer zuruecksetzen
	ldi r_temp,~OWT_MIN_RESET
	out TCNT_REG,r_temp
	EN_TIM_INT
	tst r_sendflag
	breq receive_bit ; sendflag=0 Slave empfaengt

send_bit: ; bit senden
	;nachstes bit vorbereiten
	tst r_bcount
	brne send_bit_no_handle ;noch bits da 
	rcall handle_byte ; neues Byte muss bearbeitet werden
	tst r_sendflag  
	breq send_bit_low_loop ; Nach dem Gelesen byte koennte gesendet werden muessen....
send_bit_no_handle:  ;noch bits da
	ldi r_temp2,0   ; fuer die CRC berechnung in CRCS
	ror r_rwbyte
	brcs send_bit_no_low
	SETZEROMARKER
	ldi r_temp2,1 ;WICHTIG fuer CRC berechnung
send_bit_no_low: ;ueberspringen von r_wzero=1 wenn leitung nicht auf 0 gezogen wird
	CRCS
	;naechstes byte ....
	lsl r_bcount
	sbis OW_DDR,OW_PINN
	rjmp iend ;abkuerzung wenn leitung nicht low
send_bit_low_loop:
	in r_temp,TCNT_REG
	cpi r_temp,(~OWT_MIN_RESET)+OWT_WRITE ;aller zwei us zaehlt der timer
	brlo send_bit_low_loop ;wenn kleiner 
	cbi OW_DDR,OW_PINN  ;Leitung auf hochohmig 

	rjmp iend

receive_bit: ;or reset 
	;beim lesen zuerst zeit bis zum lesen abwarten 
	;(9us nach den 6 us vom low impuls) 
	in r_temp,TCNT_REG
	cpi r_temp,(~OWT_MIN_RESET)+OWT_READ ;aller zwei us zaehlt der timer
	brlo receive_bit ;wenn kleiner 
	lsr r_rwbyte 
	ldi r_temp2,1  ;fuer CRC Berechnung
	sbis OW_PIN,OW_PINN
	rjmp receive_bit_crc
	ori r_rwbyte,0x80
	ldi r_temp2,0 ;fuer CRC Berechnung
receive_bit_crc:
	sdb
	CRCR
	
	lsl r_bcount
	brne recive_bit_no_handle ;bcount nicht 0
	;naechstes byte ....
	rcall handle_byte
	tst r_sendflag  
	brne send_bit ; Nach dem Gelesen byte koennte gesendet werden muessen....
recive_bit_no_handle:	
iend:
#ifdef _ZERO_POLLING_
	;--------------------------------------- Polling for set low... for 4us low impuls of DS2490
	TESTZEROMARKER ; ueberspringe wenn zeromarker=0  
	rjmp zeropolling
	rjmp no_zerromaker
zeropolling:
	sbrc r_sendflag,1  ; sendflag bit 1 -> switch off zeropolling in software eg for asking convert is donne (DS2450, DS18B20 and others)
	rjmp no_zerromaker
zeropolling_h_loop:
	sbic OW_PIN,OW_PINN ;warten bis leitung wieder h 
	rjmp zeropolling_wait  ;leitung ist low ->Schleie
	in r_temp,TCNT_REG
	cpi r_temp,(~OWT_MIN_RESET)+OWT_ZP_WAIT_LOW_TO;aller zwei us zaehlt der timer
	brsh  zeropolling_timeout ;Timeout Reset?
	rjmp zeropolling_h_loop
zeropolling_wait:
	sbis OW_PIN,OW_PINN ;warten bis leitung wieder l
	rjmp zeropolling_low_imp ;Leitung ist low raus aus schleufe
	in r_temp,TCNT_REG
	cpi r_temp,(~OWT_MIN_RESET)+OWT_ZP_WAIT_HIGH_TO ;aller zwei us zaehlt der timer
	sbis OW_PIN,OW_PINN ;noch eine Abfrage, da sonst schleife zu lang (sbis aendert keine flags)
	rjmp zeropolling_low_imp ;Leitung ist low raus aus schleufe
	brlo zeropolling_wait
	rjmp zeropolling_timeout 
zeropolling_low_imp:
	sbi OW_DDR,OW_PINN 
	RESETZEROMARKER 
	rjmp int_internal_start

zeropolling_timeout:
	//in r_temp,TCNT_REG
	//clr r_sendflag
	//RESETZEROMARKER 

no_zerromaker:	
#endif
	CLEAR_INTERRUPT_FLAG  ; wichtig falls inzwischen wider ein Interrupt aufgelaufen ist
	sts sendflag,r_sendflag
	sts bcount,r_bcount
	sts rwbyte,r_rwbyte
	pop r_sendflag
	pop r_rwbyte
	pop r_bcount
	pop r_temp
	cdb
	cbi OW_DDR,OW_PINN  ;vorsichtsmassname Nicht dauerhaft auf low
	pop r_temp2                       //; die benutzten Register wiederherstellen
    out _SFR_IO_ADDR(SREG),r_temp2
    pop r_temp2
    reti




handle_byte:
	push zl
	push zh
	push r_mode
	push r_bytep
	//cdb
	lds r_mode,mode
	lds r_bytep,bytep


	ldi zl,lo8(pm(handle_stable)) 
	ldi zh,hi8(pm(handle_stable)) 
	add zl,r_mode
#if ((handle_stable&0xFE00)!=(handle_stable_end&0xFE00))
	ldi r_temp,0 
	adc zh,r_temp
#endif
	ijmp 
handle_end_sleep:
	clr r_bcount
	ldi r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end_no_bcount
handle_end_inc:
	inc r_bytep
handle_end:
	ldi r_bcount,1
handle_end_no_bcount:
	sts mode,r_mode
	sts bytep,r_bytep
	//sdb
	pop r_bytep
	pop r_mode
	pop zh
	pop zl
	ret
