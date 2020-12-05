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


.comm crc16,2
.comm crc8,2
#define _CRC8_16_

CRC16_alg:
	;crc
	
	lds r_crc,crc16;lade low byte fuer vergleich mit bit
	mov r_temp,r_crc
	andi r_temp,0x01 ;nur bit 0
	cp r_temp2,r_temp  ;r_temp2 wird in interruptroutine gesetzt
	brne crc2_16 ;nur schieben
	;schieben und XOR
	lds r_temp2,crc16+1
	lsr	r_temp2
	ror	r_crc
	ldi	r_temp, 0x01	
	eor	r_crc, r_temp
	ldi	r_temp, 0xA0	;
	eor	r_temp2, r_temp
	rjmp crce_16
crc2_16:
	lds r_temp2,crc16+1
	lsr	r_temp2
	ror	r_crc
crce_16:
	sts crc16,r_crc
	sts crc16+1,r_temp2
	
	ret

.macro CRCS ; CRC beim Senden
	push r_crc
	push r_temp2
	rcall CRC16_alg
	pop r_temp2
		;crc
	lds r_crc,crc8
	mov r_temp,r_crc
	andi r_temp,0x01
	cp r_temp2,r_temp
	brne crc2_8
	lsr r_crc
	ldi r_temp,0x8c
	eor r_crc,r_temp
	rjmp crce_8
crc2_8:
	lsr r_crc
crce_8:
	sts crc8,r_crc
	pop r_crc
.endm

.macro CRCR; CRC beim Empfangen
	;clr r_temp2
	;sbrc r_rwbyte,7
	;ldi r_temp2,1
	push r_crc
	rcall CRC16_alg
	pop r_crc
.endm

.macro CRCInit1; CRC initialisierung bei READ_COMMAND
	ldi r_temp,0
	sts crc16,r_temp
	sts crc16+1,r_temp
.endm

.macro CRCInit2; CRC initialisierung bei READ_DATA
	ldi r_temp,0
	sts crc8,r_temp
.endm
