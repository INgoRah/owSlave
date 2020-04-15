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


.comm crc,2
#define _CRC16_

CRC16_alg:
	;crc
	push r_crc
	lds r_crc,crc;lade low byte fuer vergleich mit bit
	mov r_temp,r_crc
	andi r_temp,0x01 ;nur bit 0
	cp r_temp2,r_temp  ;r_temp2 wird in interruptroutine gesetzt
	brne crc2 ;nur schieben
	;schieben und XOR
	lds r_temp2,crc+1
	lsr	r_temp2
	ror	r_crc
	ldi	r_temp, 0x01	
	eor	r_crc, r_temp
	ldi	r_temp, 0xA0	;
	eor	r_temp2, r_temp
	rjmp crce
crc2:
	lds r_temp2,crc+1
	lsr	r_temp2
	ror	r_crc
crce:
	sts crc,r_crc
	sts crc+1,r_temp2
	pop r_crc
	ret

.macro CRCS ; CRC beim Senden
	rcall CRC16_alg
.endm

.macro CRCR; CRC beim Empfangen
	;clr r_temp2
	;sbrc r_rwbyte,7
	;ldi r_temp2,1
	rcall CRC16_alg
.endm

.macro CRCInit1; CRC initialisierung bei READ_COMMAND
	ldi r_temp,0
	sts crc,r_temp
	sts crc+1,r_temp
.endm

.macro CRCInit2; CRC initialisierung bei READ_DATA
.endm
