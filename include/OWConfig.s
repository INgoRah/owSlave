
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



#include <avr/io.h>

#define r_temp  16
#define r_rwbyte  17
#define r_temp2  18
#define r_bcount  19
#define r_mode  20
#define r_sendflag 21
#define r_bytep 22
#define r_crc 23

#define r_idm1 25  
#define r_idm2 24
#define r_idn1 15  
#define r_idn2 14

#define xl 26
#define xh 27
#define yl 28
#define yh 29
#define zl 30
#define zh 31

.comm mode,1  ; Aktueller Zustand nach dem die Unterprogramme aufgerufen werden
.comm srbyte,1 ; aktuelles Byte fuer Searchrom
.comm bytep,1 ; pointer fuer Zugriffe auf owid usw
.comm bcount,1 ;bit counter, bit wird durchgeschoben 
.comm rwbyte,1 ; alktuelles byte beim Senden oder Empfangen
.comm sendflag,1; sendfalg= 1 -> Senden sonst Empfangen
.comm gcontrol,1  ;im Test
.comm reset_indicator,1  ; zeigt an wenn ein Reset empfangen wurde (Fuer das C Programm)
.comm alarmflag,1
#ifndef _DIS_FLASH_
.comm flashmarker,1
#endif

.macro configZ m,offs
	ldi  zl,lo8(\m)       
	; daten im gleichen 256 Segment
//#if (((handle_stable>>1)&0xFF00)!= (m&0xFF00)) 
    ldi  zh,hi8(\m)
//#endif
	add  zl,\offs
//#if ((pack&0x00FF)>(0xC0))
	clr r_temp	
	adc zh,r_temp
//#endif
	
.endm


#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__)||defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#include "OWSet_ATTINYX4.s"
#endif

#if  defined(__AVR_ATmega48__)||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168A__) ||defined(__AVR_ATmega328__) ||defined(__AVR_ATmega328P__) ||defined(__AVR_ATmega328PB__) 
#include "OWSet_ATMEGA168.s"
#endif 

#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
#include "OWSet_ATTINYX5.s"
#endif

