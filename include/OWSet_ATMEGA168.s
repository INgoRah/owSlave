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


#define OW_PORT _SFR_IO_ADDR(PORTD) //1 Wire Port
#define OW_PIN _SFR_IO_ADDR(PIND) //1 Wire Pin as number
#define OW_PINN PORTD2
#define OW_DDR _SFR_IO_ADDR(DDRD)  //pin direction register

#define TCNT_REG _SFR_IO_ADDR(TCNT0)


#define DB_PORT _SFR_IO_ADDR(PORTB) //DEBUG
#define DB_PIN _SFR_IO_ADDR(PINB) //DEBUG
#define DB_DDR _SFR_IO_ADDR(DDRB) //DEBUG
#define DB_PINN PORTB1

#define SETZEROMARKER sbi _SFR_IO_ADDR(DDRC),6
#define RESETZEROMARKER cbi _SFR_IO_ADDR(DDRC),6
#define TESTZEROMARKER sbic _SFR_IO_ADDR(DDRC),6

#ifdef _DB_
#define sdb sbi _SFR_IO_ADDR(PORTB),DB_PINN
#define cdb cbi _SFR_IO_ADDR(PORTB),DB_PINN
#else
#define sdb
#define cdb
#endif

#define TIMER_INTERRUPT TIMER0_OVF_vect
#define PIN_INTERRUPT INT0_vect


//#define OWT_MIN_RESET 160 
//#define OWT_RESET2 40
//#define OWT_RESET_PRESENT 15
//#define OWT_PRESENT 50
//#define OWT_WRITE 18
//#define OWT_READ 4
#ifdef __4MHZ__
#define OWT_MIN_RESET 80
#define OWT_RESET2 30
#define OWT_RESET_PRESENT 15
#define OWT_PRESENT 60
#define OWT_WRITE 16
#define OWT_READ 5
#define OWT_ZP_WAIT_LOW_TO 36  ; Zeit fuer die Maximale Lowzeit bei der Zerro Polling pruefung. Ist die Leitung laenger low, ist es vieleicht ein Reset 
#define OWT_ZP_WAIT_HIGH_TO 54 ; Zeit fuer die Maximale Hight zeit bis zu einem Low Impuls vom Master. Nach laengeren Pausen wird nicht mehr gepollt sondern der Interrup vererndet.
#else
#define OWT_MIN_RESET 180
#define OWT_RESET2 80
#define OWT_RESET_PRESENT 30
#define OWT_PRESENT 130
#define OWT_WRITE 35
#define OWT_READ 12
#define OWT_ZP_WAIT_LOW_TO 60  ; Zeit fuer die Maximale Lowzeit bei der Zerro Polling pruefung. Ist die Leitung laenger low, ist es vieleicht ein Reset 
#define OWT_ZP_WAIT_HIGH_TO 90 ; Zeit fuer die Maximale Hight zeit bis zu einem Low Impuls vom Master. Nach laengeren Pausen wird nicht mehr gepollt sondern der Interrup vererndet.
#endif

.macro CLEAR_TOV_FLAG
	ldi r_temp,1
	out _SFR_IO_ADDR(TIFR0),r_temp
.endm

.macro JMP_NO_TOV
	in r_temp, _SFR_IO_ADDR(TIFR0)
	sbrc r_temp,TOV0 ; wenn ueberlauf gleich weiter
.endm

.macro CLEAR_INTERRUPT_FLAG 
	ldi r_temp,(1<<INTF0);inerrupt flags durch 1 loeschen..... 0 macht nix
	out _SFR_IO_ADDR(EIFR),r_temp
.endm

.macro EN_TIM_INT
	ldi r_temp,(1<<TOIE0)
	sts TIMSK0,r_temp
	ldi r_temp,(1<<TOV0) ;inerrupt flags durch 1 loeschen..... 0 macht nix
	out _SFR_IO_ADDR(TIFR0),r_temp
.endm

.macro DIS_TIM_INT
	ldi r_temp,0
	sts TIMSK0,r_temp
.endm

//Wiederherstellen von Idle mode und umschalten von Low-Level interrupt zu Falling edge
.macro SET_FALLING_RESET_SLEEP
	lds r_temp,EICRA
	ori r_temp,(1<<ISC01)
	sts	EICRA,r_temp
	lds r_temp,SMCR
	andi r_temp,~(1<<SM1)
	sts	SMCR,r_temp
.endm

.macro HW_INIT  ;r_temp is pushed other Registers should be saved
	;set clock to 8 MHz
	ldi r_temp,0x80;
	sts CLKPR,r_temp
	//ldi r_temp,(1<<CLKPS0)
#ifdef __4MHZ__
	ldi r_temp,0x01;
#else
	ldi r_temp,0
#endif
	sts CLKPR,r_temp
	;Disable Timer int
	ldi r_temp,0
	sts TIMSK0,r_temp ;; is default
	;Enable Pin int
	ldi r_temp,(1<<INT0)
	out _SFR_IO_ADDR(EIMSK),r_temp
	;Set Timerclock to Clock / 8 (2us bei 4MHz) bzw 1us bei 8 MHz
	ldi r_temp,(1<<CS01)
	out _SFR_IO_ADDR(TCCR0B),r_temp
	;OWPin as input
	cbi OW_DDR,OW_PINN ;; is default.... 
	cbi OW_PORT,OW_PINN ;; vereinfachung im Hauptprogram (PORTB=0xFF) wegen pullup
	;set falling edge
	ldi r_temp,(1<<ISC01)
	sts EICRA,r_temp
#ifdef _DB_
	sbi DB_DDR,DB_PINN
#endif
.endm

.macro JMP_FLASHER
	ldi r_temp,0x00
	push r_temp
	ldi r_temp,0x3E
	push r_temp
	ret ; Direkter Sprung zum Bootloader
.endm

.macro CHECK_BOOTLOADER_PIN
.endm

