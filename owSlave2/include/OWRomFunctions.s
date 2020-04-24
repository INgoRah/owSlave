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



.macro cjmp val,addr
	cpi r_rwbyte,\val
	breq \addr
.endm
.macro cljmp val,addr; Weiter sprung, wenn das ziel zu weit entfernt fuer brxx
	cpi r_rwbyte,\val
	brne 1f
	rjmp \addr
1:
.endm

.macro cset val,mod ;Nur der Mode wird gesetzt. Abkuerzung da oft nur das noch bleibt
	cpi r_rwbyte,\val
	brne 1f
	ldi r_mode,\mod
	rjmp handle_end
1:
.endm




#define OW_SLEEP 0
#define OW_READ_ROM_COMMAND 1
#define OW_MATCHROM 2
#define OW_SEARCHROMS 3  ;next send two bit
#define OW_SEARCHROMR 4  ; next resive master answer
#define OW_READROM 5
#define OW_READ_COMMAND 6
#define OW_FWCONFIGINFO 7


#ifdef _CHANGEABLE_ID_
#define OW_WRITE_NEWID 8
#define OW_READ_NEWID 9
#define OW_SET_NEWID 10
#define OW_FIRST_COMMAND 11
.comm newid,8

.macro CHANGE_ID_COMMANDS
	cset 0x75,OW_WRITE_NEWID
	cljmp 0xA7,hrc_set_readid
	cljmp 0x79,hrc_set_setid
.endm

#else
#define OW_FIRST_COMMAND 8
#endif

#ifndef _DIS_FLASH_
; test auf run flasher command 0x88 in h_readcommand
.macro FLASH_COMMANDS
	cpi r_rwbyte,0x88
	brne 1f
	rjmp hrc_jmp_flasher
1: ldi r_temp,0 ;Anderes Kommando flashmarker zuruecksetzen...
	sts flashmarker,r_temp
.endm
#endif

.macro FW_CONFIG_INFO
	cljmp 0x85,hrc_fw_configinfo
.endm


#ifdef _CHANGEABLE_ID_
; lesen der ID aus dem EEPROM beim Start
read_EEPROM_ID:
	push r_bytep
	push r_rwbyte//r_temp2 and Z is not in gnu C save area
	ldi r_temp2,lo8(E2END)
	ldi zh,hi8(E2END)
	subi r_temp2,7
	out _SFR_IO_ADDR(EEARH), zh
	ldi r_bytep,0
	ldi  zl,lo8(owid)       
    ldi  zh,hi8(owid)
read_EEPROM_ID_loop:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp read_EEPROM_ID_loop
	out _SFR_IO_ADDR(EEARL),r_temp2
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_rwbyte,_SFR_IO_ADDR(EEDR)
	cpi r_rwbyte,0xFF
	breq read_EEPROM_ID_end
	st Z+,r_rwbyte
	inc r_bytep
	inc r_temp2
	cpi r_bytep,8
	brne read_EEPROM_ID_loop
read_EEPROM_ID_end:
	pop r_rwbyte
	pop r_bytep
	ret
#endif






handle_stable: 
		rjmp handle_end_no_bcount // sleep eventuell reset, nichts tun und auf Timeout warten
		rjmp h_readromcommand 
		rjmp h_matchrom 
		rjmp h_searchroms 
		rjmp h_searchromr
		rjmp h_readrom
		rjmp h_readcommand 
		rjmp h_fwconfiginfo
#ifdef _CHANGEABLE_ID_
		rjmp h_writeid
		rjmp h_readid
		rjmp h_setid
#endif
		COMMAND_TABLE



h_readromcommand:
	clr r_bytep
	cset 0x55,OW_MATCHROM 
	cjmp 0xF0,hrc_set_searchrom
	cjmp 0xCC,hrc_start_read_command ;skip rom
	cjmp 0x33,hrc_set_read_rom
	cjmp 0xEC,hrc_set_alarm_search
	
	rjmp handle_end_sleep

#ifndef _DIS_FLASH_
;sprung zum flasher
hrc_jmp_flasher:
	lds r_temp,flashmarker
	cpi r_temp,2
	brne hrc_jmp_flasher_inc
	JMP_FLASHER
hrc_jmp_flasher_inc:
	inc r_temp
	sts flashmarker,r_temp
	rjmp handle_end_sleep
#endif


hrc_set_searchrom:	
	lds r_rwbyte,owid ;erstes Byte lesen
	rjmp h_searchrom_next_bit



hrc_start_read_command: ;Skip rom und Matchrom ok...
	ldi r_mode,OW_READ_COMMAND
	CRCInit1
	rjmp handle_end

hrc_set_read_rom:
	ldi r_mode,OW_READROM
	ldi r_sendflag,1
	rjmp h_readrom

hrc_set_alarm_search:
	lds r_temp,alarmflag
	tst r_temp
	brne hrc_set_searchrom ;alarm flag nicht 0 also gehe zu searchrom
	; sonst tue nichts
	rjmp handle_end_sleep


hrc_fw_configinfo:
	ldi r_mode,OW_FWCONFIGINFO
	ldi r_sendflag,1
	CRCInit2
	rjmp h_fwconfiginfo


;---------------------------------------------------
;   MATCH ROM
;---------------------------------------------------
	

h_matchrom:
	configZ owid,r_bytep
	ld r_temp,Z
	cp r_temp,r_rwbyte
	breq hmr_next_byte
	rjmp handle_end_sleep

hmr_next_byte:
	cpi r_bytep,7
	breq hrc_start_read_command ;Starten von Read Command
	rjmp handle_end_inc



;---------------------------------------------------
;   SEARCH ROM
;---------------------------------------------------


h_searchrom_next_bit:  ;Setup next Bit of ID
	sts srbyte,r_rwbyte ;erstes Byte speichern von der Aufrufenden Ebene
	mov r_temp2,r_rwbyte
	com r_rwbyte ; negieren
	ror r_temp2 ; erstes unnegiertes bit in Carry
	rol r_rwbyte ;und dann als erstes bit in r_rwbyte
	ldi r_sendflag,1
	ldi r_bcount,0x40 ; zwei bits sensden dann zu Searchromr 
	ldi r_mode,OW_SEARCHROMR
	rjmp handle_end_no_bcount



h_searchroms:  ; Modus Send zwei bit
	clr r_temp
	sbrc r_rwbyte,7 ; bit gesetz (1 empfangen)
	ldi r_temp,1
	lds r_bcount,srbyte ;r_bcount wird am ende gesetzt
	eor r_temp,r_bcount
	sbrs r_temp,0
	rjmp h_searchroms_next ; Vergleich des letzen gelesenen bits mit der id
	;Ungleich....
	;goto sleep
	;clr r_sendflag
	; ist ja auf lesen
	rjmp handle_end_sleep
h_searchroms_next: ; Setup next bit
	inc r_bytep  ; zaehler der Bits erhoehen
	sbrc r_bytep,6 ; 64 bit erreicht 
	rjmp h_searchrom_end_ok ;alles ok auf Command warten
	mov r_temp,r_bytep 
	andi r_temp,0x07
	brne h_searchroms_next_bit  ; bit zwischen 0 und 8
	mov r_bcount,r_bytep  ; next Byte lesen
	lsr r_bcount	
	lsr r_bcount
	lsr r_bcount

	configZ owid,r_bcount
	ld r_rwbyte,Z
	sts srbyte,r_rwbyte ;#################### Doppelung ist schon in h_searchrom_next_bit
	rjmp h_searchrom_next_bit
		
h_searchroms_next_bit: ;next Bit lesen
	;sts srbytep,r_bcount
	lds r_rwbyte,srbyte
	lsr r_rwbyte ;aktuelles byte weiterschieben r_rwbyte hier zweckefrei verwendet
	rjmp h_searchrom_next_bit  ;algemeine routine zum vorbereiten
h_searchrom_end_ok:
	clr r_sendflag
	rjmp hrc_start_read_command

h_searchromr:
	clr r_sendflag
	ldi r_mode,OW_SEARCHROMS
	ldi r_bcount,0
	rjmp handle_end_no_bcount


;---------------------------------------------------
;   READ ROM
;---------------------------------------------------

h_readrom:
	cpi  r_bytep,8
	breq h_readrom_all
	configZ owid,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_readrom_all:
	rjmp handle_end_sleep


;---------------------------------------------------
;   FW_CONFIG_INFO
;---------------------------------------------------

h_fwconfiginfo:
	cpi  r_bytep,24
	breq h_fwconfiginfo_crc
#if defined(_CRC8_)  || defined( _CRC8_16_) 
	cpi  r_bytep,25
	breq h_fwconfiginfo_all
#elif defined _CRC16_
	cpi  r_bytep,26
	breq h_fwconfiginfo_all
#else
	cpi  r_bytep,25
	breq h_fwconfiginfo_all
#warning No CRC known code implemented
#endif
h_fwconfiginfo_end:
	configZ config_info,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_fwconfiginfo_crc:
#ifdef _CRC8_
	lds r_rwbyte,crc
	rjmp handle_end_inc
#elif defined _CRC16_
	lds r_temp,crc
	com r_temp
	sts config_info+24,r_temp
	lds r_temp,crc+1
	com r_temp
	sts config_info+25,r_temp
	rjmp h_fwconfiginfo_end
#endif
h_fwconfiginfo_all:
	rjmp handle_end_sleep


;---------------------------------------------------
;   CHANGE ROM FUNCTIONS
;---------------------------------------------------


#ifdef _CHANGEABLE_ID_

h_writeid:
	configZ newid,r_bytep
	st   Z,r_rwbyte
	cpi  r_bytep,7
	breq h_writeid_all
	rjmp handle_end_inc
h_writeid_all:
	rjmp handle_end_sleep


hrc_set_readid:
	ldi r_mode,OW_READ_NEWID
	ldi r_sendflag,1
h_readid:
	cpi  r_bytep,8
	breq h_readid_all
	configZ newid,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_readid_all:
	clr  r_sendflag
	rjmp handle_end_sleep

hrc_set_setid:
	ldi r_mode,OW_SET_NEWID
	;ldi r_bytep,1 ;start to write in 2
	rjmp handle_end_inc ;set r_bytep to 1!!!

h_setid:
	configZ owid,r_bytep
	ld r_temp,Z
	cp r_rwbyte,r_temp
	brne h_setid_bad_code_all
	cpi r_bytep,1
	breq h_setid_set2
	cpi r_bytep,5 
	breq h_setid_set3
	cpi r_bytep,6
	breq h_setid_copy_id
	rjmp h_setid_bad_code_all ;sollte eigentlich nicht passieren
h_setid_set2:
	ldi r_temp,3
	add r_bytep,r_temp
h_setid_set3:
	inc r_bytep
	rjmp handle_end
h_setid_copy_id:
	ldi r_temp2,lo8(E2END)
	ldi zh,hi8(E2END)
	ldi r_temp,7
	sub r_temp2,r_temp
	;ldi r_temp,0 ;kommt nicht vor das ein E2ROM genau n*256+(0 bis 7) byte gross ist
	;sbc zh
	out _SFR_IO_ADDR(EEARH),zh
	ldi zl,lo8(newid)
	ldi zh,hi8(newid)
	ldi r_bytep,0
h_setid_EEPROM_write:
	sbic _SFR_IO_ADDR(EECR), EEPE	
	rjmp h_setid_EEPROM_write
	ldi r_temp, (0<<EEPM1)|(0<<EEPM0)
	out _SFR_IO_ADDR(EECR), r_temp
	;nur adresse L schreiben H bleibt aus oben genannten grund gleich.
	out _SFR_IO_ADDR(EEARL),r_temp2
	ld  r_rwbyte,Z+
	out _SFR_IO_ADDR(EEDR), r_rwbyte
	sbi _SFR_IO_ADDR(EECR), EEMPE
	sbi _SFR_IO_ADDR(EECR), EEPE
	inc r_bytep
	inc r_temp2
	cpi r_bytep,8
	brne h_setid_EEPROM_write
	rcall read_EEPROM_ID
h_setid_bad_code_all:
	rjmp handle_end_sleep



#endif


spause:
	nop
	nop
	nop
	nop
	ret


.global OWINIT
OWINIT:
	push r_temp
#ifndef _DIS_FLASH_
	CHECK_BOOTLOADER_PIN 
#endif
	HW_INIT  //Microcontroller specific
	CHIP_INIT //1-Wire device specific
#ifdef _CHANGEABLE_ID_
	rcall read_EEPROM_ID
#endif
	ldi r_temp,0
	sts mode,r_temp
	sts bcount,r_temp
	sts alarmflag,r_temp
	RESETZEROMARKER
	pop r_temp
	ret


.global EXTERN_SLEEP
EXTERN_SLEEP:
	cli
	push r_temp
	ldi r_temp,0
	sts mode,r_temp ;SLEEP
	sts gcontrol,r_temp
	sts sendflag,r_temp
	sts bcount,r_temp
	RESETZEROMARKER
	pop r_temp
	sei
	ret