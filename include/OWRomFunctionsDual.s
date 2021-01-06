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
#define OW_SEARCHROMR 4  ; next receive master answer
#define OW_READ_COMMAND1 5
#define OW_READ_COMMAND2 6
#define OW_READ_COMMAND12 7 ;Skip ROM.... eigentlich nicht mit mehreren geraeten, aber bei loxone schon (CC 44)
#define OW_FWCONFIGINFO1 8
#define OW_FWCONFIGINFO2 9
#define OW_FWWRITECONFIG 10
#define OW_FWWRITECONFIG2 11

.comm idtable,64

#ifdef _CHANGEABLE_ID_
#define OW_WRITE_NEWID 12
#define OW_READ_NEWID 13
#define OW_SET_NEWID 14
#define OW_FIRST_COMMAND 15
.comm newid,8

   
.macro CHANGE_ID_COMMANDS
	cset 0x75,OW_WRITE_NEWID
	cljmp 0xA7,hrc_set_readid
	cljmp 0x79,hrc_set_setid
.endm


#else
#define OW_FIRST_COMMAND 12
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
	cset 0x86,OW_FWWRITECONFIG
	cljmp 0x85,hrc_fw_configinfo1
.endm
.macro FW_CONFIG_INFO2
	cset 0x86,OW_FWWRITECONFIG2
	cljmp 0x85,hrc_fw_configinfo2
.endm

handle_stable: 
		rjmp handle_end_no_bcount // sleep eventuell reset, nichts tun und auf Timeout warten
		rjmp h_readromcommand 
		rjmp h_matchrom 
		rjmp h_searchroms 
		rjmp h_searchromr
		rjmp h_readcommand 
		rjmp h_readcommand2
#ifdef _HANDLE_CC_COMMAND_
		rjmp h_readcommand12
#else
		rjmp handle_end_no_bcount
#endif
		rjmp h_fwconfiginfo1
		rjmp h_fwconfiginfo2
		rjmp h_fwwriteconfig1
		rjmp h_fwwriteconfig2
#ifdef _CHANGEABLE_ID_
		rjmp h_writeid
		rjmp h_readid
		rjmp h_setid
#endif
		COMMAND_TABLE

h_readromcommand:
	clr r_bytep
	cjmp 0x55,hrc_set_matchrom
	cjmp 0xF0,hrc_set_searchrom
	cjmp 0xEC,hrc_set_alarm_search
#ifdef _HANDLE_CC_COMMAND_
	cjmp 0xCC,hrc_start_read_command12
#endif
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

hrc_set_matchrom:
	ldi r_temp,3
	sts srbyte,r_temp ; Beide geraete nehmen an searchrom teil
	ldi r_mode,OW_MATCHROM
	rjmp handle_end

hrc_set_searchrom:	
	ldi r_temp,3
	sts srbyte,r_temp ; Beide geraete nehmen an searchrom teil
	configZ idtable,r_bytep
	rjmp h_searchrom_next_bit

hrc_start_read_command: ;Skip rom und Matchrom ok...
	CRCInit1
	lds r_temp,srbyte
	cpi r_temp,1
	breq hrc_start_read_command1
	cpi r_temp,2
	breq hrc_start_read_command2
	rjmp handle_end_sleep
hrc_start_read_command1:
	ldi r_mode,OW_READ_COMMAND1
	rjmp handle_end
hrc_start_read_command2:
	ldi r_mode,OW_READ_COMMAND2
	rjmp handle_end

#ifdef _HANDLE_CC_COMMAND_
hrc_start_read_command12:
	ldi r_mode,OW_READ_COMMAND12
	rjmp handle_end
#endif

hrc_set_alarm_search:
	lds r_temp,alarmflag
	tst r_temp
	;alarm flag nicht 0 also gehe zu searchrom
	brne hrc_set_searchrom_id1
	; sonst tue nichts
	rjmp handle_end_sleep

hrc_fw_configinfo1:
#ifdef _NO_CONFIGBYTES_
	rjmp handle_end_sleep
#else
	ldi r_mode,OW_FWCONFIGINFO1
	ldi r_sendflag,1
	CRCInit2
	rjmp h_fwconfiginfo1
#endif

hrc_fw_configinfo2:
#ifdef _NO_CONFIGBYTES_
	rjmp handle_end_sleep
#else
	ldi r_mode,OW_FWCONFIGINFO2
	ldi r_sendflag,1
	CRCInit2
	rjmp h_fwconfiginfo2
#endif

;---------------------------------------------------
;   MATCH ROM
;---------------------------------------------------
	
h_matchrom:
	lds r_bcount,srbyte
	sbrs r_bcount,0 ;ueberspringe wenn bit 1 =0 also geraet 1 nich mehr im rennen
	rjmp h_matchrom_id2
	configZ owid1,r_bytep
	ld r_temp2,Z
	cp r_temp2,r_rwbyte
	breq h_matchrom_id2
	cbr r_bcount,1 ; loesche geraet
	breq h_matchrom_sleep
h_matchrom_id2:
	configZ owid2,r_bytep
	ld r_temp2,Z
	cp r_temp2,r_rwbyte
	breq hmr_next_byte
	cbr r_bcount,2 ; loesche geraet
	breq h_matchrom_sleep

hmr_next_byte:
	sts srbyte,r_bcount
	cpi r_bytep,7
	breq hrc_start_read_command ;Starten von Read Command
	rjmp handle_end_inc

h_matchrom_sleep:
	sts srbyte,r_bcount
	rjmp handle_end_sleep

;---------------------------------------------------
;   SEARCH ROM
;---------------------------------------------------

hrc_set_searchrom_id1:
	ldi r_temp,1
	; srbyte: aktuelles Byte fuer Searchrom
	sts srbyte,r_temp ; only owid1 participates in searchrom
	configZ idtable,r_bytep
	rjmp h_searchrom_next_bit

h_searchrom_next_bit:  ;Setup next Bit of ID
	ld r_temp2,Z
	lds r_temp,srbyte  ;srbyte ist ein zeiger auf die bits fuer ein bit im Table
h_searchrom_next_bit_l2:
	cpi r_temp,3
	breq h_searchrom_next_bit_l1
	lsr r_temp2
	lsr r_temp2
	inc r_temp
	rjmp h_searchrom_next_bit_l2
h_searchrom_next_bit_l1:
	lsr r_temp2
	rol r_rwbyte  ; negiertes bit in rwbyte
	lsr r_temp2
	rol r_rwbyte  ; bit in rwbyte
	ldi r_sendflag,1
	ldi r_bcount,0x40 ; zwei bits sensden dann zu Searchromr 
	ldi r_mode,OW_SEARCHROMR
	rjmp handle_end_no_bcount

h_searchroms:  ; Modus Send zwei bit
	configZ idtable,r_bytep
	ld r_temp2,Z+
	lds r_temp,srbyte
	cpi r_temp,3
	breq h_searchroms_idd
	cpi r_temp,1
	breq h_searchroms_id1
	cpi r_temp,2
	breq h_searchroms_id2
	rjmp handle_end_sleep ; zur Sicherheit.....
h_searchroms_idd:
	andi r_rwbyte,0x80
	breq h_searchroms_idd_zero
	; Master send 1
	sbrc r_temp2,0 ;springe wenn nicht beide bits 0 (id 1 negiert und id 2 negiert)
	rjmp handle_end_sleep ;
	sbrc r_temp2,4  ;id1 set? then skip
	cbr r_temp,1  ; loesche bit 1 in srbyte
	sbrc r_temp2,2 ; springe wenn id 2 gesetzt ist
	cbr r_temp,2 ;  loesche bit 2 in srbyte 
	sts srbyte,r_temp
	rjmp h_searchroms_idX_end
h_searchroms_idd_zero:
	sbrc r_temp2,1 ;springe wenn nicht beide 1 (id 1  und id 2 )
	rjmp handle_end_sleep ;beide 1 gehe schlafen
	sbrs r_temp2,4  ;id1 0? then skip
	cbr r_temp,1  ; loesche bit 1 in srbyte
	sbrs r_temp2,2 ; springe wenn id 2  null ist
	cbr r_temp,2  ; loesche bit 2 in srbyte
	sts srbyte,r_temp
	rjmp h_searchroms_idX_end
h_searchroms_id1:
	andi r_rwbyte,0x80
	breq h_searchroms_id1_zero
	; Master send 1
	sbrs r_temp2,5  ;id1 set? then skip
	rjmp handle_end_sleep ;
	rjmp h_searchroms_idX_end
h_searchroms_id1_zero:		
	sbrs r_temp2,4  ;id1 set? then skip
	rjmp handle_end_sleep ;
	rjmp h_searchroms_idX_end
h_searchroms_id2:
	andi r_rwbyte,0x80
	breq h_searchroms_id2_zero
	; Master send 1
	sbrs r_temp2,3  ;id1 set? then skip
	rjmp handle_end_sleep ;
	rjmp h_searchroms_idX_end
h_searchroms_id2_zero:		
	sbrs r_temp2,2  ;id1 set? then skip
	rjmp handle_end_sleep ;
	rjmp h_searchroms_idX_end
h_searchroms_idX_end:
	lds r_temp,srbyte
	tst r_temp
	brne h_searchroms_idX_end1
	rjmp handle_end_sleep
h_searchroms_idX_end1:
	inc r_bytep
	cpi r_bytep,64
	breq h_searchrom_end_ok  ;unterschied nur das letzt bit wird wohl nie vorkommen
	rjmp h_searchrom_next_bit

h_searchrom_end_ok:
	clr r_sendflag
	rjmp hrc_start_read_command

h_searchromr:  ; stelle um auf empfangen
	clr r_sendflag
	ldi r_mode,OW_SEARCHROMS
	ldi r_bcount,0  ;gehe nach einem bit zu SEARCHROMS
	rjmp handle_end_no_bcount


;---------------------------------------------------
;   FW_CONFIG_INFO
;---------------------------------------------------

h_fwconfiginfo1:
#ifdef _NO_CONFIGBYTES_
h_fwconfiginfo2:
	rjmp handle_end_sleep
#else
	configZ config_info1,r_bytep
	rjmp h_fwconfiginfo_go
h_fwconfiginfo2:
	configZ config_info2,r_bytep
/*#ifdef _CRC16_
	cpi  r_bytep,24
	breq h_fwconfiginfo_crc
	cpi  r_bytep,26
	breq h_fwconfiginfo_all
//h_fwconfiginfo_end:
	//configZ config_info1,r_bytep  //crc16 wird in config_info1 gespeichert, auch bei config_info2 
	configZ config_info2,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc
#endif
*/

h_fwconfiginfo_go:
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
	//configZ config_info1,r_bytep  //crc16 wird in config_info1 gespeichert, auch bei config_info2 
	//configZ config_info1,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc
h_fwconfiginfo_crc:
#ifdef _CRC8_ 
	lds r_rwbyte,crc
	rjmp handle_end_inc
#elif defined _CRC16_
	lds r_temp,crc
	com r_temp
	sts config_info1+24,r_temp
	lds r_temp,crc+1
	com r_temp
	sts config_info1+25,r_temp
	ldi r_mode,OW_FWCONFIGINFO1  //auch CRC vom Dev 2 wird in Configinfo 1 geschrieben also da weiter machen
	configZ config_info1,r_bytep
	ld   r_rwbyte,Z
	rjmp handle_end_inc

#endif
h_fwconfiginfo_all:
	rjmp handle_end_sleep
#endif

h_fwwriteconfig1:
	configZ config_info1,r_bytep
	st   Z,r_rwbyte
	cpi  r_bytep,22
	breq h_writeconfig_all
	rjmp handle_end_inc

h_fwwriteconfig2:
	configZ config_info2,r_bytep
	st   Z,r_rwbyte
	cpi  r_bytep,22
	breq h_writeconfig_all
	rjmp handle_end_inc

h_writeconfig_all:
	ldi r_temp,16
	sts gcontrol,r_temp
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
	lds r_bcount,srbyte
	cpi r_bcount,2
	breq h_setid2
h_setid1:
	configZ owid1,r_bytep
	rjmp h_setido
h_setid2:
	configZ owid2,r_bytep
h_setido:
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
	ldi r_temp2,lo8(0)
	ldi zh,hi8(0)
	ldi r_temp,7
	sbrc r_bcount,1
	ldi r_temp,15
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
	//rcall read_EEPROM_ID1
	//rcall read_EEPROM_ID2
	push r_idm1
	push r_idm2
	push xl
	push xh
	rcall init_idtable
	pop xh
	pop xl
	pop r_idm2
	pop r_idm1
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
	
; check for bootloader jumper
	;vor allen anderen Registerconfigs
	push r_temp
#ifndef _DIS_FLASH_
	CHECK_BOOTLOADER_PIN 
#endif
	HW_INIT  //Microcontroller specific
	CHIP_INIT //1-Wire device specific
	pop r_temp
init_idtable:
	push yl
	push yh
	push r_temp
	push r_rwbyte
	push r_idn1
	push r_idn2
	ldi r_bytep,8
	ldi r_temp,0
	ldi  zl,lo8(idtable)
    ldi  zh,hi8(idtable)
	ldi  xl,lo8(owid1)
	ldi  xh,hi8(owid1)
	ldi	 yl,lo8(owid2)
	ldi  yh,hi8(owid2)
owinit_odgen1:
	ld r_idm1,X+
	ld r_idm2,Y+
	mov r_idn1,r_idm1
	com r_idn1
	mov r_idn2,r_idm2
	com r_idn2
	ldi r_bcount,8
	mov r_temp,r_idm1
	and r_temp,r_idm2
	mov r_temp2,r_idn1
	and r_temp2,r_idn2
owinit_odgen2:
	ldi r_mode,0
	lsr r_idm1
	rol r_mode  ;6. Bit id1 
	lsr r_idn1  
	rol r_mode ; 5. Bit id1negiert
	lsr r_idm2
	rol r_mode  ;;4. Bit id2 
	lsr r_idn2  
	rol r_mode  ;3. Bit id2 negiert
	lsr r_temp 
	rol r_mode ;zweites bit  id1 und id2
	lsr r_temp2
	rol r_mode   ;erstes bit id1 negiert und id2  negiert
	st  Z+,r_mode
	dec r_bcount
	brne owinit_odgen2
	dec r_bytep
	brne owinit_odgen1

	ldi r_temp,0
	sts mode,r_temp
	sts bcount,r_temp
	sts alarmflag,r_temp
	RESETZEROMARKER
	pop r_idn2
	pop r_idn1
	pop r_rwbyte
	pop r_temp
	pop yh
	pop yl
	
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