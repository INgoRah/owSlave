// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
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

#define _ZERO_POLLING_

#include "../include/OWConfig.s"
#include "../include/OWCRC8_16.s"

.extern pack,8
.comm addr,1 ;zweites Adressbyte ist unnoetig (Warum auch immer fuer 32 Byte 16 Bit Adressen verwendet werden....)
.comm crcsave,1 ; zwischenspeicherspeicher fuer crc nur zweites byte....
//.extern  am2302_temp,2
.comm stat_to_sample,1
#ifdef DS1820_SUPPORT
.extern packt,8
.comm block,1 ; Block der augegeben, geschrieben wird (Parameter von READ/WRITE Scratchpad)
.comm cpsp,1 ; Copy Scratchpad marker
#endif

.macro CHIP_INIT
.endm

.macro COMMAND_TABLE
		rjmp h_readpioregaddr ; OW_READ_PIO_REG_ADDR = 0
		rjmp h_readpioreg
		rjmp h_readpioregcrc1
		rjmp h_readpioregcrc2
		rjmp h_readchanel
		rjmp h_readchanel_crc
		rjmp h_writechanel
		rjmp h_writecomchanel
		rjmp h_writesendaa ; OW_WRITE_SENDAA
		rjmp h_writesendchanel
		rjmp h_resetactivity ; OW_RESET_ACTIVITY
		rjmp h_writeregaddr
		rjmp h_writereg ; OW_WRITE_REG = 12
#ifdef TIMER_SUPPORT
		rjmp h_pintimer ; OW_PIN_TIMER = 13
		rjmp h_pintimer_crc1 ; OW_PIN_TIMER_CRC1
		rjmp h_pintimer_crc2
		rjmp h_pintimer_done
#endif
#ifdef DS1820_SUPPORT
		rjmp h_readscratchpad
		rjmp h_writescratchpad
		rjmp h_convert_run
#endif
.endm

#ifdef DUAL_ROM
#include "../include/OWRomFunctionsDual.s"
#else
#include "../include/OWRomFunctions.s"
#endif
#include "../include/OWTimerInterrupt.s"


; Ab hier Geraeteabhaenging
#define OW_READ_PIO_REG_ADDR OW_FIRST_COMMAND+0
#define OW_READ_PIO_REG OW_FIRST_COMMAND+1
#define OW_READ_PIO_REG_CRC1 OW_FIRST_COMMAND+2
#define OW_READ_PIO_REG_CRC2 OW_FIRST_COMMAND+3
#define OW_READ_CHANEL OW_FIRST_COMMAND+4
#define OW_READ_CHANEL_CRC OW_FIRST_COMMAND+5
#define OW_WRITE_CHANEL OW_FIRST_COMMAND+6
#define OW_WRITE_COMCHANEL OW_FIRST_COMMAND+7
#define OW_WRITE_SENDAA OW_FIRST_COMMAND+8
#define OW_WRITE_SEND_CHANEL OW_FIRST_COMMAND+9
#define OW_RESET_ACTIVITY OW_FIRST_COMMAND+10
#define OW_WRITE_REG_ADDR OW_FIRST_COMMAND+11
#define OW_WRITE_REG OW_FIRST_COMMAND+12
#ifdef TIMER_SUPPORT
#define OW_PIN_TIMER OW_FIRST_COMMAND+13
#define OW_PIN_TIMER_CRC1 OW_FIRST_COMMAND+14
#define OW_PIN_TIMER_CRC2 OW_FIRST_COMMAND+15
#define OW_PIN_TIMER_DONE OW_FIRST_COMMAND+16
#define OW_NEXT_CMD 17
#else
#define OW_NEXT_CMD 13
#endif
#ifdef DS1820_SUPPORT
#define OW_READ_SCRATCHPAD OW_FIRST_COMMAND+OW_NEXT_CMD
#define OW_WRITE_SCRATCHPAD OW_FIRST_COMMAND+OW_NEXT_CMD+1
#define OW_CONVERT_RUN OW_FIRST_COMMAND+OW_NEXT_CMD+2
#endif
#ifdef DS2450_SUPPORT
#endif

;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------
#ifdef DS2450_SUPPORT
#endif

#ifdef DS1820_SUPPORT

h_readcommand2:
	clr r_bytep
	cjmp 0xBE,hrc_set_readscratchpad
	cjmp 0x4E,hrc_set_writescratchpad
	cjmp 0x44,hrc_set_convertT
	cjmp 0x48,hrc_copy_scratchpad
	cjmp 0xB8,hrc_recall_eeprom1
	FW_CONFIG_INFO2
	ldi r_mode,OW_SLEEP
	rjmp handle_end

hrc_set_readscratchpad:
	ldi r_mode,OW_READ_SCRATCHPAD
	ldi r_sendflag,1
	CRCInit2
	rjmp h_readscratchpad

hrc_set_writescratchpad:
	ldi r_mode,OW_WRITE_SCRATCHPAD
	ldi r_bytep,2 ;start to write in 2
	rjmp handle_end

#ifdef _HANDLE_CC_COMMAND_
hrc_set_convertT12:
	rjmp hrc_set_convertT
#endif

hrc_set_convertT:
	ldi r_temp,0x20
	sts gcontrol,r_temp
hrc_set_convertT12b:
	ldi r_mode,OW_CONVERT_RUN
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_run:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount

hrc_recall_eeprom1:
#if 1
	rjmp handle_end_sleep
#else
	rcall hrc_recall_eeprom_func1
	rjmp handle_end
#endif

hrc_copy_scratchpad:
	ldi r_bytep,2
	configZ packt,r_bytep
	clr r_bytep
hrc_copy_scratchpad_EEPROM_write1:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp hrc_copy_scratchpad_EEPROM_write1
	ldi r_temp, (0<<EEPM1)|(0<<EEPM0)
	out _SFR_IO_ADDR(EECR), r_temp
	ldi r_temp,0
	out _SFR_IO_ADDR(EEARH),r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	ld  r_rwbyte,Z+
	out _SFR_IO_ADDR(EEDR), r_rwbyte
	sbi _SFR_IO_ADDR(EECR), EEMPE
	sbi _SFR_IO_ADDR(EECR), EEPE
	inc r_bytep
	cpi r_bytep,3
	brne hrc_copy_scratchpad_EEPROM_write1
	rjmp handle_end

#if 0
hrc_recall_eeprom_func1:
	ldi r_bytep,2
	configZ packt,r_bytep
	clr r_bytep
	clr r_temp
hrc_recall_eeprom_EEPROM_read1:
	sbic _SFR_IO_ADDR(EECR), EEPE
	rjmp hrc_recall_eeprom_EEPROM_read1
	out _SFR_IO_ADDR(EEARH), r_temp
	out _SFR_IO_ADDR(EEARL), r_bytep
	sbi _SFR_IO_ADDR(EECR), EERE
	in r_rwbyte,_SFR_IO_ADDR(EEDR)
	st Z+,r_rwbyte
	inc r_bytep
	cpi r_bytep,3
	brne hrc_recall_eeprom_EEPROM_read1
	ret
#endif

;---------------------------------------------------
;   READ SCRATCHPAD
;---------------------------------------------------

h_readscratchpad:
	cpi  r_bytep,8
	breq h_readscratchpad_crc
	cpi  r_bytep,9
	breq h_readscratchpad_all
	configZ packt,r_bytep
	ld   r_rwbyte,Z
	rjmp h_readscratchpad_endc
h_readscratchpad_crc:
	lds  r_rwbyte,crc8
h_readscratchpad_endc:
	inc  r_bytep
	ldi  r_bcount,1
	rjmp handle_end
h_readscratchpad_all:
	clr r_temp
	sts alarmflag2,r_temp
	rjmp handle_end_sleep

;---------------------------------------------------
;   WRITE SCRATCHPAD
;---------------------------------------------------

h_writescratchpad:
	configZ packt,r_bytep
	inc  r_bytep
	cpi  r_bytep,5
	breq h_writescratchpad_all
	st   Z,r_rwbyte
	rjmp handle_end
h_writescratchpad_all:
	;ori r_rwbyte,0x1F ; Alle unteren Bits sind immer 1 -->VOC use different
	st   Z,r_rwbyte
	rjmp handle_end_sleep

#endif
; end of DS1820_SUPPORT

;---------------------------------------------------
;   DS2408 instance 1
;---------------------------------------------------
h_readcommand:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xF0,OW_READ_PIO_REG_ADDR
	cljmp 0xF5,hrc_readchanel
	cset 0x5A,OW_WRITE_CHANEL
	cljmp 0xC3,hrc_reset_activity
	cset 0xCC,OW_WRITE_REG_ADDR
	FW_CONFIG_INFO
#ifdef TIMER_SUPPORT	
	cset 0xC5,OW_PIN_TIMER
#endif
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end

h_readpioregaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readpioreg_addr_byte1 ;nein dann weiter
	//andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	subi r_rwbyte,0x89  ;beim lesen von 0x88 --> 0xFF inc addr -> 0x00
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readpioreg_addr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_PIO_REG ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
h_readpioreg:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	cpi r_bytep,0x08
	breq h_readpioreg_init_crc
	brge h_readpioreg_end ; groeser dann nix senden
	configZ pack,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readpioreg_init_crc:; init erstes CRC byte
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_PIO_REG_CRC1
	;ldi r_bcount,1
	rjmp handle_end
h_readpioreg_end:
	ldi  r_mode,OW_SLEEP
	clr r_sendflag
	rjmp handle_end
h_readpioregcrc1:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_PIO_REG_CRC2
	rjmp handle_end
h_readpioregcrc2: ; 2. CRC Byte gesendet
	rjmp h_readpioreg_end


hrc_readchanel:
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	ldi r_mode,OW_READ_CHANEL
	ldi r_temp,8
	sts gcontrol,r_temp
	rjmp h_readchanel1
h_readchanel:
	ldi r_temp,4
	sts gcontrol,r_temp
h_readchanel1:
	cpi r_bytep,31
	brge h_readchanelcrc1
	lds r_rwbyte,stat_to_sample
	sts pack,r_rwbyte //sample
	rjmp handle_end_inc
h_readchanelcrc1:
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_CHANEL_CRC
	;ldi r_bcount,1
	rjmp handle_end
h_readchanel_crc:
	clr r_bytep
	ldi r_mode,OW_READ_CHANEL
	lds r_rwbyte,crcsave
	rjmp handle_end

h_writechanel:
	sts crcsave,r_rwbyte
	ldi r_mode,OW_WRITE_COMCHANEL
	rjmp handle_end
h_writecomchanel:
	com r_rwbyte
	lds r_temp,crcsave
	cp r_rwbyte,r_temp
	breq h_writeok
	rjmp handle_end_sleep
h_writeok:
	sts pack+1,r_rwbyte
	ldi r_temp2,1
	sts gcontrol,r_temp2
	clr r_sendflag
	ldi r_rwbyte,0xAA
	ldi r_mode,OW_WRITE_SENDAA
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	rjmp handle_end
h_writesendaa:
	lds r_rwbyte,pack
	ldi r_mode,OW_WRITE_SEND_CHANEL
	rjmp handle_end
h_writesendchanel:
	rjmp handle_end_sleep


hrc_reset_activity:
	clr r_temp
	sts alarmflag,r_temp
	sts pack+2,r_temp 	; pack.PIO_Activity_Latch_State = 0
	sts pack+5,r_temp	; pack.Status = 0
	ldi r_temp,2
	sts gcontrol,r_temp
	ldi r_rwbyte,0xAA
	ldi r_mode,OW_RESET_ACTIVITY
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	rjmp handle_end
h_resetactivity:
	rjmp handle_end_sleep


h_writeregaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writeregddr_byte1 ;nein dann weiter
	//andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	subi r_rwbyte,0x8B
	brmi h_writereg_end
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_writeregddr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_REG ;weiter zu write Memory
	;;ldi r_bcount,1 ;ist unten
	clr r_bytep
	rjmp handle_end
h_writereg:
	lds r_temp,addr
	configZ pack+3,r_temp
	st Z,r_rwbyte
	cpi r_temp,5
	brge h_writereg_end
	inc r_temp
	sts addr,r_temp
	rjmp handle_end_sleep

h_writereg_end:
	rjmp handle_end_sleep

#ifdef TIMER_SUPPORT
h_pintimer:
	; receiving bytes for pin timer
	configZ pin_tmr,r_bytep
	st   Z,r_rwbyte
	inc  r_bytep
	cpi  r_bytep,4
	; received 4 bytes, so ack
	breq h_pintimer_sendack
	rjmp handle_end
h_pintimer_sendack:
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_temp,0x40
	sts gcontrol,r_temp
	ldi r_mode,OW_PIN_TIMER_CRC1
	ldi r_sendflag,1 ; now slave is transmitting
	rjmp handle_end
h_pintimer_crc1:
	lds r_rwbyte,crcsave
	ldi r_mode,OW_PIN_TIMER_CRC2
	rjmp handle_end

h_pintimer_crc2:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_PIN_TIMER_DONE
	rjmp handle_end

h_pintimer_done:
	rjmp handle_end_sleep
#endif /* TIMER_SUPPORT */
#include "../include/OWPinInterrupt.s"
.end