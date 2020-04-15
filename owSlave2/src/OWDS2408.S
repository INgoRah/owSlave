
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

#define _CHANGEABLE_ID_
#define _ZERO_POLLING_

#include "../include/OWConfig.s"
#include "../include/OWCRC16.s"

.extern pack,8
.comm addr,1 ;zweites Adressbyte ist unnoetig (Warum auch immer fuer 32 Byte 16 Bit Adressen verwendet werden....)
.comm crcsave,1 ; zwischenspeicherspeicher fuer crc nur zweites byte....
//.extern  am2302_temp,2
.comm stat_to_sample,1



.macro CHIP_INIT 	
.endm

.macro COMMAND_TABLE
		rjmp h_readpioregaddr
		rjmp h_readpioreg
		rjmp h_readpioregcrc1
		rjmp h_readpioregcrc2
		rjmp h_readchanel
		rjmp h_readchanel_crc
		rjmp h_writechanel
		rjmp h_writecomchanel
		rjmp h_writesendaa
		rjmp h_writesendchanel
		rjmp h_resetactivity
		rjmp h_writeregaddr
		rjmp h_writereg
.endm

#include "../include/OWRomFunctions.s"
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

;---------------------------------------------------
;	READ COMMAND and start operation
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
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
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
	lds r_rwbyte,crc
	com r_rwbyte
	lds r_temp,crc+1
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





#include "../include/OWPinInterrupt.s"
.end