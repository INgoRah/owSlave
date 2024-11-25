;---------------------------------------------------
;	READ COMMAND and start operation
;---------------------------------------------------

h_readcommand2:
	clr r_bytep
#ifndef _DIS_FLASH_
	FLASH_COMMANDS ; muss zu erst sein....
#endif
	cset 0xAA,OW_READ_MEMORY_ADDR
	cset 0x55,OW_WRITE_MEMORY_ADDR
	cset 0x3C,OW_CONVERT
	FW_CONFIG_INFO2
#ifdef _CHANGEABLE_ID_
	CHANGE_ID_COMMANDS
#endif
	ldi r_mode,OW_SLEEP
	rjmp handle_end


h_readmemoryaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_readmemory_addr_byte1 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	rjmp handle_end_inc
h_readmemory_addr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_READ_MEMORY ;weiter zu read Memory
	;;ldi r_bcount,1 ;ist unten
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	clr r_bytep
	rjmp h_readmemory2
h_readmemory:
	lds r_bytep,addr
	inc r_bytep
	sts addr,r_bytep
	andi r_bytep,0x07
	breq h_readmemory_init_crc
h_readmemory2:
	lds r_bytep,addr
	;andi r_bytep,0x1F ist oben
	configZ packadc,r_bytep
	ld   r_rwbyte,Z
	;ldi r_bcount,1
	rjmp handle_end ;sendet das Byte und geht zu h_readmemory
h_readmemory_init_crc:; init erstes CRC byte
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_READ_MEMORY_CRC1
	;ldi r_bcount,1
	rjmp handle_end
h_readmemory_end:
	//ldi  r_mode,OW_SLEEP
	//clr r_sendflag
	rjmp handle_end_sleep
h_readmemorycrc1:;init zweites CRC Byte
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_READ_MEMORY_CRC2
	rjmp handle_end
h_readmemorycrc2:;weiteres senden..... nach zweitem Byte
	lds r_temp,addr
	andi r_temp,0xE0
	brne h_readmemory_end; ende des speichers
	ldi r_mode,OW_READ_MEMORY
	CRCInit1 ;Start with new CRC
	rjmp h_readmemory2

h_writememoryaddr:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_writememory_addr_byte1 ;nein dann weiter
	andi r_rwbyte,0x1F  ; nur Adressen zwischen 0 und 0x1F zulassen
	sts addr,r_rwbyte  ;speichern des ersten bytes
	inc r_bytep
	;ldi r_bcount,1
	rjmp handle_end
h_writememory_addr_byte1:  ;zweiters Addressbyte wird nicht gespeichert!
	ldi r_mode,OW_WRITE_MEMORY ;weiter zu read Memory
	;ldi r_bcount,1 ;; _________________________________________________in handle_end integrieren.....
	lds r_bytep,addr
	rjmp handle_end ;read Memory Byte
h_writememory:
	lds r_bytep,addr
	configZ packadc,r_bytep
	st Z,r_rwbyte
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC1
	ldi r_sendflag,1 ;jetzt sendet der Slave zum Master
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	rjmp handle_end
h_writememorycrc1:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_CRC2
	rjmp handle_end
h_writememorycrc2:
	lds r_temp,addr
	configZ packadc,r_temp
	ld r_rwbyte,Z
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY_READBACK
	rjmp handle_end
h_writememoryreadback:
	ldi r_temp,0x00
	sts crc16+1,r_temp
	lds r_temp,addr
	inc r_temp
	sts addr,r_temp
	sts crc16,r_temp
	ldi r_sendflag,0
	;ldi r_bcount,1
	ldi r_mode,OW_WRITE_MEMORY
	rjmp handle_end

h_convert:
	cpi r_bytep,0  ;erstes Adressbyte ?
	brne h_convert_byte1 ;nein dann weiter
	inc r_bytep
	sts packadc+0x20,r_rwbyte
	;ldi r_bcount,1
	rjmp handle_end
h_convert_byte1: ;zweies byte glesen go crc16#
	sts packadc+0x21,r_rwbyte
	lds r_rwbyte,crc16
	com r_rwbyte
	lds r_temp,crc16+1
	com r_temp
	sts crcsave,r_temp
	ldi r_mode,OW_CONVERT_CRC1
	;ldi r_bcount,1
	ldi r_sendflag,1
	rjmp handle_end	
h_convertcrc1:
	lds r_rwbyte,crcsave
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CRC2
	rjmp handle_end
h_convertcrc2:
	ldi r_temp,0x20
	sts gcontrol,r_temp
	;ldi r_bcount,1
	ldi r_mode,OW_CONVERT_CONV
	;clr r_sendflag
	ldi r_sendflag,3 ;set bit 0 and 1 for no zero polling
h_convert_conv:
	ldi r_bcount,0
	ldi r_rwbyte,0
	rjmp handle_end_no_bcount	
