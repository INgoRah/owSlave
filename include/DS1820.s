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
