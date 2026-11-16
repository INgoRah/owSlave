/* owSlave 1-Wire bootloader: what the bootloader, the application and
 * the master (in10dfs, include/fw_update.h there) have to agree on.
 * Include it after <avr/io.h>.
 *
 * The bootloader sits in the boot section (ATmega: 2 KB at the end of
 * the flash, BOOTSZ = 1024 words, BOOTRST programmed; ATtiny84/85 see
 * BL_DATA). After a reset it
 * starts the application right away - unless the application asked
 * for it (BL_MAGIC in r24:r25 when jumping to BL_START) or there is no
 * valid one (its first word erased). It then answers on the bus as
 * the device itself (the same ROM, from the EEPROM), to these function
 * commands after a Match or Skip ROM. "~crc" is the inverted CRC16 of
 * the command and all bytes of it so far, as for a DS2408 register read.
 *
 * BL_CMD_INFO   -> 'B' 'L' version page_size start_lo start_hi
 *                  sig0 sig1 sig2 app_valid last_result ~crc_lo ~crc_hi
 * BL_CMD_WRITE  <- addr_lo addr_hi page_size bytes
 *               -> ~crc_lo ~crc_hi
 *               <- BL_CONFIRM, anything else drops the page
 *                  (page erase, write and verify: up to 10 ms, the
 *                  device does not answer meanwhile)
 *               -> result (BL_OK or BL_ERR_*), read until not 0xff
 * BL_CMD_READ   <- addr_lo addr_hi
 *               -> page_size bytes of the flash, ~crc_lo ~crc_hi
 * BL_CMD_EXIT   -> BL_OK and starts the application (watchdog reset),
 *                  BL_ERR_NOAPP and stays if there is none
 *
 * The application enters it by the config write (0x86) of
 * BL_ENTER_CMD BL_ENTER_KEY ~BL_ENTER_KEY (config control CC_BL_ENTER),
 * see ow_loop(): it jumps to
 * BL_START (ijmp, rjmp does not reach it on all) with BL_MAGIC.
 */
#ifndef OWBOOTLOADER_H
#define OWBOOTLOADER_H

#define BL_VERSION	2
#ifdef RWWSRE
/* the boot section, BOOTSZ = 00 */
#define BL_SIZE		2048
#else
/* No boot section (ATtiny84/85): a "virtual" one at the end of the
   flash, SELFPRGEN programmed. Reset always starts at 0: writing page 0
   the bootloader puts its own reset vector there and keeps the one of
   the application on the last page of the flash (BL_DATA), to start
   the application by. A read of page 0 gives it as written. */
#define BL_SIZE		1600
#define BL_DATA		(FLASHEND + 1 - SPM_PAGESIZE)
#endif
#define BL_START	(FLASHEND + 1 - BL_SIZE)
/* in r24:r25 when the application jumps to BL_START */
#define BL_MAGIC	0xB007

#define BL_ENTER_CMD	0xB0
#define BL_ENTER_KEY	0x07

#define BL_CMD_INFO	0xB1
#define BL_CMD_WRITE	0xB2
#define BL_CMD_READ	0xB3
#define BL_CMD_EXIT	0xB4

#define BL_CONFIRM	0xAA
#define BL_OK		0xAA
#define BL_ERR_ADDR	0x01
#define BL_ERR_VERIFY	0x02
#define BL_ERR_NOAPP	0x03
#define BL_ERR_ABORT	0x04
#define BL_ERR_NONE	0x00

#endif
