/* owSlave 1-Wire bootloader, protocol see include/owbootloader.h.
 * ATmega88 and ATmega168 (boot section), ATtiny84 and ATtiny85 (virtual boot
 * partition, see BL_DATA).
 *
 * A polled 1-Wire slave: no interrupts, a timer running at 1 tick per
 * microsecond (8 MHz / 8) is the clock for the slot timing - Timer1
 * where it has 16 bit, on the ATtiny85 Timer0 extended to 16 bit in
 * software. Each bit starts with the falling edge of the master (the
 * edge flag of INT0, polled): a
 * write slot is sampled T_SAMPLE after it, a read slot answered with a
 * 0 by holding the line low until T_READ0. The work on a byte (CRC,
 * storing) is done in the rest of the slot of its last bit, so it
 * never delays the next edge. A low longer than T_RESET is a reset:
 * presence pulse and back to the ROM command, with the stack dropped.
 *
 * The flash is programmed with the master waiting (BL_CMD_WRITE), the
 * bus is not watched meanwhile.
 */
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include "../include/owbootloader.h"

#if F_CPU != 8000000UL && F_CPU != 16000000UL
#error "the slot timing needs F_CPU of 8 or 16 MHz"
#endif

/* the 1-Wire line, INT0 as for the application */
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny84A__) || defined(__AVR_ATtiny85__)
#define OW_DDR		DDRB
#define OW_PORT		PORTB
#define OW_PIN		PINB
#define OW_BIT		_BV(PB2)
#else
#define OW_DDR		DDRD
#define OW_PORT		PORTD
#define OW_PIN		PIND
#define OW_BIT		_BV(PD2)
#endif
#define OW_LOW()	(OW_DDR |= OW_BIT)
#define OW_RELEASE()	(OW_DDR &= ~OW_BIT)
#define OW_IS_HIGH()	(OW_PIN & OW_BIT)
/* The falling edge, latched by the hardware (INT0, its interrupt not
   enabled): seen however short the high before it was, and if the
   bootloader looks a little late. */
#ifdef GIFR
#define OW_EDGE		GIFR
#define ow_edge_init()	do { GIMSK = 0; MCUCR = _BV(ISC01); } while (0)
#else
#define OW_EDGE		EIFR
#define ow_edge_init()	do { EIMSK = 0; EICRA = _BV(ISC01); } while (0)
#endif
#define OW_FELL()	(OW_EDGE & _BV(INTF0))
#define OW_FELL_CLEAR()	(OW_EDGE = _BV(INTF0))

/* timer ticks, prescaler 8 */
#define US(us)		((uint16_t)((us) * (F_CPU / 8000000UL)))
#define T_RESET		US(400)
#define T_SAMPLE	US(25)
/* A 0 sent: the DS2482 samples 14 us after its fall, 30 us were too
   short on a long line with several devices. Not the 50 us of the
   application (OWT_WRITE) either: counted by the RC oscillator, which
   may be 10 % slow, and the next slot starts 69 us after this one -
   the work between two bytes did not fit in what was left, the slot
   was missed. */
#define T_READ0		US(40)
/* Ringing of a long line: after an edge the line may cross the input
   threshold again for a moment. A fall starts a slot only if the line
   is low again at any time from T_STABLE to 2 * T_STABLE after it (a
   bump may cover a part), below the master's shortest low (a write 1
   or read slot, 6 us and more). Falls up to T_SAMPLE after it belong
   to the same slot (the rise after a short low has settled then), as
   for the application, whose interrupt takes about that long.
   The high between two slots is not looked at: the DS2482 keeps the
   line low for 64 us in a write 0 slot and starts the next one 5.25 us
   later, of which the rise of a loaded line takes its part. A high
   that had to be seen for 2..3 us was missed there, and with it the
   next slot (all bits shifted: no answer, or a wrong CRC). */
#define T_STABLE	US(2)
/* the line high this long: the master is between two bytes (the
   DS2482 is given each by I2C), not between two bits (61 us). Counted
   in 8 bit. */
#define T_IDLE		US(120)
#define T_PRESENCE_WAIT	US(30)
#define T_PRESENCE	US(120)

/* idle time before going back to the application, in timer overflows
   (65.5 ms at 8 MHz): about a minute */
#define IDLE_MAX	((uint16_t)(60000UL * 1000UL / (65536UL / (F_CPU / 8000000UL))))

#ifndef DEFAULT_OWID_ADR
#define DEFAULT_OWID_ADR 0x5
#endif
#ifndef DEFAULT_OWID_BUS
#define DEFAULT_OWID_BUS 0x1
#endif

/* no .data, no cleared .bss (see start.S): all set in bl_main(), and
   in .noinit, so that the code to clear .bss is not linked in */
#define NOINIT __attribute__((section(".noinit")))
static uint8_t rom[8] NOINIT;
static uint8_t page[SPM_PAGESIZE] NOINIT;
static uint16_t t_fall NOINIT;
static uint16_t idle NOINIT;
static uint8_t last_result NOINIT;
/* the CRC16 of a command, see crc_bit() */
static uint16_t crc NOINIT;

void bl_main(uint16_t magic) __attribute__((noreturn, used));
/* not inlined: presence() jumps to it with a fresh stack */
void bl_bus(void) __attribute__((noreturn, noinline, used));

#ifdef TCNT1H
/* 16 bit Timer1 */
static void timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = _BV(CS11);
	TIMSK1 = 0;
	TIFR1 = _BV(TOV1);
}

static inline uint16_t now(void)
{
	return TCNT1;
}

/* a timer overflow since the last call */
static inline uint8_t overflow(void)
{
	if (!(TIFR1 & _BV(TOV1)))
		return 0;
	TIFR1 = _BV(TOV1);
	return 1;
}
#else
/* 8 bit Timer0, the high byte counted here: now() has to be called at
   least every 256 ticks, as the wait loops do */
static uint8_t t_hi NOINIT;
static uint8_t t_wrap NOINIT;

static void timer_init(void)
{
	TCCR0A = 0;
	TCCR0B = _BV(CS01);
	TIMSK = 0;
	TIFR = _BV(TOV0);
	t_hi = 0;
	t_wrap = 0;
}

static uint16_t now(void)
{
	uint8_t lo = TCNT0;

	if (TIFR & _BV(TOV0)) {
		TIFR = _BV(TOV0);
		/* the overflow may have come after the read */
		lo = TCNT0;
		if (++t_hi == 0)
			t_wrap = 1;
	}
	return ((uint16_t)t_hi << 8) | lo;
}

static uint8_t overflow(void)
{
	now();
	if (!t_wrap)
		return 0;
	t_wrap = 0;
	return 1;
}
#endif

/* the low byte of now(), for the short checks of a few us */
#ifdef TCNT1H
#define now8()	TCNT1L
#else
#define now8()	TCNT0
#endif

#ifdef BL_DATA
/* rjmp: 12 bit relative, wrapping around on 8 KB */
#define WORDS_MASK	((FLASHEND + 1) / 2 - 1)

static uint16_t rjmp_to(uint16_t at, uint16_t target)
{
	return 0xc000 | ((target - at - 1) & 0x0fff);
}

#define BL_VECTOR	rjmp_to(0, BL_START / 2)

/* The word address the application starts at, 0 if there is none: by
   its reset vector, kept on BL_DATA once the bootloader wrote page 0,
   or still at 0 as an ISP upload left it */
static uint16_t app_entry(void)
{
	uint16_t v = pgm_read_word(0);

	if (v == BL_VECTOR)
		v = pgm_read_word(BL_DATA);
	if ((v & 0xf000) != 0xc000)
		return 0;
	v = (v + 1) & WORDS_MASK;
	return v < BL_START / 2 ? v : 0;
}

static uint8_t app_valid(void)
{
	return app_entry() != 0;
}

static void __attribute__((noreturn)) app_start(void)
{
	asm volatile("ijmp" :: "z" (app_entry()));
	for (;;)
		;
}

/* the flash as the master wrote it: page 0 with its reset vector */
static uint8_t flash_byte(uint16_t addr)
{
	if (addr < 2 && pgm_read_word(0) == BL_VECTOR)
		addr += BL_DATA;
	return pgm_read_byte(addr);
}
#else
static uint8_t app_valid(void)
{
	return pgm_read_word(0) != 0xffff;
}

static void __attribute__((noreturn)) app_start(void)
{
	/* no jmp on the 8 KB parts (ATmega88) */
	asm volatile("ijmp" :: "z" (0));
	for (;;)
		;
}

#define flash_byte(addr) pgm_read_byte(addr)
#endif

/* To the application by a watchdog reset: it starts from a clean
   state. r24:r25 cleared, the reset must not see BL_MAGIC. Stays when
   there is no application. */
static void leave(void)
{
	if (!app_valid()) {
		idle = 0;
		return;
	}
	wdt_enable(WDTO_15MS);
	asm volatile(
		"clr r24\n\t"
		"clr r25\n"
		"1:\trjmp 1b\n\t" ::: "r24", "r25");
}

static void delay_from(uint16_t t, uint16_t ticks)
{
	while ((uint16_t)(now() - t) < ticks)
		;
}

static void __attribute__((noreturn)) presence(void)
{
	uint16_t t;

	while (!OW_IS_HIGH())
		now();
	t = now();
	delay_from(t, T_PRESENCE_WAIT);
	OW_LOW();
	t = now();
	delay_from(t, T_PRESENCE);
	OW_RELEASE();
	/* other devices may hold their presence pulse longer: that is
	   not the next reset, nor a slot */
	t_fall = now();
	OW_FELL_CLEAR();
	/* whatever was going on is over: on with an empty stack. In
	   assembler: a frame set up in this function (the compiler may
	   have inlined callers' work into it) must not be left over. */
	asm volatile(
		"ldi r28, lo8(%0)\n\t"
		"ldi r29, hi8(%0)\n\t"
		"out __SP_H__, r29\n\t"
		"out __SP_L__, r28\n\t"
		"rjmp bl_bus\n\t" :: "i" (RAMEND));
	for (;;)
		;
}

/* The start of the next slot: a fall with the line low again from
   T_STABLE to twice that after it. A reset if the line stays low
   since the last one; leaves when idle too long. */
static void wait_fall(void)
{
	uint8_t d;

	for (;;) {
		while (!OW_FELL()) {
			if (OW_IS_HIGH()) {
				if (overflow() && ++idle == IDLE_MAX)
					leave();
			/* low: not by a fall just now, after the look above */
			} else if (!OW_FELL() && (uint16_t)(now() - t_fall) >= T_RESET) {
				presence();
			}
		}
		/* first: a fall while checking this one is kept */
		OW_FELL_CLEAR();
		t_fall = now();
		do {
			d = now8() - (uint8_t)t_fall;
			if (d >= T_STABLE && !OW_IS_HIGH())
				return;
		} while (d <= 2 * T_STABLE);
	}
}

/* to the sample point of the slot; what fell until then was its own */
static void wait_sample(void)
{
	delay_from(t_fall, T_SAMPLE);
	OW_FELL_CLEAR();
}

static uint8_t rx_bit(void)
{
	wait_fall();
	wait_sample();
	return OW_IS_HIGH() ? 1 : 0;
}

static void tx_bit(uint8_t b)
{
	if (b) {
		wait_fall();
		wait_sample();
		return;
	}
	/* A 0: pulled low within a few cycles of the master's fall, as the
	   application's zero polling does, not after the checks of
	   wait_fall(). The master releases the line after a few us, with
	   its active pullup on a late pull fights it and the 0 may be read
	   as a 1. A reset instead of the read slot ends this as well, it
	   is seen by the next wait_fall(). */
	while (!OW_FELL())
		;
	OW_LOW();
	t_fall = now();
	delay_from(t_fall, T_READ0);
	OW_FELL_CLEAR();
	OW_RELEASE();
}

/* The CRC16 (x^16 + x^15 + x^2 + 1, as _crc16_update()) of all bits
   received and sent, a bit at a time in its slot: after the last bit
   of a byte there is no work left before the next slot. */
static void crc_bit(uint8_t b)
{
	b ^= crc;
	crc >>= 1;
	if (b & 1)
		crc ^= 0xA001;
}

static uint8_t rx(void)
{
	uint8_t i, v, b = 0;

	for (i = 0; i < 8; i++) {
		v = rx_bit();
		crc_bit(v);
		b >>= 1;
		if (v)
			b |= 0x80;
	}
	return b;
}

static void tx(uint8_t b)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
		tx_bit(b & 1);
		crc_bit(b);
		b >>= 1;
	}
}

/* sends ~crc, low byte first */
static void tx_crc(void)
{
	uint16_t c = ~crc;

	tx(c & 0xff);
	tx(c >> 8);
}

static uint8_t crc8(const uint8_t* p, uint8_t len)
{
	uint8_t c = 0;

	while (len--)
		c = _crc_ibutton_update(c, *p++);
	return c;
}

/* the ROM of the application, see cfg_init() there */
static void rom_init(void)
{
	eeprom_read_block(rom, (const void*)0, 8);
	if (rom[0] != 0x29 || crc8(rom, 7) != rom[7]) {
		rom[0] = 0x29;
		rom[1] = DEFAULT_OWID_ADR;
		rom[2] = DEFAULT_OWID_BUS;
		rom[3] = ~rom[1];
		rom[4] = ~rom[2];
		rom[5] = 0x66;
		rom[6] = 0x77;
		rom[7] = crc8(rom, 7);
	}
}

static void page_erase(uint16_t addr)
{
	boot_page_erase(addr);
	boot_spm_busy_wait();
}

/* writes the page buffer filled before */
static void page_write(uint16_t addr)
{
	boot_page_write(addr);
	boot_spm_busy_wait();
#ifdef RWWSRE
	boot_rww_enable();
#endif
}

static uint8_t program(uint16_t addr)
{
	uint16_t i;

	if ((addr & (SPM_PAGESIZE - 1)) != 0 || addr >= BL_START)
		return BL_ERR_ADDR;
	eeprom_busy_wait();
#ifdef BL_DATA
	/* page 0: no application until its reset vector is kept, the
	   reset to the bootloader meanwhile */
	uint16_t vector = page[0] | (page[1] << 8);
	if (addr == 0) {
		page_erase(BL_DATA);
		page[0] = BL_VECTOR & 0xff;
		page[1] = BL_VECTOR >> 8;
	}
#endif
	page_erase(addr);
	for (i = 0; i < SPM_PAGESIZE; i += 2)
		boot_page_fill(addr + i, page[i] | (page[i + 1] << 8));
	page_write(addr);
	for (i = 0; i < SPM_PAGESIZE; i++)
		if (pgm_read_byte(addr + i) != page[i])
			return BL_ERR_VERIFY;
#ifdef BL_DATA
	if (addr == 0 && vector != 0xffff) {
		/* the rest of the page buffer is 0xff after the write */
		boot_page_fill(BL_DATA, vector);
		page_write(BL_DATA);
		if (pgm_read_word(BL_DATA) != vector)
			return BL_ERR_VERIFY;
	}
#endif
	return BL_OK;
}

/* 0 when deselected by the search */
static uint8_t search(void)
{
	uint8_t i, b;

	for (i = 0; i < 64; i++) {
		b = (rom[i >> 3] >> (i & 7)) & 1;
		tx_bit(b);
		tx_bit(!b);
		if (rx_bit() != b)
			return 0;
	}
	return 1;
}

static void command(void)
{
	uint8_t cmd, i;
	uint16_t addr;

	idle = 0;
	crc = 0;
	cmd = rx();
	switch (cmd) {
	case BL_CMD_INFO: {
		const uint8_t info[] = {
			'B', 'L', BL_VERSION, SPM_PAGESIZE & 0xff,
			BL_START & 0xff, BL_START >> 8,
			SIGNATURE_0, SIGNATURE_1, SIGNATURE_2,
			app_valid(), last_result
		};
		for (i = 0; i < sizeof(info); i++)
			tx(info[i]);
		tx_crc();
		break;
	}
	case BL_CMD_WRITE:
	case BL_CMD_READ:
		addr = rx();
		addr |= (uint16_t)rx() << 8;
		if (cmd == BL_CMD_READ) {
			for (i = 0; i < SPM_PAGESIZE; i++)
				tx(flash_byte(addr + i));
			tx_crc();
			break;
		}
		for (i = 0; i < SPM_PAGESIZE; i++)
			page[i] = rx();
		tx_crc();
		if (rx() != BL_CONFIRM) {
			last_result = BL_ERR_ABORT;
			break;
		}
		last_result = program(addr);
		/* the master may be reading already (0xff, page 0 of the
		   ATtinys takes 18 ms): the result from the start of a byte,
		   and no slot of those taken for the next one */
		i = now8();
		do {
			if (!OW_IS_HIGH())
				i = now8();
		} while ((uint8_t)(now8() - i) < T_IDLE);
		OW_FELL_CLEAR();
		tx(last_result);
		break;
	case BL_CMD_EXIT:
		if (!app_valid()) {
			tx(BL_ERR_NOAPP);
			break;
		}
		/* its last bit is a 1: the master has sampled it */
		tx(BL_OK);
		leave();
		break;
	default:
		break;
	}
}

/* after a reset and presence pulse: the ROM command, then the command
   if selected */
void bl_bus(void)
{
	uint8_t b = rx();

	switch (b) {
	case 0xCC:
		break;
	case 0x55:
		for (b = 0; b < 8; b++)
			if (rx() != rom[b])
				goto deselected;
		break;
	case 0x33:
		for (b = 0; b < 8; b++)
			tx(rom[b]);
		break;
	case 0xF0:
		search();
		goto deselected;
	default:
		goto deselected;
	}
	command();
deselected:
	/* until the next reset */
	for (;;)
		rx_bit();
}

void bl_main(uint16_t magic)
{
	if (magic != BL_MAGIC && app_valid())
		app_start();
	/* the watchdog of the application must not reset us */
	MCUSR = 0;
	wdt_disable();
	/* entered from the application, not by a reset: the timer of all
	   the slot timing may be switched off (power reduction, as it does)
	   or its prescaler held; the clock divided (CKDIV8) */
	PRR = 0;
	GTCCR = 0;
	clock_prescale_set(clock_div_1);
	OW_RELEASE();
	OW_PORT &= ~OW_BIT;
	timer_init();
	ow_edge_init();
	OW_FELL_CLEAR();
	idle = 0;
	last_result = BL_ERR_NONE;
	rom_init();
	t_fall = now();
	/* until the first reset */
	for (;;)
		rx_bit();
}
