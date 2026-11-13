/* End to end test of the 1-Wire bootloader in simavr: the application
 * and the bootloader of env <mcu>-bl / <mcu>-boot (atmega168 and
 * atmega168-boot for the ATmega168), driven over the INT0 pin by a
 * 1-Wire master with the standard speed timing of a DS2482 (as in10dfs
 * uses it), cycle by cycle.
 *
 *   bl_sim <mcu> <bootloader.elf> <application.elf>
 *
 * mcu: atmega168, attiny84 or attiny85. See run.sh. Needs the simavr
 * headers (libsimavr-dev).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/sim_interrupts.h>
#include <simavr/avr_flash.h>
#include "../../include/owbootloader.h"

/* what differs, data space addresses */
struct mcu {
	const char* name;
	int flash, page;
	// BL_START; with a boot section (BOOTRST) or a virtual one
	int boot, bootrst;
	char port;
	int ddr, portr, bit;
	// the INT0 flag register and bit
	int ifr, intf0;
	// the power reduction register, the bit of the bootloader's timer
	int prr, prtim;
	uint8_t sig[3];
	/* The application's 1-Wire (interrupt driven) works in simavr: on
	   the ATtiny84/85 simavr 1.6 keeps INT0 low level triggered after
	   the application switched it to the falling edge - it takes the
	   reset pulse for bits. There the jump of the application to the
	   bootloader is made here, see enter(). */
	int app_bus;
};

static const struct mcu mcus[] = {
	{ "atmega168", 16384, 128, 0x3800, 1, 'D', 0x2a, 0x2b, 2, 0x3c, 0, 0x64, 3, { 0x1e, 0x94, 0x06 }, 1 },
	{ "attiny84", 8192, 64, 0x19c0, 0, 'B', 0x37, 0x38, 2, 0x5a, 6, 0x20, 3, { 0x1e, 0x93, 0x0c }, 0 },
	{ "attiny85", 8192, 64, 0x19c0, 0, 'B', 0x37, 0x38, 2, 0x5a, 6, 0x40, 2, { 0x1e, 0x93, 0x0b }, 0 },
};

static const struct mcu* m;
static avr_t* avr;
static avr_irq_t* ow_irq;
static int master_low;
static int level = -1;
/* the time a write 0 and a read slot hold the line, and the recovery
   after a slot, in µs */
static int t_w0 = 60, t_rl = 6, t_msr = 14, t_slot = 70, t_rec = 5;
static int failures;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		failures++; \
		printf("FAIL %s:%d: ", __FILE__, __LINE__); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} \
} while (0)

/* simavr 1.6 does not clear a pending INT0 when the firmware writes a
   1 to its flag (EIFR, GIFR), as the application does after its
   presence pulse: the pulse would be taken as a bit. Done here, as the
   hardware does it, after the handler simavr has for the register (the
   ATtinys need theirs, e.g. to wake up), if any - else the flags
   written as 1 are cleared here as well. */
static avr_int_vector_t* int0;
static avr_io_write_t ifr_old;
static void* ifr_old_param;

static void ifr_write(avr_t* a, avr_io_addr_t addr, uint8_t v, void* param)
{
	(void)param;
	if (ifr_old)
		ifr_old(a, addr, v, ifr_old_param);
	else
		a->data[addr] &= ~v;
	if ((v & (1 << m->intf0)) && int0)
		avr_clear_interrupt(a, int0);
}

/* simavr 1.6 has no self-programming on its ATtiny84/85 cores: SPM does
   nothing, SPMEN stays set. Its flash module of the ATmegas, without
   RWW, with SPMCSR (0x57) of the ATtinys and no interrupt, is what the
   ATtinys have. */
static avr_flash_t tiny_flash = {
	.flags = 0,
	.r_spm = 0x57,
	.spm_pagesize = 64,
	.selfprgen = AVR_IO_REGBIT(0x57, 0),
	.pgers = AVR_IO_REGBIT(0x57, 1),
	.pgwrt = AVR_IO_REGBIT(0x57, 2),
	.blbset = AVR_IO_REGBIT(0x57, 3),
};

static int slave_low(void)
{
	return (avr->data[m->ddr] & (1 << m->bit)) && !(avr->data[m->portr] & (1 << m->bit));
}

static int line(void)
{
	return !(master_low || slave_low());
}

static void update(void)
{
	int l = line();

	if (l != level) {
		level = l;
		avr_raise_irq(ow_irq, l);
	}
}

static void wait_us(double us)
{
	avr_cycle_count_t end = avr->cycle + (avr_cycle_count_t)(us * 8);

	while (avr->cycle < end) {
		int s = avr_run(avr);
		if (s == cpu_Done || s == cpu_Crashed) {
			printf("cpu stopped: %d\n", s);
			exit(2);
		}
		update();
	}
}

static void drive(int low)
{
	master_low = low;
	update();
}

static int ow_reset(void)
{
	int presence;

	drive(1);
	wait_us(560);
	drive(0);
	wait_us(70);
	presence = !line();
	wait_us(410);
	return presence;
}

static void write_bit(int b)
{
	drive(1);
	wait_us(b ? 8 : t_w0);
	drive(0);
	wait_us(t_slot - (b ? 8 : t_w0) + t_rec);
}

static int read_bit(void)
{
	int b;

	drive(1);
	wait_us(t_rl);
	drive(0);
	wait_us(t_msr - t_rl);
	b = line();
	wait_us(t_slot - t_msr + t_rec);
	return b;
}

static void wr(uint8_t v)
{
	for (int i = 0; i < 8; i++, v >>= 1)
		write_bit(v & 1);
}

static uint8_t rd(void)
{
	uint8_t v = 0;

	for (int i = 0; i < 8; i++)
		v |= read_bit() << i;
	return v;
}

static uint16_t crc16(uint16_t crc, uint8_t b)
{
	crc ^= b;
	for (int i = 0; i < 8; i++)
		crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
	return crc;
}

/* reads the inverted CRC16 and checks it against crc */
static int crc_ok(uint16_t crc)
{
	uint16_t got = rd();

	got |= rd() << 8;
	return (uint16_t)~got == crc;
}

static uint8_t rom[8];

static int bl_info(uint8_t* info)
{
	uint16_t crc;

	if (!ow_reset())
		return 0;
	wr(0xcc);
	wr(BL_CMD_INFO);
	crc = crc16(0, BL_CMD_INFO);
	for (int i = 0; i < 11; i++) {
		info[i] = rd();
		crc = crc16(crc, info[i]);
	}
	return crc_ok(crc);
}

static uint8_t bl_write(uint16_t addr, const uint8_t* data)
{
	uint16_t crc;
	uint8_t r;

	if (!ow_reset())
		return 0xfe;
	/* by Match ROM, as in10dfs does */
	wr(0x55);
	for (int i = 0; i < 8; i++)
		wr(rom[i]);
	wr(BL_CMD_WRITE);
	crc = crc16(0, BL_CMD_WRITE);
	wr(addr & 0xff);
	crc = crc16(crc, addr & 0xff);
	wr(addr >> 8);
	crc = crc16(crc, addr >> 8);
	for (int i = 0; i < m->page; i++) {
		wr(data[i]);
		crc = crc16(crc, data[i]);
	}
	if (!crc_ok(crc))
		return 0xfd;
	wr(BL_CONFIRM);
	wait_us(12000);
	for (int i = 0; i < 10; i++) {
		r = rd();
		if (r != 0xff)
			return r;
		wait_us(2000);
	}
	return 0xfc;
}

static int bl_read(uint16_t addr, uint8_t* data)
{
	uint16_t crc;

	if (!ow_reset())
		return 0;
	wr(0xcc);
	wr(BL_CMD_READ);
	crc = crc16(0, BL_CMD_READ);
	wr(addr & 0xff);
	crc = crc16(crc, addr & 0xff);
	wr(addr >> 8);
	crc = crc16(crc, addr >> 8);
	for (int i = 0; i < m->page; i++) {
		data[i] = rd();
		crc = crc16(crc, data[i]);
	}
	return crc_ok(crc);
}

static uint8_t bl_exit(void)
{
	if (!ow_reset())
		return 0xfe;
	wr(0xcc);
	wr(BL_CMD_EXIT);
	return rd();
}

/* a search with one device on the bus */
static int search(uint8_t* out)
{
	if (!ow_reset())
		return 0;
	wr(0xf0);
	memset(out, 0, 8);
	for (int i = 0; i < 64; i++) {
		int b = read_bit(), nb = read_bit();
		if (b == nb)
			return 0;
		out[i >> 3] |= b << (i & 7);
		write_bit(b);
	}
	return 1;
}

/* the application to the bootloader: config write 0xDEAD B0 07, or
   (app_bus 0) its jump made here, with the power reduction it sets */
static void enter(const char* what)
{
	if (m->app_bus) {
		CHECK(ow_reset(), "application presence, %s", what);
		wr(0xcc);
		wr(0x86);
		wr(0xde);
		wr(0xad);
		wr(BL_ENTER_CMD);
		wr(BL_ENTER_KEY);
		for (int i = 4; i < 23; i++)
			wr(0);
	} else {
		avr->data[m->prr] |= 1 << m->prtim;
		avr->data[24] = BL_MAGIC & 0xff;
		avr->data[25] = BL_MAGIC >> 8;
		avr->pc = m->boot;
		avr->state = cpu_Running;
	}
	wait_us(50000);
	CHECK(avr->pc >= m->boot, "in the bootloader, %s, pc %#x", what, avr->pc);
	/* simavr keeps the timers running in spite of the power reduction
	   the application switched on: checked here, on a device the
	   bootloader hangs with it */
	CHECK(!(avr->data[m->prr] & (1 << m->prtim)), "timer off, %s (PRR %#x)", what,
		avr->data[m->prr]);
}

static void load(const char* file)
{
	elf_firmware_t fw = { 0 };

	if (elf_read_firmware(file, &fw) != 0) {
		printf("cannot read %s\n", file);
		exit(2);
	}
	avr_loadcode(avr, fw.flash, fw.flashsize, fw.flashbase);
}

static void test_protocol(const uint8_t* app_flash)
{
	const int ps = m->page;
	const uint16_t test = (m->boot / 2) & ~(ps - 1);
	uint8_t info[11], buf[128], page[128], got[8];

	CHECK(bl_info(info), "INFO crc");
	CHECK(info[0] == 'B' && info[1] == 'L' && info[2] == BL_VERSION,
		"INFO %c%c %d", info[0], info[1], info[2]);
	CHECK(info[3] == ps, "page size %d", info[3]);
	CHECK((info[4] | info[5] << 8) == m->boot, "start %#x", info[4] | info[5] << 8);
	CHECK(memcmp(info + 6, m->sig, 3) == 0,
		"signature %02x %02x %02x", info[6], info[7], info[8]);
	/* as the ISP upload left it */
	CHECK(info[9] == 1, "app valid %d", info[9]);

	/* Read ROM: the default one (erased EEPROM) */
	CHECK(ow_reset(), "presence");
	wr(0x33);
	for (int i = 0; i < 8; i++)
		rom[i] = rd();
	CHECK(rom[0] == 0x29, "family %#x", rom[0]);
	CHECK(search(got) && memcmp(got, rom, 8) == 0, "search");

	/* a page: written, verified by the device and read back */
	for (int i = 0; i < ps; i++)
		page[i] = i * 7 + 3;
	CHECK(bl_write(test, page) == BL_OK, "write");
	CHECK(bl_read(test, buf) && memcmp(buf, page, ps) == 0, "read back");
	CHECK(bl_write(test + 1, page) == BL_ERR_ADDR, "unaligned");
	CHECK(bl_write(m->boot, page) == BL_ERR_ADDR, "boot section");
	CHECK(bl_write(m->flash - ps, page) == BL_ERR_ADDR, "last page");
	CHECK(bl_read(m->boot, buf) && buf[0] == avr->flash[m->boot], "read boot");

	/* an update as in10dfs does it: page 0 erased first, written last;
	   in between there is no application */
	memset(page, 0xff, ps);
	CHECK(bl_write(0, page) == BL_OK, "erase page 0");
	CHECK(bl_read(0, buf) && memcmp(buf, page, ps) == 0, "page 0 reads erased");
	CHECK(bl_info(info) && info[9] == 0, "no app");
	CHECK(bl_exit() == BL_ERR_NOAPP, "exit without app");
	/* broken off here: a reset goes to the bootloader, it stays */
	avr_reset(avr);
	wait_us(20000);
	CHECK(avr->pc >= m->boot, "reset without app: in the bootloader, pc %#x", avr->pc);
	CHECK(bl_info(info) && info[9] == 0, "reset without app: answers");

	for (int a = ps; a < test + ps; a += ps)
		if (memcmp(app_flash + a, avr->flash + a, ps) != 0)
			CHECK(bl_write(a, app_flash + a) == BL_OK, "write %#x", a);
	CHECK(bl_write(0, app_flash) == BL_OK, "page 0");
	CHECK(bl_read(0, buf) && memcmp(buf, app_flash, ps) == 0, "page 0 reads as written");
	if (m->bootrst) {
		CHECK(memcmp(avr->flash, app_flash, test + ps) == 0, "flash as the image");
	} else {
		/* the reset vector to the bootloader, the application's on the
		   last page */
		uint16_t v = avr->flash[0] | avr->flash[1] << 8;
		uint16_t data = m->flash - ps;
		CHECK(v == (0xc000 | ((m->boot / 2 - 1) & 0xfff)), "reset vector %#06x", v);
		CHECK(memcmp(avr->flash + data, app_flash, 2) == 0, "the application's vector kept");
		CHECK(memcmp(avr->flash + 2, app_flash + 2, test + ps - 2) == 0, "flash as the image");
	}
	CHECK(bl_info(info) && info[9] == 1, "app valid again");
}

int main(int argc, char** argv)
{
	uint8_t info[11];
	static uint8_t app_flash[16384];

	if (argc < 4) {
		printf("usage: %s mcu bootloader.elf application.elf\n", argv[0]);
		return 2;
	}
	for (size_t i = 0; i < sizeof(mcus) / sizeof(mcus[0]); i++)
		if (strcmp(argv[1], mcus[i].name) == 0)
			m = &mcus[i];
	if (!m) {
		printf("unknown mcu %s\n", argv[1]);
		return 2;
	}
	avr = avr_make_mcu_by_name(m->name);
	avr_init(avr);
	if (!m->bootrst)
		avr_flash_init(avr, &tiny_flash);
	avr->frequency = 8000000;
	memset(avr->flash, 0xff, m->flash);
	load(argv[2]);
	load(argv[3]);
	memcpy(app_flash, avr->flash, m->flash);
	/* BOOTRST, or the reset vector of the application */
	avr->reset_pc = m->bootrst ? m->boot : 0;
	avr->pc = avr->reset_pc;
	ow_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(m->port), m->bit);
	for (int i = 0; i < avr->interrupts.vector_count; i++)
		if (avr->interrupts.vector[i]->vector == 1)
			int0 = avr->interrupts.vector[i];
	/* in place of simavr's handler, which ifr_write() calls */
	ifr_old = avr->io[AVR_DATA_TO_IO(m->ifr)].w.c;
	ifr_old_param = avr->io[AVR_DATA_TO_IO(m->ifr)].w.param;
	avr->io[AVR_DATA_TO_IO(m->ifr)].w.c = ifr_write;
	avr->io[AVR_DATA_TO_IO(m->ifr)].w.param = NULL;
	update();
	printf("%s\n", m->name);

	/* no magic: on to the application */
	wait_us(1000);
	CHECK(avr->pc < m->boot, "started the application, pc %#x", avr->pc);
	wait_us(300000);
	enter("from the application");

	test_protocol(app_flash);

	/* and with the tighter timing of a fast master */
	t_rec = 2;
	t_msr = 12;
	CHECK(bl_info(info), "INFO, fast master");
	t_rec = 5;
	t_msr = 14;

	/* back to the (new) application by a watchdog reset */
	CHECK(bl_exit() == BL_OK, "exit");
	wait_us(100000);
	CHECK(avr->pc < m->boot, "application again, pc %#x", avr->pc);
	wait_us(300000);

	/* and the next update: in from the application once more */
	enter("again");
	CHECK(bl_info(info) && info[9] == 1, "INFO again");
	CHECK(bl_exit() == BL_OK, "exit again");
	wait_us(400000);
	CHECK(avr->pc < m->boot, "application at last, pc %#x", avr->pc);
	if (m->app_bus) {
		CHECK(ow_reset(), "application presence at last");
		/* the application answers its own commands: read config (0x85) */
		wr(0xcc);
		wr(0x85);
		(void)rd();
	}

	printf("%s: %d failures\n", failures ? "FAILED" : "PASSED", failures);
	return failures != 0;
}
