OW_FAMILY?=A8

FD_DEFS=-DOW_FAMILY=0x$(OW_FAMILY) -DFD_VERSION=1 -DFD_SERIAL=3

ifeq ($(OW_FAMILY), A8)
TARGET=atmega88
endif
ifeq ($(OW_FAMILY), A3)
TARGET=attiny85
endif
ifeq ($(OW_FAMILY), A2)
TARGET=attiny85
#TARGET = attiny25
#TARGET=atmega168
endif

#AVR_PATH="C:\Program Files (x86)\Arduino\hardware\tools\avr\bin"
# ISP=usbasp -P usb -B 2000 -i200
# ISP=stk500v2 PORT=/dev/ttyUSB0
ISP=stk500v2 -P COM9 -B 20 -i10
#ISP=stk500v1 -P COM6 -b 57600 -B 5000 -i1000
#ISP=arduino  -P COM3 -b 19200 -B 5000 -i1000
CLOCK=8000000UL

CC = $(AVR_PATH)avr-gcc
OBJCOPY        = $(AVR_PATH)avr-objcopy
OBJDUMP        = $(AVR_PATH)avr-objdump
OBJSIZE        = $(AVR_PATH)avr-size

PRG         = owfam_$(OW_FAMILY)
OBJ         = main.o irsend.o wiring.o
OPTIMIZE    = -Os
DEFS        = -I. -DF_CPU=$(CLOCK) -DTARGET=$(TARGET) $(FD_DEFS)
LIBS        =
UART = 0

ifeq ($(TARGET), atmega8)
HFUSE=0xd9
LFUSE=0xe4
endif

ifeq ($(TARGET), atmega88)
HFUSE=0xdf
LFUSE=0xf2
PROG_TARGET=atmega88p
else
PROG_TARGET=$(TARGET)
endif

ifeq ($(TARGET), attiny25)
# HFUSE=0xdf LFUSE=0xb2 ???
LFUSE=0xb2
HFUSE=0xdf
endif
ifeq ($(TARGET), attiny85)
# HFUSE=0xdf LFUSE=0xb2 ???
LFUSE=0xd2
HFUSE=0xdf
DEFS+=-DIR_SENDER
endif

ifeq ($(UART), 1)
DEFS += -DBAUD=9600 -DHAVE_UART -DUART_RX_BUFFER_SIZE=16 -DUART_TX_BUFFER_SIZE=16
OBJ += uart.o uart_addon.o
#OBJ += printf.o cli.o
endif

all: clean_main $(PRG).elf $(PRG).bin $(PRG).hex text disasm

clean_main:
	rm -f main.o $(PRG).elf $(PRG).bin

help:
	echo "Usage: make                same as make help"
	@echo "       make help           same as make"
	@echo "       make main.hex       create main.hex"
	@echo "       make clean          remove redundant data"
	@echo "       make disasm         disasm main"
	@echo "       make flash          upload main.hex into flash"
	@echo "       make fuses          program fuses"
	@echo "       make avrdude        test avrdude"
	@echo "Current values:"
	@echo "       TARGET=${TARGET}"
	@echo "       LFUSE=${LFUSE}"
	@echo "       HFUSE=${HFUSE}"
	@echo "       CLOCK=${CLOCK}"
	@echo "       ISP=${ISP}"
	@echo "       PORT=${PORT}"
	@echo "       gcc -g -Wall $(OPTIMIZE) -mmcu=$(TARGET) $(DEFS) $(LIBS) $(LDFLAGS)"
	

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

$(PRG).elf: $(OBJ)
	echo $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(OBJSIZE) --mcu=$(TARGET) $@

#	$(CC) -o $(PRG).elf -Wl,-Map,$(PRG).map $@ $^

clean:
	@echo cleaning
	rm -f *.o $(PRG).elf $(PRG).bin *.map *.lst *.map
	rm -f $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.size: %.elf
	$(OBJSIZE) --mcu=$(TARGET) $<

# Rules for building the .text rom images
text: hex bin # srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

$(PRG).hex:	$(PRG).elf
	$(AVR_PATH)avr-objcopy -j .text -j .data -O ihex $(PRG).elf $(PRG).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

disasm:	$(PRG).elf
	$(AVR_PATH)avr-objdump -d $(PRG).elf > $(PRG).asm

# Rules for building the .eeprom rom images
eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

install: $(PRG).hex
	$(AVR_PATH)avrdude -c ${ISP} -p ${PROG_TARGET} -U flash:w:$(PRG).hex

fusesr:
	$(AVR_PATH)avrdude -c ${ISP} -p ${TARGET} -F -u -U hfuse:r:-:h -U lfuse:r:-:h

fusel:
	$(AVR_PATH)avrdude -c ${ISP} -p ${TARGET} -P ${PORT} -u -U lfuse:w:$(LFUSE):m

fuses:
	$(AVR_PATH)avrdude -c ${ISP} -p ${PROG_TARGET} -u -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m

test:
	$(AVR_PATH)avrdude -F -c  ${ISP} -p ${PROG_TARGET} -vvv

dump:
	$(AVR_PATH)avrdude -c ${ISP} -p ${PROG_TARGET} -U eeprom:r:-:h



# Fuse atmega8 high byte HFUSE:
# 0x99 = 1 0 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
# 0xD9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (full output swing)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ WDTON (WDT not always on)
#        +-------------------- RSTDISBL (reset pin is enabled)
# Fuse atmega8 low byte LFUSE:
# 0xE4 = 1 1 1 0   0 1 0 0
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (internal >8M crystal)
#        | |  +--------------- SUT 1..0 (slow rising, BOD disabled)
#        | +------------------ BODEN (BrownOut Detector enabled)
#        +-------------------- BODLEVEL (2.7V)
#
# Fuse atmega48 / ATtiny25 high byte hfuse:
# 0xdf = 1 1 0 1   1 1 1 1     factory setting
#        ^ ^ ^ ^   ^ \-+-/
#        | | | |   |   +------ BODLEVEL (Brown out disabled)
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (WDT not always on)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ DWEN (debug wire is disabled)
#        +-------------------- RSTDISBL (reset pin is enabled)
# 0xdd = ext.reset, no DW, SPI, no watchdog, no save eeprom, BOD 2.7V
# Fuse atmega48 / ATtiny25 low byte lfuse:
# 0x62 = 0 1 1 0   0 0 1 0     factory setting
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (internal 8Mhz Oszillator)
#        | |  +--------------- SUT 1..0 (start-up time)
#        | +------------------ CKOUT (no clock output)
#        +-------------------- CKDIV8 (divide clock by 8)
# 0xd2 = divide/1,no clock output,fast raising power,internal 8 Mhz
# 0xdc = divide/1,no clock output,fast raising power,low Pw Oszil. 3..8 Mhz
# 0xe0 = divide/1,no clock output,fast raising power,external Oszil.
# 0xff = divide/1,no clock output,slow raising power,low Pw Oszil 8..  Mhz
