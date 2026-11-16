#!/bin/sh
# Builds the application and the bootloader for the ATmega168, ATtiny84
# and ATtiny85 (or the ones given: run.sh attiny84) and runs the simavr
# test (bl_sim.c) against them. Needs the simavr headers: libsimavr-dev,
# or SIMAVR_INC set to a directory holding simavr/*.h.
set -e
rc=0
cd "$(dirname "$0")/../.."
build=${BUILD_DIR:-.pio/build}
mcus=${*:-atmega168 attiny84 attiny85}
mkdir -p "$build"
cc -O2 ${SIMAVR_INC:+-I$SIMAVR_INC} -o "$build/bl_sim" test/bootloader/bl_sim.c \
	-l:libsimavr.so.2 -lelf
for mcu in $mcus; do
	app=$mcu-bl
	[ "$mcu" = atmega168 ] && app=atmega168
	PLATFORMIO_BUILD_DIR=$build pio run -e $app -e $mcu-boot
	"$build/bl_sim" $mcu "$build/$mcu-boot/firmware.elf" "$build/$app/firmware.elf" || rc=1
done
exit $rc
