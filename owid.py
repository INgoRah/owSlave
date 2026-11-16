# The default ROM of the device (DEFAULT_OWID_ADR, DEFAULT_OWID_BUS) for
# the firmware and the bootloader: taken when the EEPROM holds none. From
# the environment, eeprom.py --upload sets it per device:
#   OWID_ADR=7 OWID_BUS=2 pio run -e attiny84
Import("env")
import os

adr = os.environ.get("OWID_ADR", "0x5")
bus = os.environ.get("OWID_BUS", "0x1")
env.Append(CPPDEFINES=[("DEFAULT_OWID_ADR", adr), ("DEFAULT_OWID_BUS", bus)])
