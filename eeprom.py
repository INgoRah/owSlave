import subprocess
import argparse

#AVRDUDE = "%USERPROFILE%\\.platformio\\packages\\tool-avrdude\\avrdude.exe"
AVRDUDE = "avrdude"
PROG = "-c stk500v2 -P /dev/ttyACM"
#attiny84
dev =  "attiny84"

db = {}
#     |ID  |bus                     |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7  |FEA |OFF |MAJ |MIN |TYP |   OFF   |   FACT  |S   |IO  |TH  |TL  |TYP |THR |DIM |TM1 |TM2 |SWA0|SWA1|SWA2|SWA3|SWA4|SWA5|SWA6.
# 00   01   02   03   04   05   06   07   08   09   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44   45   46   47   48   49   50  
# OWID 1    2    3    4    5    6    7
# cfg_info                           0    1    2    3    4    5    6    7    8    9    10                                           19                        
# cfg_info2                                                                                                                                                  0    1    2    3    4    5    6    7 
# cfg_custom1                                                                                                                                                                                         0    1    2    3    4    5 ...
#       |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7  |FEA |OFF |MAJ |MIN |TYP |   OFF   |   FACT  |S   |IO  |TH  |TL  |TYP |THR |DIMD|DIMU|DIF |TM1 |TM2 |SWA0|SWA1|SWA2|SWA3|SWA4|SWA5|SWA6.
#    06   07   08   09   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44   45   46   47   48   49   50  
db['0.2']  = "0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x20,0xff,0x23,0x21,0x21,0x21,0x21,0x10,0x10,0x00,0xFD,0x01,0x01,0x06,0x0B,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10,0x01,0x90,0x10,0x00,0x0A,0x04,0x04,0x00,0xff,0xff,0xff,0xff,0xff,0x21,0xff"
db['0.5']  = "0xff,0xff,0xff,0xff,0xff,0x01,0xff,0xff,0xff,0x02,0x21,0x21,0x10,0x10,0x10,0x10,0x10,0x05,0xFE,0x06,0x02,0x02,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10"
db['0.7']  = "0xff,0xff,0xff,0xff,0xff,0x01,0x02,0x01,0x02,0xff,0x21,0x21,0x11,0x10,0x10,0x10,0x10,0x10,0xFE,0x01,0x01,0x06,0x0B,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10"
# EG
db['1.1']  = "0xff,0xff,0xff,0xff,0xff,0x03,0xff,0xff,0xff,0x02,0x21,0x21,0x21,0x07,0x10,0x10,0x10,0x10,0xFE,0x06,0x02,0x02,0x01,0x02,0x0c,0x22,0xe0"
db['1.7']  = "0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0xFE,0x01,0x06,0x0B,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10"
db['1.12'] = "0xff,0xff,0xff,0xff,0xff,0x03,0xff,0x02,0xff,0xff,0x21,0x21,0x21,0x07,0x10,0x10,0x10,0x10,0xFE,0x06,0x02,0x02,0x01,0x02,0x0c,0x22,0xe0,0xff,0xff,0xff,0xff"
# OG
#       |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7  |FEA |OFF |MAJ |MIN |TYP |   OFF   |   FACT  |S   |IO  |TH  |TL  |TYP |THR |DIMD|DIMU|DIF |TM1 |TM2 |SWA0|SWA1|SWA2|SWA3|SWA4|SWA5|SWA6.
#    06   07   08   09   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44   45   46   47   48   49   50  
db['2.7']  = "0xff,0xff,0xff,0x20,0xff,0xff,0xff,0xff,0xff,0xff,0x23,0x02,0x00,0x00,0xff,0xff,0xff,0xff,0xFF,0xff,0x02,0x02,0x02,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10,0x01,0x90,0x10,0x00,0x0A,0x04,0x04,0xff,0x21,0xff,0xff,0xff,0xff,0xff,0xff"
db['2.2']  = "0xff,0xff,0xff,0x01,0xff,0xff,0xff,0xff,0xff,0x21,0x00,0x10,0x10,0x00,0x00,0x00,0x00,0xff,0xff,0x02,0x02,0x01,0x02,0x0c,0x22,0xe0"
db['2.4']  = "0xff,0xff,0xff,0xff,0xff,0xff,0x01,0xff,0x02,0xff,0x21,0x21,0x21,0x00,0x10,0x00,0x10,0x00,0xff,0xff,0x02,0x02,0x01,0x00,0x00,0x00,0x01"
# Out
db['3.1']  = "0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x02,0x00,0xff,0xff,0xff,0xff,0xFF,0xff,0x02,0x02,0x02,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10,0x01,0x90,0x10,0x00,0x0A,0x04,0x04,0xff,0x21,0xff,0xff,0xff,0xff,0xff,0xff"

devices = {
	"0.2": "atmega88pa",
	"2.7": "t85",
	"3.1": "t85",
}

# Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
def crc8(data):
	crc = 0
	# Iterate bytes in data
	for byte in data:
		# Iterate bits in byte
		for _ in range(0, 8):
			mix = (crc ^ byte) & 0x01
			crc >>= 1
			if mix:
				crc ^= 0x8C
			byte >>= 1

	return crc

#  x^16 + x^15 + x^2 + 1 (0xa001)
def crc16(data):
	crc = 0
	# Iterate bytes in data
	for byte in data:
		# Iterate bits in byte
		for _ in range(0, 8):
			mix = (crc ^ byte) & 0x01
			crc >>= 1
			if mix:
				crc ^= 0xA001
			byte >>= 1

	return crc ^ 0xffff

def calc_crc():
	# example
	buf = [0xc5, 0xe5, 0x0, 0x78, 0x0]
	s = ""
	for val in buf:
		s += (hex(val) + ",")

	s += hex(crc16(buf))
	return s

def _owid(bus, id):
	buf = [0x29, id, bus, 0xff - id, 0xff - bus,0x66,0x77]
	s = ""
	for val in buf:
		s += (hex(val) + ",")

	s += hex(crc8(buf))
	return s

def pro(owid):
	try:
		_db = db[f"{args.id[0]}.{args.id[1]}"]
		#print("    |ID |bus                    |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7")
		#print(f"{owid},{_db}")
		if not args.dry:
			subprocess.check_output(f"{AVRDUDE} -p {dev} {PROG}{args.port} -U eeprom:w:{owid},{_db}:m", shell=True).decode()
		else:
			print(f"{AVRDUDE} -p {dev} {PROG}{args.port} -U eeprom:w:{owid},{_db}:m")
		if args.verbose:
			dump_data(f"{owid},{_db}")
		if args.pins:
			pins(f"{owid},{_db}")
	except Exception as e:
		print(e)
def out_pin(c):
	s = c # "N/A"
	if c == "0x21":
		s = "OUT"
	elif c == "0x23":
		s = "PWM"
	elif c == "0x10":
		s = "BTN"
	elif c == "0x11":
		s = "SW"
	elif c == "0x05":
		s = "PASS"
	elif c == "0x06":
		s = "INV"
	elif c == "0x07":
		s = "INV_PU"
	elif c == "0x02" or c == "0x2":
		s = "ACT_HIGH (PIR/Touch)"
	return s

def dump_data(res):
	ar = res.split(",")
	i = 0
	dat = ""
	header = ""
	for c in ar:
		a = int(c, 16)
		header = f"{header} {i:#0{2}d}  "
		dat = f"{dat} {a:#0{4}x}"
		i += 1
		if i > 62:
			break
	# remove first space (again)
	header = header[1:-1]
	dat = dat[1:-1]
	print("    |ID  |bus                     |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7  |FEA |OFF |MAJ |MIN |TYP |   OFF   |   FACT  |S   |IO  |TH  |TL  |TYP |THR |DIMD|DIMU|DIF |TM1 |TM2 |SWA0|SWA1|SWA2|SWA3|SWA4...")
	print(header)
	print(dat)
	if args.pins:
		pins(dat)

def pins(_db):
	ar = _db.split(" ")
	off = 8
	for i in range(2, 2+8):
		c = ar[off + 8 + i]
		s = out_pin(c)
		if ar[off + i] != "0xff":
			print(f"PIN{i-2}: {s} | SW {ar[off + i]}")
		else:
			print(f"PIN{i-2}: {s}")
	# extended config for timers
	off = 40
	if ar[off] != "0xff":
		print(f"THR: {ar[off]}")
	off += 1
	if ar[off] != "0xff":
		print(f"DIMD: {ar[off]}")
	off += 1
	if ar[off] != "0xff":
		print(f"DIMU: {ar[off]}")
	off += 1
	if ar[off] != "0xff":
		print(f"DIF: {ar[off]}")
	off += 1
	if ar[off] != "0xff":
		print(f"TMR1: {ar[off]}")
	off += 1
	if ar[off] != "0xff":
		print(f"TMR2: {ar[off]}")
	off += 1
	for i in range(off, off + 8):
		try:
			c = ar[i]
			pin_nr = i - off
			if c != "0xff" and c != "0x00":
				print(f"PIN{pin_nr}: -> SW {c}")
		except Exception as e:
			pass

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('id', nargs='*', type=int, help='comma separated list of id: bus adr')
	parser.add_argument('-p', '--port', default = "0", help='Port of the Programmer', required=False)
	parser.add_argument('-d', '--dump', default = False, help='Port of the Programmer', action="store_true")
	parser.add_argument('--dry', default = False, help='Dry Run, not programming', action="store_true")
	parser.add_argument('--pins', default = False, help='Port of the Programmer', action="store_true")
	parser.add_argument('-v', '--verbose', default = False, help='Verbose output', action="store_true")
	args = parser.parse_args()
	id = f"{args.id[0]}.{args.id[1]}"
	if id in devices:
		dev = devices[id]
	if args.verbose:
		print(f"Device: {dev}")
	if args.dump:
		if args.verbose:
			print(f"{AVRDUDE} -p {dev} {PROG}{args.port} -U eeprom:r:-:h")
		if args.dry:
			owid = _owid(args.id[0], args.id[1])
			_db = db[f"{args.id[0]}.{args.id[1]}"]
			res = f"{owid},{_db}"
		else:
			res = subprocess.check_output(f"{AVRDUDE} -p {dev} {PROG}{args.port} -U eeprom:r:-:h", shell=True).decode()
		dump_data(res)
	else:
		s = _owid(args.id[0], args.id[1])
		pro(s)

