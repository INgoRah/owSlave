import subprocess
import argparse

AVRDUDE = "%USERPROFILE%\\.platformio\\packages\\tool-avrdude\\avrdude.exe"
DEV = "-c stk500v2 -p attiny84 -P COM"

db = {}
#           RES       |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7  |FEA |OFF |MAJ |MIN |TYP |   OFF   |   FACT  |S   |IO  |TH  |TL
db['0.5'] = "0xff,0xff,0xff,0xff,0xff,0x01,0xff,0xff,0xff,0x02,0x21,0x21,0x10,0x10,0x10,0x10,0x10,0x05,0xFE,0x06,0x02,0x02,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10"
db['1.1'] = "0xff,0xff,0xff,0xff,0xff,0x03,0xff,0xff,0xff,0x02,0x21,0x21,0x21,0x07,0x10,0x10,0x10,0x10,0xFE,0x06,0x02,0x02,0x01,0x02,0x0c,0x22,0xe0"
db['1.7'] = "0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x00,0xFE,0x01,0x06,0x0B,0x00,0x00,0x00,0x01,0x01,0x01,0x14,0x10"

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
        print("    |ID |bus                    |CRC |  RES    |SW   1    2    3    4    5    6    7   | CFG  1    2    3    4    5    6    7")
        print(f"{owid},{_db}")
        subprocess.check_output(f"{AVRDUDE} {DEV}{args.port} -U eeprom:w:{owid},{_db}:m", shell=True).decode()
    except Exception as e:
        print(e)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', nargs='*', type=int, help='comma separated list of id: bus,adr')
    parser.add_argument('-p', '--port', default = "4", help='Port of the Programmer', required=False)
    parser.add_argument('-d', '--dump', default = False, help='Port of the Programmer', action="store_true")
    args = parser.parse_args()
    if args.dump:
        res = subprocess.check_output(f"{AVRDUDE} {DEV}{args.port} -U eeprom:r:-:h", shell=True).decode()
        print(res[0:220])
    else:
        s = _owid(args.id[0], args.id[1])
        pro(s)
