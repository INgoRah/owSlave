#ifdef BMP280_SUPPORT

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <i2cmaster.h>

#include "bmx280.h"

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

int16_t bmp280_compensate_T16(int32_t adc_T)
{
	double var1, var2;
	int32_t t_fine;
	int16_t T;

	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	/* original it was double without *16 ...
	   This converts to the 1820 value */
	T = (int16_t) (t_fine / 5120.0 * 16);

	return T;
}

static uint16_t bmp280_read16(uint8_t adr)
{
	uint8_t lsb, msb;

	i2c_start(0xEC + I2C_WRITE);
	i2c_write(adr);
	i2c_rep_start(0xEC + I2C_READ);
	lsb = i2c_read(ACK);
	msb = i2c_read(NACK);
	i2c_stop();

	return msb << 8 | lsb;
}

void bmp280_calib()
{
	dig_T1 = bmp280_read16(0x88);
	dig_T2 = bmp280_read16(0x8A);
	dig_T3 = bmp280_read16(0x8C);
}

int bmp280_init()
{
	uint8_t ret;

	i2c_init();
	// set device address (EC 8 bit = 76 7 bit) and write mode
	ret = i2c_start(0xEC + I2C_WRITE);
	if (ret) {
		i2c_stop();
		return 0;
	}
	ret = i2c_write(0xD0);
	if (ret) {
		i2c_stop();
		return 0;
	}
	i2c_stop();
	i2c_start (0xEC + I2C_READ);
	ret = i2c_read(NACK);
	if (ret != 0x58)
		return 0;
	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xF4);
	/* write mode to forced */
	i2c_write(0x45);
	i2c_stop();
	bmp280_calib();

    return 1;
}

int32_t bmp280_readT()
{
	uint8_t msb, lsb, c;

	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xF4);
	/* write mode to forced */
	i2c_write(0x45);
	i2c_stop();
	_delay_ms(1);
	i2c_start(0xEC + I2C_WRITE);
	i2c_write(0xFA);
	i2c_rep_start(0xEC + I2C_READ);
	msb = i2c_read(ACK);
	lsb = i2c_read(ACK);
	c = i2c_read(NACK);
	i2c_stop();
	return (int32_t)msb << 12 | ((int16_t)lsb << 4) | (c >> 4);
}

#endif /* BMP280_SUPPORT */