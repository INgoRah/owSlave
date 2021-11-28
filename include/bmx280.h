#ifndef BMX280_H
#define BMX280_H

extern int bmp280_init();
extern int32_t bmp280_readT();
extern int16_t bmp280_compensate_T16(int32_t adc_T);

#endif /* BMX280_H */
