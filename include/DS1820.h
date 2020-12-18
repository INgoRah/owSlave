#ifndef DS1820_H
#define DS1820_H

typedef union {
	volatile uint8_t bytes[8];
	struct {
		int16_t temp;  //0
		uint8_t TH;  //2
		uint8_t TL;  //3
		uint8_t config;  //4
		uint8_t rrFF; //5
		uint8_t rr00; //6
		uint8_t rr10; //7
	};
} packt_t;

void temp_setup();
void temp_loop();

#endif /* DS1820_H */
