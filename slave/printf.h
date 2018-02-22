#ifndef _PRINTF_P_H_
#define _PRINTF_P_H_

#include <avr/pgmspace.h>

extern void _printf_P (char* dest, char const *fmt0, ...);

#define printres(format, args...) \
	if (usb_cmd) \
		_printf_P (res_buf, PSTR (format), ## args); \
	else { \
		_printf_P (NULL, PSTR (format), ## args); \
		_printf_P (NULL, CLI_CRLF); \
	}

#define printf(format, args...)   _printf_P(NULL, PSTR(format) , ## args)
#define sprintf(buf, format, args...)   _printf_P(buf, PSTR(format) , ## args)

#endif
