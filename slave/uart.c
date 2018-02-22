#ifdef HAVE_UART
#include <avr/io.h>

#define BAUDRATE 9600

void serial_init(void)
{
	/* Set the baud rate */
	UBRR0H = (unsigned char) (((F_CPU / (BAUDRATE) / 8) - 1)) >> 8;
	UBRR0L = (unsigned char) (((F_CPU / (BAUDRATE) / 8) - 1));
	/* double speed to archive higher precision on 12 Mhz */
	UCSR0A = _BV(U2X0);
	/* set the framing to 8N1 */
	UCSR0C = (3 << UCSZ00);
	/* Engage! */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	return;
}

void serial_write(unsigned char c)
{
	while ( !(UCSR0A & (1 << UDRE0)) )
		;
	UDR0 = c;
}

#endif /* uart1 */

