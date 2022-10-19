/* SERIAL.H: This code is designed to act as a low-level serial driver for
	higher-level programming.  Ideally, one could simply call init_serial()
	to initialize the serial port, then use serial_send("data", 4) to send
	an array of data (8-bit unsigned character strings).

	WRITTEN BY : Paul Miller <pamiller@uiuc.edu>
	$Id: serial.h,v 1.2 2003/08/08 18:50:32 paul Exp $
*/

#ifndef __28027_SERIAL_H__
#define __28027_SERIAL_H__

#include <coecsl.h>

typedef struct serial_s {
	volatile struct buffer_s TX;
	volatile struct SCI_REGS *sci;
	void (*got_data)(struct serial_s *s, char data);
} serial_t;

extern serial_t SerialA;

err_t init_serial(serial_t *s, Uint32 baud, void (*got_func)(serial_t *s, char data));
void uninit_serial(serial_t *s);
Uint16 serial_send(serial_t *s, char *data, Uint16 len);
Uint16 serial_printf(serial_t *s, char *fmt, ...);

#endif /* __28335_SERIAL_H__ */

