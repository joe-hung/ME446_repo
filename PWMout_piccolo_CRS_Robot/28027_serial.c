/* SERIAL.C: This code is designed to act as a low-level serial driver for
	higher-level programming.  Ideally, one could simply call init_serial()
	to initialize the serial port, then use serial_send("data", 4) to send
	an array of data (8-bit unsigned character strings).

	WRITTEN BY : Paul Miller <pamiller@uiuc.edu>
	$Id: serial.c,v 1.4 2003/08/08 16:08:56 paul Exp $
*/
#define DSP28_DATA_TYPES
typedef long            int32;
typedef int             int16;
#include <coecsl.h>
#include <28027_serial.h>

//#define NEW_BLOCK_CHAR	0xFF00

//#pragma DATA_SECTION(SerialA, "fastdata");
serial_t SerialA;

interrupt void TXAINT_data_sent(void);
#ifndef DSP28_BIOS
interrupt void RXAINT_recv_ready(void);
#endif


//#pragma CODE_SECTION(init_serial, "initfuncs");
err_t init_serial(serial_t *s, Uint32 baud, void (*got_func)(serial_t *s, char data))
{
	volatile struct SCI_REGS *sci;
	Uint32 clk;

	if (s == &SerialA) {
		sci = &SciaRegs;

//		EALLOW;
//
//#ifndef DSP28_BIOS
//		PieVectTable.SCITXINTA = &TXAINT_data_sent;
//		PieVectTable.SCIRXINTA = &RXAINT_recv_ready;
//#endif
//
//		GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
//		GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//		GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
//
//		GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;	   // Disable pull-up for GPIO29 (SCITXDA)
//		GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
//
//
//		GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
//		GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
//
//
//		EDIS;

		EALLOW;

#ifndef DSP28_BIOS
		PieVectTable.SCITXINTA = &TXAINT_data_sent;
		PieVectTable.SCIRXINTA = &RXAINT_recv_ready;
#endif

		GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up for GPIO7 (SCIRXDA)
		GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;  // Asynch input GPIO7 (SCIRXDA)
		GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;

		GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;	   // Disable pull-up for GPIO12 (SCITXDA)
		GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;


		GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
		GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;


		EDIS;

	} else {
		return E_CRITICAL;
	}

	s->sci = sci;
	s->got_data = got_func;

	init_buffer(&s->TX);

	/* init for standard baud,8N1 comm */
	sci->SCICTL1.bit.SWRESET = 0;		// init SCI state machines and opt flags
	sci->SCICCR.all = 0x0;
	sci->SCICTL1.all = 0x0;
	sci->SCICTL2.all = 0x0;
	sci->SCIPRI.all = 0x0;
	clk = LSPCLK_HZ;					// set baud rate
	clk /= baud*8;
	clk--;
	sci->SCILBAUD = clk & 0xFF;
	sci->SCIHBAUD = (clk >> 8) & 0xFF;

	sci->SCICCR.bit.SCICHAR = 0x7;		// (8) 8 bits per character
	sci->SCICCR.bit.PARITYENA = 0;		// (N) disable party calculation
	sci->SCICCR.bit.STOPBITS = 0;		// (1) transmit 1 stop bit
	sci->SCICCR.bit.LOOPBKENA = 0;		// disable loopback test
	sci->SCICCR.bit.ADDRIDLE_MODE = 0;	// idle-line mode (non-multiprocessor SCI comm)

	sci->SCIFFCT.bit.FFTXDLY = 0;		// TX: zero-delay

	sci->SCIFFTX.bit.SCIFFENA = 1;		// enable SCI fifo enhancements
	sci->SCIFFTX.bit.TXFIFOXRESET = 0;	// TX: fifo reset
	sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
	sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
	sci->SCIFFTX.bit.TXFFIENA = 0;		// TX: disable fifo interrupt
	sci->SCIFFTX.bit.TXFIFOXRESET = 1;	// TX: re-enable fifo

	sci->SCIFFRX.bit.RXFIFORESET = 0;	// RX: fifo reset
	sci->SCIFFRX.bit.RXFFINTCLR = 1;	// RX: clear interrupt flag
	sci->SCIFFRX.bit.RXFFIENA = 1;		// RX: enable fifo interrupt
	sci->SCIFFRX.bit.RXFFIL = 0x1;		// RX: fifo interrupt
	sci->SCIFFRX.bit.RXFIFORESET = 1;	// RX: re-enable fifo

	sci->SCICTL2.bit.RXBKINTENA = 0;	// disable receiver/error interrupt
	sci->SCICTL2.bit.TXINTENA = 0;		// disable transmitter interrupt

	sci->SCICTL1.bit.TXWAKE = 0;
	sci->SCICTL1.bit.SLEEP = 0;			// disable sleep mode
	sci->SCICTL1.bit.RXENA = 1;			// enable SCI receiver
	sci->SCICTL1.bit.RXERRINTENA = 0;	// disable receive error interrupt
	sci->SCICTL1.bit.TXENA = 1;			// enable SCI transmitter
	sci->SCICTL1.bit.SWRESET = 1;		// re-enable SCI

	/* enable PIE interrupts */
	if (s == &SerialA) {
		PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
		PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
	}
	IER |= M_INT9;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

	return SUCCESS;
}

//#pragma CODE_SECTION(uninit_serial, "initfuncs");
void uninit_serial(serial_t *s)
{
	volatile struct SCI_REGS *sci = s->sci;

	/* disable PIE interrupts */
	if (s == &SerialA) {
		PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
		PieCtrlRegs.PIEIER9.bit.INTx2 = 0;
	}

	sci->SCICTL1.bit.RXERRINTENA = 0;	// disable receive error interrupt
	sci->SCICTL2.bit.RXBKINTENA = 0;	// disable receiver/error interrupt
	sci->SCICTL2.bit.TXINTENA = 0;		// disable transmitter interrupt

	sci->SCICTL1.bit.RXENA = 0;			// disable SCI receiver
	sci->SCICTL1.bit.TXENA = 0;			// disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/

Uint16 serial_send(serial_t *s, char *data, Uint16 len)
{
	Uint16 i = 0;
	if (len && s->TX.size < BUF_SIZE) {
		for (i = 0; i < len; i++) {
			if (buf_write_1(&s->TX, data[i] & 0x00FF) != SUCCESS) break;
		}
		s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
		s->sci->SCIFFTX.bit.TXFFIENA = 1;		// TX: enable fifo interrupt
	}
	return i;
}



/***************************************************************************
 * SERIAL_TX()
 *
 * Transmit data to serial port.  If data is the "NEW_BLOCK_CHAR", send a
 * "Block start signal."
 ***************************************************************************/

//static inline void serial_TX(serial_t *s, char data)
//{
//	buf_remove(&s->TX, 1);
//	if (data == NEW_BLOCK_CHAR) {
//		s->sci->SCICTL1.bit.TXWAKE = 1;
//		s->sci->SCITXBUF = 'X';
//	} else {
//		s->sci->SCITXBUF = data;
//	}
//
//}

int dandubugcnt =0;
/***************************************************************************
 * TXxINT_DATA_SENT()
 *
 * Executed when transmission is ready for additional data.  These functions
 * read the next char of data and put it in the TXBUF register for transfer.
 ***************************************************************************/
#ifndef USEJUST_INTERNAL_MEM
#pragma CODE_SECTION(TXAINT_data_sent, "ramfuncs");
#endif
interrupt void TXAINT_data_sent(void)
{
	char data;
	if (buf_read_1(&SerialA.TX,0,&data) == SUCCESS) {
		while ( (buf_read_1(&SerialA.TX,0,&data) == SUCCESS) 
				&& (SerialA.sci->SCIFFTX.bit.TXFFST != 0x4) ) {
			buf_remove(&SerialA.TX, 1);
			SerialA.sci->SCITXBUF = data;
			dandubugcnt++;
		}
	} else {
		SerialA.sci->SCIFFTX.bit.TXFFIENA = 0;		// TX: disable fifo interrupt
	}
	SerialA.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}




/***************************************************************************
 * RXxINT_RECV_READY()
 *
 * Executed when data is received.
 ***************************************************************************/

static inline void serial_recv_ready(serial_t *s)
{
	char data = s->sci->SCIRXBUF.all;

	/* SCI PE or FE error */
	if (data & 0xC000) {
		s->sci->SCICTL1.bit.SWRESET = 0;
		s->sci->SCICTL1.bit.SWRESET = 1;
		s->sci->SCIFFRX.bit.RXFIFORESET = 0;
		s->sci->SCIFFRX.bit.RXFIFORESET = 1;
	} else if (s->got_data) {
		s->got_data(s, data & 0x00FF);
	}
}
#ifndef USEJUST_INTERNAL_MEM
#pragma CODE_SECTION(RXAINT_recv_ready, "ramfuncs");
#endif
#if DSP28_BIOS
void RXAINT_recv_ready(void)
#else
interrupt void RXAINT_recv_ready(void)
#endif
{
	serial_recv_ready(&SerialA);
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}




/***************************************************************************
 * SERIAL_PRINTF()
 *
 * Simple printf command to print out a serial port
 ***************************************************************************/
 
Uint16 serial_printf(serial_t *s, char *fmt, ...)
{
	va_list ap;
	char buf[BUF_SIZE];

	va_start(ap,fmt);
	vsprintf(buf,fmt,ap);
	va_end(ap);

	return serial_send(s,buf,strlen(buf));
}

