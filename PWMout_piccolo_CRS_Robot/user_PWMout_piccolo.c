
#define DSP28_DATA_TYPES

#define SPI_MASTER 0

typedef int             int16;
typedef long             int32;
#include <coecsl.h>
#include "user_includes.h"
void init_PWMandDIR(enum epwm ep);

unsigned int SPIdata;
unsigned int wirelessbeginnewdata = 0, wirelessdatacollect = 0, timeint = 0;
char wirelessMessageArray[100];
int pwmnum = 0;
short u = 0;
short u_max = 0;

int down = 0;
int usonic = 0;
short pwm1 = 0;
short pwm2 = 0;
short pwm3 = 0;
short pwm4 = 0;
short pwm5 = 0;
short ss = 0;
short u_msb = 0;
short u_lsb = 0;
short u_old1 = 0;
short u_old2 = 0;
short u_old3 = 0;
short u_old4 = 0;
short u_old5 = 0;
short errorbits = 0;
long int pwmoff = 0;
float tunable = 0.08;

long int generalerror = 0;

short dummy1 = 0;
short dummy2 = 0;
short dummy3 = 0;
short dummy4 = 0;
short dummy5 = 0;
int errorcount = 0;
long int pwmnumerr = 0;
int errorflag = 0;
int greater = 0;
int lesser = 0;
short u_error = 0;
int pwm_error = 0;
short u_msberror = 0;
short u_lsberror = 0;
short decimal = 0;

int lesser_greater = 0;
int u_nochange1 = 0;
int u_nochange2 = 0;
int u_nochange3 = 0;
int u_nochange4 = 0;
int u_nochange5 = 0;
long int u_big1 = 0;
long int u_big2 = 0;
long int u_big3 = 0;
long int u_big4 = 0;
long int u_big5 = 0;
short ubig_threshold = 1200;
long int lg = 0;
int lg_threshold = 3;

int pwmseq = 0x1;

int errorflag_spi = 0;
int errorflag_lg = 0;
int errorflag_uconsant = 0;
int errorflag_ubig = 0;
int errorflag_pwmnum = 0;
int error_message = 0;

long int totalinterrupts= 0;
long int pwm1interrupts = 0;
long int pwm2interrupts = 0;
long int pwm3interrupts = 0;
long int pwm4interrupts = 0;
long int pwm5interrupts = 0;
long int pwmDCinterrupts = 0;

float duty1 = 0;
float duty2 = 0;
float duty3 = 0;
float duty4 = 0;
float duty5 = 0;

float tunable1 = 0;
float tunable2 = 0;
float tunable3 = 0;
float tunable4 = 0;
float tunable5 = 0;

short dec1 = 0;
short dec2 = 0;
short dec3 = 0;
short dec4 = 0;
short dec5 = 0;



int TXUsonic = 0;
int StartUsonic = 0;

void Debug3Bits(unsigned int bits) {
	switch (bits) {
	case 0:
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		break;
	case 1:
		GpioDataRegs.GPASET.bit.GPIO0 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		break;
	case 2:
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

		break;
	case 3:
		GpioDataRegs.GPASET.bit.GPIO0 = 1;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

		break;
	case 4:
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioDataRegs.GPASET.bit.GPIO5 = 1;

		break;
	case 5:
		GpioDataRegs.GPASET.bit.GPIO0 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioDataRegs.GPASET.bit.GPIO5 = 1;

		break;
	case 6:
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
		GpioDataRegs.GPASET.bit.GPIO5 = 1;

		break;
	case 7:
		GpioDataRegs.GPASET.bit.GPIO0 = 1;
		GpioDataRegs.GPASET.bit.GPIO2 = 1;
		GpioDataRegs.GPASET.bit.GPIO5 = 1;

		break;
	default:
		break;
	}
}
// used for debugging
//int usonicarray[50];
//int arrayindex = 0;
void RXserialA(serial_t *s, char data) {

	if (!wirelessbeginnewdata) {// Only true if have not yet begun a message
    	if ((unsigned char)data == 82) {// Check for start char 'R'
	        wirelessdatacollect = 0;       // amount of data collected in message set to 0
            wirelessbeginnewdata = 1;      // flag to indicate we are collecting a message
       	}
  	} else {    // Filling data
                // Dont go too large... limit message to 100  chars
 		if ((wirelessdatacollect < 3) && ((unsigned char)data != 13)) {
       		wirelessMessageArray[wirelessdatacollect] = data;
           	wirelessdatacollect++;
      	} else {  // too much data or 13 char received
			if ((unsigned char)data != 13) {
           		wirelessdatacollect = 0;
                wirelessbeginnewdata = 0;
           	} else {
            	wirelessMessageArray[wirelessdatacollect] = '\0'; 	// Null terminate the string 

            	usonic = atoi(wirelessMessageArray);

//            	usonicarray[arrayindex] = usonic;
//            	arrayindex++;
//            	if (arrayindex == 50) {
//            		arrayindex = 0;
//            	}
            	if (TXUsonic == 1) {
            		SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
            		SpiaRegs.SPITXBUF = 0xDD00 | ((unsigned)usonic&0xFF);
            		TXUsonic = 0;
            	}

				wirelessbeginnewdata = 0;
                wirelessdatacollect = 0;
          	}
       	}
  	}
}

int dummyread;

void main(void)
{

  	InitPieCtrl();
	// Enable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    		
	// Enables PIE to drive a pulse into the CPU 
	PieCtrlRegs.PIEACK.all = 0xFFFF;

	// initialize serial port A to 9600 baud
	init_serial(&SerialA,9600,RXserialA);  // need to change this to correct f28027 pins

	EALLOW;

		GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
		GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
		GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO3 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

		// Code for sensing Magnet  Commented out to use GPIO5 as debug output
//		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
//		GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;  // input
//		GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0; // enable pullup on GPIO5
//		GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0x2;  // qual on 6 samples
//		GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0x20;  // QualPeriod of 32*Sysclkout

		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;

		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;

		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;

		GpioCtrlRegs.AIOMUX1.bit.AIO10 = 1;
		GpioCtrlRegs.AIOMUX1.bit.AIO2 = 1;
		GpioCtrlRegs.AIODIR.bit.AIO10 = 1;
		GpioCtrlRegs.AIODIR.bit.AIO2 = 1;






	EDIS;

	/*********  SPI  *************************************/

	if (SPI_MASTER) {
		EALLOW;

			// CS pins GPIOs
			//GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
			GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
			// CS pins Outputs
			//GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
			GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
			// CS start hi
			//GpioDataRegs.GPASET.bit.GPIO6 = 1;
			GpioDataRegs.GPASET.bit.GPIO19 = 1;
		
		EDIS;

		InitSpiaGpio();

		SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

		SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
		SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

		SpiaRegs.SPICCR.bit.SPICHAR = 15;   // set to transmitt 8 bits

		SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
		SpiaRegs.SPICTL.bit.TALK = 1;

		SpiaRegs.SPICTL.bit.SPIINTENA = 0;

		//SpiaRegs.SPISTS.all=0x0000;

		SpiaRegs.SPIBRR = 47;   // divide 15MZ (60Mhz already divide by 4) by  12 = 1.25 Mhz

		SpiaRegs.SPIFFTX.bit.SPIRST = 1;
		SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
		SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

		SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
		SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFIL = 3;  // 4 is the max on 28027
		SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;

		SpiaRegs.SPIFFCT.all=0x00;

		SpiaRegs.SPIPRI.bit.FREE = 1;
		SpiaRegs.SPIPRI.bit.SOFT = 0;


		SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

		SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

		SpiaRegs.SPICTL.bit.SPIINTENA = 1;
		SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

	} else {
		InitSpiaGpio_slave();

		SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

		SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
		SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

		SpiaRegs.SPICCR.bit.SPICHAR = 7;   // set to transmitt 8 bits

		SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;
		SpiaRegs.SPICTL.bit.TALK = 1;

		SpiaRegs.SPICTL.bit.SPIINTENA = 0;

		SpiaRegs.SPISTS.all=0x0000;


		SpiaRegs.SPIBRR = 47;   // divide 15MZ (60Mhz already divide by 4) by  12 = 1.25 Mhz

		SpiaRegs.SPIFFTX.bit.SPIRST = 1;
		SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
		SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

		SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
		SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

		SpiaRegs.SPIFFCT.all=0x00;

		SpiaRegs.SPIPRI.bit.FREE = 1;
		SpiaRegs.SPIPRI.bit.SOFT = 0;


		SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

		SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

		SpiaRegs.SPICTL.bit.SPIINTENA = 1;
		SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
		SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
		//SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

		// added 7/23/2012
		//modified 6/3/2016
		SpiaRegs.SPIFFRX.bit.RXFFIL = 4;// 4 is the max on 28027


	}
	
	//init pwm
	init_PWMandDIR(EPWM1);
	init_PWMandDIR(EPWM2);
	init_PWMandDIR(EPWM3);
	init_PWMandDIR(EPWM4);

	// SPI
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt

	IFR &= ~M_INT6;  // Make sure no ints pending.
	IER |= M_INT6; // Enable CPU Interrupt E for SPI and later for McBSP

/*********  SPI  *************************************/
/*****************************************************/


}


void SPI_RXint(void) {



	//6/7/2016
	//NOTES
	// slowed down SPI CLK from master. Initially was too fast and not entering into interrupt function
	// Masked RXbuf values to only keep desired bits
	// Changed pwmnum from 0x0200 -> 0x2


	pwmnum = SpiaRegs.SPIRXBUF & 0xFF; //First message is the "pwmnum" which corresponds to the which joint the control effort is for.
	u_msb = SpiaRegs.SPIRXBUF & 0xFF; //MSB of the control effort
	u_lsb = SpiaRegs.SPIRXBUF & 0xFF; //LSB of the control effort
	decimal = SpiaRegs.SPIRXBUF & 0xFF; //decimal value of control effort if high res PWM is used

	while (SpiaRegs.SPIFFRX.bit.RXFFST != 0)
	{
		dummyread = SpiaRegs.SPIRXBUF;
	}


	totalinterrupts++; //Used for debugging SPI Issues. Increments each time SPI interrupt is called.



	if(pwmnum == 0xDC) // The delfino will send 0xDC as the first message to signify SPI communication is functioning again.

	{
		pwmDCinterrupts++;				//Checking if we are receiving the error acknowledgment message from the Delfino
		u = (u_msb << 8) + u_lsb;		//If we successfuly receive it we set the errorflag_spi back to 0 and zero the pwm's one last time before they are updated with correct values
		if((u == 0x0BEF) && (decimal == 0)) // 0xDC is followed by 0x0BEF.
		{
			errorflag_spi = 0;
			errorbits &= ~(0x0400); //clear errorbit to send to delfino
			pwm1 = 0;
			pwm2 = 0;
			pwm3 = 0;
			pwm4 = 0;
			pwm5 = 0;
			//Once we get into this IF statement we know SPI communication is resynced. We 0 the PWMs once more and then come out of the SPI error state.
		}
	}


	//The following checks for each pwmnum verify that we aren't receiving bad data. All of the error detection happens here and increments specific error counters for each issue.
	//The actual error flags are set further down along with the tresholds for when an error is set.


	if((pwmnum == 0x1) && (pwmseq == 0x1))
	{

		pwm1interrupts++; //counting how many times we enter this specific pwmnum.


		u = (u_msb << 8) + u_lsb;

			if(u>1500)
			{								//Check if u is greater than or less than 1500 or -1500. We should never receive anything larger or smaller.
				u = 1500;					//Each time this happens we set the counters to 15.
				greater = 15;				//The PWM output will be 0 for all 5 joints until the count decrements from 15 to 0.
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(u<-1500)
			{
				lesser = 15;
				u = -1500;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(errorflag_lg == 0x1) 	//The lines below decrement the counter back to 0.
			{
				if(lesser>0)
				{
					lesser--;
				}

				if(greater>0)
				{
					greater--;
				}
			}



		if(u == u_old1) 			//Check to see if our control effort is changing.
									//Currently we are not doing anything for a constant control effort.
									//Can be changed in the future if desired.
		{
			//Do Nothing
		}
		else
		{
			u_nochange1 = 0;
		}


		if(fabs(u)>ubig_threshold)    //Checking to see if our control effort is a large constant value.
		{									 //A counter increments in the ControlFunc that is reset by this part of the code.
											 // If we hit the threshold for the error we will 0 PWM output until it is fixed.
			//Do Nothing
		}
		else
		{
			u_big1 = 0;
		}


		pwm1 = u;   //Assign control effort to the pwm output
		dec1 = decimal; //Assign control effort decimal value
		u_old1 = u; //refresh old control effort value
		errorcount = 0; //zero the error count if we get this far. Used for debugging
		pwmseq = 0x2; //increment the PWM Sequence. This ensures that the pwm values will always output in the right order. If we get out of sequence we wont send random values to motors.
		if(pwmnumerr>0)
		{
			pwmnumerr--; //used for debugging. Decrement each time we get into this IF statement because we know there is no longer a pwm error. Treshold is set in ControlFunc.
		}

	}
	else if((pwmnum == 0x2) && (pwmseq == 0x2))
	{

		pwm2interrupts++;


		u = (u_msb << 8) + u_lsb;

			if(u>1500)
			{
				u = 1500;
				greater = 15;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(u<-1500)
			{
				lesser = 15;
				u = -1500;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(errorflag_lg == 0x1)
			{
				if(lesser>0)
				{
					lesser--;
				}

				if(greater>0)
				{
					greater--;
				}
			}


		if(u == u_old2)
		{
			//Do Nothing
		}
		else
		{
			u_nochange2 = 0;
		}


		if(fabs(u)>ubig_threshold)
		{
			//Do Nothing
		}
		else
		{
			u_big2 = 0;
		}

		pwm2 = u;
		dec2 = decimal;
		u_old2 = u;
		errorcount = 0;
		pwmseq = 0x3;
		if(pwmnumerr>0)
		{
			pwmnumerr--;
		}
	}
	else if((pwmnum == 0x3) && (pwmseq == 0x3))
	{
		pwm3interrupts++;

		u = (u_msb << 8) + u_lsb;

			if(u>1500)
			{
				u = 1500;
				greater = 15;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(u<-1500)
			{
				lesser = 15;
				u = -1500;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(errorflag_lg == 0x1)
			{
				if(lesser>0)
				{
					lesser--;
				}

				if(greater>0)
				{
					greater--;
				}
			}


		if(u == u_old3)
		{
			//Do Nothing
		}
		else
		{
			u_nochange3 = 0;
		}

		if(fabs(u)>ubig_threshold)
		{
			//Do Nothing
		}
		else
		{
			u_big3 = 0;
		}

		pwm3 = u;
		dec3 = decimal;
		u_old3 = u;
		errorcount = 0;
		pwmseq = 0x4;
		if(pwmnumerr>0)
		{
			pwmnumerr--;
		}
	}
	else if((pwmnum == 0x4) && (pwmseq == 0x4))
	{
		pwm4interrupts++;

		u = (u_msb << 8) + u_lsb;

			if(u>1500)
			{
				u = 1500;
				greater = 15;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(u<-1500)
			{
				lesser = 15;
				u = -1500;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(errorflag_lg == 0x1)
			{
				if(lesser>0)
				{
					lesser--;
				}

				if(greater>0)
				{
					greater--;
				}
			}



		if(u == u_old4)
		{
			//Do Nothing
		}
		else
		{
			u_nochange4 = 0;
		}

		if(fabs(u)>ubig_threshold)
		{
			//Do Nothing
		}
		else
		{
			u_big4 = 0;
		}


		pwm4 = u;
		dec4 = decimal;
		u_old4 = u;
		errorcount = 0;
		pwmseq = 0x5;
		if(pwmnumerr>0)
		{
			pwmnumerr--;
		}
	}
	else if((pwmnum == 0x5) && (pwmseq == 0x5))
	{
		pwm5interrupts++;

		u = (u_msb << 8) + u_lsb;

			if(u>1500)
			{
				u = 1500;
				greater = 15;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(u<-1500)
			{
				lesser = 15;
				u = -1500;
				lg++; //Overall lg counter. Each time we get an lg error this will increment.
			}
			else if(errorflag_lg == 0x1)
			{
				if(lesser>0)
				{
					lesser--;
				}

				if(greater>0)
				{
					greater--;
				}
			}



		if(u == u_old5)
		{
			//Do Nothing
		}
		else
		{
			u_nochange5 = 0;
		}

		if(fabs(u)>ubig_threshold)
		{
			//Do Nothing
		}
		else
		{
			u_big5 = 0;
		}

		pwm5 = u;
		dec5 = decimal;
		u_old5 = u;
		errorcount = 0;
		pwmseq = 0x1;

		if(pwmnumerr>0)
		{
			pwmnumerr--;
		}


		//Check to see if any of our error flags are set. If they are we 0 the PWM values and increment the pwmoff counter.
		if(errorflag_spi == 0x1 || errorflag_lg == 0x1 || errorflag_pwmnum == 0x1 || errorflag_ubig == 0x1){ //Error Mode

			PWMandDIR_out(0x1,0,0);
			PWMandDIR_out(0x2,0,0);
			PWMandDIR_out(0x3,0,0);
			PWMandDIR_out(0x4,0,0);
			PWMandDIR_out(0x5,0,0);
			pwmoff++;

		}
		else{

			//Output PWM values normally
			PWMandDIR_out(0x1,pwm1,dec1);
			PWMandDIR_out(0x2,pwm2,dec2);
			PWMandDIR_out(0x3,pwm3,dec3);
			PWMandDIR_out(0x4,pwm4,dec4);
			PWMandDIR_out(0x5,pwm5,dec5);

		}

	}
	else
	{
		pwmnumerr++; //Error counters used for debugging
		generalerror++; //Error counters used for debugging
	}


	//SPI error messages to be sent to Delfino. The delfino will receive these and send a specific message back to resync the SPI communication.
	if(errorflag_spi == 0x1)
	{
		SpiaRegs.SPITXBUF = 0xE33E;
		SpiaRegs.SPITXBUF = 0xE33E;
		SpiaRegs.SPITXBUF = 0xE33E;
		SpiaRegs.SPITXBUF = 0xE33E;
	}
	else
	{
		//Normal messages to Delfino are errorbit status.
		SpiaRegs.SPITXBUF = errorbits;
		SpiaRegs.SPITXBUF = errorbits;
		SpiaRegs.SPITXBUF = errorbits;
		SpiaRegs.SPITXBUF = errorbits;
	}

	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE





}


char sendarray[15];


void ControlFunc(void) {

	timeint++;


	//error counters
	errorcount++;

	//incrementing every 1ms. Used to check if u isn't changing
	//6/10/2016  not currently used
	u_nochange1++;
	u_nochange2++;
	u_nochange3++;
	u_nochange4++;
	u_nochange5++;


	//incrementing every 1ms. Used to check if u is >1000 for extended period of time
	u_big1++;
	u_big2++;
	u_big3++;
	u_big4++;
	u_big5++;



	if(lesser >= 1 || greater >= 1) //lesser/greater threshold can be changed here. We want to keep this low because this is always going to be an error and not random.
	{
		errorflag_lg = 0x1;	 //set error flag
		errorbits |= 0x0100; //set errorbit to send to delfino
		u_max = u; //Store this u for debugging purposes.
	}
	else if((lesser == 0) && (greater == 0))
	{
		errorflag_lg = 0x0;		//clear error flag
		errorbits &= ~(0x0100); //clear errorbit to send to delfino
	}

	if(lg>lg_threshold){ //lg threshold is the amount of times we will allow an lg error before we permanently turn off the arm.

		errorflag_lg = 0x1; //set error flag
		errorbits |= 0x0100; //set errorbit to send to delfino
	}

	if(pwmnumerr>=5) //threshold for how many pwmerrors we can get before we turn the arm off.
	{
		errorflag_pwmnum = 0x1; //set error flag
		errorbits |= 0x0200; //set errorbit to send to delfino
	}
	else if(pwmnumerr == 0)
	{
		errorflag_pwmnum = 0x0; //clear error flag
		errorbits &= ~(0x0200); //clear errorbit to send to delfino

	}


	if(errorcount >= 100) //Threshold for missed SPI messages.
	{

		errorflag_spi = 0x1; //No longer receiving SPI messages. Set error flag
		errorbits |= 0x0400; //set errorbit to send to delfino

	}
	else
	{
		//error flag is reset in the SPI interrupt
	}


	if(u_big1 >= 500 || u_big2 >= 500 || u_big3 >= 500 || u_big4 >= 500 ||u_big5 >= 500) //threshold for how many u_big error we can get before we go into error.
	{

		errorflag_ubig = 0x1; //set error flag
		errorbits |= 0x0800; //set errorbit to send to delfino
	}
	else
	{
		errorflag_ubig = 0x0; //clear error flag
		errorbits &= ~(0x0800); //clear errorbit to send to delfino
	}

//No change tresholds. Currently not in use.
//	if((u_nochange1>=100) || (u_nochange2>=100) || (u_nochange3>=100) || (u_nochange4>=100) || (u_nochange5>=100)){
//		errorflag_uconstant = 1;
//	}
//	else{
//		errorflag_uconstant = 0;
//	}



	//The code below lights up the error LEDs on the board. We only have 3 LEDs to work with.
	if((timeint%100) == 0)
	{

		if(errorbits & 0x0400) //spi error
		{

			GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// Set LED1

		}
		else{
			GpioDataRegs.GPBSET.bit.GPIO34 = 1; // Clear LED1
		}
		if(errorbits & 0x0100) //lg error
		{

			GpioDataRegs.AIOCLEAR.bit.AIO10 = 1;	// Set LED2

		}
		else{
			GpioDataRegs.AIOSET.bit.AIO10 = 1; //Clear LED2
		}
		if(errorbits & 0x0800) //ubig error
		{

			GpioDataRegs.AIOCLEAR.bit.AIO2 = 1;	//Set LED4

		}
		else{
			GpioDataRegs.AIOSET.bit.AIO2 = 1; //Clear LED4
		}
		if(errorbits & 0x0200) //pwmnum error
		{

			GpioDataRegs.AIOCLEAR.bit.AIO2 = 1;	// Set LED4
			GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// Set LED1
			GpioDataRegs.AIOCLEAR.bit.AIO10 = 1;	// Set LED2

		}
		else{

			if(errorflag_spi == 0){
				GpioDataRegs.GPBSET.bit.GPIO34 = 1; // Clear LED1				//Double checking these LEDs since they are shared with other errors.
			}
			if(errorflag_lg == 0){
				GpioDataRegs.AIOSET.bit.AIO10 = 1; //Clear LED2
			}
			if(errorflag_ubig == 0){
				GpioDataRegs.AIOSET.bit.AIO2 = 1; // Clear LED4
			}




		}


	}


//	if (StartUsonic == 1) {
//		Debug3Bits(6);
//		GpioDataRegs.GPASET.bit.GPIO3 = 1;	// pulse GPIO34 (Ultrasonic RX1) high for 1ms
//		StartUsonic = 2;
//	} else if (StartUsonic == 2) {
//		Debug3Bits(7);
//		GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
//		StartUsonic = 0;
//		TXUsonic = 1;
//	}


	//Turn off arm for SPI and u_big error mode.
	if(errorflag_spi == 0x1 || errorflag_ubig == 0x1){ //Error Mode

		PWMandDIR_out(0x1,0,0);
		PWMandDIR_out(0x2,0,0);
		PWMandDIR_out(0x3,0,0);
		PWMandDIR_out(0x4,0,0);
		PWMandDIR_out(0x5,0,0);
		pwmoff++;


	}



	


}

