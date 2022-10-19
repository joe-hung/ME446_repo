#define DSP28_DATA_TYPES
typedef long            int32;
typedef int             int16;
#include <coecsl.h>
#include <28027_pwm.h>

int dummy = 0;

void init_PWM(enum epwm ep)
{
	dummy = PWM_PR;
	switch (ep) {


	case EPWM1:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
		EDIS;

		EPwm1Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm1Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm1Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm1Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm1Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm1Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm1Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm1Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM2:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
		EDIS;

		EPwm2Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm2Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm2Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm2Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm2Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm2Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM3:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
		EDIS;

		EPwm3Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm3Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm3Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm3Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm3Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm3Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM4:
		
		EALLOW;
		//GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
		EDIS;

		EPwm4Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm4Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm4Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm4Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm4Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm4Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm4Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm4Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm4Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm4Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	}
		
}


void init_dualPWM(enum epwm ep)
{
	switch (ep) {


	case EPWM1:
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO0 = 0;
		GpioDataRegs.GPACLEAR.bit.GPIO1 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
		GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
		EDIS;
		

		EPwm1Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm1Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm1Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm1Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm1Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm1Regs.CMPA.half.CMPA = 0;  // Both signals zero
		EPwm1Regs.CMPB = 0;
		EPwm1Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm1Regs.CMPCTL.bit.LOADBMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm1Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD
		EPwm1Regs.AQCTLB.bit.CBU = 1; // Clear EPWM1B pin when CTR=CMPB
		EPwm1Regs.AQCTLB.bit.PRD = 2; // Set EPWM1B pin when CTR=PRD

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
		EDIS;

		break;

/*	case EPWM2:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
		EDIS;

		EPwm2Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm2Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm2Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm2Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm2Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm2Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM3:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
		EDIS;

		EPwm3Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm3Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm3Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm3Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm3Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm3Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM4:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
		EDIS;

		EPwm4Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm4Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm4Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm4Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm4Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm4Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm4Regs.CMPA.half.CMPA = PWM_DUTY50;  // Start off with a square wave
		EPwm4Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm4Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm4Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	
*/
	}
		
}


/* PWM_out(): update PWM's duty cycle -10.0<u<10.0
 *
 * parameters:
 *     pwm = PWM to update
 *     u   = duty cycle (-10 < u < 10)
 */
void PWM_out(enum epwm ep, float u)
{
	unsigned int val;

	if (u > 10.0F) {
		u = 10.0F;
	} else if (u < -10.0F) {
		u = -10.0F;
	}
	val = PWM_PR*u/20.0F + PWM_DUTY50;

	switch (ep) {
	case EPWM1:		EPwm1Regs.CMPA.half.CMPA = val;	break;
	case EPWM2:		EPwm2Regs.CMPA.half.CMPA = val;	break;
	case EPWM3:		EPwm3Regs.CMPA.half.CMPA = val;	break;
	case EPWM4:		EPwm4Regs.CMPA.half.CMPA = val;	break;
	}
}

/* dualPWM_out(): update PWM's duty cycle -10.0<u<10.0
 *
 * parameters:
 *     pwm = PWM to update
 *     u   = duty cycle (-10 < u < 10)
 */
void dualPWM_out(enum epwm ep, float u)
{
	unsigned int val;

	if (u > 10.0F) {
		u = 10.0F;
	} else if (u < -10.0F) {
		u = -10.0F;
	}
	val = PWM_PR*fabs(u)/10.0F;

	switch (ep) {
	case EPWM1:	
		if (u >= 0.0) { 
			EPwm1Regs.CMPA.half.CMPA = val;
			EPwm1Regs.CMPB = 0;
		} else {
			EPwm1Regs.CMPA.half.CMPA = 0;
			EPwm1Regs.CMPB = val;
		}
	break;

/*	case EPWM2:		EPwm2Regs.CMPA.half.CMPA = val;	break;
	case EPWM3:		EPwm3Regs.CMPA.half.CMPA = val;	break;
	case EPWM4:		EPwm4Regs.CMPA.half.CMPA = val;	break;
*/
	}
}




void init_PWMandDIR(enum epwm ep)
{


	switch (ep) {


	case EPWM1:

		EALLOW;

		//PWM pins
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO0 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;

		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO1 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;

		//DIR Pins
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;

		EDIS;

		dummy = PWM_PR;
		EPwm1Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm1Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm1Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm1Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm1Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm1Regs.CMPA.half.CMPA = 0;  // Set Signal to Zero zero
		EPwm1Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm1Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		EPwm1Regs.CMPB = 0; //Set signal to zero
		EPwm1Regs.CMPCTL.bit.LOADBMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLB.bit.PRD = 2; // Set EPWM1B pin when CTR=PRD
		EPwm1Regs.AQCTLB.bit.CBU = 1; // Clear EPWM1B pin when CTR=CMPB

//		//HR PWM Registers
//		EALLOW;
//		//EPwm1Regs.HRCNFG.bit.AUTOCONV = 1; //Auto convert CMPAHR values by MEP Scalar
//		EPwm1Regs.HRCNFG.bit.HRLOAD = 1; //Load on CTR = PRD
//		EPwm1Regs.HRCNFG.bit.EDGMODE = 2; //MEP Control of Falling Edge
//		EPwm1Regs.HRMSTEP = 111; //Use 60 MHz clock and Data Sheet MEP Value of 150 (1/60MHz)/(!50 Pico Seconds) = 111.1)
//		EDIS;



		break;

	case EPWM2:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO2 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;

		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO3 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;

		//DIR Pins
		GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
		GpioCtrlRegs.AIOMUX1.bit.AIO4 = 1;
		GpioCtrlRegs.AIODIR.bit.AIO4 = 1;


		EDIS;

		EPwm2Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm2Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm2Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm2Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm2Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm2Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		EPwm2Regs.CMPB = 0; //Set signal to zero
		EPwm2Regs.CMPCTL.bit.LOADBMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLB.bit.PRD = 2; // Set EPWM1B pin when CTR=PRD
		EPwm2Regs.AQCTLB.bit.CBU = 1; // Clear EPWM1B pin when CTR=CMPB

//		//HR PWM Registers
//		EALLOW;
//		//EPwm2Regs.HRCNFG.bit.AUTOCONV = 1; //Auto convert CMPAHR values by MEP Scalar
//		EPwm2Regs.HRCNFG.bit.HRLOAD = 1; //Load on CTR = PRD
//		EPwm2Regs.HRCNFG.bit.EDGMODE = 2; //MEP Control of Falling Edge
//		EPwm2Regs.HRMSTEP = 111; //Use 60 MHz clock and Data Sheet MEP Value of 150 (1/60MHz)/(!50 Pico Seconds) = 111.1)
//		EDIS;

		break;

	case EPWM3:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO4 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO4 = 0;

		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; //set to 1 for pwm
		//GpioDataRegs.GPACLEAR.bit.GPIO5 = 0;
		//GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;

		//PWM Pins
		GpioCtrlRegs.AIOMUX1.bit.AIO6 = 1;
		GpioCtrlRegs.AIODIR.bit.AIO6 = 1;

		GpioCtrlRegs.AIOMUX1.bit.AIO12 = 1;
		GpioCtrlRegs.AIODIR.bit.AIO12 = 1;
		EDIS;

		EPwm3Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
		EPwm3Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm3Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm3Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm3Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm3Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD
		EPwm3Regs.CMPB = 0; //Set signal to zero
		EPwm3Regs.CMPCTL.bit.LOADBMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLB.bit.PRD = 2; // Set EPWM1B pin when CTR=PRD
		EPwm3Regs.AQCTLB.bit.CBU = 1; // Clear EPWM1B pin when CTR=CMPB

//		//HR PWM Registers
//		EALLOW;
//		//EPwm3Regs.HRCNFG.bit.AUTOCONV = 1; //Auto convert CMPAHR values by MEP Scalar
//		EPwm3Regs.HRCNFG.bit.HRLOAD = 1; //Load on CTR = PRD
//		EPwm3Regs.HRCNFG.bit.EDGMODE = 2; //MEP Control of Falling Edge
//		EPwm3Regs.HRMSTEP = 111; //Use 60 MHz clock and Data Sheet MEP Value of 150 (1/60MHz)/(!50 Pico Seconds) = 111.1)
//		EDIS;

		break;




	case EPWM4:

		//Not setup on PCB

//
//
//		EALLOW;
//		GPIO for EPWM4A,4B is used for DIR1 and DIR2
//		EDIS;
//
//		EPwm4Regs.TBPRD = PWM_PR;  // Set to Period for 20Khz carrier
//		EPwm4Regs.TBPHS.half.TBPHS = 0;  // not using Phase
//		EPwm4Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
//		EPwm4Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
//		EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
//		EPwm4Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
//		EPwm4Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
//		EPwm4Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
//		EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count up
//
//		EPwm4Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
//		EPwm4Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
//		EPwm4Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
//		EPwm4Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD
//
	break;
	}

}




void PWMandDIR_out(int num, float u, short decval)
{

	short val = fabs(u); //integer value of duty cycle
	short dec = decval*256; //decimal value of duty cycle

	switch (num) {

	case (0x1):
		EPwm1Regs.CMPA.half.CMPA = val;
		//EPwm1Regs.CMPA.half.CMPAHR = dec;
		if (u >= 0) {
			GpioDataRegs.GPASET.bit.GPIO6 = 1;
		} else {
			GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
		}
		break;

	case (0x2):
		EPwm1Regs.CMPB = val;
		if (u >= 0) {
			GpioDataRegs.GPASET.bit.GPIO7 = 1;
		} else {
			GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
		}
		break;

//		EPwm2Regs.CMPA.half.CMPA = val;
//		EPwm2Regs.CMPA.half.CMPAHR = dec;
//		if (u >= 0) {
//			GpioDataRegs.GPASET.bit.GPIO7 = 1;   //High Res PWM
//		} else {
//			GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
//		}
//
//	break;


	case (0x3):
			EPwm2Regs.CMPA.half.CMPA = val;
			if (u >= 0) {
				GpioDataRegs.GPASET.bit.GPIO12 = 1;
			} else {
				GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
			}

		break;

//		EPwm3Regs.CMPA.half.CMPA = val;
//		EPwm3Regs.CMPA.half.CMPAHR = dec;
//		if (u >= 0) {
//			GpioDataRegs.GPASET.bit.GPIO12 = 1;
//		} else {
//			GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
//		}
//
//	break;

	case (0x4):
			EPwm2Regs.CMPB = val;
			if (u >= 0) {
				GpioDataRegs.AIOSET.bit.AIO4 = 1;
			} else {
				GpioDataRegs.AIOCLEAR.bit.AIO4 = 1;
			}

		break;

//		EPwm1Regs.CMPB = val;
//		if (u >= 0) {
//			GpioDataRegs.AIOSET.bit.AIO4 = 1;
//		} else {
//			GpioDataRegs.AIOCLEAR.bit.AIO4 = 1;
//		}
//
//		break;
	case (0x5):
				EPwm3Regs.CMPA.half.CMPA = val;
				if (u >= 0) {
					GpioDataRegs.AIOSET.bit.AIO6 = 1;
				} else {
					GpioDataRegs.AIOCLEAR.bit.AIO6 = 1;
				}

			break;

//			EPwm2Regs.CMPB = val;
//			if (u >= 0) {
//				GpioDataRegs.AIOSET.bit.AIO6 = 1;
//			} else {
//				GpioDataRegs.AIOCLEAR.bit.AIO6 = 1;
//			}
//
//			break;

	case (0x6):
		//Not Used
//
//					EPwm3Regs.CMPB = val;
//					if (u >= 0) {
//						GpioDataRegs.AIOSET.bit.AIO12 = 1;
//					} else {
//						GpioDataRegs.AIOSET.bit.AIO12 = 1;
//					}
//
				break;


	}
}
