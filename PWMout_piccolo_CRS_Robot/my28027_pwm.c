#define DSP28_DATA_TYPES
typedef long            int32;
typedef int             int16;
#include <coecsl.h>
#include <28027_pwm.h>

//#define DAN_PWM_CARRIER_HZ	80000L
#define DAN_PWM_CARRIER_HZ	40000L  // The limit of the A3953 chip is 70Khz

#define DAN_PWM_TPS			0x0
#define DAN_PWM_TCLK		(LSPCLK_HZ >> (DAN_PWM_TPS))
#define DAN_PWM_PR			(DAN_PWM_TCLK / DAN_PWM_CARRIER_HZ)
#define DAN_PWM_DUTY50		(DAN_PWM_PR >> 1)

void init_PWM(enum epwm ep)
{
	switch (ep) {


	case EPWM1:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
		EDIS;

		EPwm1Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm1Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm1Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm1Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm1Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm1Regs.CMPA.half.CMPA = DAN_PWM_DUTY50;  // Start off with a square wave
		EPwm1Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm1Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM2:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
		EDIS;

		EPwm2Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm2Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm2Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm2Regs.CMPA.half.CMPA = DAN_PWM_DUTY50;  // Start off with a square wave
		EPwm2Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm2Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM3:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
		EDIS;

		EPwm3Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm3Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm3Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm3Regs.CMPA.half.CMPA = DAN_PWM_DUTY50;  // Start off with a square wave
		EPwm3Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA 
		EPwm3Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case EPWM4:
		
		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
		EDIS;

		EPwm4Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm4Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm4Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm4Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm4Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm4Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm4Regs.CMPA.half.CMPA = DAN_PWM_DUTY50;  // Start off with a square wave
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
		

		EPwm1Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
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
	val = DAN_PWM_PR*u/20.0F + DAN_PWM_DUTY50;

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
	val = DAN_PWM_PR*fabs(u)/10.0F;

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
