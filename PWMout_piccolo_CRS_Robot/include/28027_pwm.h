#ifndef __28027_PWM_H__
#define __28027_PWM_H__

#define PWM_CARRIER_HZ	20000L

#define PWM_TPS			0x0
#define PWM_TCLK		(LSPCLK_HZ >> (PWM_TPS)) //LSPCLK is at 60 MHz
#define PWM_PR			(PWM_TCLK / PWM_CARRIER_HZ)
#define PWM_DUTY50		(PWM_PR >> 1)

void init_PWM(enum epwm ep);
void init_PWMandDIR(enum epwm ep);
void PWM_out(enum epwm ep, float u);
void init_dualPWM(enum epwm ep);
void dualPWM_out(enum epwm ep, float u);
void PWMandDIR_out(int num, float u, short decval);

#endif /* __28027_PWM_H__ */

