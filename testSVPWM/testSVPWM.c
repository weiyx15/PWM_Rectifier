//###########################################################################
// Description
//! \addtogroup f2833x_example_list
//! <h1> ADC Start of Conversion (adc_soc)</h1>
//!
//! This ADC example uses ePWM1 to generate a periodic ADC SOC on SEQ1.
//! Two channels are converted, ADCINA3 and ADCINA2.
//!
//! \b Watch \b Variables \n
//! - Voltage1[10]	- Last 10 ADCRESULT0 values
//! - Voltage2[10]	- Last 10 ADCRESULT1 values
//! - ConversionCount	- Current result number 0-9
//! - LoopCount		- Idle loop counter
//
//
//###########################################################################
// $TI Release: F2833x/F2823x Header Files and Peripheral Examples V142 $
// $Release Date: November  1, 2016 $
// $Copyright: Copyright (C) 2007-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"

// math constants
#define pi (3.14159)
#define sqrt23 (0.81649658)
#define sqrt2 (1.414213562)
// PWM period and CMPA parameters
#define EPWM_PERIOD (1875)			// TBCLK = 18750kHz, EPWMCLK = 10kHz
#define EPWM_CMPA_MAX (EPWM_PERIOD-1)// fully on over a PWM period
#define EPWM_CMPA_MIN (0)			// fully off over a PWM period
#define EPWM_OFF	(EPWM_PERIOD-1) // uncontrolled rectifier: all IGBTs off
// Maximum Dead Band values
#define EPWM_MAX_DB   0x013F
#define EPWM_MIN_DB   0x013F
// voltage external loop values
#define STAND_VD	(100)			// output DC voltage reference
#define UNC_VD		(40)	// <= UNC_VD, use uncontrolled rectifier
#define Kp_i	10
#define Ki_i 	0.001
#define Kp_u	10
#define Ki_u	0.001
// PLL values
#define PHASE_SHIFT (0)
#define Kp_pll 0.1
#define Ki_pll 0.007
#define OMG (100*pi)			// angular frequency of line voltage
// Loop control values
#define cntMAX (200)
#define overMAX (100)

// Loop control variables
Uint16 LoopCnt;
Uint16 downCnt;
Uint16 upCnt;

// PLL global variables
float theta, omega, Uq_p, Uq;
float phaA, phaB, phaC;
float viewAB, viewBC, viewCA,AB90,BC90,CA90;

// ADC global variables
Uint16 UAB;									// AC line-line voltage Uab
Uint16 UBC;									// AC line-line voltage Ubc
Uint16 UCA;									// AC line-line voltage Uca
Uint16 UD;									// DC output voltage
Uint16 IA;									// AC phase A current
Uint16 IB;									// AC phase B current
Uint16 IC;									// AC phase C current


// voltage external loop global variables
Uint16 epwm_flag;
float Ia,Ib,Ic,Ud,Uab,Ubc,Uca;
// for GRAPH watching
float Vab[cntMAX],Vbc[cntMAX],Vca[cntMAX];
float iA[cntMAX],iB[cntMAX],iC[cntMAX];

// voltage external loop global variables
Uint16 sector;
float Vdc_err, Vdc_err_p, Id, Idx, Idy_p, Idy, Iq, Iqx, Iqy, Iqy_p, ud1, ud2, uq1, uq2, ud, uq;
float v_alpha, v_beta, Ta,Tb,Tc,t1,t2,t12,vd,vq,Va,Vb,Vc;
int cmpa,cmpb,cmpc;


// Prototype statements for functions found within this file.
void InitEPwm4();
void InitEPwm5();
void InitEPwm6();
void InitEPwm4Gpio(void);
void InitEPwm5Gpio(void);
void InitEPwm6Gpio(void);
void PLL(void);
float calcRMS(float U[cntMAX]);
__interrupt void adc_isr(void);


main()
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clock
// This example function is found in the DSP2833x_SysCtrl.c file.

	InitSysCtrl();
   EALLOW;
  #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
	#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
  #endif
  #if (CPU_FRQ_100MHZ)
	#define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
  #endif
  EDIS;

   // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
   // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
   EALLOW;
   SysCtrlRegs.HISPCP.all = ADC_MODCLK;
   EDIS;

// Step 2. Initialize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2833x_EPwm.c file
  InitEPwm4Gpio();
  InitEPwm5Gpio();
  InitEPwm6Gpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   InitAdc();  // For this example, init the ADC

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
  EDIS;
  InitEPwm4();
  InitEPwm5();
  InitEPwm6();
  EALLOW;

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

// Step 5. User specific code, enable interrupts:

// Enable ADCINT in PIE
   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
   IER |= M_INT1; // Enable CPU Interrupt 1
   EINT;          // Enable Global interrupt INTM
   ERTM;          // Enable Global realtime interrupt DBGM

// Configure ADC
   AdcRegs.ADCMAXCONV.all = 0x6;       // Setup 7 conv's
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA1 as 1st SEQ1 conv.
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA2 as 2nd SEQ1 conv.
   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA3 as 3rd SEQ1 conv.
   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; // Setup ADCINA4 as 4th SEQ1 conv.
   AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; // Setup ADCINA5 as 5th SEQ1 conv.
   AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; // Setup ADCINA6 as 6th SEQ1 conv.
   AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6; // Setup ADCINA7 as 7th SEQ1 conv.
   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

// Assumes ePWM1 clock is already enabled in InitSysCtrl();
   EPwm4Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
   EPwm4Regs.ETSEL.bit.SOCASEL = 4;       // Select SOC from from CPMA on upcount
   EPwm4Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event


// Wait for ADC interrupt
// Step 6. Customlize initialization
   Vdc_err=0; Vdc_err_p=0; Id=0; Idx=0; Idy_p=0; Idy=0;
   Iq=0; Iqx=0; Iqy=0; Iqy_p=0; ud1=0; ud2=0; uq1=0; uq2=0; ud=0; uq=0;
   Ta=0; Tb=0; Tc=0; t1=0; t2=0; t12=0; vd=0; vq=0; Va=0; Vb=0; Vc=0; v_alpha=0; v_beta=0;
   epwm_flag = 0;
   LoopCnt = 0; downCnt = 0; upCnt = 0;
   for(;;){}
}

__interrupt void  adc_isr(void)
{
	if(LoopCnt == cntMAX-1)
   {
	   LoopCnt = 0;
   }
   else
   {
	   LoopCnt++;
   }

  // Read ADC register
  UAB = AdcRegs.ADCRESULT0 >>4;
  UBC = AdcRegs.ADCRESULT1 >>4;
  UCA = AdcRegs.ADCRESULT2 >>4;
  IA  = AdcRegs.ADCRESULT3 >>4;
  IB  = AdcRegs.ADCRESULT4 >>4;
  IC  = AdcRegs.ADCRESULT5 >>4;
  UD  = AdcRegs.ADCRESULT6 >>4;

  // ADC Uint16 -> float -> Actual parameter
  Ia  = (float) IA/4095*3;							// 12 bit accuracy, 0~3V input
  Ib  = (float) IB/4095*3;
  Ic  = (float) IC/4095*3;
  Uab = (float) UAB/4095*3;
  Ubc = (float) UBC/4095*3;
  Uca = (float) UCA/4095*3;
  Ud  = (float) UD/4095*3;

	Ia = 5.187114*Ia-7.861663;
	Ib = 5.150712*Ib-7.9364;
	Ic = 5.2224*Ic-8.127;
	Uab = 249.782*Uab-381.4252;
	Ubc = 274.752*Ubc-406.4098;
	Uca = 238.9212*Uca-362.46788;
	Ud = 125.3*Ud-3.2916168;

	// 20180120 adjustion
	Ia = 0.9817*Ia-0.1336;
	Ib = 1.0345*Ib+0.0794;
	Ic = 0.9498*Ic-0.1216;
	Uab = 0.6816*Uab+31.74;
	Ubc = 0.6675*Ubc+36.403;
	Uca = 0.6423*Uca+34.437;
	Ud = 0.4672*Ud+4.0202;

	// Three phase PLL
	PLL();
//
//  // for GRAPH watch
//	Vab[LoopCnt] = Uab;
//	Vbc[LoopCnt] = Ubc;
//	Vca[LoopCnt] = Uca;
//	iA[LoopCnt] = Ia;
//	iB[LoopCnt] = Ib;
//	iC[LoopCnt] = Ic;
//
//	if(Ud<UNC_VD)
//	{
//		downCnt++;
//		if(downCnt == overMAX)
//		{
//			epwm_flag = 0;
//		}
//	}
//	else
//	{
//		downCnt = 0;
//	}
//
//	if(Ud>UNC_VD)
//	{
//		upCnt++;
//	}
//	else
//	{
//		upCnt = 0;
//		if(upCnt == overMAX)
//		{
//			epwm_flag = 1;
//		}
//	}
//
//  // Initial uncontrolled rectifier
//  if(epwm_flag==0)						// uncontrolled rectifier: all IGBTs off
//  {
//
//	  EPwm4Regs.DBCTL.bit.OUT_MODE = 0;
//	  EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
//	  EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
//	  EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;
//	  EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;
//	  EPwm4Regs.DBCTL.bit.POLSEL = 0;
//
//	  EPwm5Regs.DBCTL.bit.OUT_MODE = 0;
//	  EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
//	  EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
//	  EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;
//	  EPwm5Regs.AQCTLB.bit.CAD = AQ_SET;
//	  EPwm5Regs.DBCTL.bit.POLSEL = 0;
//
//	  EPwm6Regs.DBCTL.bit.OUT_MODE = 0;
//	  EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
//	  EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
//	  EPwm6Regs.AQCTLB.bit.CAU = AQ_SET;
//	  EPwm6Regs.AQCTLB.bit.CAD = AQ_SET;
//	  EPwm6Regs.DBCTL.bit.POLSEL = 0;
//  }
//  else if(epwm_flag==1)								// PWM rectifier
//  {
//
//	// Set actions
//	EPwm4Regs.DBCTL.bit.OUT_MODE = 3;
//	EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
//	EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
//	EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;
//	EPwm4Regs.AQCTLB.bit.CAD = AQ_CLEAR;
//	EPwm4Regs.DBCTL.bit.POLSEL = 1;
//
//	EPwm5Regs.DBCTL.bit.OUT_MODE = 3;
//	EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
//	EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
//	EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;
//	EPwm5Regs.AQCTLB.bit.CAD = AQ_CLEAR;
//	EPwm5Regs.DBCTL.bit.POLSEL = 1;
//
//	EPwm6Regs.DBCTL.bit.OUT_MODE = 3;
//	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
//	EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
//	EPwm6Regs.AQCTLB.bit.CAU = AQ_SET;
//	EPwm6Regs.AQCTLB.bit.CAD = AQ_CLEAR;
//	EPwm6Regs.DBCTL.bit.POLSEL = 1;
//
//	// control loop
//	Vdc_err = STAND_VD - Ud;
//	Idx += Ki_i*Vdc_err + Kp_i*(Vdc_err-Vdc_err_p);
//	if(Idx>6)
//	{
//		Idx = 6;
//	}
//	else if(Idx<-6)
//	{
//		Idx = -6;
//	}
//	Vdc_err_p = Vdc_err;
//
//	Id = 0.666667*(Ia*cos(phaA)+Ib*cos(phaB)+Ic*cos(phaC));
//	Iq = -0.666667*(Ia*sin(phaA)+Ib*sin(phaB)+Ic*sin(phaC));
//
//	Idy = Idx - Id;
//	Iqy = Iqx - Iq;
//
//	uq1 += Ki_u*Idy + Kp_u*(Idy-Idy_p);
//	if(uq1>200)
//	{
//		uq1 = 200;
//	}
//	else if(uq1<-200)
//	{
//		uq1 = -200;
//	}
//	Idy_p = Idy;
//	uq2 = 100*pi*0.01*Iq;
//
//	ud1 = 100*pi*0.01*Id;
//	ud2 += Ki_u*Iqy + Kp_u*(Iqy-Iqy_p);
//	if(ud2>200)
//	{
//		ud2 = 200;
//	}
//	else if(ud2<-200)
//	{
//		ud2 = -200;
//	}
//	Iqy_p = Iqy;
//
//	ud = 0.222222*((Uab-Uca)*cos(phaA)+(Ubc-Uab)*cos(phaB)+(Uca-Ubc)*cos(phaC));
//	uq = -0.222222*((Uab-Uca)*sin(phaA)+(Ubc-Uab)*sin(phaB)+(Uca-Ubc)*sin(phaC));
//
//	vd = uq-uq1-uq2;
//	vq = ud+ud1-ud2;
//
//	if(Ud>1)
//	{
//		v_alpha = (cos(phaA)*vd-sin(phaA)*vq)/(0.666667*Ud);
//		v_beta = (sin(phaA)*vd+cos(phaA)*vq)/(0.666667*Ud);
//	}
//	else
//	{
//		v_alpha = (cos(phaA)*vd-sin(phaA)*vq)/0.666667;
//		v_beta = (sin(phaA)*vd+cos(phaA)*vq)/0.666667;
//	}

	// PWM output

//	sector = 0;
//
//	Va = Uab/50;
//	Vb = Ubc/50;
//	Vc = Uca/50;
//
//	if(Va>0)
//	{
//		sector = 1;
//	}
//	if(Vb>0)
//	{
//		sector += 2;
//	}
//	if(Vc>0)
//	{
//		sector += 4;
//	}
//
//	if(sector == 0)
//	{
//		Ta = 0.5;
//		Tb = 0.5;
//		Tc = 0.5;
//	}
//	else if(sector == 1)
//	{
//		t1 = Vc;
//		t2 = Vb;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Tb = 0.5*(1-t1-t2);
//		Ta = Tb + t1;
//		Tc = Ta + t2;
//	}
//	else if(sector == 2)
//	{
//		t1 = Vb;
//		t2 = -Va;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Ta = 0.5*(1-t1-t2);
//		Tc = Ta+t1;
//		Tb = Tc+t2;
//	}
//	else if(sector == 3)
//	{
//		t1 = -Vc;
//		t2 = Va;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Ta = 0.5*(1-t1-t2);
//		Tb = Ta+t1;
//		Tc = Tb+t2;
//	}
//	else if(sector == 4)
//	{
//		t1 = -Va;
//		t2 = Vc;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Tc = 0.5*(1-t1-t2);
//		Tb = Tc+t1;
//		Ta = Tb+t2;
//	}
//	else if(sector == 5)
//	{
//		t1 = Va;
//		t2 = -Vb;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Tb = 0.5*(1-t1-t2);
//		Tc = Tb+t1;
//		Ta = Tc+t2;
//	}
//	else if(sector == 6)
//	{
//		t1 = -Vb;
//		t2 = -Vc;
//		t12 = t1+t2;
//		if(t12>1)
//		{
//			t1 = t1/t12;
//			t2 = t2/t12;
//		}
//		Tc = 0.5*(1-t1-t2);
//		Ta = Tc+t1;
//		Tb = Ta+t2;
//	}
//
//	cmpa = (int) (Ta*EPWM_PERIOD);
//	cmpb = (int) (Tb*EPWM_PERIOD);
//	cmpc = (int) (Tc*EPWM_PERIOD);
//	if(cmpa<0)
//	{
//		cmpa = 0;
//	}
//	else if(cmpa>EPWM_PERIOD-1)
//	{
//		cmpa = EPWM_PERIOD-1;
//	}
//	if(cmpb<0)
//	{
//		cmpb = 0;
//	}
//	else if(cmpb>EPWM_PERIOD-1)
//	{
//		cmpb = EPWM_PERIOD-1;
//	}
//	if(cmpc<0)
//	{
//		cmpc = 0;
//	}
//	else if(cmpc>EPWM_PERIOD-1)
//	{
//		cmpc = EPWM_PERIOD-1;
//	}
//
//	EPwm4Regs.CMPA.half.CMPA = cmpa;
//	EPwm5Regs.CMPA.half.CMPA = cmpb;
//	EPwm6Regs.CMPA.half.CMPA = cmpc;

	// for test
	EPwm4Regs.CMPA.half.CMPA = EPWM_CMPA_MAX*0.8;
	EPwm5Regs.CMPA.half.CMPA = EPWM_CMPA_MAX*0.2;
	EPwm6Regs.CMPA.half.CMPA = EPWM_CMPA_MAX*0.2;



  // Reinitialize for next ADC sequence
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}

void InitEPwm4()
{
	EPwm4Regs.TBPRD = EPWM_PERIOD;                         // Set timer period
	EPwm4Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
	EPwm4Regs.TBCTR = 0x0000;                       // Clear counter


	// Setup TBCLK
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT

	// Setup compare
	EPwm4Regs.CMPA.half.CMPA = EPWM_CMPA_MAX;

	// Set actions
	EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;


	EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;
	EPwm4Regs.AQCTLB.bit.CAD = AQ_CLEAR;

	// Active high complementary PWMs - Setup the deadband
	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm4Regs.DBCTL.bit.POLSEL = 1;
	EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm4Regs.DBRED = EPWM_MAX_DB;
	EPwm4Regs.DBFED = EPWM_MAX_DB;
}

void InitEPwm5()
{
   EPwm5Regs.TBPRD = EPWM_PERIOD;                        // Set timer period
   EPwm5Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm5Regs.TBCTR = 0x0000;                      // Clear counter

   // Setup TBCLK
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT

   // Setup compare
   EPwm5Regs.CMPA.half.CMPA = EPWM_CMPA_MAX;

   // Set actions
   EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
   EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;


   EPwm5Regs.AQCTLB.bit.CAU = AQ_SET;
   EPwm5Regs.AQCTLB.bit.CAD = AQ_CLEAR;

   // Active Low complementary PWMs - setup the deadband
   EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm5Regs.DBCTL.bit.POLSEL = 1;
   EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm5Regs.DBRED = EPWM_MAX_DB;
   EPwm5Regs.DBFED = EPWM_MAX_DB;
}

void InitEPwm6()
{
   EPwm6Regs.TBPRD = EPWM_PERIOD;                  // Set timer period
   EPwm6Regs.TBPHS.half.TBPHS = 0x0000;            // Phase is 0
   EPwm6Regs.TBCTR = 0x0000;                       // Clear counter


   // Setup TBCLK
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT

   // Setup compare
   EPwm6Regs.CMPA.half.CMPA = EPWM_CMPA_MAX;

   // Set actions
   EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
   EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;


   EPwm6Regs.AQCTLB.bit.CAU = AQ_SET;
   EPwm6Regs.AQCTLB.bit.CAD = AQ_CLEAR;

   // Active high complementary PWMs - Setup the deadband
   EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm6Regs.DBCTL.bit.POLSEL = 1;
   EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm6Regs.DBRED = EPWM_MAX_DB;
   EPwm6Regs.DBFED = EPWM_MAX_DB;
}

void
InitEPwm4Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO1 (EPWM1B)

    //
    // Configure ePWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM1 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

//
// InitEPwm5Gpio - This function initializes GPIO pins to function as ePWM2
//
void
InitEPwm5Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

    //
    // Configure ePWM-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM2 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

//
// InitEPwm6Gpio - This function initializes GPIO pins to function as ePWM3
//
void
InitEPwm6Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO5 (EPWM3B)

    //
    // Configure ePWM-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM3 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}


void PLL(void)
{

	// αβ transformation & dq transformation
//	Ud = 2/3*(Uab-1/2*Ubc-1/2*Uca)*cos(theta)+sqrt(3)/3*(Ubc-Uca)*sin(theta);
	Uq = sqrt(3)/3*(Ubc-Uca)*cos(theta)-2/3*(Uab-1/2*Ubc-1/2*Uca)*sin(theta);

//	delta = Kp_pll*(Uq-Uq_p)+Ki_pll*Uq+delta;
//	omega = OMG + delta;

	omega += Kp_pll*(Uq-Uq_p)+Ki_pll*Uq;
	// Saturation
	if(omega<300)
	{
		omega = 300;
	}
	else if(omega>350)
	{
		omega = 350;
	}

	theta = theta + omega/10000;

	// theta must be in the range of [0,2pi)
	theta = fmod(theta,2*pi);

	Uq_p = Uq;

	phaA = theta - pi/6 -PHASE_SHIFT;
	if(phaA<0)
	{
		phaA += 2*pi;
	}
	else if(phaA > 2*pi)
	{
		phaA -= 2*pi;
	}
	phaB = theta - pi/6 - 2*pi/3 - PHASE_SHIFT;
	if(phaB<0)
	{
		phaB += 2*pi;
	}
	else if(phaB > 2*pi)
	{
		phaB -= 2*pi;
	}
	phaC = theta - pi/6 - 4*pi/3 - PHASE_SHIFT;
	if(phaC<0)
	{
		phaC += 2*pi;
	}
	else if(phaC > 2*pi)
	{
		phaC -= 2*pi;
	}

	if(theta <= omega/10000)
	{
		viewAB = Uab;
		viewBC = Ubc;
		viewCA = Uca;
	}
	if(fabs(theta-pi*2/3) <= omega/10000)
	{
		AB90 = Uab;
		BC90 = Ubc;
		CA90 = Uca;
	}

	return;

}
