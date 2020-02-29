// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:25 $
//###########################################################################
//
// FILE:	DSP2833x_Gpio.c
//
// TITLE:	DSP2833x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 
void InitGpio(void)
{
	//GPIO0/2/4/6/8, Set in InitEPwmGpio() of epwm.c
	InitEPwmGpio();
	//GPIO30/31,Set in ECan.c
	InitECanGpio();
	//GPI14/15,Set in SCI.c
	InitSciGpio();
	//GPIO16/17/18/19,Set in Spi.c
	InitSpiaGpio();
	//GPIO20/21 and GPIO24/25,Set in EQep.c
	InitEQepGpio();
	//GPIO28/34/36~38/40~47/64~86,Set in Xintf.c
	InitXintf();
	//GPIO32/33,Set in I2C.c
	InitI2CGpio();

	EALLOW;
	//Sampling period
//	GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0x0A;//GPIO0-7
	GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0x0A;//GPIO8-15
	GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 0x0A;//GPIO16-23
//	GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 0x0A;//GPIO24-31
//	GpioCtrlRegs.GPBCTRL.bit.QUALPRD0 = 0x0A;//GPIO32-39
//	GpioCtrlRegs.GPBCTRL.bit.QUALPRD1 = 0x0A;//GPIO40-47
//	GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 0x0A;//GPIO48-55
//	GpioCtrlRegs.GPBCTRL.bit.QUALPRD3 = 0x0A;//GPIO56-63


	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;//Select as Output,Signal for FPGA, indicated DSP is in error

	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;//Select as Output,Reserved

	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;//Select as Output,Reserved

	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;//Select as Output,SP switch

	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;//Select as Input,interrupt source

	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;//Select as Input,interrupt source

	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;//Select as Input,interrupt source

	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;//Select as Input,Signal for FPGA, indicated DSP is On
//	GpioDataRegs.GPACLEAR.bit.GPIO12=1;

	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;//Select as Input,Signal for FPGA, PWM enable
	
	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 1;  // 3 Samples for input GPIO14 
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;//Select as Intput,Reserved

	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;//Ensable Internal Pull-up
	GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 1;  // 3 Samples for input GPIO15 
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;//Select as Intput,Reserved

	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 1; // 3 Samples for input GPIO22 
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;//Select as Intput,RSD for eQep1S

	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 1; // 3 Samples for input GPIO23 
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;//Select as Intput,RSD for eQep1I

	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 1; // 3 Samples for input GPIO26 
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;//Select as Intput,RSD for eQep2I

	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 1; // 3 Samples for input GPIO27 
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;//Select as Intput,RSD for eQep2S
	
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;//Select as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;//Select as Output,Reserved

	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;//Select as Output,Reserved
	
	GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;//Select as Output,Reserved
	
	GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;//Select as Output,Signal to FPGA,as RS485 control
	
	GpioCtrlRegs.GPBPUD.bit.GPIO49 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 1; // 3 Samples for input GPIO49 
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;//Select as Intput,RSD for FPGA

	GpioCtrlRegs.GPBPUD.bit.GPIO50 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0;//Select as Input,Signal from AD7606-1,as FDATA
	
	GpioCtrlRegs.GPBPUD.bit.GPIO51 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;//Select as Input,Signal from AD7606-1,as Busy
	
	GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;//Enable Internal Pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;//Select as Output,Signal to AD7606-1,as CS

	GpioCtrlRegs.GPBPUD.bit.GPIO53 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;//Select as Output,Signal to AD7606-1 and 2,as Reset
	
	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;//Select as Output,Signal to AD7606-1 and 2,as Convst
	
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 1; // 3 Samples for input GPIO55 
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0;//Select as Input,Signal from AD7606-2,as FDATA
	
	GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;//Select as Input,Signal from AD7606-2,as Busy
	
	GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;//Select as Output,Signal to AD7606-2,as CS

	GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;//Select as Output,NC
	
	GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;//Select as Output,NC
	
	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;//Enable Internal Pull-up 
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;//Select as Output,NC
	
	GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0;//Select as Input,Signal from FPGA Init
	
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 1; // 3 Samples for input GPIO62 
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0;//Select as Intput,RSD for FPGA
	
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 1; // 3 Samples for input GPIO63 
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;//Select as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0;//Select as Intput, INTR for DPRAM

	GpioCtrlRegs.GPCPUD.bit.GPIO84 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;//Select as GPIO
	GpioCtrlRegs.GPCDIR.bit.GPIO84 = 0;//Select as Intput,RSD for Switch M0

	GpioCtrlRegs.GPCPUD.bit.GPIO85 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;//Select as GPIO
	GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0;//Select as Intput,RSD for Switch M1

	GpioCtrlRegs.GPCPUD.bit.GPIO86 = 1;//Disable Internal Pull-up
	GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;//Select as GPIO
	GpioCtrlRegs.GPCDIR.bit.GPIO86 = 0;//Select as Intput,RSD for Switch M2
	
	GpioCtrlRegs.GPCPUD.bit.GPIO87 = 1;//Disable Internal Pull-up 
	GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;//Select as GPIO
	GpioCtrlRegs.GPCDIR.bit.GPIO87 = 0;//Select as Intput,RSD for Switch M3 

	EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
