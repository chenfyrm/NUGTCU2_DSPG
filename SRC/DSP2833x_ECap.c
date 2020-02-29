// TI File $Revision: /main/2 $
// Checkin $Date: March 15, 2007   16:54:36 $
//###########################################################################
//
// FILE:   DSP2833x_ECap.c
//
// TITLE:  DSP2833x eCAP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitECap:
//---------------------------------------------------------------------------
// This function initializes the eCAP(s) to a known state.
//
void InitECap1(void)
{
	ECap1Regs.ECCTL2.bit.TSCTRSTOP=0;

//停止TSCTR计数
	ECap1Regs.TSCTR=0;
	ECap1Regs.CAP1=0;
	ECap1Regs.CAP2=0;
	ECap1Regs.CAP3=0;
	ECap1Regs.CAP4=0;
//计数器清零，捕获寄存器清零
	ECap1Regs.ECCTL1.bit.FREE_SOFT=3;//遇到仿真断点继续运行
	ECap1Regs.ECCTL1.bit.PRESCALE=0;//捕获不分频
	ECap1Regs.ECCTL1.bit.CAPLDEN=1;//使能捕获

	ECap1Regs.ECCTL1.bit.CTRRST4=0;
//捕获后是否复位计数器
//0：不复位
//1：复位
	ECap1Regs.ECCTL1.bit.CAP4POL=1;//下降沿触发
//捕获极性
//0：上升沿
//1：下降沿
	ECap1Regs.ECCTL1.bit.CTRRST3=0;
	ECap1Regs.ECCTL1.bit.CAP3POL=0;
	ECap1Regs.ECCTL1.bit.CTRRST2=0;
	ECap1Regs.ECCTL1.bit.CAP2POL=0;
	ECap1Regs.ECCTL1.bit.CTRRST1=1;
	ECap1Regs.ECCTL1.bit.CAP1POL=0;

	ECap1Regs.ECCTL2.bit.APWMPOL=0;
	ECap1Regs.ECCTL2.bit.CAP_APWM=0;//工作在捕获模式
	ECap1Regs.ECCTL2.bit.SWSYNC=0;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL=3;
	ECap1Regs.ECCTL2.bit.SYNCI_EN=0;
	ECap1Regs.ECCTL2.bit.REARM=1;
	ECap1Regs.ECCTL2.bit.STOP_WRAP=0;//在CAP1后复位
	ECap1Regs.ECCTL2.bit.CONT_ONESHT=0;//工作在连续模式

	ECap1Regs.ECCLR.all=0x00FF;//清除之前的各种中断标志位

	ECap1Regs.ECEINT.bit.CTR_EQ_CMP=0;
	ECap1Regs.ECEINT.bit.CTR_EQ_PRD=0;
	ECap1Regs.ECEINT.bit.CTROVF=0;
	ECap1Regs.ECEINT.bit.CEVT4=0;
	ECap1Regs.ECEINT.bit.CEVT3=0;
	ECap1Regs.ECEINT.bit.CEVT2=0;
	ECap1Regs.ECEINT.bit.CEVT1=1;//使能CAP1中断

	ECap1Regs.ECCTL2.bit.TSCTRSTOP=1;//计数器计数开始
}
void InitECap2(void)
{
	ECap2Regs.ECCTL2.bit.TSCTRSTOP=0;//TSCTR Stopped

	ECap2Regs.TSCTR=0;//Clear TSCTR
	ECap2Regs.CAP1=0;//Clear CAP1
	ECap2Regs.CAP2=0;//Clear CAP2
	ECap2Regs.CAP3=0;//Clear CAP3
	ECap2Regs.CAP4=0;//Clear CAP4

	ECap2Regs.ECCTL1.bit.FREE_SOFT=3;//Run freely
	ECap2Regs.ECCTL1.bit.PRESCALE=0;//Event not prescale
	ECap2Regs.ECCTL1.bit.CAPLDEN=1;//Enable loading of CAP1-4

//Capture polarity
//0:rising edge
//1:falling edge

//Reset or not reset TSCTR when a capture event happen
//0:not reset
//1:reset
	ECap2Regs.ECCTL1.bit.CAP1POL=0;//Rising edge
	ECap2Regs.ECCTL1.bit.CAP2POL=1;//Falling edge
	ECap2Regs.ECCTL1.bit.CAP3POL=0;
	ECap2Regs.ECCTL1.bit.CAP4POL=0;

	ECap2Regs.ECCTL1.bit.CTRRST1=1;//Reset TSCTR
	ECap2Regs.ECCTL1.bit.CTRRST2=1;//Reset TSCTR
	ECap2Regs.ECCTL1.bit.CTRRST3=0;
	ECap2Regs.ECCTL1.bit.CTRRST4=0;


	ECap2Regs.ECCTL2.bit.APWMPOL=0;
	ECap2Regs.ECCTL2.bit.CAP_APWM=0;//CAP moad
	ECap2Regs.ECCTL2.bit.SWSYNC=0;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL=3;
	ECap2Regs.ECCTL2.bit.SYNCI_EN=0;
	ECap2Regs.ECCTL2.bit.REARM=1;//Rearm action
	ECap2Regs.ECCTL2.bit.STOP_WRAP=1;//warp after cap2 event
	ECap2Regs.ECCTL2.bit.CONT_ONESHT=0;//continues mode

	ECap2Regs.ECCLR.all=0x00FF;//Claer flag
	ECap2Regs.ECEINT.bit.CTR_EQ_CMP=0;
	ECap2Regs.ECEINT.bit.CTR_EQ_PRD=0;
	ECap2Regs.ECEINT.bit.CTROVF=0;
	ECap2Regs.ECEINT.bit.CEVT4=0;
	ECap2Regs.ECEINT.bit.CEVT3=0;
	ECap2Regs.ECEINT.bit.CEVT2=1;//Enable CAP2 interrupt
	ECap2Regs.ECEINT.bit.CEVT1=1;//Enable CAP1 interrupt

	ECap2Regs.ECCTL2.bit.TSCTRSTOP=1;//Start capture
	
}
void InitECap3(void)
{
	ECap3Regs.ECCTL2.bit.TSCTRSTOP=0;//TSCTR Stopped

	ECap3Regs.TSCTR=0;//Clear TSCTR
	ECap3Regs.CAP1=0;//Clear CAP1
	ECap3Regs.CAP2=0;//Clear CAP2
	ECap3Regs.CAP3=0;//Clear CAP3
	ECap3Regs.CAP4=0;//Clear CAP4

	ECap3Regs.ECCTL1.bit.FREE_SOFT=3;//Run freely
	ECap3Regs.ECCTL1.bit.PRESCALE=0;//Event not prescale
	ECap3Regs.ECCTL1.bit.CAPLDEN=1;//Enable loading of CAP1-4

//Capture polarity
//0:rising edge
//1:falling edge

//Reset or not reset TSCTR when a capture event happen
//0:not reset
//1:reset
	ECap3Regs.ECCTL1.bit.CAP1POL=0;//Rising edge
	ECap3Regs.ECCTL1.bit.CAP2POL=1;//Falling edge
	ECap3Regs.ECCTL1.bit.CAP3POL=0;
	ECap3Regs.ECCTL1.bit.CAP4POL=0;

	ECap3Regs.ECCTL1.bit.CTRRST1=1;//Reset TSCTR
	ECap3Regs.ECCTL1.bit.CTRRST2=1;//Reset TSCTR
	ECap3Regs.ECCTL1.bit.CTRRST3=0;
	ECap3Regs.ECCTL1.bit.CTRRST4=0;


	ECap3Regs.ECCTL2.bit.APWMPOL=0;
	ECap3Regs.ECCTL2.bit.CAP_APWM=0;//CAP moad
	ECap3Regs.ECCTL2.bit.SWSYNC=0;
	ECap3Regs.ECCTL2.bit.SYNCO_SEL=3;
	ECap3Regs.ECCTL2.bit.SYNCI_EN=0;
	ECap3Regs.ECCTL2.bit.REARM=1;//Rearm action
	ECap3Regs.ECCTL2.bit.STOP_WRAP=1;//warp after cap2 event
	ECap3Regs.ECCTL2.bit.CONT_ONESHT=0;//continues mode

	ECap3Regs.ECCLR.all=0x00FF;//Claer flag
	ECap3Regs.ECEINT.bit.CTR_EQ_CMP=0;
	ECap3Regs.ECEINT.bit.CTR_EQ_PRD=0;
	ECap3Regs.ECEINT.bit.CTROVF=0;
	ECap3Regs.ECEINT.bit.CEVT4=0;
	ECap3Regs.ECEINT.bit.CEVT3=0;
	ECap3Regs.ECEINT.bit.CEVT2=1;//Enable CAP2 interrupt
	ECap3Regs.ECEINT.bit.CEVT1=1;//Enable CAP1 interrupt

	ECap3Regs.ECCTL2.bit.TSCTRSTOP=1;//Start capture
	
}
void InitECap5(void)
{
	ECap5Regs.ECCTL2.bit.TSCTRSTOP=0;
//停止TSCTR计数
	ECap5Regs.TSCTR=0;
	ECap5Regs.CAP1=0;
	ECap5Regs.CAP2=0;
	ECap5Regs.CAP3=0;
	ECap5Regs.CAP4=0;
//计数器清零，捕获寄存器清零
	ECap5Regs.ECCTL1.bit.FREE_SOFT=3;//遇到仿真断点继续运行
	ECap5Regs.ECCTL1.bit.PRESCALE=0;//捕获不分频
	ECap5Regs.ECCTL1.bit.CAPLDEN=1;//使能捕获

	ECap5Regs.ECCTL1.bit.CTRRST4=0;
//捕获后是否复位计数器
//0：不复位
//1：复位
	ECap5Regs.ECCTL1.bit.CAP4POL=0;
//捕获极性
//0：上升沿
//1：下降沿
	ECap5Regs.ECCTL1.bit.CTRRST3=0;
	ECap5Regs.ECCTL1.bit.CAP3POL=0;
	ECap5Regs.ECCTL1.bit.CTRRST2=1;
	ECap5Regs.ECCTL1.bit.CAP2POL=1;
	ECap5Regs.ECCTL1.bit.CTRRST1=1;
	ECap5Regs.ECCTL1.bit.CAP1POL=0;

	ECap5Regs.ECCTL2.bit.APWMPOL=0;
	ECap5Regs.ECCTL2.bit.CAP_APWM=0;//工作在捕获模式
	ECap5Regs.ECCTL2.bit.SWSYNC=0;
	ECap5Regs.ECCTL2.bit.SYNCO_SEL=3;
	ECap5Regs.ECCTL2.bit.SYNCI_EN=0;
	ECap5Regs.ECCTL2.bit.REARM=1;
	ECap5Regs.ECCTL2.bit.STOP_WRAP=1;//在CAP2后复位
	ECap5Regs.ECCTL2.bit.CONT_ONESHT=0;//工作在连续模式

	ECap5Regs.ECCLR.all=0x00FF;//清除之前的各种中断标志位

	ECap5Regs.ECEINT.bit.CTR_EQ_CMP=0;
	ECap5Regs.ECEINT.bit.CTR_EQ_PRD=0;
	ECap5Regs.ECEINT.bit.CTROVF=0;
	ECap5Regs.ECEINT.bit.CEVT4=0;
	ECap5Regs.ECEINT.bit.CEVT3=0;
	ECap5Regs.ECEINT.bit.CEVT2=0;//使能CAP2中断
	ECap5Regs.ECEINT.bit.CEVT1=1;//使能CAP1中断

	ECap5Regs.ECCTL2.bit.TSCTRSTOP=1;//计数器计数开始

}
void InitECap6(void)
{	
	ECap6Regs.ECCTL2.bit.TSCTRSTOP=0;
	ECap6Regs.TSCTR=0;
	ECap6Regs.CAP1=0;
	ECap6Regs.CAP2=0;
	ECap6Regs.CAP3=0;
	ECap6Regs.CAP4=0;

	ECap6Regs.ECCTL1.bit.FREE_SOFT=3;
	ECap6Regs.ECCTL1.bit.PRESCALE=0;
	ECap6Regs.ECCTL1.bit.CAPLDEN=1;
	ECap6Regs.ECCTL1.bit.CTRRST4=0;
	ECap6Regs.ECCTL1.bit.CAP4POL=0;
//捕获极性
//0：上升沿
//1：下降沿
	ECap6Regs.ECCTL1.bit.CTRRST3=0;
	ECap6Regs.ECCTL1.bit.CAP3POL=0;
	ECap6Regs.ECCTL1.bit.CTRRST2=0;
	ECap6Regs.ECCTL1.bit.CAP2POL=0;
	ECap6Regs.ECCTL1.bit.CTRRST1=0;
	ECap6Regs.ECCTL1.bit.CAP1POL=1;

	ECap6Regs.ECCTL2.bit.APWMPOL=0;
	ECap6Regs.ECCTL2.bit.CAP_APWM=0;
	ECap6Regs.ECCTL2.bit.SWSYNC=0;
	ECap6Regs.ECCTL2.bit.SYNCO_SEL=3;
	ECap6Regs.ECCTL2.bit.SYNCI_EN=0;
	ECap6Regs.ECCTL2.bit.REARM=1;
	ECap6Regs.ECCTL2.bit.STOP_WRAP=0;
	ECap6Regs.ECCTL2.bit.CONT_ONESHT=1;//1:one-shot模式   0:连续模式

	ECap6Regs.ECCLR.all=0x00FF;

	ECap6Regs.ECEINT.bit.CTR_EQ_CMP=0;
	ECap6Regs.ECEINT.bit.CTR_EQ_PRD=0;
	ECap6Regs.ECEINT.bit.CTROVF=0;
	ECap6Regs.ECEINT.bit.CEVT4=0;
	ECap6Regs.ECEINT.bit.CEVT3=0;
	ECap6Regs.ECEINT.bit.CEVT2=0;
	ECap6Regs.ECEINT.bit.CEVT1=1;

	ECap6Regs.ECCTL2.bit.TSCTRSTOP=1;

	
}

//---------------------------------------------------------------------------
// Example: InitECapGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ECAP pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each eCAP peripheral
// Only one GPIO pin should be enabled for ECAP operation.
// Comment out other unwanted lines.

void InitECapGpio()
{

   InitECap1Gpio();
#if (DSP28_ECAP2)
   InitECap2Gpio();
#endif // endif DSP28_ECAP2
#if (DSP28_ECAP3)
   InitECap3Gpio();
#endif // endif DSP28_ECAP3
#if (DSP28_ECAP4)
   InitECap4Gpio();
#endif // endif DSP28_ECAP4
#if (DSP28_ECAP5)
   InitECap5Gpio();
#endif // endif DSP28_ECAP5
#if (DSP28_ECAP6)
   InitECap6Gpio();
#endif // endif DSP28_ECAP6
}

void InitECap1Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;      // Enable pull-up on GPIO5 (CAP1)
   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;     // Enable pull-up on GPIO24 (CAP1)
// GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;     // Enable pull-up on GPIO34 (CAP1)


// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5 (CAP1)
   GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;   // Synch to SYSCLKOUT GPIO24 (CAP1)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO34 = 0;   // Synch to SYSCLKOUT GPIO34 (CAP1)

/* Configure eCAP-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP1 functional pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3;     // Configure GPIO5 as CAP1
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;    // Configure GPIO24 as CAP1
// GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 1;    // Configure GPIO24 as CAP1

    EDIS;
}

#if DSP28_ECAP2
void InitECap2Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up on GPIO7 (CAP2)
// GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // Enable pull-up on GPIO25 (CAP2)
// GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;    // Enable pull-up on GPIO37 (CAP2)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0;    // Synch to SYSCLKOUT GPIO7 (CAP2)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;   // Synch to SYSCLKOUT GPIO25 (CAP2)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO37 = 0;   // Synch to SYSCLKOUT GPIO37 (CAP2)

/* Configure eCAP-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP2 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 3;    // Configure GPIO7 as CAP2
// GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;   // Configure GPIO25 as CAP2
// GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;   // Configure GPIO37 as CAP2

    EDIS;
}
#endif // endif DSP28_ECAP2

#if DSP28_ECAP3
void InitECap3Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;      // Enable pull-up on GPIO9 (CAP3)
// GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;     // Enable pull-up on GPIO26 (CAP3)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 0;    // Synch to SYSCLKOUT GPIO9 (CAP3)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;   // Synch to SYSCLKOUT GPIO26 (CAP3)

/* Configure eCAP-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP3 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 3;     // Configure GPIO9 as CAP3
// GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;    // Configure GPIO26 as CAP3

    EDIS;
}
#endif // endif DSP28_ECAP3


#if DSP28_ECAP4
void InitECap4Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pull-up on GPIO11 (CAP4)
// GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;   // Enable pull-up on GPIO27 (CAP4)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0; // Synch to SYSCLKOUT GPIO11 (CAP4)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0; // Synch to SYSCLKOUT GPIO27 (CAP4)

/* Configure eCAP-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP4 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 3;  // Configure GPIO11 as CAP4
// GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;  // Configure GPIO27 as CAP4

    EDIS;
}
#endif // endif DSP28_ECAP4


#if DSP28_ECAP5
void InitECap5Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;     // Enable pull-up on GPIO3 (CAP5)
// GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (CAP5)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 0;  // Synch to SYSCLKOUT GPIO3 (CAP5)
// GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0; // Synch to SYSCLKOUT GPIO48 (CAP5)

/* Configure eCAP-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP5 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;   // Configure GPIO3 as CAP5
// GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;  // Configure GPIO48 as CAP5

    EDIS;
}
#endif // endif DSP28_ECAP5


#if DSP28_ECAP6
void InitECap6Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;     // Enable pull-up on GPIO1 (CAP6)
// GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;    // Enable pull-up on GPIO49 (CAP6)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 0;  // Synch to SYSCLKOUT GPIO1 (CAP6)
// GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 0; // Synch to SYSCLKOUT GPIO49 (CAP6)

/* Configure eCAP-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAP6 functional pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 2;   // Configure GPIO1 as CAP6
// GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 1;  // Configure GPIO49 as CAP6

    EDIS;
}
#endif // endif DSP28_ECAP6



//===========================================================================
// End of file.
//===========================================================================
