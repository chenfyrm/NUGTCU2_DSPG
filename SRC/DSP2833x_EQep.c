// TI File $Revision: /main/3 $
// Checkin $Date: July 27, 2007   11:55:20 $
//###########################################################################
//
// FILE:   DSP2833x_EQep.c
//
// TITLE:  DSP2833x eQEP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitEQep:
//---------------------------------------------------------------------------
// This function initializes the eQEP(s) to a known state.
//初始化QEP模块
//定时器的频率为SYSCLKOUT/(2^CCPS)
//单位位移事件的脉冲数为2^UPPS个
//光电变盘器的分辨率为RES count/rev.
//以最低转速SPEEDMIN rpm运行时，脉冲频率为SPEEDMIN/60*RES Hz
//单位位移事件对应的时间为2^UPPS/(SPEEDMIN/60*RES) s
//单位位移事件对应的定时器计数值为2^UPPS/(SPEEDMIN/60*RES)/(SYSCLKOUT/(2^CCPS))
//
//系统时钟为150MHz
//单位位移事件的脉冲数为32个
//脉冲分辨率设为2^14=16384->2^12=4096
//Rated Frequency 80Hz,Synchronous Rotation Speed 2400rpm, Max. Speed 3000rpm
//Max. Speed is 3000rpm计，Min. Speed is 1rpm
//以最低转速运行时单位位移事件对Φ氖奔湮?.01171875 s->0.046875
//以最低转速运行时单位位移事件对应的定时器计数值为13732.91016->54931
//
//单位时间窗设置为10ms，以最高转速运行时正交时钟计数器值为13333.33333->13653

void InitEQep(void)
{
	// Initialize eQEP1
	#if (CPU_FRQ_150MHZ)
	  EQep1Regs.QUPRD=750000;			// 5ms calculated once,Unit Timer for 100Hz at 150MHz SYSCLKOUT
	#endif
    #if (CPU_FRQ_100MHZ)
	  EQep1Regs.QUPRD=500000;			// 5ms Unit Timer for 100Hz at 100MHz SYSCLKOUT
	#endif
//Decoder control register
	EQep1Regs.QDECCTL.bit.XCR=0;        // 2x resolution (cnt falling and rising edges)
	EQep1Regs.QDECCTL.bit.QSRC=0;		// QEP quadrature count mode,pulse count
	EQep1Regs.QDECCTL.bit.QAP=0;
	EQep1Regs.QDECCTL.bit.QBP=0;
	EQep1Regs.QDECCTL.bit.QIP=0;		// No Index signal

//EQEP control register
	EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep1Regs.QEPCTL.bit.PCRM=1;		// QPOSCNT reset on unit time evnt
	EQep1Regs.QEPCTL.bit.IEI=0;
	EQep1Regs.QEPCTL.bit.IEL=1;
	EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timer Enable
	EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep1Regs.QPOSMAX=0xFFFFFFFF;
	EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

	#if (CPU_FRQ_150MHZ)
	  EQep1Regs.QCAPCTL.bit.UPPS=2;   	// 1/4 for unit position at 150MHz SYSCLKOUT
	#endif
	#if (CPU_FRQ_100MHZ)
	  EQep1Regs.QCAPCTL.bit.UPPS=3;   	// 1/8 for unit position at 100MHz SYSCLKOUT
	#endif

	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable

	

   // Initialize eQEP2
	// Initialize eQEP1
	#if (CPU_FRQ_150MHZ)
	  EQep2Regs.QUPRD=750000;			// 20ms calculated once,Unit Timer for 100Hz at 150MHz SYSCLKOUT
	#endif
    #if (CPU_FRQ_100MHZ)
	  EQep2Regs.QUPRD=500000;			// 20ms,Unit Timer for 100Hz at 100MHz SYSCLKOUT
	#endif
//Decoder control register
	EQep2Regs.QDECCTL.bit.XCR=0;        // 2x resolution (cnt falling and rising edges)
	EQep2Regs.QDECCTL.bit.QSRC=0;		// QEP quadrature count mode,pulse count
	EQep2Regs.QDECCTL.bit.QAP=0;
	EQep2Regs.QDECCTL.bit.QBP=0;
	EQep2Regs.QDECCTL.bit.QIP=0;		// No Index signal

//EQEP control register
	EQep2Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep2Regs.QEPCTL.bit.PCRM=1;		// QPOSCNT reset on unit time evnt
	EQep2Regs.QEPCTL.bit.IEI=0;
	EQep2Regs.QEPCTL.bit.IEL=1;
	EQep2Regs.QEPCTL.bit.UTE=1; 		// Unit Timer Enable
	EQep2Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep2Regs.QPOSMAX=0xFFFFFFFF;
	EQep2Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

	#if (CPU_FRQ_150MHZ)
	  EQep2Regs.QCAPCTL.bit.UPPS=2;   	// 1/4 for unit position at 150MHz SYSCLKOUT
	#endif
	#if (CPU_FRQ_100MHZ)
	  EQep2Regs.QCAPCTL.bit.UPPS=3;   	// 1/8 for unit position at 100MHz SYSCLKOUT
	#endif

	EQep2Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep2Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable

}
//---------------------------------------------------------------------------
// Example: InitEQepGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as eQEP pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each eQEP peripheral
// Only one GPIO pin should be enabled for EQEPxA operation.
// Only one GPIO pin should be enabled for EQEPxB operation.
// Only one GPIO pin should be enabled for EQEPxS operation.
// Only one GPIO pin should be enabled for EQEPxI operation.
// Comment out other unwanted lines.

void InitEQepGpio(void)
{
#if DSP28_EQEP1
   InitEQep1Gpio();
#endif  // endif DSP28_EQEP1
#if DSP28_EQEP2
   InitEQep2Gpio();
#endif // endif DSP28_EQEP2
}

#if DSP28_EQEP1
void InitEQep1Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pull-up on GPIO22 (EQEP1S)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pull-up on GPIO23 (EQEP1I)

//    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // Enable pull-up on GPIO50 (EQEP1A)
//    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // Enable pull-up on GPIO51 (EQEP1B)
//    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;   // Enable pull-up on GPIO52 (EQEP1S)
//    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // Enable pull-up on GPIO53 (EQEP1I)


// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;   // Sync to SYSCLKOUT GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;   // Sync to SYSCLKOUT GPIO21 (EQEP1B)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0;   // Sync to SYSCLKOUT GPIO22 (EQEP1S)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;   // Sync to SYSCLKOUT GPIO23 (EQEP1I)

//    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;   // Sync to SYSCLKOUT GPIO50 (EQEP1A)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;   // Sync to SYSCLKOUT GPIO51 (EQEP1B)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 0;   // Sync to SYSCLKOUT GPIO52 (EQEP1S)
//    GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0;   // Sync to SYSCLKOUT GPIO53 (EQEP1I)

/* Configure eQEP-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eQEP1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B
//    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EQEP1S
//    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EQEP1I

//    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;   // Configure GPIO50 as EQEP1A
//    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;   // Configure GPIO51 as EQEP1B
//    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 1;   // Configure GPIO52 as EQEP1S
//    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;   // Configure GPIO53 as EQEP1I


    EDIS;
}
#endif // if DSP28_EQEP1



#if DSP28_EQEP2
void InitEQep2Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;    // Enable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // Enable pull-up on GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;    // Enable pull-up on GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27 (EQEP2S)

// Inputs are synchronized to SYSCLKOUT by default.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;  // Sync to SYSCLKOUT GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;  // Sync to SYSCLKOUT GPIO25 (EQEP2B)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;  // Sync to SYSCLKOUT GPIO26 (EQEP2I)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  // Sync to SYSCLKOUT GPIO27 (EQEP2S)

/* Configure eQEP-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eQEP2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;   // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;   // Configure GPIO25 as EQEP2B
//    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;   // Configure GPIO26 as EQEP2I
//    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2;   // Configure GPIO27 as EQEP2S


    EDIS;
}
#endif // endif DSP28_EQEP2




//===========================================================================
// End of file.
//===========================================================================
