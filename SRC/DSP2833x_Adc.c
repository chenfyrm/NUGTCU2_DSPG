// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#define ADC_usDELAY  5000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{		
	extern void DSP28x_usDelay(Uint32 Count);

	AdcRegs.ADCTRL1.bit.RESET=1;
	asm(" RPT #1 || NOP");

    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

//************initialized in DSP2833x_SysCtrl.c***************
	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;
//************************************************************	
    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_CLOCK_SPEED define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
// Specific ADC setup for this example:
   AdcRegs.ADCTRL1.bit.ACQ_PS = 0x3;		// S/H width in ADC module periods        = 6 ADC cycle  
   						// Sequential mode: Sample rate   = 1/[(2+ACQ_PS)*ADC clock in ns]
                        //                     = 1/(3*40ns) =8.3MHz (for 150 MHz SYSCLKOUT)
					    //                     = 1/(3*80ns) =4.17MHz (for 100 MHz SYSCLKOUT)
					    // If Simultaneous mode enabled: Sample rate = 1/[(3+ACQ_PS)*ADC clock in ns]
   AdcRegs.ADCTRL1.bit.CPS = 0; 
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // Cascaded mode
   AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // Setup start/stop run
   AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;		//Simultaneous sampling mode
//   AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;       // Enable Sequencer override feature
   AdcRegs.ADCTRL3.bit.ADCCLKPS = 5;	// ADC module clock = HSPCLK/10     = 75MHz/(10)   = 7.5 MHz
   
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;         // Initialize all ADC channel 
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;
   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;
   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;
   AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;          
   AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;
   AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
   AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;
//   AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;
//   AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0xD;
//   AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0x3;
//   AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xF;
//   AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0x0;

   AdcRegs.ADCMAXCONV.all = 0x07;  // convert and store in 16 results registers

   AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
//   AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;

   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0;
   AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;
   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0;
   AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 0;
 
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 0;
   AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2 = 0;
   AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 0;
   AdcRegs.ADCTRL2.bit.SOC_SEQ2 = 0;

   AdcRegs.ADCTRL1.bit.CONT_RUN = 1;       //每次启动AD转换只有当最大转换通道转换完成才置标志位
   AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;       // Sequencer override feature，转换完成之后通道指针重新开始
   //AdcRegs.ADCOFFTRIM.all = 0x004A;  //AD校正
}
//===========================================================================
// End of file.
//===========================================================================
