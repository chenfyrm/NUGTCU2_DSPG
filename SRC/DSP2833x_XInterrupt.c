#include "DSP2833x_Device.h"    
#include "DSP2833x_Examples.h"

void InitXInterrupt(void)
{
	EALLOW;
//	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=18;//Set GPIO18 as interrupt source
//	XIntruptRegs.XINT1CR.bit.POLARITY=0;//interrupt generated on a falling edge
//	XIntruptRegs.XINT1CR.bit.ENABLE=1;//Enable the interrupt
//	EDIS;

	IER|=M_INT1;
	IER|=M_INT3;
//	IER|=M_INT9;
//	PieCtrlRegs.PIEIER1.bit.INTx4=1;
	PieCtrlRegs.PIEIER1.bit.INTx7=1;
	PieCtrlRegs.PIEIER3.bit.INTx1=1;
//	PieCtrlRegs.PIEIER3.bit.INTx4=1;
//	PieCtrlRegs.PIEIER9.bit.INTx3=1;
   	EINT;
   	ERTM;

	EDIS;
}


