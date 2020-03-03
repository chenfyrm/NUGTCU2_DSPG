//################################################################################
// FILE:    Main.c
// TITLE:	NUGTCU_MCU
// DESCRIPTION: NUGTCU,Tint=250uS
//################################################################################
#include	"DSP2833x_Device.h"     
#include	"DSP2833x_Examples.h"  
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
#define		RLEN		0x1f;
//===========================================================================
//#define TX485()	GpioDataRegs.GPBSET.bit.GPIO48=1
//#define RX485()	GpioDataRegs.GPBCLEAR.bit.GPIO48=1
//===========================================================================
#define EN_PWM()	GpioDataRegs.GPASET.bit.GPIO3=1
#define DIS_PWM()	GpioDataRegs.GPACLEAR.bit.GPIO3=1
//===========================================================================
//#define EN_DA()	GpioDataRegs.GPBSET.bit.GPIO54=1
//#define DIS_DA()	GpioDataRegs.GPBCLEAR.bit.GPIO54=1
//===========================================================================
//#define FPGA_ALL_RST_EN()	GpioDataRegs.GPASET.bit.GPIO1=1
//#define FPGA_ALL_RST_DIS()	GpioDataRegs.GPACLEAR.bit.GPIO1=1
//===========================================================================
//#define FPGA_Err_RST_EN()	GpioDataRegs.GPASET.bit.GPIO3=1
//#define FPGA_Err_RST_DIS()	GpioDataRegs.GPACLEAR.bit.GPIO3=1
//===========================================================================
#define SP01_EN()	GpioDataRegs.GPASET.bit.GPIO7=1
#define SP01_DIS()	GpioDataRegs.GPACLEAR.bit.GPIO7=1
//===========================================================================
#define EN_INT()	GpioDataRegs.GPASET.bit.GPIO1=1
#define DIS_INT()	GpioDataRegs.GPACLEAR.bit.GPIO1=1
//===========================================================================
#define DSP_WDH()	GpioDataRegs.GPBSET.bit.GPIO48=1
#define DSP_WDL()	GpioDataRegs.GPBCLEAR.bit.GPIO48=1
//===========================================================================
#define CONV_ADH()     GpioDataRegs.GPBSET.bit.GPIO54=1
#define CONV_ADL()     GpioDataRegs.GPBCLEAR.bit.GPIO54=1
//===========================================================================
#define CS_AD1H()     GpioDataRegs.GPBSET.bit.GPIO52=1
#define CS_AD1L()     GpioDataRegs.GPBCLEAR.bit.GPIO52=1
//===========================================================================
#define CS_AD2H()     GpioDataRegs.GPBSET.bit.GPIO57=1
#define CS_AD2L()     GpioDataRegs.GPBCLEAR.bit.GPIO57=1
//===========================================================================
//#define EN_LTPEN()     GpioDataRegs.GPBCLEAR.bit.GPIO60=1
//#define DIS_LTPEN()     GpioDataRegs.GPBSET.bit.GPIO60=1
//===========================================================================
//#define EN_OPTOEN()     GpioDataRegs.GPBSET.bit.GPIO61=1
//#define DIS_OPTOEN()     GpioDataRegs.GPBCLEAR.bit.GPIO61=1
//********************************************************************
//--------------------------------------------------------------------------------
int16	*XintfZone0=(int16 *)0x004000;
int16	*XintfZone6=(int16 *)0x100000;
int16	*XintfZone7=(int16 *)0x200000;
//--------------------------------------------------------------------------------
int16	PWM1AQ_SET,PWM2AQ_SET,PWM3AQ_SET,PWM_PRD,PWM1A_CMP,PWM2A_CMP,PWM3A_CMP,PWM4A_CMP,PWM5A_CMP,PWM1A_test,PWM2A_test,PWM3A_test;
Uint16	Cnt_PWM,Cnt_APP_1ms,Cnt_250us,Cnt_ms,Cnt_sec,Cnt_Dir01F,Cnt_Dir01B,Cnt_Dir02F,Cnt_Dir02B,Cnt_Spd1,Cnt_Spd2;
Uint16	Cnt_MCU,Cnt_DSP,Cnt_main,Cnt_Err_DSP,INT_ID1,INT_ID2;
// ----------------------------
Uint16 otime,OVCUN,HMI_RBUF[10],HMI_TBUF[10],Pin,Pout,rtemp,scitx_sta;
//--------------------------------------------------------------------------------
float32	TMP_Rt,Zero_Ia,Zero_Ib;
//--------------------------------------------------------------------------------
int16 AD_CH[30],DA_CH[5],FPGA_RD[10],FPGA_WR[10],DPRAM_RD[40],DPRAM_WR[40],EEPROM_RD[40],EEPROM_WR[40];
Uint16 ERR_OS[5];

float32 AI[30];
Uint16 PWM_OS[11];

Uint16 DI_OS[1];
Uint16 DO_OS[1];
Uint16 ERR_EXTR[3];

Uint16 CFG_IN[4];
Uint16 STA_IN[8];
Uint16 ERR_DSP[3];
Uint16 CFG_OUT[3];
Uint16 STA_OUT[4];

float32	CMD_OS[40],CTRL_OS[20],TMP_Rt,Zero_Ia,Zero_Ib;


//--------------------------------------------------------------------------------
volatile struct Ro_speed Ro_SP01,Ro_SP02,Ro_SP03,Ro_SP04;
//tasks_ini_t tasks_ini;
long unixtime = 0;
                                                                                              
//--------------------------------------------------------------------------------
void ADC_Process(void);
void ADC_LS_IOB_Process(void);
void ADC_LS_IOE_Process(void);
void FLASH_IOE_Process(void);
void FLASH_OPT_Process(void);
void DAC_Process(void);
void DPRAM_PRO_WR(void);
void DPRAM_PRO_RD(void);
void FPGA_PRO_WR(void);
void FPGA_PRO_RD(void);
void InitFPGA(void);
void InitDRV(void);
void Time_Process(void);
void SP_PRO(void);
void InitAD7606(void);
void InitVariables(void);
void SPWM(void);
void MLOOP_Infra(void);

Uint16 Readchar();
void codedelt(void);       
void txarray(Uint16 txl);
void recvdelt(void);

interrupt void DSP_isr(void);
interrupt void EPWM1_isr(void);
//interrupt void EPWM4_isr(void);
interrupt void RTOStimer_isr(void);
interrupt void ScibRx_isr(void);

//=================================================================================
 void main(void)
 {
   	InitSysCtrl();//Set in SysCtrl.c
   	InitGpio();//Set in Gpio.c,define all of the GPIOs
	
 	InitEPwm();//Set in EPwm.c
//	InitSci();//Set in Sci.c
//	InitECan();//Set in ECan.c
	InitEQep();//Set in EQep.c
//	InitI2C();//Set in I2C.c
	InitSpi();
//	InitAdc();//Set in Adc.c,Initial Internal ADC
	InitAD7606();//Set in main.c
	InitVariables();//Set in main.c
	
	DIS_INT();
	DIS_PWM();
	
	InitPieCtrl();//Set in PieCtrl.c
	InitPieVectTable();//Set in PieVect.c
	EALLOW;
	//	PieVectTable.XINT1=&DSP_isr;
	PieVectTable.EPWM1_INT=&EPWM1_isr;
	PieVectTable.TINT0=&RTOStimer_isr;
//	PieVectTable.EPWM4_INT=&EPWM4_isr;
//	PieVectTable.SCIRXINTB=&ScibRx_isr;
	EDIS;

	InitCpuTimers();
//	ConfigCpuTimer(&CpuTimer0, 150, 300);  // 250us 150MHz CPU Freq, 1 second Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 150, 200);
	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
	InitXInterrupt();//Set interrupt source


	EN_INT();
//	EPwm4Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
//	InitFPGA();
	InitDRV();
//	DPRAM_WR[0] = 0x0401;

	INIT_EV();

	while(1)
	{ 	  					
        MLOOP_Infra();

        ADC_LS_IOB_Process();
        DELAY_US(5L);
//        ADC_LS_IOE_Process();
        DELAY_US(5L);
//        FLASH_IOE_Process();
        DELAY_US(5L);
//        FLASH_OPT_Process();
        DELAY_US(5L);
//        DAC_Process();
//        DELAY_US(200L);
        INT_ID1=GpioDataRegs.GPBDAT.bit.GPIO63;
        DELAY_US(1L);
        INT_ID2=GpioDataRegs.GPBDAT.bit.GPIO63;
        if((INT_ID1==0) && (INT_ID2==0))
        {
        	DPRAM_PRO_RD();
        	INT_ID1=1;
        	INT_ID2=1;
        }

        if((((Uint16)CMD_OS[35]) & 0x0001)==1)
        	CTRL_OS[1] = 0x0000;
        if(((((Uint16)CMD_OS[35]) >>1) & 0x0001)==1)
            CTRL_OS[1] = 0x0002;
        if(((((Uint16)CMD_OS[35]) >>2) & 0x0001)==1)
            CTRL_OS[1] = 0x0004;
        if(((((Uint16)CMD_OS[35]) >>3) & 0x0001)==1)
            CTRL_OS[1] = 0x0006;
        if(((((Uint16)CMD_OS[35]) >>4) & 0x0001)==1)
            CTRL_OS[1] = 0x0008;
        if(((((Uint16)CMD_OS[35]) >>5) & 0x0001)==1)
            CTRL_OS[1] = 0x000A;
        if(((((Uint16)CMD_OS[35]) >>6) & 0x0001)==1)
            CTRL_OS[1] = 0x0013;
	}
}

void MLOOP_Infra(void)
{
	Cycle_OS();
}

//==============================================================================
interrupt void RTOStimer_isr(void)   
{  
	Time_Process();
//------------------------------------------------------------------
//	if((Cnt_sec % 2) == 1)
//		DSP_WDH();//Set the GPIO to indicate interrupt is OK
//		DELAY_US(200L);
//	if((Cnt_sec % 2) == 0)
//		DSP_WDL();
//------------------------------------------------------------------
//	Time_Process();//Set the time,Eg. us,sec,min,hour,etc,0.7us.
	INT_RTOS();
//------------------------------------------------------------------
//	ADC_Process();//Inner ADC and 7606 Sampling and Basically process,2.85us
//------------------------------------------------------------------
//	DPRAM_PRO_WR();//Write the data to DPRAM,3.4us,20 data
//------------------------------------------------------------------
	FPGA_PRO_WR();//Write the data to FPGA,XXus
	FPGA_PRO_RD();//Read the data from FPGA,XXus

//------------------------------------------------------------------
//	SP01_EN();
//	if((Cnt_ms % 20) == 1)
//		SP01_DIS();
//	if((Cnt_ms % 20) == 11)
	SP01_EN();
	SP_PRO();//Theta Caculation,1.45us
//------------------------------------------------------------------
//	DAC_Process();//Write data to DA,4.8us
			
//---------------------------------------------------------------------
	CpuTimer0Regs.TCR.bit.TIF = 1;//clear flag
	CpuTimer0Regs.TCR.bit.TRB = 1;//reload counter of timer0
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP1;

//	EPwm4Regs.ETCLR.bit.INT=1;
//  PieCtrlRegs.PIEACK.all|=PIEACK_GROUP3;

}

interrupt void EPWM1_isr(void)
{
	//------------------------------------------------------------------
	ADC_Process();//Inner ADC and 7606 Sampling and Basically process,2.85us
	//------------------------------------------------------------------
	INT_PWM();
	if((Cnt_sec % 2) == 1)
		DSP_WDH();//Set the GPIO to indicate interrupt is OK

//	DPRAM_PRO_WR();//Write the data to DPRAM,3.4us,20 data

//	DELAY_US(100L);
	SPWM();//SPWM limitation,1.5us
	if((Cnt_sec % 2) == 0)
		DSP_WDL();

	EPwm1Regs.ETCLR.bit.INT=1;
	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP3;
}

//==============================================================================
void Time_Process(void)
{
	Cnt_main=0;
	Cnt_DSP++;
	if(Cnt_DSP>=32767)
		Cnt_DSP=0;
	Cnt_PWM++;
	if(Cnt_PWM>4350)
		{		
			Cnt_PWM=0;
		}
	Cnt_250us++;
	if(Cnt_250us>4)
		{
			Cnt_ms++;		
			Cnt_250us=0;
		}
	if(Cnt_ms>1000)
		{		
			Cnt_ms=0;
			Cnt_sec++;
		}
	if(Cnt_sec>60)
	{
		Cnt_sec=0;
	}
}

//==============================================================================
void ADC_Process(void)
{
	//Enable Inner ADC
//	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

//	FPGA_WR[0] &= 0x7FFF;
//	*(XintfZone0 + 9) = FPGA_WR[0];//IOB_GPDO.all;
	CONV_ADH();
	DELAY_US(4L);
	CONV_ADL();

//	FPGA_WR[0] |= 0x8000;
//	*(XintfZone0 + 9) = FPGA_WR[0];

	DELAY_NS(500L);
	//Read AD7606
	CS_AD1L();
	DELAY_US(1L);

	AD_CH[0] = *(XintfZone0+8);//PH1           
	AD_CH[1] = *(XintfZone0+8);//PH2   
	AD_CH[2] = *(XintfZone0+8);//PH-ref
	AD_CH[3] = *(XintfZone0+8);//PH+ref
	AD_CH[4] = *(XintfZone0+8);//IDC
	AD_CH[5] = *(XintfZone0+8);//UDC
	AD_CH[6] = *(XintfZone0+8);//UDC+ref
	AD_CH[7] = *(XintfZone0+8);//EF+ref
	CS_AD1H();

	CS_AD2L();
	DELAY_US(1L);
	AD_CH[8] = *(XintfZone0+8);//AI23
	AD_CH[9] = *(XintfZone0+8);//AI24
	AD_CH[10] = *(XintfZone0+8);//AI25
	AD_CH[11] = *(XintfZone0+8);//AI26
	AD_CH[12] = *(XintfZone0+8);//AI28
	AD_CH[13] = *(XintfZone0+8);//AI29
	AD_CH[14] = *(XintfZone0+8);//AI30
	AD_CH[15] = *(XintfZone0+8);//AI31
	CS_AD2H();
//-----------IOB-------------
	if(CTRL_OS[1] <= 0x02)
	{
		Zero_Ia = Zero_Ia * 0.999 + (float32)AD_CH[0] * 0.001;
		Zero_Ib = Zero_Ib * 0.999 + (float32)AD_CH[1] * 0.001;
	}
	AD_CH[0] -= (int16)Zero_Ia;
	AD_CH[1] -= (int16)Zero_Ib;

	AI[11] = -((float32)AD_CH[0] * 0.0914412);//Ia//0.0304804
	AI[12] = -((float32)AD_CH[1] * 0.0914412);//Ib
	AI[2] = -(AI[0] + AI[1]);//Ic
	AI[3] = AD_CH[2] * 0.1;
	AI[4] = AD_CH[3] * 0.1;

	AI[5] = AI[5] * 0.9 + ((float32)AD_CH[4] * 0.0469515 * 0.1);//Idc
	AI[6] = AI[6] * 0.9 + ((float32)AD_CH[5] * 0.0914412 * 0.1);//Udc

	AI[7] = AD_CH[6] * 0.1;//Udc+ref
	AI[8] = AD_CH[7] * 0.1;//EF+ref

//-----------IOE-------------
	AI[9] = AD_CH[8] * 0.1;//AI23
	AI[10] = AD_CH[9] * 0.1;//AI24
	AI[11] = AD_CH[10] * 0.1;//AI25
	AI[12] = AD_CH[11] * 0.1;//AI26
	AI[13] = AD_CH[12] * 0.1;//AI28
	AI[14] = AD_CH[13] * 0.1;//AI29
	AI[15] = AD_CH[14] * 0.1;//AI30
	AI[16] = AD_CH[15] * 0.1;//AI31

	AI[17] = 110;//Vbattery
}

//==============================================================================
void ADC_LS_IOB_Process(void)
{
	adc_low.res = 8;
	adc_low.BIP = 1;
	adc_low.LSBF = 1;
	adc_low.LEN = 2;
	adc_low.ADDR = 0x09;
	// Transmit data
   FPGA_WR[2] &= 0xFE;
   *(XintfZone7 + 1) = FPGA_WR[2];

   SpiaRegs.SPITXBUF=0x8020;//spi读取命令,0通道
   /*
   0b1000000000100000
   */
    // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) {;}
    // Check against sent data
//    AD_CH[16] = SpiaRegs.SPIRXBUF;

    FPGA_WR[2] |= 0x01;
    *(XintfZone7 + 1) = FPGA_WR[2];

    DELAY_US(1L);

    FPGA_WR[2] &= 0xFE;
    *(XintfZone7 + 1) = FPGA_WR[2];

    SpiaRegs.SPITXBUF=0x8420;//spi读取命令,1通道
     //       SpiaRegs.SPITXBUF=0x009E;//0x5555;
    	     // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) {;}
    	     // Check against sent data
    AD_CH[16] = SpiaRegs.SPIRXBUF;//读取0通道

//    DELAY_US(1L);

    FPGA_WR[2] |= 0x01;
	*(XintfZone7 + 1) = FPGA_WR[2];

	DELAY_US(1L);

	FPGA_WR[2] &= 0xFE;
	*(XintfZone7 + 1) = FPGA_WR[2];

	SpiaRegs.SPITXBUF=0x8820;//spi读取命令,2通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) {;}
	// Check against sent data
	AD_CH[17] = SpiaRegs.SPIRXBUF;//读取1通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x8C20;//spi读取命令,3通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[18] = SpiaRegs.SPIRXBUF;//读取2通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x9020;//spi读取命令,4通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[19] = SpiaRegs.SPIRXBUF;//读取3通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x9420;//spi读取命令,5通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[20] = SpiaRegs.SPIRXBUF;//读取4通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x9820;//spi读取命令,6通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[21] = SpiaRegs.SPIRXBUF;//读取5通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x9C20;//spi读取命令,7通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[22] = SpiaRegs.SPIRXBUF;//读取6通道

//	DELAY_US(10L);

//	SpiaRegs.SPITXBUF=0x8020;//spi读取命令,0通道
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
//	AD_CH[23] = SpiaRegs.SPIRXBUF;//读取7通道

    FPGA_WR[2] |= 0x01;
    *(XintfZone7 + 1) = FPGA_WR[2];
    DELAY_US(1L);

	//Wait for Inner ADC

	TMP_Rt = (float32)AD_CH[16] * 0.039950284;//AD_CH[6] /4096 * 3 /7.333333 *1000 / 2.5;//0.0264651
	AI[18] = (TMP_Rt - 100) * 2.731579;

	TMP_Rt = (float32)AD_CH[17] * 0.039950284;//AD_CH[6] /4096 * 3 /7.333333 *1000 / 2.5
	AI[19] = (TMP_Rt - 100) * 2.731579;//2.631579

	TMP_Rt = (float32)AD_CH[18] * 0.079900568;//AD_CH[6] /4096 * 3 /3.666667 *1000 / 2.5
	AI[20] = (TMP_Rt - 100) * 2.731579;

	TMP_Rt = (float32)AD_CH[19] * 0.079900568;//AD_CH[6] /4096 * 3 /3.666667 *1000 / 2.5
	AI[21] = (TMP_Rt - 100) * 2.731579;
	AI[22] = 25;
	AI[23] = 25;

}

//==============================================================================
void ADC_LS_IOE_Process(void)
{
	adc_low.res = 8;
	adc_low.BIP = 1;
	adc_low.LSBF = 1;
	adc_low.LEN = 2;
	adc_low.ADDR = 0x09;
	// Transmit data
   FPGA_WR[2] &= 0xF7;
   *(XintfZone7 + 1) = FPGA_WR[2];

   SpiaRegs.SPITXBUF=0x0C00;//0x5555;
//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
    // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
    // Check against sent data
    AD_CH[23] = SpiaRegs.SPIRXBUF;

    DELAY_US(10L);

	SpiaRegs.SPITXBUF=0x1C00;//0x5555;
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	AD_CH[23] = SpiaRegs.SPIRXBUF;

	SpiaRegs.SPITXBUF=0x2C00;//0x5555;
	//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
	// Wait until data is received
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	AD_CH[27] = SpiaRegs.SPIRXBUF;

    FPGA_WR[2] |= 0x08;
    *(XintfZone7 + 1) = FPGA_WR[2];
    DELAY_US(1L);
}

//==============================================================================
void FLASH_IOE_Process(void)
{
	// Transmit data
   FPGA_WR[2] &= 0xEF;
   *(XintfZone7 + 1) = FPGA_WR[2];

//   SpiaRegs.SPITXBUF=0x000C;//0x5555;
//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
    // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
    // Check against sent data

    FPGA_WR[2] |= 0x10;
    *(XintfZone7 + 1) = FPGA_WR[2];
    DELAY_US(1L);
}

//==============================================================================
void FLASH_OPT_Process(void)
{
	// Transmit data
   FPGA_WR[2] &= 0xFD;
   *(XintfZone7 + 1) = FPGA_WR[2];

//   SpiaRegs.SPITXBUF=0x000C;//0x5555;
//       SpiaRegs.SPITXBUF=0x009E;//0x5555;
    // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
    // Check against sent data

    FPGA_WR[2] |= 0x02;
    *(XintfZone7 + 1) = FPGA_WR[2];
    DELAY_US(1L);
}

//==============================================================================
void DAC_Process(void) 
{
    // Transmit data
   FPGA_WR[2] &= 0xFB;
   *(XintfZone7 + 1) = FPGA_WR[2];

   dac_low.Addr = 0;
   dac_low.Date = (DPRAM_RD[2] & 0x7FF);
   SpiaRegs.SPITXBUF=0x1100 + (DPRAM_RD[2] & 0x00FF);//0x0200,A,512;
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	DA_CH[0] = SpiaRegs.SPIRXBUF;

	FPGA_WR[2] &= 0xDF;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_NS(50L);
	FPGA_WR[2] |= 0x20;
	*(XintfZone7 + 1) = FPGA_WR[2];

	dac_low.Addr = 4;
	dac_low.Date = (DPRAM_RD[3] & 0x7FF);
	SpiaRegs.SPITXBUF=0x1200 + (DPRAM_RD[3] & 0x00FF);//0x4200,B,1000;
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	DA_CH[0] = SpiaRegs.SPIRXBUF;

	FPGA_WR[2] &= 0xDF;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_NS(50L);
	FPGA_WR[2] |= 0x20;
	*(XintfZone7 + 1) = FPGA_WR[2];

	dac_low.Addr = 8;
	dac_low.Date = (DPRAM_RD[4] & 0x7FF);
	SpiaRegs.SPITXBUF=0x1400 + (DPRAM_RD[4] & 0x00FF);//0x8400,C,1000;
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	DA_CH[0] = SpiaRegs.SPIRXBUF;

	FPGA_WR[2] &= 0xDF;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_NS(50L);
	FPGA_WR[2] |= 0x20;
	*(XintfZone7 + 1) = FPGA_WR[2];

	dac_low.Addr = 12;
	dac_low.Date = (DPRAM_RD[4] & 0x7FF);
	SpiaRegs.SPITXBUF=0x1800 + (DPRAM_RD[4] & 0x00FF);//0x8400,C,1000;
//	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	// Check against sent data
	DA_CH[0] = SpiaRegs.SPIRXBUF;

	FPGA_WR[2] &= 0xDF;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_NS(50L);
	FPGA_WR[2] |= 0x24;
	*(XintfZone7 + 1) = FPGA_WR[2];
}

//==============================================================================
void SP_PRO(void)
{
	//Ro_SP01 Direction,0=CCW/reverse, 1=CW/forward
	if(EQep1Regs.QEPSTS.bit.QDF == 0)
	{
		Cnt_Dir01F = 0;
		Cnt_Dir01B++;
		if(Cnt_Dir01B>=1)
		{
			Ro_SP01.dir = 0x11;
			Cnt_Dir01B=1;
		}
	}
	else if(EQep1Regs.QEPSTS.bit.QDF == 1)
	{
		Cnt_Dir01B = 0;
		Cnt_Dir01F++;
		if(Cnt_Dir01F>=1)
		{
			Ro_SP01.dir = 0x22;
			Cnt_Dir01F=1;
		}
	}
//High and low speed, more than 375RPM,less than 15RPM
	if(EQep1Regs.QFLG.bit.UTO==1)                  // 5ms interrupt
	{
		// Differentiator	
	 	Ro_SP01.pos_pre = Ro_SP01.pos_cur;
		if((abs(EQep1Regs.QPOSLAT - Ro_SP01.pos_pre)) >= 10)
		{
			Ro_SP01.pos_cur=EQep1Regs.QPOSLAT;                    // Latched POSCNT value
			if (Ro_SP01.dir == 0x11)	
			{
				if (Ro_SP01.pos_cur > Ro_SP01.pos_pre)//锟斤拷�锟较达拷说锟斤拷锟斤拷锟剿伙拷械锟斤拷锟?
				{
					Ro_SP01.pos_delta = Ro_SP01.pos_pre + 0xFFFFFFFF - Ro_SP01.pos_cur;
				}
				else
				{
					Ro_SP01.pos_delta = Ro_SP01.pos_pre - Ro_SP01.pos_cur;
				}
			}
			else if (Ro_SP01.dir == 0x22)
			{
				if (Ro_SP01.pos_cur < Ro_SP01.pos_pre)//锟斤拷�锟斤拷小说锟斤拷锟斤拷锟剿伙拷械锟斤拷锟?
				{
					Ro_SP01.pos_delta = Ro_SP01.pos_cur + 0xFFFFFFFF - Ro_SP01.pos_pre;
				}
				else
				{
					Ro_SP01.pos_delta = Ro_SP01.pos_cur - Ro_SP01.pos_pre;
				}			
			}
			Ro_SP01.omega_m_high = ((float)Ro_SP01.pos_delta) * 37.5 / Cnt_Spd1;//Ro_SP01.omega_m_high * 0.99 +  * 0.01//1.25,Ro_SP01.pos_delta * (150MHz/QUPRD) / 4 / 20 / 4; 20ms,4 pulses per PRD,20 pulses per TR per Min,4 div frequency
			Cnt_Spd1 = 1;			
		}
		else
		{
			Cnt_Spd1++;
			if(Cnt_Spd1>=10000)
				Cnt_Spd1 = 10000;
			if(Cnt_Spd1<=1)
				Cnt_Spd1 = 1;
		}			
		//=======================================
		EQep1Regs.QCLR.bit.UTO=1;					// Clear interrupt flag
	}
// Medium speed, between 15RPM and 375RPM
	if(EQep1Regs.QEPSTS.bit.UPEVNT==1)              // Unit Position Event
	{
		if(EQep1Regs.QEPSTS.bit.COEF == 0)// No Capture overflow
		{
			//锟斤拷锟芥单位位锟斤拷时锟斤拷锟斤拷�
			Ro_SP01.up_prd = ((float)EQep1Regs.QCPRDLAT) * 0.85333333;//QCK=128/150=0.853333us
			Ro_SP01.omega_m_low = 750000 / Ro_SP01.up_prd;//Ro_SP01.omega_m_low * 0.99 +  * 0.01//2500,2^UPPS * 10^6 / 4 /Ro_SP01.up_prd /20,4 div frequency;
		}
		else
		{
			Ro_SP01.up_prd = 0xFFFF * 0.8533;
			Ro_SP01.omega_m_low = 0;
		}			
		EQep1Regs.QEPSTS.all=0x88;					    // Clear Unit position event flag												     	// Clear overflow error flag
	}
	Ro_SP01.omega_m = Ro_SP01.omega_m_high;
	if((Ro_SP01.omega_m >= 15) && (Ro_SP01.omega_m <= 3750))
		Ro_SP01.omega_m = Ro_SP01.omega_m_low;
	if(Ro_SP01.omega_m <= 0.01)
		Ro_SP01.omega_m = 0;
	else if(Ro_SP01.omega_m >= 5000)
		Ro_SP01.omega_m = 5000;
//	
//Ro_SP01 Direction,0=CCW/reverse, 1=CW/forward
	if(EQep2Regs.QEPSTS.bit.QDF == 0)
	{
		Cnt_Dir02F = 0;
		Cnt_Dir02B++;
		if(Cnt_Dir02B>=1)
		{
			Ro_SP02.dir = 0x11;
			Cnt_Dir02B=1;
		} 
	}
	else if(EQep2Regs.QEPSTS.bit.QDF == 1)
	{
		Cnt_Dir02B = 0;
		Cnt_Dir02F++;
		if(Cnt_Dir02F>=1)
		{
			Ro_SP02.dir = 0x22;
			Cnt_Dir02F=1;
		}	 
	}
//High and low speed, more than 375RPM,less than 15RPM
	if(EQep2Regs.QFLG.bit.UTO==1)                  // 5ms interrupt
	{
		// Differentiator
	 	Ro_SP02.pos_pre = Ro_SP02.pos_cur;
		if((abs(EQep2Regs.QPOSLAT - Ro_SP02.pos_pre)) >= 10)
		{
			Ro_SP02.pos_cur=EQep2Regs.QPOSLAT;                    // Latched POSCNT value
			if (Ro_SP02.dir == 0x11)
			{
				if (Ro_SP02.pos_cur > Ro_SP02.pos_pre)//锟斤拷�锟较达拷说锟斤拷锟斤拷锟剿伙拷械锟斤拷锟?
				{
					Ro_SP02.pos_delta = Ro_SP02.pos_pre + 0xFFFFFFFF - Ro_SP02.pos_cur;
				}
				else
				{
					Ro_SP02.pos_delta = Ro_SP02.pos_pre - Ro_SP02.pos_cur;
				}
			}
			else if (Ro_SP02.dir == 0x22)
			{
				if (Ro_SP02.pos_cur < Ro_SP02.pos_pre)//锟斤拷�锟斤拷小说锟斤拷锟斤拷锟剿伙拷械锟斤拷锟?
				{
					Ro_SP02.pos_delta = Ro_SP02.pos_cur + 0xFFFFFFFF - Ro_SP02.pos_pre;
				}
				else
				{
					Ro_SP02.pos_delta = Ro_SP02.pos_cur - Ro_SP02.pos_pre;
				}
			}
			Ro_SP02.omega_m_high = ((float)Ro_SP02.pos_delta) * 37.5 / Cnt_Spd2;//Ro_SP01.omega_m_high * 0.99 +  * 0.01//1.25,Ro_SP01.pos_delta * (150MHz/QUPRD) / 4 / 20 / 4; 20ms,4 pulses per PRD,20 pulses per TR per Min,4 div frequency
			Cnt_Spd2 = 1;
		}
		else
		{
			Cnt_Spd2++;
			if(Cnt_Spd2>=10000)
				Cnt_Spd2 = 10000;
			if(Cnt_Spd2<=1)
				Cnt_Spd2 = 1;
		}
		//=======================================
		EQep2Regs.QCLR.bit.UTO=1;					// Clear interrupt flag
	}
// Medium speed, between 15RPM and 375RPM
	if(EQep2Regs.QEPSTS.bit.UPEVNT==1)              // Unit Position Event
	{
		if(EQep2Regs.QEPSTS.bit.COEF == 0)// No Capture overflow
		{
			//锟斤拷锟芥单位位锟斤拷时锟斤拷锟斤拷�
			Ro_SP02.up_prd = ((float)EQep2Regs.QCPRDLAT) * 0.85333333;//QCK=128/150=0.853333us
			Ro_SP02.omega_m_low = 750000 / Ro_SP02.up_prd;//Ro_SP01.omega_m_low * 0.99 +  * 0.01//2500,2^UPPS * 10^6 / 4 /Ro_SP01.up_prd /20,4 div frequency;
		}
		else
		{
			Ro_SP02.up_prd = 0xFFFF * 0.8533;
			Ro_SP02.omega_m_low = 0;
		}
		EQep2Regs.QEPSTS.all=0x88;					    // Clear Unit position event flag												     	// Clear overflow error flag
	}
	Ro_SP02.omega_m = Ro_SP02.omega_m_high;
	if((Ro_SP02.omega_m >= 15) && (Ro_SP02.omega_m <= 3750))
		Ro_SP02.omega_m = Ro_SP02.omega_m_low;
	if(Ro_SP02.omega_m <= 0.01)
		Ro_SP02.omega_m = 0;
	else if(Ro_SP02.omega_m >= 5000)
		Ro_SP02.omega_m = 5000;

	AI[24] = Ro_SP01.omega_m;
	AI[25] = Ro_SP02.omega_m;
	AI[26] = Ro_SP01.omega_m;
	AI[27] = Ro_SP02.omega_m;
	AI[28] = (Ro_SP01.dir & 0x000F) + ((Ro_SP02.dir & 0x000F)<<4) + ((Ro_SP01.dir & 0x000F)<<8) + ((Ro_SP02.dir & 0x000F)<<12);

}

//==============================================================================
void DPRAM_PRO_RD(void)
{
	Uint16 i=0;

	CMD_OS[3] = (*(XintfZone6 + 0) & 0x00FF);//Motor quantity
	CMD_OS[4] = (*(XintfZone6 + 1) & 0x00FF);//Motor poles

	CMD_OS[5] = ((*(XintfZone6 + 0x02) & 0x00FF) + ((*(XintfZone6 + 0x03) << 8) & 0xFF00)) * 0.0000001;
	CMD_OS[6] = ((*(XintfZone6 + 0x04) & 0x00FF) + ((*(XintfZone6 + 0x05) << 8) & 0xFF00)) * 0.0000001;
	CMD_OS[7] = ((*(XintfZone6 + 0x06) & 0x00FF) + ((*(XintfZone6 + 0x07) << 8) & 0xFF00)) * 0.0000001;
	CMD_OS[8] = ((*(XintfZone6 + 0x08) & 0x00FF) + ((*(XintfZone6 + 0x09) << 8) & 0xFF00)) * 0.0000001;
	CMD_OS[9] = ((*(XintfZone6 + 0x0A) & 0x00FF) + ((*(XintfZone6 + 0x0B) << 8) & 0xFF00)) * 0.0000001;

	CMD_OS[10] = ((*(XintfZone6 + 0x0C) & 0x00FF) + ((*(XintfZone6 + 0x0D) << 8) & 0xFF00)) * 0.1;
	CMD_OS[11] = ((*(XintfZone6 + 0x0E) & 0x00FF) + ((*(XintfZone6 + 0x0F) << 8) & 0xFF00)) * 0.1;
	CMD_OS[12] = ((*(XintfZone6 + 0x10) & 0x00FF) + ((*(XintfZone6 + 0x11) << 8) & 0xFF00));
	CMD_OS[13] = ((*(XintfZone6 + 0x12) & 0x00FF) + ((*(XintfZone6 + 0x13) << 8) & 0xFF00));
	CMD_OS[14] = ((*(XintfZone6 + 0x14) & 0x00FF) + ((*(XintfZone6 + 0x15) << 8) & 0xFF00));
	CMD_OS[15] = ((*(XintfZone6 + 0x16) & 0x00FF) + ((*(XintfZone6 + 0x17) << 8) & 0xFF00));

	CMD_OS[16] = (*(XintfZone6 + 0x18) & 0x00FF);//Motor contr Mode

	CMD_OS[17] = (*(XintfZone6 + 0x19) & 0x00FF);//Motor contr Mode

	CMD_OS[18] = ((*(XintfZone6 + 0x1A) & 0x00FF) + ((*(XintfZone6 + 0x1B) << 8) & 0xFF00)) * 0.01;
	CMD_OS[19] = ((*(XintfZone6 + 0x1C) & 0x00FF) + ((*(XintfZone6 + 0x1D) << 8) & 0xFF00));
	CMD_OS[20] = ((*(XintfZone6 + 0x1E) & 0x00FF) + ((*(XintfZone6 + 0x1F) << 8) & 0xFF00)) * 0.1;
	CMD_OS[21] = ((*(XintfZone6 + 0x20) & 0x00FF) + ((*(XintfZone6 + 0x21) << 8) & 0xFF00)) * 0.1;

	AI[29] = ((*(XintfZone6 + 0x22) & 0x00FF) + ((*(XintfZone6 + 0x23) << 8) & 0xFF00)) * 0.01;//ambient temp
	DPRAM_RD[0] = ((*(XintfZone6 + 0x24) & 0x00FF) + ((*(XintfZone6 + 0x25) << 8) & 0xFF00));
	DPRAM_RD[1] = ((*(XintfZone6 + 0x26) & 0x00FF) + ((*(XintfZone6 + 0x27) << 8) & 0xFF00)) * 0.1;
	DPRAM_RD[2] = 0x000F;//((*(XintfZone6 + 0x28) & 0x00FF) + ((*(XintfZone6 + 0x29) << 8) & 0xFF00));//Udc ref
	DPRAM_RD[3] = 0x002F;//((*(XintfZone6 + 0x2A) & 0x00FF) + ((*(XintfZone6 + 0x2B) << 8) & 0xFF00));//Iac ref
	DPRAM_RD[4] = 0x0007;//((*(XintfZone6 + 0x2C) & 0x00FF) + ((*(XintfZone6 + 0x2D) << 8) & 0xFF00));//EF ref

	CMD_OS[23] = ((*(XintfZone6 + 0x40) & 0x00FF) + ((*(XintfZone6 + 0x41) << 8) & 0xFF00)) * 0.01;
	CMD_OS[24] = ((*(XintfZone6 + 0x42) & 0x00FF) + ((*(XintfZone6 + 0x43) << 8) & 0xFF00)) * 0.01;
	CMD_OS[25] = ((*(XintfZone6 + 0x44) & 0x00FF) + ((*(XintfZone6 + 0x45) << 8) & 0xFF00)) * 0.01;
	CMD_OS[26] = ((*(XintfZone6 + 0x46) & 0x00FF) + ((*(XintfZone6 + 0x47) << 8) & 0xFF00)) * 0.01;
	CMD_OS[27] = ((*(XintfZone6 + 0x48) & 0x00FF) + ((*(XintfZone6 + 0x49) << 8) & 0xFF00)) * 0.01;

	CMD_OS[28] = ((*(XintfZone6 + 0x4A) & 0x00FF) + ((*(XintfZone6 + 0x4B) << 8) & 0xFF00)) * 0.01;
	CMD_OS[29] = ((*(XintfZone6 + 0x4C) & 0x00FF) + ((*(XintfZone6 + 0x4D) << 8) & 0xFF00)) * 0.01;
	CMD_OS[30] = ((*(XintfZone6 + 0x4E) & 0x00FF) + ((*(XintfZone6 + 0x4F) << 8) & 0xFF00)) * 0.01;
	CMD_OS[31] = ((*(XintfZone6 + 0x50) & 0x00FF) + ((*(XintfZone6 + 0x51) << 8) & 0xFF00)) * 0.01;
	CMD_OS[32] = ((*(XintfZone6 + 0x52) & 0x00FF) + ((*(XintfZone6 + 0x53) << 8) & 0xFF00)) * 0.01;

	CMD_OS[33] = ((*(XintfZone6 + 0x54) & 0x00FF) + ((*(XintfZone6 + 0x55) << 8) & 0xFF00)) * 0.01;
	CMD_OS[34] = ((*(XintfZone6 + 0x56) & 0x00FF) + ((*(XintfZone6 + 0x57) << 8) & 0xFF00)) * 0.01;
	CMD_OS[35] = ((*(XintfZone6 + 0x58) & 0x00FF) + ((*(XintfZone6 + 0x59) << 8) & 0xFF00));//status requirement
	CMD_OS[36] = ((*(XintfZone6 + 0x5A) & 0x00FF) + ((*(XintfZone6 + 0x5B) << 8) & 0xFF00)) * 0.01;
	CMD_OS[37] = ((*(XintfZone6 + 0x5C) & 0x00FF) + ((*(XintfZone6 + 0x5D) << 8) & 0xFF00)) * 0.01;

	DPRAM_PRO_WR();

	CMD_OS[1] = (*(XintfZone6 + 0x1FB) & 0x00FF) + ((*(XintfZone6 + 0x1FC) << 8) & 0xFF00);//MCU status
	CMD_OS[0] = (*(XintfZone6 + 0x1FD) & 0x00FF) + ((*(XintfZone6 + 0x1FE) << 8) & 0xFF00);//MCU version

	CMD_OS[2] = *(XintfZone6 + 0x3FF);//

	if(Cnt_MCU == (Uint16)CMD_OS[2])
	{
		Cnt_Err_DSP++;
		if(Cnt_Err_DSP>=3)
			{Cnt_Err_DSP=0;ERR_OS[4] |= 0x0004;}
	}
	Cnt_MCU = (Uint16)CMD_OS[2];

}

//==============================================================================
void DPRAM_PRO_WR(void)
{
	Uint16 i=0;

	*(XintfZone6 + 0x1FF) = ((int16)CTRL_OS[3] & 0xFF);//Power In
	*(XintfZone6 + 0x200) = (((int16)CTRL_OS[3] >>8) & 0xFF);//Power In
	*(XintfZone6 + 0x201) = ((int16)CTRL_OS[4] & 0xFF);//Power Out
	*(XintfZone6 + 0x202) = (((int16)CTRL_OS[4] >>8) & 0xFF);//Power Out
	*(XintfZone6 + 0x203) = ((int16)CTRL_OS[5] & 0xFF);//Power In
	*(XintfZone6 + 0x204) = (((int16)CTRL_OS[5] >>8) & 0xFF);//Power In
	*(XintfZone6 + 0x205) = 11;//(CTRL_OS[4] & 0xFF);//Power Out
	*(XintfZone6 + 0x206) = ((int16)CTRL_OS[13] & 0xFF);//Power Out
	*(XintfZone6 + 0x207) = ((int16)CTRL_OS[6] & 0xFF);//Chopper R temp
	*(XintfZone6 + 0x208) = (((int16)CTRL_OS[6] >>8) & 0xFF);//Chopper R temp
	*(XintfZone6 + 0x209) = ((int16)CTRL_OS[7] & 0xFF);//Inductor temp
	*(XintfZone6 + 0x20A) = (((int16)CTRL_OS[7] >>8) & 0xFF);//Inductor temp
	*(XintfZone6 + 0x20B) = ((int16)(CTRL_OS[8] * 10) & 0xFF);//Ia RMS
	*(XintfZone6 + 0x20C) = (((int16)(CTRL_OS[8] * 10) >>8) & 0xFF);//Ia RMS
	*(XintfZone6 + 0x20D) = ((int16)(CTRL_OS[9] * 10) & 0xFF);//Ib RMS
	*(XintfZone6 + 0x20E) = (((int16)(CTRL_OS[9] * 10) >>8) & 0xFF);//Ib RMS
	*(XintfZone6 + 0x20F) = ((int16)(CTRL_OS[10] * 10) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x210) = (((int16)(CTRL_OS[10] * 10) >>8) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x211) = ((int16)CTRL_OS[11] & 0xFF);//Ic RMS
	*(XintfZone6 + 0x212) = (((int16)CTRL_OS[11] >>8) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x213) = ((int16)CTRL_OS[12] & 0xFF);//Ic RMS
	*(XintfZone6 + 0x214) = (((int16)CTRL_OS[12] >>8) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x215) = ((int16)CTRL_OS[14] & 0xFF);//Ic RMS
	*(XintfZone6 + 0x216) = (((int16)CTRL_OS[14] >>8) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x217) = ((int16)CTRL_OS[15] & 0xFF);//Ic RMS
	*(XintfZone6 + 0x218) = (((int16)CTRL_OS[15] >>8) & 0xFF);//Ic RMS
	*(XintfZone6 + 0x219) = ((int16)CTRL_OS[16] & 0xFF);//Ic RMS
	*(XintfZone6 + 0x21A) = (((int16)CTRL_OS[16] >>8) & 0xFF);//Ic RMS
//temperature
	for(i=0;i<=5;i++)
	{
		*(XintfZone6 + 0x21B + (2 * i)) = ((int16)(AI[(18+i)] * 100) & 0xFF);
		*(XintfZone6 + 0x21C + (2 * i)) = (((int16)(AI[(18+i)] * 100) >> 8) & 0xFF);
	}
	*(XintfZone6 + 0x227) = ((int16)(AI[(30)] * 100) & 0xFF);
	*(XintfZone6 + 0x228) = (((int16)(AI[(30)] * 100) >> 8) & 0xFF);
//rotation speed
	for(i=0;i<=4;i++)
	{
		*(XintfZone6 + 0x229 + (2 * i)) = ((int16)AI[(24+i)] & 0xFF);
		*(XintfZone6 + 0x22A + (2 * i)) = (((int16)AI[(24+i)] >> 8) & 0xFF);
	}
//sampling AI
	for(i=0;i<=17;i++)
	{
		*(XintfZone6 + 0x233 + (2 * i)) = ((int16)(AI[(0+i)] * 10) & 0xFF);
		*(XintfZone6 + 0x234 + (2 * i)) = (((int16)(AI[(0+i)] * 10) >> 8) & 0xFF);
	}
//error collections
	for(i=0;i<=4;i++)
	{
		*(XintfZone6 + 0x2F0 + (2 * i)) = (ERR_OS[0 + i] & 0xFF);
		*(XintfZone6 + 0x2F1 + (2 * i)) = ((ERR_OS[0 + i] >> 8) & 0xFF);
	}

	*(XintfZone6 + 0x3FA) = ((int16)CTRL_OS[1] & 0xFF);//MCU Status
	*(XintfZone6 + 0x3FB) = (((int16)CTRL_OS[1] >>8) & 0xFF);//MCU Status
	*(XintfZone6 + 0x3FC) = ((2 * 10) & 0xFF);//MCU Status,CTRL_OS[0]
	*(XintfZone6 + 0x3FD) = (((int16)CTRL_OS[0] * 10) & 0xFF);//MCU Status
	*(XintfZone6 + 0x3FE) = (Cnt_DSP & 0xFF);//锟斤拷锟叫达拷锟斤拷锟斤拷锟斤拷锟较拷锟斤拷锟绞癸拷锟�TR锟斤拷锟矫碉�
}

//==============================================================================
void FPGA_PRO_WR(void)
{


//	*(XintfZone0 + 9) = IOB_GPDO.all;
//	*(XintfZone0 + 11) = FPGA_WR[2];
//	*(XintfZone0 + 10) = 0x2222;
}

//==============================================================================
void FPGA_PRO_RD(void)
{
	FPGA_RD[0] = *(XintfZone0 + 9);//DIDO
	FPGA_RD[1] = (Uint16)*(XintfZone0 + 10);//MVB Addr
	FPGA_RD[2] = (Uint16)*(XintfZone0 + 11);//FPGA Error
	if(((FPGA_WR[0] >>10) & 0x0001)==1)
	{
		FPGA_RD[0] = 0;
		FPGA_RD[1] = 0;
		FPGA_RD[2] = 0;
	}
	if(((FPGA_WR[0] >>13) & 0x0001)==0)
	{
		FPGA_RD[2] = 0;
	}

}

//==============================================================================
void SPWM(void)
{

	PWM1AQ_SET = abs(PWM1AQ_SET-1);
	PWM2AQ_SET = abs(PWM2AQ_SET-1);
	PWM3AQ_SET = abs(PWM3AQ_SET-1);
	PWM_PRD = 18029;
	if(PWM1AQ_SET==0)
	{
		PWM1A_test += 2000;
		if(PWM1A_test>=16000)
			PWM1A_test = 4000;
	}
	if(PWM2AQ_SET==0)
	{
		PWM2A_test += 1500;
		if(PWM2A_test>=16000)
			PWM2A_test = 3000;
	}
	if(PWM3AQ_SET==0)
	{
		PWM3A_test += 1000;
		if(PWM3A_test>=16000)
			PWM3A_test = 2000;
	}

	if(PWM1AQ_SET==0)//Mode:1->0
		PWM1A_CMP = PWM1A_test;
	else
		PWM1A_CMP = (PWM_PRD - PWM1A_test);

	if(PWM2AQ_SET==0)//Mode:1->0
			PWM2A_CMP = PWM2A_test;
		else
			PWM2A_CMP = (PWM_PRD - PWM2A_test);

	if(PWM3AQ_SET==0)//Mode:1->0
			PWM3A_CMP = PWM3A_test;
		else
			PWM3A_CMP = (PWM_PRD - PWM3A_test);

	PWM4A_CMP += 1500;
	if(PWM4A_CMP>=16000)
		PWM4A_CMP = 2000;

	if((PWM_PRD<=30000) && (PWM_PRD>=300))
	{
		EPwm1Regs.TBPRD =  (Uint16)PWM_PRD;// TBCLK counts 9375*2/75 = 250us
		EPwm2Regs.TBPRD =  (Uint16)PWM_PRD;
		EPwm3Regs.TBPRD =  (Uint16)PWM_PRD;
	}
//PWM1
	if(PWM1AQ_SET==0)//Mode:1->0
	{
		EPwm1Regs.CMPA.half.CMPA = (Uint16)PWM1A_CMP;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm1Regs.CMPB = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		EPwm1Regs.AQCTLA.all = 0x212;
	}
	if(PWM1AQ_SET==1)//Mode:0->1
	{
		EPwm1Regs.CMPA.half.CMPA = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm1Regs.CMPB = (Uint16)PWM1A_CMP;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		if(PWM1A_CMP>0)
			EPwm1Regs.AQCTLA.all = 0x211;
		else
			EPwm1Regs.AQCTLA.all = 0x212;
	}

//PWM2
	if(PWM2AQ_SET==0)//Mode:1->0
	{
		EPwm2Regs.CMPA.half.CMPA = (Uint16)PWM2A_CMP;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm2Regs.CMPB = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		EPwm2Regs.AQCTLA.all = 0x212;
	}
	if(PWM2AQ_SET==1)//Mode:0->1
	{
		EPwm2Regs.CMPA.half.CMPA = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm2Regs.CMPB = (Uint16)PWM2A_CMP;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		if(PWM2A_CMP>0)
			EPwm2Regs.AQCTLA.all = 0x211;
		else
			EPwm2Regs.AQCTLA.all = 0x212;
	}
//*/
	if(PWM3AQ_SET==0)//Mode:1->0
	{
		EPwm3Regs.CMPA.half.CMPA = (Uint16)PWM3A_CMP;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm3Regs.CMPB = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		EPwm3Regs.AQCTLA.all = 0x212;
	}
	if(PWM3AQ_SET==1)//Mode:0->1
	{
		EPwm3Regs.CMPA.half.CMPA = (Uint16)PWM_PRD + 1;//TB_PWM_PERIOD * 0.2;//2000;//
		EPwm3Regs.CMPB = (Uint16)PWM3A_CMP;//TB_PWM_PERIOD * 0.4;//(Uint16)Ua_CMPB;//
		if(PWM3A_CMP>0)
			EPwm3Regs.AQCTLA.all = 0x211;
		else
			EPwm3Regs.AQCTLA.all = 0x212;
	}
		
	EPwm4Regs.CMPA.half.CMPA = (Uint16)PWM4A_CMP;//TB_PWM_PERIOD * 0.8;//
//	EPwm4Regs.CMPB = TB_PWM_PERIOD * 0.2;//(Uint16)Ua_CMPB;
}

//==============================================================================================
void InitAD7606(void)
{	
	EALLOW;
//No Oversampling
//	GpioDataRegs.GPBCLEAR.bit.GPIO50=1;//OS0=0
//	GpioDataRegs.GPBCLEAR.bit.GPIO51=1;//OS0=0
//	GpioDataRegs.GPBCLEAR.bit.GPIO52=1;//OS0=0
//Range is 锟斤�0V
//	GpioDataRegs.GPBSET.bit.GPIO59=1;//1->锟斤�0V//Nonsense now
//Disable Standby
//	GpioDataRegs.GPBSET.bit.GPIO53=1;//1->No Standby
//Reset AD7606,Typically 50ns,here 100ns
	GpioDataRegs.GPBCLEAR.bit.GPIO53=1;
	DELAY_US(5L);
		
	GpioDataRegs.GPBSET.bit.GPIO53=1;
	DELAY_NS(100L);
	GpioDataRegs.GPBCLEAR.bit.GPIO53=1;
	DELAY_US(2L);

	EDIS;

	FPGA_WR[2] &= 0xFE;
	*(XintfZone7 + 1) = FPGA_WR[2];

	SpiaRegs.SPITXBUF=0xAAA0;//spi初始化配置参�
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	/*
	0b1010101010100000 �到第5位无效位，第6到第13位为通道0-3的范围���4到第16位为写范围寄存器1指令�01
	例第12、第13位为通道0的范围�择配置，当前�1 +-5V
	00�-10V
	01�-5V
	10�-2.5V
	11: 0-10V
	*/
	FPGA_WR[2] |= 0x01;
	*(XintfZone7 + 1) = FPGA_WR[2];

	DELAY_US(1L);

	FPGA_WR[2] &= 0xFE;
	*(XintfZone7 + 1) = FPGA_WR[2];

	SpiaRegs.SPITXBUF=0xCAA0;//spi初始化配置参�
	while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {;}
	/*
	0b1100101010100000 �到第5位无效位，第6到第13位为通道4-7的范围�择，�4到第16位为写范围寄存器2指令�10
	例第12、第13位为通道4的范围�择配置，当前�1 +-5V
	00�-10V
	01�-5V
	10�-2.5V
	11: 0-10V
	*/
	FPGA_WR[2] |= 0x01;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_US(1L);
} 

//==============================================================================================
void InitFPGA(void)
{
//	FPGA_ALL_RST_EN();
	FPGA_WR[0] |= 0x0400;
	DELAY_US(30L);
//	FPGA_ALL_RST_DIS();
	FPGA_WR[0] &= 0xFBFF;
	DELAY_US(5L);
}

//==============================================================================================
void InitDRV(void)
{
//	*(XintfZone0 + 9) = 0x4100;
//	DELAY_US(100L);
//	*(XintfZone0 + 9) = 0x8100;

//	FPGA_Err_RST_EN();
	FPGA_WR[2]=0xFFFF;
	*(XintfZone7 + 1) = FPGA_WR[2];

	FPGA_WR[2] &= 0xBF;
	*(XintfZone7 + 1) = FPGA_WR[2];
	DELAY_US(10L);

	FPGA_WR[2] |= 0x40;
	*(XintfZone7 + 1) = FPGA_WR[2];

//	MCU_Err_DIS();
}

//==============================================================================================
void InitDGMVB(void)
{
//	DIS_MVBRST();
	DELAY_US(5L);
//	EN_MVBRST();
}

//==============================================================================================
void InitVariables(void)
{
	Uint16 i=0;
	Pin=0;INT_ID1=1;INT_ID2=1;

	PWM1AQ_SET=2;PWM2AQ_SET=2;PWM3AQ_SET=2;PWM_PRD=18029;PWM1A_CMP=9105;PWM2A_CMP=4500;PWM3A_CMP=2500;PWM4A_CMP=1000;PWM1A_test=2000;PWM2A_test=2000;PWM3A_test=2000;

	Cnt_PWM=0;Cnt_APP_1ms=0;Cnt_250us=0;Cnt_ms=0;Cnt_sec=0;Cnt_MCU=0;Cnt_DSP=0;Cnt_main=0;Cnt_Err_DSP=0;
	Cnt_Dir01F=0;Cnt_Dir01B=0;Cnt_Dir02F=0;Cnt_Dir02B=0;Cnt_Spd1=1;Cnt_Spd2=1;
	Cnt_main=0;Cnt_Err_DSP=0;
	Ro_SP01.dir=1;Ro_SP01.omega_m=0;Ro_SP01.omega_m_medium=0;Ro_SP01.omega_m_high=0;Ro_SP01.omega_m_low=0;Ro_SP01.pos_cur=0;Ro_SP01.pos_delta=0;Ro_SP01.pos_pre=0;
	Ro_SP02.dir=2;Ro_SP02.omega_m=0;Ro_SP02.omega_m_medium=0;Ro_SP02.omega_m_high=0;Ro_SP02.omega_m_low=0;Ro_SP02.pos_cur=0;Ro_SP02.pos_delta=0;Ro_SP02.pos_pre=0;
	TMP_Rt=0;Zero_Ia=0;Zero_Ib=0;
	Pin=0;Pout=0;
//------------------------------------------------------------
	for(i=0;i<5;i++)
		{
		ERR_OS[i]=0;
		DA_CH[i]=0;
		}
//------------------------------------------------------------
	for(i=0;i<10;i++)
	{
		HMI_TBUF[i]=0;
		HMI_RBUF[i]=0;
		FPGA_RD[i]=0;
		FPGA_WR[i]=0;
		PWM_OS[i]=0;
	}
//------------------------------------------------------------
	for(i=0;i<20;i++)
	{
		CTRL_OS[i]=0;
	}
//------------------------------------------------------------
	for(i=0;i<30;i++)
	{
		AD_CH[i]=0;

	}
//------------------------------------------------------------
	for(i=0;i<40;i++)
	{
		EEPROM_WR[i]=0;
		EEPROM_RD[i]=0;
		AI[i]=0;
		CMD_OS[i]=0;
		DPRAM_RD[i]=0;
		DPRAM_WR[i]=0;
	}
//------------------------------------------------------------
	CONV_ADL();
}

//===============================================================================================
//=========================================THE END===============================================
//===============================================================================================
