/*
 * NUG_App.c
 *
 *  Created on: 2020-3-2
 *      Author: 700363
 */

// **************************************************************************
// the includes
#include	"DSP2833x_Device.h"
#include	"DSP2833x_Examples.h"

// **************************************************************************
// the defines

//======================== DSP state ==========================
#define  ChpIni			0x00    	//芯片初始化状态
#define  DSPIni			0x01		//DSP控制参数初始化状态
#define  DSPIniFn		0x02		//系统初始化完成
#define  OVPTst			0x03		//OVP测试状态
#define  OVPTstFn		0x04		//OVP测试完成
#define  PreFlx			0x05		//预励磁状态
#define  PreFlxFn		0x06		//预励磁完成
#define  TqOut			0x07		//转矩输出状态
#define  TqOutFn		0x08		//转矩输出结束状态
#define  DisChg			0x09		//放电状态
#define  DisChgFn		0x0A		//放电完成状态
#define  FltStt			0x0B		//故障状态
#define  NX_DSPSt       (os.STA_OUTHandle->DSPSt)

//======================== MCU request ==========================
#define  DspRst			(1<<0) 			//1    bit0:  reset DSP   请求DSP软件重启DSP reset
#define  OvpTsEn		(1<<1)			//2    bit1:  Ovp test enabled  请求激活OVP OVP test disabled
#define  Br 			(1<<2) 		    //4    bit2:  brake 制动状态   brake
#define  DspClr			(1<<3) 			//8    bit3:  clear DSP  DSP故障复位 DSP clear
#define  CtOp			(1<<4)			//16   bit4:  contact open  请求激活斩波   contactor open
#define  CtOpHL			(1<<5)			//32   bit5:  contact open HIGH LEVEL 请求激活高等级斩波，斩波故障时，直接跳入斩波完成
#define  PrEtEn			(1<<6)			//64   bit6:  Pre-excitation enable 请求激活预励磁
#define  TqOutEn		(1<<7)			//128  bit7:  torque output enable 请求激活转矩输出
#define  mTqOutFn		(1<<8)			//256  bit8:  torque output finish 请求激活转矩输出结束
#define  CvSpFg			(1<<9)			//512  bit9:  covertor stop flag 变流器停止标志
#define  CvSpFn			(1<<10)			//1024 bit10: covertor stop completed 变流器停止完成标志
#define  CvStAg			(1<<11)			//2048 bit11: covertor start again 变流器二次启动请求
#define  CvReSt			(1<<12)			//4096 bit12: covertor Re-start  变流器重启请求
#define  MCUHwIniFn		(1<<13)			//8192 bit13: MCU hardware initial finish MCU硬件初始化完成
#define  NX_MCUCmd		(os.STA_INHandle->MCUCmd.all)

#define  M_CvSa()		(os.DO_OSHandle->DO1.bit.L_InvEn = 1)
#define	 M_CvSo()       (os.DO_OSHandle->DO1.bit.L_InvEn = 0)

// **************************************************************************
// the typedefs
typedef struct _AI_Obj_ {
	float32 XTpFt_MdAm_Flt; //环温  AI05
	float32 XTpFt_HCOT_Flt; //散热器温度 AI06
	float32 XTpFt_Mot1_Flt; //1号电机绕组温度 AI07
	float32 XTpFt_Mot2_Flt; //2号电机绕组温度 AI08
	float32 XTpFt_Mot3_Flt; //3号电机绕组温度 AI21 IOE

	float32 XTpFt_Mot4_Flt; //4号电机绕组温度 AI22 IOE
	float32 XTpFt_Brd_Flt; //板卡温度
	float32 XVFt_Spd1; //1号电机转速  SP01
	float32 XVFt_Spd2; //2号电机转速 SP02
	float32 XVFt_Spd3; //3号电机转速 SP03

	float32 XVFt_Spd4; //4号电机转速 SP04
	float32 XIFt_IA; //A相电流 AI01
	float32 XIFt_IB; //B相电流 AI02
	float32 XIFt_IacHwPrPos_Lim; //交流电流硬保正阀值
	float32 XIFt_IacHwPrNeg_Lim; //交流电流硬保负阀值

	float32 XIFt_IDC; //直流电流 AI04
	float32 XUFt_UDC1; //直流电压 AI03
	float32 XUFt_UDCHwPr_Lim; //直流电压硬保阀值
	float32 XIFt_EFLekPr_Lim; //接地漏电流硬保阀值
	float32 XUFt_UPh; //交流电压互感器 AI23

	float32 XIFt_IPh1; //交流电流互感器 AI24
	float32 XIFt_IBt; //蓄电池电流 AI25
	float32 XUFt_UDC2; //直流母线电压，备用 AI26
	float32 XIFt_IBtCg; //蓄电池充电电流 AI28
	float32 XIFt_IC; //C相电流 AI29

	float32 XUFt_UDC3; //直流母线电压，备用 AI30
	float32 XIFt_IPh2; //交流电流互感器 AI31
	float32 XUFt_Bt; //蓄电池电压 AI27
	float32 rsvd1;
	float32 rsvd2;
} AI_Obj, *AI_Handle;

struct DI1_BITS {
	Uint16 L_OptoFb :1;
	Uint16 L_InvFb :1;
	Uint16 rsvd :14;
};
union DI1_REG {
	Uint16 all;
	struct DI1_BITS bit;
};
typedef struct _DI_OS_Obj_ {
	union DI1_REG DI1;
} DI_OS_Obj, *DI_OS_Handle;

typedef struct _CFG_IN_Obj_ {
	Uint16 NX_MCUVer;
	Uint16 NX_FPGAVer;
	Uint16 rsvd1;
	Uint16 rsvd2;
} CFG_IN_Obj, *CFG_IN_Handle;

struct MCU_CMD_BITS {
	Uint16 L_DspRst :1;				//1    bit0:  reset DSP   请求DSP软件重启DSP reset
	Uint16 L_OvpTsEn :1;//2    bit1:  Ovp test enabled  请求激活OVP OVP test disabled
	Uint16 L_Br :1; 			 		    //4    bit2:  brake 制动状态   brake
	Uint16 L_DspClr :1;			 	//8    bit3:  clear DSP  DSP故障复位 DSP clear
	Uint16 L_CtOp :1;		//16   bit4:  contact open  请求激活斩波   contactor open

	Uint16 L_CtOpHL :1;	//32   bit5:  contact open HIGH LEVEL 请求激活高等级斩波，斩波故障时，直接跳入斩波完成
	Uint16 L_PrEtEn :1;				//64   bit6:  Pre-excitation enable 请求激活预励磁
	Uint16 L_TqOutEn :1;			//128  bit7:  torque output enable 请求激活转矩输出
	Uint16 L_TqOutFn :1;		//256  bit8:  torque output finish 请求激活转矩输出结束
	Uint16 L_CvSpFg :1;					//512  bit9:  covertor stop flag 变流器停止标志

	Uint16 L_CvSpFn :1;			//1024 bit10: covertor stop completed 变流器停止完成标志
	Uint16 L_CvStAg :1;				//2048 bit11: covertor start again 变流器二次启动请求
	Uint16 L_CvReSt :1;					//4096 bit12: covertor Re-start  变流器重启请求
	Uint16 L_MCUHwIniFn :1;	//8192 bit13: MCU hardware initial finish MCU硬件初始化完成
	Uint16 rsvd :2;
};
union MCU_CMD_REG {
	Uint16 all;
	struct MCU_CMD_BITS bit;
};
typedef struct _STA_IN_Obj_ {
	Uint16 NX_MCUSt;
	Uint16 NX_MCUTck;
	Uint16 NX_DSPTck;
	Uint16 NX_FPGATck;
	Uint16 NX_PWMTck;

	Uint16 NX_RTOSTck;
	union MCU_CMD_REG MCUCmd;
	Uint16 rsvd2;
} STA_IN_Obj, *STA_IN_Handle;

struct ERR_DSP1_BITS {
	Uint16 L_DPRAMRd :1;
	Uint16 L_DPRAMWr :1;
	Uint16 L_MCUHw :1;
	Uint16 L_FPGAHw :1;
	Uint16 L_RTOSInt :1;

	Uint16 L_PWMInt :1;
	Uint16 L_AD1 :1;
	Uint16 L_AD2 :1;
	Uint16 L_LowAD :1;
	Uint16 L_SP1 :1;

	Uint16 L_SP2 :1;
	Uint16 L_SP3 :1;
	Uint16 L_SP4 :1;
	Uint16 L_DA :1;
	Uint16 rsvd :2;
};
union ERR_DSP1_REG {
	Uint16 all;
	struct ERR_DSP1_BITS bit;
};
typedef struct _ERR_DSP_Obj_ {
	union ERR_DSP1_REG ERR_DSP1;
	Uint16 rsvd1;
	Uint16 rsvd2;
} ERR_DSP_Obj, *ERR_DSP_Handle;

struct ERR_EXTR1_BITS {
	Uint16 L_PWM1A :1;
	Uint16 L_PWM1B :1;
	Uint16 L_PWM2A :1;
	Uint16 L_PWM2B :1;
	Uint16 L_PWM3A :1;

	Uint16 L_PWM3B :1;
	Uint16 L_PWM4A :1;
	Uint16 L_PWM4B :1;
	Uint16 L_PWM5A :1;
	Uint16 L_PWM5B :1;

	Uint16 rsvd :4;
	Uint16 L_PWM :1;

	Uint16 L_Hw :1;
};
union ERR_EXTR1_REG {
	Uint16 all;
	struct ERR_EXTR1_BITS bit;
};
typedef struct _ERR_EXTR_Obj_ {
	union ERR_EXTR1_REG ERR_EXTR1;
	Uint16 rsvd1;
	Uint16 rsvd2;
} ERR_EXTR_Obj, *ERR_EXTR_Handle;

typedef struct _PWM_OS_Obj_ {
	Uint16 YTm_PwmPdVv;
	Uint16 YX_PwmMo;
	Uint16 YX_Pwm1AVv;
	Uint16 YX_Pwm2AVv;
	Uint16 YX_Pwm3AVv;

	Uint16 YTm_Pwm4PdVv;
	Uint16 YX_Pwm4AVv;
	Uint16 YX_Pwm4BVv;
	Uint16 YTm_Pwm5PdVv;
	Uint16 YX_Pwm5AVv;

	Uint16 rsvd1;
} PWM_OS_Obj, *PWM_OS_Handle;

struct DO1_BITS {
	Uint16 L_InvEn :1;
	Uint16 rsvd :15;
};
union DO1_REG {
	Uint16 all;
	struct DO1_BITS bit;
};
typedef struct _DO_OS_Obj_ {
	union DO1_REG DO1;
} DO_OS_Obj, *DO_OS_Handle;

typedef struct _CFG_OUT_Obj_ {
	Uint16 NX_DSPAppVer;
	Uint16 rsvd1;
	Uint16 rsvd2;
} CFG_OUT_Obj, *CFG_OUT_Handle;

typedef struct _STA_OUT_Obj_ {
	Uint16 DSPSt;
	Uint16 rsvd1;
	Uint16 rsvd2;
	Uint16 rsvd3;
} STA_OUT_Obj, *STA_OUT_Handle;

typedef struct _HSTPDA_Obj_ {
	Uint16 NX_MtNo;
	float32 PUi_Np;
	float32 PFt_Lm;
	float32 PFt_Ls;
	float32 PFt_Lr;

	float32 PFt_Rs;
	float32 PFt_Rr;
	float32 PFt_Id_Nom;
	float32 PFt_IMot_Max;
	float32 PFt_FMot_Nom;

	float32 PFt_UDclk_Nom;
	float32 PFt_IDclk_Nom;
	float32 PFt_IAC_Nom;
	float32 NX_SpdOrTq;//电机控制模式
	float32 WX_CpDrt;//斩波占空比

	float32 PR_BrRs;
	float32 PTm_BrRsDp;
	float32 PK_BrRs;
	float32 PX_BrRsTpCft;
	float32 PR_DcNd;

	float32 PTm_DcNdDp;
	float32 PK_DcNd;
	float32 PX_DcNdTpCft;
	float32 PX_SpdLoKp;
	float32 PX_SpdLoKi;

	float32 PX_DclkFlt1;
	float32 PX_DclkFlt2;

} HSTPDA_Obj, *HSTPDA_Handle;

typedef struct _HSTIDA_Obj_ {
	Uint16 NX_BusDir;
	float32 XVFt_BusSpd;
	Uint16 C_DIR;
	float32 CTq_TQ;
	float32 NXFt_Wgh;

	float32	rsvd1;
	float32	rsvd2;
	float32	rsvd3;
	float32	rsvd4;
	float32	rsvd5;
} HSTIDA_Obj, *HSTIDA_Handle;

typedef struct _HSTODA_Obj_ {
	float32 XP_In;
	float32 XP_Out;
	float32 XM_OutTq;
	float32 XH_BrRsTp_Est;
	float32 XH_DcNdTp_Est;
	float32 XIFt_IA_Rms;
	float32 XIFt_IB_Rms;
	float32 XIFt_IC_Rms;
	float32 rsvd;
} HSTODA_Obj, *HSTODA_Handle;

typedef struct _OS_Obj_ {
	AI_Handle AIHandle;
	DI_OS_Handle DI_OSHandle;
	CFG_IN_Handle CFG_INHandle;
	STA_IN_Handle STA_INHandle;
	ERR_DSP_Handle ERR_DSPHandle;

	ERR_EXTR_Handle ERR_EXTRHandle;
	PWM_OS_Handle PWM_OSHandle;
	DO_OS_Handle DO_OSHandle;
	CFG_OUT_Handle CFG_OUTHandle;
	STA_OUT_Handle STA_OUTHandle;
} OS_Obj, *OS_Handle;

struct COM_BITS {							// S_FltSt1
	Uint16 TA0 :1;		// Udc过压/欠压
	Uint16 TA1 :1;		// Idc过流
	Uint16 TA2 :1;		// Idc采样失效
	Uint16 TA3 :1;		// Udc采样失效
	Uint16 TA4 :1;		// DSP过载
	Uint16 TA5 :1;		// Ia过流
	Uint16 TA6 :1;		// Ib过流
	Uint16 TA7 :1;		// Ic过流
	Uint16 TA8 :1;		// Iac（交流电流）不平衡
	Uint16 TA9 :1;		// Ia失相
	Uint16 TA10 :1;		// Ib失相
	Uint16 TA11 :1;		// Ic失相
	Uint16 TA12 :1;		// Ia观测故障
	Uint16 TA13 :1;		// Ib观测故障
	Uint16 TA14 :1;		// Ic观测故障
	Uint16 TA15 :1;		// Iac（交流电流）采样失效
};

union WARN_REG {
	Uint16 all;
	struct COM_BITS bit;
};

//------------------------------
typedef struct _XX_UIInStc_t_ {
	float32 XUFt_UDC;   		    		// DC-link voltage, V
	float32 XUFt_U3Ph;						// 3 phase line voltage, V
	float32 XIFt_IDC;						// DC-link current, A
	float32 XIFt_IA;						// phase A current, A
	float32 XIFt_IB;						// phase B current, A
} XX_UIInStc_t;

typedef struct _XX_SpdDrInStc_t_ {
	float32 XVFt_Spd1; 		    	// rotate speed 1, r/min
	float32 XVFt_Spd2;		    		// rotate speed 2, r/min
	float32 XVFt_Spd3;		    		// rotate speed 3, r/min
	float32 XVFt_Spd4;		    		// rotate speed 4, r/min
	Uint16 SX_MotDir_Flt;				// motor direction of train
} XX_SpdDrInStc_t;

typedef struct _YX_PwmOutStc_t_ {
	Uint16 YX_PwmMo;				// PWM mode, inverter
	Uint16 YTm_PwmPdVv;   		// PWM period value, inverter
	Uint16 YX_Pwm1AVv;   			// PWM1A value
	Uint16 YX_Pwm2AVv;			// PWM2A value
	Uint16 YX_Pwm3AVv;			// PWM3A value
	Uint16 YTm_Pwm4PdVv;			// PWM period value, chopper
	Uint16 YX_Pwm4AVv;			// PWM4A value, chopper 1
	Uint16 YX_Pwm4BVv;			// PWM4B value, chopper 2
} YX_PwmOutStc_t;

struct DSPAPPFLT1_BITS {
	Uint16 L_Stt :1;		//状态机故障
	Uint16 L_Init :1;		//初始化故障
	Uint16 L_OvpTst :1;	//ovp测试故障
	Uint16 L_PreFlxSyn :1;	//预励磁同步失败
	Uint16 L_DisChg :1;	//放电故障
	Uint16 rsvd :11;
};
union DSPAPPFLT1_REG {
	Uint16 all;
	struct DSPAPPFLT1_BITS bit;
};

struct DSPAPPFLT2_BITS {
	Uint16 L_DCOV :1;		//直流过压
	Uint16 L_DCLV :1;		//直流欠压
	Uint16 L_DCOI :1;		//直流过流
	Uint16 L_IAOI :1;		//A相过流
	Uint16 L_IBOI :1;		//B相过流
	Uint16 L_ICOI :1;		//C相过流
	Uint16 L_IUB :1;		//电流三相不平衡
	Uint16 L_PH :1;		//缺相
	Uint16 L_SpdO :1;		//超速
	Uint16 rsvd :7;
};
union DSPAPPFLT2_REG {
	Uint16 all;
	struct DSPAPPFLT2_BITS bit;
};

struct DSPAPPFLT3_BITS {
	Uint16 L_UABOV :1;		//UV线电压过电压
	Uint16 L_UBCOV :1;		//VW线电压过电压
	Uint16 L_UCAOV :1;		//WU线电压过电压
	Uint16 L_MdOT :1;		//模块过温
	Uint16 L_MtOT :1;		//电机过温
	Uint16 L_Frq :1;		//频率错误
	Uint16 rsvd :10;
};
union DSPAPPFLT3_REG {
	Uint16 all;
	struct DSPAPPFLT3_BITS bit;
};

// **************************************************************************
// the globals

volatile OS_Obj os;

volatile HSTPDA_Obj hstpda;
volatile HSTIDA_Obj hstida;
volatile HSTODA_Obj hstoda;

union WARN_REG S_FltSt0, S_FltSt1, S_FltSt2;

volatile Uint16 Nt_WarnFn = 0;
volatile Uint16 SX_NtFlt = 0;

volatile float32 WM_TqCmd = 0; 			// default: = 0
volatile float32 S_PreFlxFlg = 2;		// default: = 2;
volatile int16 SX_DisChgOK = 2;			// default: = 2;
volatile int16 SX_OvpTsOk = 2;// default: = 2;		// OVP test ok, default: 2

//-------------------------
volatile XX_UIInStc_t XX_UIIn;
volatile XX_SpdDrInStc_t XX_SpdDrIn;
volatile YX_PwmOutStc_t YX_PwmOut;

// **************************************************************************
// the function prototypes
void state_machine(void);

void input(void);
void control(void);
void ouput(void);

void protect(void);
void SaSoCv(void);
void chopper(void);

// **************************************************************************
// the functions

void INIT_EV(void) {
	Uint16 i = 0;

	os.AIHandle = (AI_Handle) &AI[0];
	os.DI_OSHandle = (DI_OS_Handle) &DI_OS[0];
	os.CFG_INHandle = (CFG_IN_Handle) &CFG_IN[0];
	os.STA_INHandle = (STA_IN_Handle) &STA_IN[0];
	os.ERR_DSPHandle = (ERR_DSP_Handle) &ERR_DSP[0];

	os.ERR_EXTRHandle = (ERR_EXTR_Handle) &ERR_EXTR[0];
	os.PWM_OSHandle = (PWM_OS_Handle) &PWM_OS[0];
	os.DO_OSHandle = (DO_OS_Handle) &DO_OS[0];
	os.CFG_OUTHandle = (CFG_OUT_Handle) &CFG_OUT[0];
	os.STA_OUTHandle = (STA_OUT_Handle) &STA_OUT[0];



}

void Cycle_OS(void) {
	state_machine();

	if (NX_DSPSt == DSPIni) {

		Nt_WarnFn = 1;			//初始化完成
		SX_NtFlt = 0;			//初始化故障
	}
}

void INT_RTOS(void) {
	chopper();
	SaSoCv();
	protect();
}

void INT_PWM(void) {
	input();
	control();
	ouput();
}

void state_machine(void) {
	//operating state
	if (NX_DSPSt == DSPIni) {
		if (Nt_WarnFn == 1) {
			if (SX_NtFlt == 0) {
				NX_DSPSt = DSPIniFn;			//DSP initialization finished
			} else {
				NX_DSPSt = FltStt;
				S_FltSt0.bit.TA1 = 1;				//DSP2 initialization failed
				SX_NtFlt = 0;
			}
		}
	} else if (NX_DSPSt == DSPIniFn)	// DSP系统初始化完成
	{
		if ((NX_MCUCmd & CtOp)||(NX_MCUCmd&CtOpHL))		//收到MCU放电请求，接触器断开了
{		NX_DSPSt = DisChg;	//放电状态
	}
	else if(NX_MCUCmd&CvReSt)	//收到MCU硬件初始化
	{
		NX_DSPSt=ChpIni;	//DSP跳到芯片初始化状态
	}
	else if(NX_MCUCmd&OvpTsEn)		//收到OVP测试请求
	{
		NX_DSPSt = OVPTst;	//OVP测试状态
	}
}
else if(NX_DSPSt==OVPTst)	//OVP测试状态
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//放电状态
	}
	else if(SX_OvpTsOk==1)			//测试通过
	{
		NX_DSPSt=OVPTstFn;		//OVP完成
	}
	else if(SX_OvpTsOk==0)		//测试未通过
	{
		NX_DSPSt = FltStt;
		S_FltSt0.bit.TA2 = 1;	//
	}
}
else if(NX_DSPSt==OVPTstFn)	//OVP测试完成状态
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//放电状态
	}
	else if(NX_MCUCmd&PrEtEn)	//收到MCU硬件初始化启动运行              //直流电源测试用
	{
		NX_DSPSt=PreFlx;		//预励磁状态
	}
}
else if(NX_DSPSt==PreFlx) //预励磁状态
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//OVP测试状态
	}
	else if(NX_MCUCmd&mTqOutFn)
	{
		if(fabs(WM_TqCmd)<15)
		{
			NX_DSPSt=TqOutFn;
		}
	}
	else if(S_PreFlxFlg==1.0)   	// time counter done in MATLAB
	{
		NX_DSPSt = PreFlxFn;	//预励磁完成
	}
	else if(S_PreFlxFlg==0.0)
	{
		NX_DSPSt = FltStt;
		S_FltSt0.bit.TA3 = 1;
	}
}
else if(NX_DSPSt==PreFlxFn)					//系统运行
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//OVP测试状态
	}
	else if(NX_MCUCmd&mTqOutFn)
	{
		if(fabs(WM_TqCmd)<15)
		{
			NX_DSPSt=TqOutFn;
		}
	}
	else if(NX_MCUCmd&TqOutEn)   	// time counter done in MATLAB
	{
		NX_DSPSt = TqOut;	//
	}
}
else if(NX_DSPSt==TqOut)					//转矩输出
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//OVP测试状态
	}
	else if(NX_MCUCmd&mTqOutFn)
	{
		if(fabs(WM_TqCmd)<15)
		{
			NX_DSPSt=TqOutFn;
		}
	}
}
else if(NX_DSPSt==TqOutFn)		//转矩输出结束
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//OVP测试状态
	}
	else if(NX_MCUCmd&CvStAg)
	{
		NX_DSPSt=DSPIniFn;
	}
	else if(NX_MCUCmd&CvSpFg)
	{
		NX_DSPSt=OVPTstFn;
	}
	else if(NX_MCUCmd&PrEtEn)
	{
		NX_DSPSt=PreFlx;
	}
}
else if(NX_DSPSt==DisChg)
{
	if(SX_DisChgOK==1)
	{
		NX_DSPSt=DisChgFn;
		SX_DisChgOK=2;
	}
	else if(SX_DisChgOK==0)
	{
		NX_DSPSt = FltStt;
		S_FltSt0.bit.TA4 = 1;
		SX_DisChgOK=2;
	}
}
else if(NX_DSPSt==DisChgFn)             //
{
	if((NX_MCUCmd&CvSpFn)||(NX_MCUCmd&CvStAg))
	{
		NX_DSPSt=DSPIniFn;
	}
}
else if(NX_DSPSt==FltStt)
{
	if((S_FltSt0.bit.TA4==1)&&(NX_MCUCmd&CtOpHL))//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChgFn;	//
		S_FltSt0.bit.TA4=0;
	}
	else if(((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))&&(S_FltSt0.bit.TA4==0))//收到MCU放电请求，接触器断开了
	{
		NX_DSPSt = DisChg;	//
		S_FltSt0.bit.TA3=0;
		S_FltSt0.bit.TA2=0;
//			S_FltSt1.all=0;
	}
	else if((NX_MCUCmd&mTqOutFn)&&(S_FltSt0.bit.TA4==0))
	{
		NX_DSPSt=TqOutFn;
//			S_FltSt1.all=0;
	}
	else if(NX_MCUCmd&CvReSt)
	{
		NX_DSPSt=ChpIni;
	}
}
else if(NX_DSPSt==ChpIni)
{
	S_FltSt0.all=0;
	S_FltSt1.all=0;
	S_FltSt2.all=0;
	Nt_WarnFn = 0;
	if(NX_MCUCmd&MCUHwIniFn)					//MCU硬件初始化完成
	{
		NX_DSPSt=DSPIni;
	}
}
else
{
	S_FltSt0.bit.TA0=1;
}

//reset
if(NX_MCUCmd&DspClr)
{
	S_FltSt0.all = 0;
	S_FltSt1.all = 0;
	S_FltSt2.all = 0;
}
}

void input(void) {
	XX_UIIn.XUFt_UDC = os.AIHandle->XUFt_UDC1;
	XX_UIIn.XUFt_U3Ph = os.AIHandle->XUFt_UPh;
	XX_UIIn.XIFt_IA = os.AIHandle->XIFt_IA;
	XX_UIIn.XIFt_IB = os.AIHandle->XIFt_IB;
	XX_UIIn.XIFt_IDC = os.AIHandle->XIFt_IDC;
}

void ouput(void) {
	os.PWM_OSHandle->YX_PwmMo = YX_PwmOut.YX_PwmMo;
	os.PWM_OSHandle->YTm_PwmPdVv = YX_PwmOut.YTm_PwmPdVv;
	os.PWM_OSHandle->YX_Pwm1AVv = YX_PwmOut.YX_Pwm1AVv;
	os.PWM_OSHandle->YX_Pwm2AVv = YX_PwmOut.YX_Pwm2AVv;
	os.PWM_OSHandle->YX_Pwm3AVv = YX_PwmOut.YX_Pwm3AVv;
}

void control(void) {

}

void chopper(void) {
	os.PWM_OSHandle->YTm_Pwm4PdVv = YX_PwmOut.YTm_Pwm4PdVv;
	os.PWM_OSHandle->YX_Pwm4AVv = YX_PwmOut.YX_Pwm4AVv;
	os.PWM_OSHandle->YX_Pwm4BVv = YX_PwmOut.YX_Pwm4BVv;
}

void protect(void) {

}

void SaSoCv(void) {
	if ((os.STA_OUTHandle->DSPSt == PreFlx)
			|| (os.STA_OUTHandle->DSPSt == PreFlxFn)
			|| (os.STA_OUTHandle->DSPSt == TqOut)) {
		M_CvSa();
	} else {
		M_CvSo();
	}
}
