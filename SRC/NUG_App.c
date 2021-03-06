/*
 * NUG_App.c
 *
 *  Created on: 2020-3-2
 *      Author: 700363
 */

// **************************************************************************
// the includes
#include	<math.h>
#include	"module.h"
#include	"DSP2833x_NUG_App.h"

// **************************************************************************
// the defines

//======================== DSP state ==========================
#define	ChpIni				0x00    	//芯片初始化状态
#define	ChpIniFn	    	0x01		//芯片初始化完成状态
#define	DSPIni				0x02		//DSP控制参数初始化状态
#define	DSPIniFn			0x03		//系统初始化完成
#define	OVPTst				0x04		//OVP测试状态
#define	OVPTstFn			0x05		//OVP测试完成
#define	PreFlxSyn			0x06		//预励磁/同步状态
#define	PreFlxSynFn		0x07		//预励磁/同步完成
#define	TqXpOut			0x08		//转矩/有功输出状态
#define	TqXpOutFn		0x09		//转矩/有功率输出结束状态
#define	DisChg				0x0A		//放电状态
#define	DisChgFn			0x0B		//放电完成状态
#define	FltStt				0x0C		//故障状态
#define	DSPOptoTst  	0x0D    //Opto测试态
#define	NX_DSPSt			(os.STA_OUTHandle->DSPSt)

//======================== MCU request ==========================
#define  	DspRst				(1<<0) 			//1    bit0:  reset DSP   请求DSP软件重启DSP reset
#define  	OvpTsEn			(1<<1)			//2    bit1:  Ovp test enabled  请求激活OVP OVP test disabled
#define  	Br 					(1<<2) 		    //4    bit2:  brake 制动状态   brake
#define  	DspClr				(1<<3) 			//8    bit3:  clear DSP  DSP故障复位 DSP clear
#define  	CtOp				(1<<4)			//16   bit4:  contact open  请求激活斩波   contactor open
#define  	CtOpHL			(1<<5)			//32   bit5:  contact open HIGH LEVEL 请求激活高等级斩波，斩波故障时，直接跳入斩波完成
#define  	PrEtEn				(1<<6)			//64   bit6:  Pre-excitation enable 请求激活预励磁
#define  	TqOutEn			(1<<7)			//128  bit7:  torque output enable 请求激活转矩输出
#define  	mTqOutFn		(1<<8)			//256  bit8:  torque output finish 请求激活转矩输出结束
#define  	CvSpFg				(1<<9)			//512  bit9:  covertor stop flag 变流器停止标志
#define  	CvSpFn				(1<<10)		//1024 bit10: covertor stop completed 变流器停止完成标志
#define  	CvStAg				(1<<11)		//2048 bit11: covertor start again 变流器二次启动请求
#define  	CvReSt				(1<<12)		//4096 bit12: covertor Re-start  变流器重启请求
#define  	MCUHwIniFn	(1<<13)		//8192 bit13: MCU hardware initial finish MCU硬件初始化完成
#define  	NX_MCUCmd	(os.STA_INHandle->MCUCmd.all)

//===========================MCU state=========================
#define 	OptoTst			(0x301)

//========================flag==============================================
//flag[0]
#define 	SL_DspIniOk						0x00
#define	SL_OvpTstOk						0x01
#define	SL_OvpTstFl						0x02
#define 	SL_PreFlxSynOk					0x03
#define 	SL_PreFlxSynFl					0x04
#define 	SL_DisChgOk					0x05
#define 	SL_DisChgFl						0x06
#define	SL_TqXpZero						0x07
#define 	SL_Run								0x08
#define	SL_CvCtrl							0x09
#define	SL_ChpCtrl						0x0A

//flag[1]


//===========================InvEn InvBc======================================
#define	M_CvEn()			(os.DO_OSHandle->DO1.bit.L_CvEn = 1)
#define  	M_CvBc()     		(os.DO_OSHandle->DO1.bit.L_CvEn = 0)

//===========================ChpEn ChpBc=====================================
#define 	M_ChpEn()		(os.DO_OSHandle->DO1.bit.L_ChpEn = 1)
#define  	M_ChpBc()     	(os.DO_OSHandle->DO1.bit.L_ChpEn = 0)

//======================================================================
#define	M_SetFlag(x)		(flag[(x & 0xF0)>>4] |= (((Uint16)0x0001)<<(x & 0x0F)))
#define	M_ClrFlag(x)		(flag[(x & 0xF0)>>4] &=~ (((Uint16)0x0001)<<(x & 0x0F)))
#define	M_NotFlag(x)	(flag[(x & 0xF0)>>4] ^= (((Uint16)0x0001)<<(x & 0x0F)))
#define	M_ChkFlag(x)	(flag[(x & 0xF0)>>4] &(((Uint16)0x0001)<<(x & 0x0F)))
#define	M_ChkCounter(x,y)	((int32)(x-y))

// **************************************************************************
// the typedefs

//---------------------------------AI---------------------------------
typedef struct _AI_Obj_
{
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
	float32 XIFt_I1; //交流电流 AI01
	float32 XIFt_I2; //交流电流 AI02
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
	float32 NX_SpDir; //转速方向
	float32 rsvd2;
} AI_Obj, *AI_Handle;

//---------------------------------DI_OS---------------------------------
struct DI1_BITS
{
	Uint16 L_TracSafe:1;				//牵引安全信号  外部
	Uint16 L_LineTrip:1;				//板载LineTrip
	Uint16 L_OptoFb :1;				//Opto使能反馈
	Uint16 L_PwmHwErr:1;			//PWM驱动板故障
	Uint16 L_SysHwErr:1;				//系统硬件保护
	Uint16 L_CvFb :1;					//逆变使能反馈
	Uint16 L_ChpFb:1;					//斩波使能反馈
	Uint16 rsvd :9;
};
union DI1_REG
{
	Uint16 all;
	struct DI1_BITS bit;
};
typedef struct _DI_OS_Obj_
{
	union DI1_REG DI1;
} DI_OS_Obj, *DI_OS_Handle;

//---------------------------------CFG_IN---------------------------------
typedef struct _CFG_IN_Obj_
{
	Uint16 NX_MCUVer;
	Uint16 NX_FPGAVer;
	Uint16 rsvd1;
	Uint16 rsvd2;
} CFG_IN_Obj, *CFG_IN_Handle;

//---------------------------------STA_IN---------------------------------
struct MCU_CMD_BITS
{
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
union MCU_CMD_REG
{
	Uint16 all;
	struct MCU_CMD_BITS bit;
};
typedef struct _STA_IN_Obj_
{
	Uint16 NX_MCUSt;
	Uint16 NX_MCUTck;
	Uint16 NX_DSPTck;
	Uint16 NX_FPGATck;
	Uint16 NX_PWMTck;

	Uint16 NX_RTOSTck;
	union MCU_CMD_REG MCUCmd;
	Uint16 rsvd2;
} STA_IN_Obj, *STA_IN_Handle;

//---------------------------------ERR_DSP---------------------------------
struct ERR_DSP1_BITS
{
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
union ERR_DSP1_REG
{
	Uint16 all;
	struct ERR_DSP1_BITS bit;
};
struct ERR_DSP2_BITS
{
	Uint16 L_Stt :1;		//状态机故障
	Uint16 L_Init :1;		//初始化故障
	Uint16 L_OvpTst :1;		//ovp测试故障
	Uint16 L_PreFlxSyn :1;	//预励磁同步失败
	Uint16 L_DisChg :1;		//放电故障

	Uint16 rsvd :11;
};
union ERR_DSP2_REG
{
	Uint16 all;
	struct ERR_DSP2_BITS bit;
};
struct ERR_DSP3_BITS
{
	Uint16 L_DCOV :1;		//直流过压
	Uint16 L_DCLV :1;		//直流欠压
	Uint16 L_DCOI :1;		//直流过流
	Uint16 L_IAOI :1;		//A相过流
	Uint16 L_IBOI :1;		//B相过流

	Uint16 L_ICOI :1;		//C相过流
	Uint16 L_IUB :1;		//电流三相不平衡
	Uint16 L_PH :1;			//缺相
	Uint16 L_SpdO :1;		//超速
	Uint16 rsvd :7;
};
union ERR_DSP3_REG
{
	Uint16 all;
	struct ERR_DSP3_BITS bit;
};
typedef struct _ERR_DSP_Obj_
{
	//底层故障
	union ERR_DSP1_REG ERR_DSP1;
	Uint16 rsvd1;
	Uint16 rsvd2;
	//应用层故障
	union ERR_DSP2_REG ERR_DSP2;		//状态类
	union ERR_DSP3_REG ERR_DSP3;		//信号类
	Uint16 rsvd3;
	Uint16 rsvd4;
	Uint16 rsvd5;
} ERR_DSP_Obj, *ERR_DSP_Handle;

//---------------------------------ERR_EXTR---------------------------------
struct ERR_EXTR1_BITS
{
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
union ERR_EXTR1_REG
{
	Uint16 all;
	struct ERR_EXTR1_BITS bit;
};
typedef struct _ERR_EXTR_Obj_
{
	union ERR_EXTR1_REG ERR_EXTR1;
	Uint16 rsvd1;
	Uint16 rsvd2;
} ERR_EXTR_Obj, *ERR_EXTR_Handle;

//---------------------------------PWM_OS---------------------------------
typedef struct _PWM_OS_Obj_
{
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

//---------------------------------DO_OS---------------------------------
struct DO1_BITS
{
	Uint16 L_CvEn :1;					//光纤逆变使能指令
	Uint16 L_ChpEn:1;					//光纤斩波使能指令
	Uint16 rsvd :14;
};
union DO1_REG
{
	Uint16 all;
	struct DO1_BITS bit;
};
typedef struct _DO_OS_Obj_
{
	union DO1_REG DO1;
} DO_OS_Obj, *DO_OS_Handle;

//---------------------------------CFG_OUT---------------------------------
typedef struct _CFG_OUT_Obj_
{
	Uint16 NX_DSPAppVer;
	Uint16 rsvd1;
	Uint16 rsvd2;
} CFG_OUT_Obj, *CFG_OUT_Handle;

//---------------------------------STA_OUT---------------------------------
typedef struct _STA_OUT_Obj_
{
	Uint16 DSPSt;
	Uint16 rsvd1;
	Uint16 rsvd2;
	Uint16 rsvd3;
} STA_OUT_Obj, *STA_OUT_Handle;

typedef struct _CUST_MCU_PAR_Obj_
{
	Uint16 rsvd1[31];   //底层用
	Uint16 MCUTxPar[60];   //低8位有效
	Uint16 rsvd2[9];
} CUST_MCU_PAR_Obj, *CUST_MCU_PAR_Handle;

typedef struct _CUST_MCU_1ms_Obj_
{
	Uint16 MCUTxVar[20];   //低8位有效
} CUST_MCU_1ms_Obj, *CUST_MCU_1ms_Handle;

typedef struct _CUST_MCU_2ms_Obj_
{
	Uint16 MCUTxVar[40];   //低8位有效
} CUST_MCU_2ms_Obj, *CUST_MCU_2ms_Handle;

typedef struct _CUST_MCU_16ms_Obj_
{
	Uint16 MCUTxVar[40];   //低8位有效
} CUST_MCU_16ms_Obj, *CUST_MCU_16ms_Handle;

typedef struct _CUST_MCU_64ms_Obj_
{
	Uint16 MCUTxVar[40];   //低8位有效
} CUST_MCU_64ms_Obj, *CUST_MCU_64ms_Handle;

typedef struct _CUST_DSP_1ms_Obj_
{
	Uint16 DSPTxVar[20];   //低8位有效
} CUST_DSP_1ms_Obj, *CUST_DSP_1ms_Handle;

typedef struct _CUST_DSP_2ms_Obj_
{
	Uint16 DSPTxVar[40];   //低8位有效
} CUST_DSP_2ms_Obj, *CUST_DSP_2ms_Handle;

typedef struct _CUST_DSP_16ms_Obj_
{
	Uint16 DSPTxVar[40];   //低8位有效
} CUST_DSP_16ms_Obj, *CUST_DSP_16ms_Handle;

typedef struct _CUST_DSP_64ms_Obj_
{
	Uint16 DSPTxVar[40];   //低8位有效
} CUST_DSP_64ms_Obj, *CUST_DSP_64ms_Handle;

typedef struct _OS_Obj_
{
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

	CUST_MCU_PAR_Handle CUST_MCU_PARHandle;
	CUST_MCU_1ms_Handle CUST_MCU_1msHandle;
	CUST_MCU_2ms_Handle CUST_MCU_2msHandle;
	CUST_MCU_16ms_Handle CUST_MCU_16msHandle;
	CUST_MCU_64ms_Handle CUST_MCU_64msHandle;

	CUST_DSP_1ms_Handle CUST_DSP_1msHandle;
	CUST_DSP_2ms_Handle CUST_DSP_2msHandle;
	CUST_DSP_16ms_Handle CUST_DSP_16msHandle;
	CUST_DSP_64ms_Handle CUST_DSP_64msHandle;
} OS_Obj, *OS_Handle;

//---------------------------------------------------------
typedef struct _HSTPDA_Obj_
{
	Uint16 NX_MtNo;
	Uint16 NX_Np;
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
	float32 NX_SpdOrTq;	//电机控制模式
	float32 WX_CpDrt;	//斩波占空比

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

typedef struct _HSTIDA_Obj_
{
	//1ms
	float32 HMI_rsvd;
	Uint16 NX_BusDir;
	float32 XVFt_BusSpd;
	Uint16 C_DIR;
	float32 CTq_TQ;
	float32 NXFt_Wgh;

	Uint16 rsvd1[5];

	//2ms
	//	Uint16 rsvd2[20];

	//16ms
	//	Uint16 rsvd3[20];

	//64ms
	//	Uint16 rsvd4[20];

} HSTIDA_Obj, *HSTIDA_Handle;

typedef struct _HSTODA_Obj_
{
	//1ms
	Uint16 BankCh1;
	Uint16 BankCh2;
	Uint16 BankCh3;
	Uint16 BankCh4;
	Uint16 BankCh5;
	Uint16 BankCh6;
	Uint16 rsvd1[4];

	//2ms
	float32 XP_In;
	float32 XP_Out;
	float32 XM_OutTq;
	float32 XH_BrRsTp_Est;
	float32 XH_DcNdTp_Est;



	//	Uint16 rsvd2[11];

	//16ms
	//	Uint16 rsvd3[20];

	//64ms
	float32 XIFt_IA_Avg;
	float32 XIFt_IB_Avg;
	float32 XIFt_IC_Avg;
	float32 XIFt_IA_Rms;
	float32 XIFt_IB_Rms;
	float32 XIFt_IC_Rms;
	float32 XIFt_IA_Sqr;
	float32 XIFt_IB_Sqr;
	float32 XIFt_IC_Sqr;
	//	Uint16 rsvd4[20];
} HSTODA_Obj, *HSTODA_Handle;

//---------------------------------------
typedef struct _XX_UIInStc_t_
{
	float32 XUFt_UDC;   		    		// DC-link voltage, V
	float32 XUFt_U3Ph;					// 3 phase line voltage, V
	float32 XIFt_IDC;						// DC-link current, A
	float32 XIFt_IA;							// phase A current, A
	float32 XIFt_IB;							// phase B current, A
	float32 XIFt_IC;							// phase C current, A
} XX_UIInStc_t;

typedef struct _XX_SpdDrInStc_t_
{
	float32 XVFt_Spd1; 		    		// rotate speed 1, r/min
	float32 XVFt_Spd2;		    		// rotate speed 2, r/min
	float32 XVFt_Spd3;		    		// rotate speed 3, r/min
	float32 XVFt_Spd4;		    		// rotate speed 4, r/min
	Uint16 SX_MotDir_Flt;				// motor direction of train
} XX_SpdDrInStc_t;

typedef struct _YX_PwmOutStc_t_
{
	Uint16 YX_PwmMo;					// PWM mode, inverter
	Uint16 YTm_PwmPdVv;   			// PWM period value, inverter
	Uint16 YX_Pwm1AVv;   				// PWM1A value
	Uint16 YX_Pwm2AVv;				// PWM2A value
	Uint16 YX_Pwm3AVv;				// PWM3A value
	Uint16 YTm_Pwm4PdVv;			// PWM period value, chopper
	Uint16 YX_Pwm4AVv;				// PWM4A value, chopper 1
	Uint16 YX_Pwm4BVv;				// PWM4B value, chopper 2
} YX_PwmOutStc_t;

typedef struct _CvCtrl_Obj_
{
	float32 Analog[4];						//电压  、alpha电流、beta电流、转速
	float32 CmdTq;							//转矩指令
	Uint16 SpDir;							//转速方向
	float32 Duty[5];							//开关周期、模式、A占空比、B占空比、C占空比
} CvCtrl_Obj, *CvCtrl_Handle;

typedef struct _XX_Pro_Obj_
{
	float32 PU_UDCOVL;
	float32 PU_UDCLVL;
	float32 PI_IDCOIL;
	float32 PI_IAOIL;
	float32 PI_IBOIL;
	float32 PI_ICOIL;
	float32 PX_SpdOL;

	Uint16 Cnt_UDCOV;
	Uint16 Cnt_UDCLV;
	Uint16 Cnt_IDCOI;
	Uint16 Cnt_IAOI;
	Uint16 Cnt_IBOI;
	Uint16 Cnt_ICOI;
	Uint16 Cnt_IUB;
	Uint16 Cnt_PH;
	Uint16 Cnt_SpdO;
} XX_Pro_Obj;

typedef struct _Counter_Obj_
{
	Uint16 Cnt1;
	Uint16 Cnt2;
	Uint16 Cnt3;
	Uint16 Cnt4;
	Uint16 Cnt5;

	Uint16 Cnt_ChpIni;
	Uint16 Cnt_DspIni;
	Uint16 Cnt_OvpTst;
	Uint16 Cnt_PreFlxSyn;
	Uint16 Cnt_DisChg;

}Counter_Obj;
// **************************************************************************
// the globals

volatile OS_Obj os;

volatile HSTPDA_Obj hstpda;
volatile HSTIDA_Obj hstida;
volatile HSTODA_Obj hstoda;

volatile Uint16 Cnt_RmsA = 0;
volatile Uint16 Cnt_RmsB = 0;
volatile Uint16 Cnt_RmsC = 0;
//--------------------------------------
volatile Uint16 Cnt_RTOS = 0;
volatile Uint16 flag[16];

//-------------------------
volatile XX_UIInStc_t XX_UIIn;
volatile XX_SpdDrInStc_t XX_SpdDrIn;
volatile YX_PwmOutStc_t YX_PwmOut;

volatile CvCtrl_Obj CvCtrl;
volatile float32 frq, syntheta, U3PhAbs, cvtheta, MRef;
volatile cfloat32 Udq, Uab;

volatile XX_Pro_Obj XX_Pro;
volatile Counter_Obj Counter;
// **************************************************************************
// the function prototypes

void InitApp(void);
void HSTPDA(void);

void sample_input(void);
void protect(void);
//void CvControl(void);
void CvControlM(void);
void pwm_ouput(void);

void state_machine(void);
void chopper(void);

void HSTIDA(void);
void HSTODA(void);

void CntCtrl(void);
//--------------------------
void OptoTest(void);


// **************************************************************************
// the functions

void INIT_EV(void)
{
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

	os.CUST_MCU_PARHandle = (CUST_MCU_PAR_Handle) &CUST_MCU_PAR[0];
	os.CUST_MCU_1msHandle = (CUST_MCU_1ms_Handle) &CUST_MCU_1ms[0];
	os.CUST_MCU_2msHandle = (CUST_MCU_2ms_Handle) &CUST_MCU_2ms[0];
	os.CUST_MCU_16msHandle = (CUST_MCU_16ms_Handle) &CUST_MCU_16ms[0];
	os.CUST_MCU_64msHandle = (CUST_MCU_64ms_Handle) &CUST_MCU_64ms[0];

	os.CUST_DSP_1msHandle = (CUST_DSP_1ms_Handle) &CUST_DSP_1ms[0];
	os.CUST_DSP_2msHandle = (CUST_DSP_2ms_Handle) &CUST_DSP_2ms[0];
	os.CUST_DSP_16msHandle = (CUST_DSP_16ms_Handle) &CUST_DSP_16ms[0];
	os.CUST_DSP_64msHandle = (CUST_DSP_64ms_Handle) &CUST_DSP_64ms[0];
}

void Cycle_OS(void)
{
	if (NX_DSPSt == DSPIni)
	{
		HSTPDA();
		InitApp();
	}

	if (1 == (Cnt_RTOS % 5))
	{
		HSTIDA();
		HSTODA();
	}

//	if (0 == (os.STA_INHandle->NX_RTOSTck % 5))
//	{
//		HSTIDA();
//		HSTODA();
//	}
}


void INT_RTOS(void)
{
	Cnt_RTOS++;
	if (Cnt_RTOS >= 10)
		Cnt_RTOS = 0;

	state_machine();

	if (0 == (Cnt_RTOS % 5))
		CntCtrl();


//	if (NX_DSPSt >= DSPIniFn){
//		M_ChpEn();
//
//		if(os.DI_OSHandle->DI1.bit.L_ChpFb)
//			M_SetFlag(SL_ChpCtrl);
//		else
//			M_ClrFlag(SL_ChpCtrl);
//
//		chopper();
//	}else
//	{
//		M_ChpBc();
//		M_ClrFlag(SL_ChpCtrl);
//	}
}

void INT_PWM(void)
{
	if (NX_DSPSt == DSPOptoTst)
	{
		M_CvEn();
		M_ChpEn();
		OptoTest();
	}
	else if (NX_DSPSt >= DSPIniFn)
	{
		//---------------------------------------
		if (((NX_DSPSt == PreFlxSyn) || (NX_DSPSt == PreFlxSynFn)
				|| (NX_DSPSt == TqXpOut)) && os.DI_OSHandle->DI1.bit.L_OptoFb)
			M_CvEn();
		else
			M_CvBc();

		if ((os.DI_OSHandle->DI1.bit.L_OptoFb)
				&& (os.DI_OSHandle->DI1.bit.L_CvFb))
			M_SetFlag(SL_CvCtrl);
		else
			M_ClrFlag(SL_CvCtrl);

		sample_input();
		protect();
//		CvControl();
		CvControlM();
		pwm_ouput();

		//---------------------------------------------
		M_ChpEn();

		if(os.DI_OSHandle->DI1.bit.L_ChpFb)
			M_SetFlag(SL_ChpCtrl);
		else
			M_ClrFlag(SL_ChpCtrl);

		chopper();
	}
	else
	{
		M_CvBc();
		M_ClrFlag(SL_CvCtrl);
		M_ChpBc();
		M_ClrFlag(SL_ChpCtrl);
	}
}

void state_machine(void)
{
	//-----------------进入测试态------------
	if (os.STA_INHandle->NX_MCUSt == 0x301)
		NX_DSPSt = DSPOptoTst;

	//---------------operating state---------------------
	if (NX_DSPSt == ChpIni)
	{
		;//底层完成与MCU的一次性参数交互后返回0x01
	}
	else if (NX_DSPSt == ChpIniFn)
	{
		if (NX_MCUCmd & MCUHwIniFn)					//MCU返回硬件初始化完成标志
			NX_DSPSt = DSPIni;
	}
	else if (NX_DSPSt == DSPIni)
	{
		if (M_ChkFlag(SL_DspIniOk))
		{
				NX_DSPSt = DSPIniFn;		//DSP initialization finished
				M_ClrFlag(SL_DspIniOk);
		}
	}
	else if (NX_DSPSt == DSPIniFn)
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;	//放电状态
		}
		else if(NX_MCUCmd&CvReSt)//重启变流器
		{
			NX_DSPSt=ChpIni;
		}
		else if(NX_MCUCmd&OvpTsEn)
		{
			NX_DSPSt = OVPTst;
		}
	}
	else if(NX_DSPSt==OVPTst)	//OVP测试状态
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;	//放电状态
		}
		else if(M_ChkFlag(SL_OvpTstOk))			//测试通过
		{
			NX_DSPSt=OVPTstFn;		//OVP完成
			M_ClrFlag(SL_OvpTstOk);
		}
		else if(M_ChkFlag(SL_OvpTstFl))		//测试失败
		{
			os.ERR_DSPHandle->ERR_DSP2.bit.L_OvpTst = 1;	//OVP test fail
			M_ClrFlag(SL_OvpTstFl);
		}
	}
	else if(NX_DSPSt==OVPTstFn)	//OVP测试完成状态
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;	//放电状态
		}
		else if(NX_MCUCmd&PrEtEn)	//收到MCU预励磁请求
		{
			NX_DSPSt=PreFlxSyn;		//预励磁状态
		}
	}
	else if(NX_DSPSt==PreFlxSyn) //预励磁状态
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;
		}
		else if(NX_MCUCmd&mTqOutFn)
		{
			if(M_ChkFlag(SL_TqXpZero))
			{
				NX_DSPSt=TqXpOutFn;
				M_ClrFlag(SL_TqXpZero);
			}
		}
		else if(M_ChkFlag(SL_PreFlxSynOk))   	// time counter done in MATLAB
		{
			NX_DSPSt = PreFlxSynFn;	//预励磁完成
			M_ClrFlag(SL_PreFlxSynOk);
		}
		else if(M_ChkFlag(SL_PreFlxSynFl))
		{
			os.ERR_DSPHandle->ERR_DSP2.bit.L_PreFlxSyn = 1;	//预励磁失败
			M_ClrFlag(SL_PreFlxSynFl);
		}
	}
	else if(NX_DSPSt==PreFlxSynFn)					//预励磁完成 系统运行
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;	//OVP测试状态
		}
		else if(NX_MCUCmd&mTqOutFn)
		{
			if(M_ChkFlag(SL_TqXpZero))
			{
				NX_DSPSt=TqXpOutFn;
				M_ClrFlag(SL_TqXpZero);
			}
		}
		else if(NX_MCUCmd&TqOutEn)   	// time counter done in MATLAB
		{
			NX_DSPSt = TqXpOut;	//
		}
	}
	else if(NX_DSPSt==TqXpOut)					//转矩输出
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;
		}
		else if(NX_MCUCmd&mTqOutFn)
		{
			if(M_ChkFlag(SL_TqXpZero))
			{
				NX_DSPSt=TqXpOutFn;
				M_ClrFlag(SL_TqXpZero);
			}
		}
	}
	else if(NX_DSPSt==TqXpOutFn)		//转矩输出结束
	{
		if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;
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
			NX_DSPSt=PreFlxSyn;
		}
	}
	else if(NX_DSPSt==DisChg)	//放电
	{
		if(M_ChkFlag(SL_DisChgOk))
		{
			NX_DSPSt=DisChgFn;
			M_ClrFlag(SL_DisChgOk);
		}
		else if(M_ChkFlag(SL_DisChgFl))
		{
			os.ERR_DSPHandle->ERR_DSP2.bit.L_DisChg = 1;
			M_ClrFlag(SL_DisChgFl);
		}
	}
	else if(NX_DSPSt==DisChgFn)             //放电完成
	{
		if((NX_MCUCmd&CvSpFn)||(NX_MCUCmd&CvStAg))
			NX_DSPSt=DSPIniFn;
	}
	else if(NX_DSPSt==FltStt)             //故障
	{
		if((os.ERR_DSPHandle->ERR_DSP2.bit.L_DisChg == 1)&&(NX_MCUCmd&CtOpHL)) //收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChgFn;	//
			os.ERR_DSPHandle->ERR_DSP2.bit.L_DisChg = 0;
		}
		else if(((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))&&(os.ERR_DSPHandle->ERR_DSP2.bit.L_DisChg == 0))//收到MCU放电请求，接触器断开了
		{
			NX_DSPSt = DisChg;	//
			os.ERR_DSPHandle->ERR_DSP2.bit.L_PreFlxSyn=0;
			os.ERR_DSPHandle->ERR_DSP2.bit.L_OvpTst=0;
		}
		else if((NX_MCUCmd&mTqOutFn)&&(os.ERR_DSPHandle->ERR_DSP2.bit.L_DisChg == 0))
		{
			NX_DSPSt=TqXpOutFn;
		}
		else if(NX_MCUCmd&CvReSt)
		{
			NX_DSPSt=ChpIni;
		}
	}
	else if(NX_DSPSt==DSPOptoTst)
	{
		if(NX_MCUCmd&CvReSt)
		{
			NX_DSPSt=ChpIni;
		}
	}
	else
	{
		os.ERR_DSPHandle->ERR_DSP2.bit.L_Stt = 1;
	}

	//error
	if (NX_DSPSt == DSPOptoTst)
	{
		;
	}
	else
	{
		if ((os.ERR_DSPHandle->ERR_DSP1.all) || (os.ERR_DSPHandle->ERR_DSP2.all)
				|| (os.ERR_DSPHandle->ERR_DSP3.all))
		{
			NX_DSPSt = FltStt;
		}
	}

	//reset error
	if (NX_MCUCmd & DspClr)
	{
		os.ERR_DSPHandle->ERR_DSP2.all = 0;
		os.ERR_DSPHandle->ERR_DSP3.all = 0;
	}

	//run flag
//	if ((os.STA_OUTHandle->DSPSt == PreFlx)
//			|| (os.STA_OUTHandle->DSPSt == PreFlxFn)
//			|| (os.STA_OUTHandle->DSPSt == TqOut))
//		M_SetFlag(SL_InvCtrl);
//	else
//		M_ClrFlag(SL_InvCtrl);

}

void InitApp(void)
{
	Uint16 i = 0;
	Uint16 *ptr = NULL;

	ptr = (Uint16*) os.ERR_DSPHandle;
	for (i = 3; i < sizeof(ERR_DSP_Obj); i++)
	{
		*(ptr + i) = 0;	//应用层故障置0
	}
	os.ERR_DSPHandle->ERR_DSP2.all = 0;
	os.ERR_DSPHandle->ERR_DSP3.all = 0;

	ptr = (Uint16*) os.PWM_OSHandle;
	for (i = 0; i < sizeof(PWM_OS_Obj); i++)
	{
		*(ptr + i) = 0;
	}
	os.PWM_OSHandle->YTm_PwmPdVv = 0.001 * (150.0e6 / 8.0);
	os.PWM_OSHandle->YX_PwmMo = 21;
	os.PWM_OSHandle->YX_Pwm1AVv = 0.5 * os.PWM_OSHandle->YTm_PwmPdVv;
	os.PWM_OSHandle->YX_Pwm2AVv = 0.5 * os.PWM_OSHandle->YTm_PwmPdVv;
	os.PWM_OSHandle->YX_Pwm3AVv = 0.5 * os.PWM_OSHandle->YTm_PwmPdVv;
	os.PWM_OSHandle->YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);
	os.PWM_OSHandle->YX_Pwm4AVv = 0;

	ptr = (Uint16*) os.DO_OSHandle;
	for (i = 0; i < sizeof(DO_OS_Obj); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) os.CFG_OUTHandle;
	for (i = 0; i < sizeof(CFG_OUT_Obj); i++)
	{
		*(ptr + i) = 0;
	}
	os.CFG_OUTHandle->NX_DSPAppVer = 0x0011;

	ptr = (Uint16*) os.STA_OUTHandle;
	for (i = 0; i < sizeof(STA_OUT_Obj); i++)
	{
		*(ptr + i) = 0;
	}
	os.STA_OUTHandle->DSPSt = DSPIni;

	ptr = (Uint16*) os.CUST_DSP_1msHandle;
	for (i = 0; i < sizeof(CUST_DSP_1ms_Obj); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) os.CUST_DSP_2msHandle;
	for (i = 0; i < sizeof(CUST_DSP_2ms_Obj); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) os.CUST_DSP_16msHandle;
	for (i = 0; i < sizeof(CUST_DSP_16ms_Obj); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) os.CUST_DSP_64msHandle;
	for (i = 0; i < sizeof(CUST_DSP_64ms_Obj); i++)
	{
		*(ptr + i) = 0;
	}

	//----------------------------------------------
	ptr = (Uint16*) &XX_UIIn;
	for (i = 0; i < sizeof(XX_UIInStc_t); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) &XX_SpdDrIn;
	for (i = 0; i < sizeof(XX_SpdDrInStc_t); i++)
	{
		*(ptr + i) = 0;
	}

	ptr = (Uint16*) &CvCtrl;
	for (i = 0; i < sizeof(CvCtrl_Obj); i++)
	{
		*(ptr + i) = 0;
	}
	CvCtrl.Duty[0] = 0.001;
	CvCtrl.Duty[1] = 1.0;
	CvCtrl.Duty[2] = 0.5;
	CvCtrl.Duty[3] = 0.5;
	CvCtrl.Duty[4] = 0.5;

	ptr = (Uint16*) &XX_Pro;
	for (i = 0; i < sizeof(XX_Pro_Obj); i++)
	{
		*(ptr + i) = 0;
	}
	XX_Pro.PU_UDCOVL = 1900.0;
	XX_Pro.PU_UDCLVL = 1000.0;
	XX_Pro.PI_IDCOIL = 200.0;
	XX_Pro.PI_IAOIL = 200.0;
	XX_Pro.PI_IBOIL = 200.0;
	XX_Pro.PI_ICOIL = 200.0;
	XX_Pro.PX_SpdOL = 4000.0;

	frq = 0.0;
	syntheta = 0.0;
	U3PhAbs = 0.0;
	cvtheta = 0.0;
	MRef = 0.0;
	Udq.re = 0.0;
	Udq.im = 0.0;
	Uab.re = 0.0;
	Uab.im = 0.0;

	for(i=0;i<16;i++){
		flag[i] = 0;
	}

	if (0 == os.ERR_DSPHandle->ERR_DSP2.bit.L_Init)
		M_SetFlag(SL_DspIniOk);		//初始化成功

}

void sample_input(void)
{
	XX_UIIn.XUFt_UDC = os.AIHandle->XUFt_UDC1;
	XX_UIIn.XUFt_U3Ph = os.AIHandle->XUFt_UPh;
	XX_UIIn.XIFt_IDC = os.AIHandle->XIFt_IDC;
	XX_UIIn.XIFt_IA = os.AIHandle->XIFt_I1 *  (-1.0);
	XX_UIIn.XIFt_IC = os.AIHandle->XIFt_I2 *  (-1.0);
	XX_UIIn.XIFt_IB = -XX_UIIn.XIFt_IA - XX_UIIn.XIFt_IC;

	XX_SpdDrIn.XVFt_Spd1 = os.AIHandle->XVFt_Spd1;
	XX_SpdDrIn.XVFt_Spd2 = os.AIHandle->XVFt_Spd2;
	XX_SpdDrIn.XVFt_Spd3 = os.AIHandle->XVFt_Spd3;
	XX_SpdDrIn.XVFt_Spd4 = os.AIHandle->XVFt_Spd4;
	XX_SpdDrIn.SX_MotDir_Flt = ((Uint32) (os.AIHandle->NX_SpDir) & 0x0000FFFF);

//	LowPass(&hstoda.XIFt_IA_Avg, fabs(XX_UIIn.XIFt_IA),
//			CvCtrl.Duty[0] * PI2 * 1.0);		//一阶滤波 截止频率1Hz
//	LowPass(&hstoda.XIFt_IB_Avg, fabs(XX_UIIn.XIFt_IB),
//			CvCtrl.Duty[0] * PI2 * 1.0);		//一阶滤波 截止频率1Hz
//	LowPass(&hstoda.XIFt_IC_Avg, fabs(XX_UIIn.XIFt_IC),
//			CvCtrl.Duty[0] * PI2 * 1.0);		//一阶滤波 截止频率1Hz
//
//	hstoda.XIFt_IA_Rms = hstoda.XIFt_IA_Avg*PIby2SQRT2;//基波有效值
//	hstoda.XIFt_IB_Rms = hstoda.XIFt_IB_Avg*PIby2SQRT2;//基波有效值
//	hstoda.XIFt_IC_Rms = hstoda.XIFt_IC_Avg*PIby2SQRT2;//基波有效值

	RmsClc(&hstoda.XIFt_IA_Rms,XX_UIIn.XIFt_IA,256,&hstoda.XIFt_IA_Sqr,&Cnt_RmsA);
	RmsClc(&hstoda.XIFt_IB_Rms,XX_UIIn.XIFt_IB,256,&hstoda.XIFt_IB_Sqr,&Cnt_RmsB);
	RmsClc(&hstoda.XIFt_IC_Rms,XX_UIIn.XIFt_IC,256,&hstoda.XIFt_IC_Sqr,&Cnt_RmsC);

}

void pwm_ouput(void)
{
	if (YX_PwmOut.YX_PwmMo == 21)
	{
		YX_PwmOut.YX_PwmMo = 0;
	}
	else
	{
		YX_PwmOut.YX_PwmMo = 21;
	}

	YX_PwmOut.YTm_PwmPdVv = (Uint16) (CvCtrl.Duty[0] * (150.0e6 / 8.0));

	if (YX_PwmOut.YX_PwmMo == 21)
	{
		YX_PwmOut.YX_Pwm1AVv = YX_PwmOut.YTm_PwmPdVv * (1.0 - CvCtrl.Duty[2]);
		YX_PwmOut.YX_Pwm2AVv = YX_PwmOut.YTm_PwmPdVv * (1.0 - CvCtrl.Duty[3]);
		YX_PwmOut.YX_Pwm3AVv = YX_PwmOut.YTm_PwmPdVv * (1.0 - CvCtrl.Duty[4]);
	}
	else
	{
		YX_PwmOut.YX_Pwm1AVv = YX_PwmOut.YTm_PwmPdVv * CvCtrl.Duty[2];
		YX_PwmOut.YX_Pwm2AVv = YX_PwmOut.YTm_PwmPdVv * CvCtrl.Duty[3];
		YX_PwmOut.YX_Pwm3AVv = YX_PwmOut.YTm_PwmPdVv * CvCtrl.Duty[4];
	}

	os.PWM_OSHandle->YX_PwmMo = YX_PwmOut.YX_PwmMo;
	os.PWM_OSHandle->YTm_PwmPdVv = YX_PwmOut.YTm_PwmPdVv;
	os.PWM_OSHandle->YX_Pwm1AVv = YX_PwmOut.YX_Pwm1AVv;
	os.PWM_OSHandle->YX_Pwm2AVv = YX_PwmOut.YX_Pwm2AVv;
	os.PWM_OSHandle->YX_Pwm3AVv = YX_PwmOut.YX_Pwm3AVv;
}

//void CvControl(void)
//{
//	float32 U3PhLdRef;
//
//	if (M_ChkFlag(SL_InvCtrl))
//	{
//		CvCtrl.Analog[0] = XX_UIIn.XIFt_IA;
//		CvCtrl.Analog[1] = XX_UIIn.XIFt_IB;
//		CvCtrl.Analog[2] = XX_UIIn.XUFt_UDC;
//		CvCtrl.Analog[3] = XX_SpdDrIn.XVFt_Spd1;
//		CvCtrl.CmdTq = hstida.CTq_TQ;
//		CvCtrl.SpDir = XX_SpdDrIn.SX_MotDir_Flt;
//
//		U3PhLdRef = hstida.HMI_rsvd * 10.0;

//		if(NX_DSPSt == PreFlx)
//		{
//			RAMP2(&frq, 50.0, 0.001, 0.001, 0.0, FALSE, FALSE);
//		}
//		else if(NX_MCUCmd&mTqOutFn)
//		{
//			RAMP2(&frq, 0.0, 0.001, 0.001, 0.0, FALSE, FALSE);
//		}
//

//		U3PhLdRef = Limit(U3PhLdRef, 0.0, 380.0);
//
//		//	U3PhAbs = FKG4(frq, 0.0, 0.0, 6.0, 0.0, 50.0, U3PhLdRef, 100.0, U3PhLdRef)
//		//							* SQRT2bySQRT3 * 1.684;
//		U3PhLdRef = FKG4(frq, 0.0, 0.0, 6.0, 0.0, 50.0, U3PhLdRef, 100.0, U3PhLdRef)
//									* SQRT2bySQRT3 * 1.684;
//		RAMP2(&U3PhAbs, U3PhLdRef, 1.0, 1.0, 0.0, FALSE, FALSE);
//
//		MRef = U3PhAbs / XX_UIIn.XUFt_UDC;
//		MRef = OvMd(MRef);
//
//		syntheta += PI2 * frq * CvCtrl.Duty[0];
//
//		//	/**/
//		//	syntheta = PI2/3.0;
//		//	MRef = Limit((os.CUST_MCU_1msHandle->MCUTxVar[0] & 0x00FF) * 0.001,0.0,0.1);
//		//	/**/
//
//		if (syntheta > PI2)
//		{
//			syntheta -= PI2;
//		}
//		else if (syntheta < 0.0)
//		{
//			syntheta += PI2;
//		}
//
//		Udq = POL2CPLX(MRef, 0.0);
//		Uab = CPLXMULT(Udq, POL2CPLX(1.0, syntheta)); //ipark
//
//		SVPWM(&CvCtrl.Duty[2], &CvCtrl.Duty[3], &CvCtrl.Duty[4], Uab);
//
//		CvCtrl.Duty[2] = Limit(CvCtrl.Duty[2], 0.06, 0.94);
//		CvCtrl.Duty[3] = Limit(CvCtrl.Duty[3], 0.06, 0.94);
//		CvCtrl.Duty[4] = Limit(CvCtrl.Duty[4], 0.06, 0.94);
//
//		CvCtrl.Duty[0] = 1.0 / 18.75e6 * ((Uint16) ((1.0 / 2700.0) * 18.75e6));
//		CvCtrl.Duty[1] = 1.0 - CvCtrl.Duty[1];
//	}
//	else
//	{
//		CvCtrl.Duty[2] = 0.5;
//		CvCtrl.Duty[3] = 0.5;
//		CvCtrl.Duty[4] = 0.5;
//
//		CvCtrl.Duty[0] = 1.0 / 18.75e6 * ((Uint16) ((1.0 / 2700.0) * 18.75e6));
//		CvCtrl.Duty[1] = 1.0 - CvCtrl.Duty[1];
//	}
//	//--------------------------------------
//M_ClrFlag(SL_PreFlxSynFl);
//	if(frq>45.0)
//		M_SetFlag(SL_PreFlxSynOk);
//	else
//		M_ClrFlag(SL_PreFlxSynOk);
//
//	if(U3PhAbs<1.0)
//		M_SetFlag(SL_TqXpZero);
//	else
//		M_ClrFlag(SL_TqXpZero);
//}

void CvControlM(void)
{
	float32 FrqRef;
	float32 U3PhLdRef;

	if (M_ChkFlag(SL_CvCtrl))
	{
		CvCtrl.Analog[0] = XX_UIIn.XIFt_IA;
		CvCtrl.Analog[1] = XX_UIIn.XIFt_IB;
		CvCtrl.Analog[2] = XX_UIIn.XUFt_UDC;
		CvCtrl.Analog[3] = XX_SpdDrIn.XVFt_Spd1;
		CvCtrl.CmdTq = hstida.CTq_TQ;
		CvCtrl.SpDir = XX_SpdDrIn.SX_MotDir_Flt;

		FrqRef = hstida.HMI_rsvd;
		FrqRef = Limit(FrqRef,0,50.0);

		if(NX_MCUCmd&mTqOutFn)
			RAMP2(&frq, 0.0, 0.0005, 0.0005, 0.0, FALSE, FALSE);
		else
			RAMP2(&frq, FrqRef, 0.0005, 0.0005, 0.0, FALSE, FALSE);

		U3PhLdRef = FKG4(frq, 0.0, 0.0, 3.0, 30.0, 50.0, 380.0, 100.0, 380.0)
									* SQRT2bySQRT3;
		RAMP2(&U3PhAbs, U3PhLdRef, 0.5, 0.5, 0.0, FALSE, FALSE);

		MRef = U3PhAbs / XX_UIIn.XUFt_UDC;
		MRef = Limit(MRef,0,0.577);

//		syntheta += PI2 * frq * CvCtrl.Duty[0];
		syntheta -= PI2 * frq * CvCtrl.Duty[0];

		if (syntheta > PI2)
		{
			syntheta -= PI2;
		}
		else if (syntheta < 0.0)
		{
			syntheta += PI2;
		}

		Udq = POL2CPLX(MRef, 0.0);
		Uab = CPLXMULT(Udq, POL2CPLX(1.0, syntheta)); //ipark

		SVPWM(&CvCtrl.Duty[2], &CvCtrl.Duty[3], &CvCtrl.Duty[4], Uab);

		CvCtrl.Duty[2] = Limit(CvCtrl.Duty[2], 0.06, 0.94);
		CvCtrl.Duty[3] = Limit(CvCtrl.Duty[3], 0.06, 0.94);
		CvCtrl.Duty[4] = Limit(CvCtrl.Duty[4], 0.06, 0.94);

		CvCtrl.Duty[0] = 1.0 / 18.75e6 * ((Uint16) ((1.0 / 2700.0) * 18.75e6));
		CvCtrl.Duty[1] = 1.0 - CvCtrl.Duty[1];
	}
	else
	{
		CvCtrl.Duty[2] = 0.5;
		CvCtrl.Duty[3] = 0.5;
		CvCtrl.Duty[4] = 0.5;

		CvCtrl.Duty[0] = 1.0 / 18.75e6 * ((Uint16) ((1.0 / 2700.0) * 18.75e6));
		CvCtrl.Duty[1] = 1.0 - CvCtrl.Duty[1];
	}
	//--------------------------------------
	if(frq>0.9*FrqRef)
		M_SetFlag(SL_PreFlxSynOk);
	else
		M_ClrFlag(SL_PreFlxSynOk);

	if(U3PhAbs<1.0)
		M_SetFlag(SL_TqXpZero);
	else
		M_ClrFlag(SL_TqXpZero);
}

void chopper(void)
{
	if(M_ChkFlag(SL_ChpCtrl))
	{
		if (NX_DSPSt == OVPTst)
		{
			YX_PwmOut.YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);
			YX_PwmOut.YX_Pwm4AVv = 0.5 * YX_PwmOut.YTm_Pwm4PdVv;

			if(TRUE)
			{
				M_SetFlag(SL_OvpTstOk);
				Counter.Cnt_OvpTst = 0;
			}
			else
			{
				if(M_ChkCounter(Counter.Cnt_OvpTst,100)>0)
					M_SetFlag(SL_OvpTstFl);
			}
		}
		else if (NX_DSPSt == DisChg)
		{
			YX_PwmOut.YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);
			YX_PwmOut.YX_Pwm4AVv = 0.25 * YX_PwmOut.YTm_Pwm4PdVv;

			if(TRUE)
			{
				M_SetFlag(SL_DisChgOk);
				Counter.Cnt_DisChg = 0;
			}
			else
			{
				if(M_ChkCounter(Counter.Cnt_DisChg,500)>0)
					M_SetFlag(SL_DisChgFl);
			}
		}
		else
		{
			if (XX_UIIn.XUFt_UDC > 1900)
			{
				YX_PwmOut.YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);
				YX_PwmOut.YX_Pwm4AVv = 0.75 * YX_PwmOut.YTm_Pwm4PdVv;
			}
			else if (XX_UIIn.XUFt_UDC < 1700)
			{
				YX_PwmOut.YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);
				YX_PwmOut.YX_Pwm4AVv = 0;
			}
		}

		os.PWM_OSHandle->YTm_Pwm4PdVv = YX_PwmOut.YTm_Pwm4PdVv;
		os.PWM_OSHandle->YX_Pwm4AVv = YX_PwmOut.YX_Pwm4AVv;
	}
	else
	{
		os.PWM_OSHandle->YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);//
		os.PWM_OSHandle->YX_Pwm4AVv = 0;
	}
}

void protect(void)
{
	//UDCOV
	if (XX_UIIn.XUFt_UDC > XX_Pro.PU_UDCOVL)
	{
		if (XX_Pro.Cnt_UDCOV > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_DCOV = 1;
		else
			XX_Pro.Cnt_UDCOV++;
	}
	else
		XX_Pro.Cnt_UDCOV = 0;
	//UDCLV
	//	if ((XX_UIIn.XUFt_UDC < XX_Pro.PU_UDCLVL) && (SX_Run == 1))
	//	{
	//		if (XX_Pro.Cnt_UDCLV > 3)
	//			os.ERR_DSPHandle->ERR_DSP3.bit.L_DCLV = 1;
	//		else
	//			XX_Pro.Cnt_UDCLV++;
	//	}
	//	else
	//		XX_Pro.Cnt_UDCLV = 0;
	//IDCOI
	if (fabs(XX_UIIn.XIFt_IDC) > XX_Pro.PI_IDCOIL)
	{
		if (XX_Pro.Cnt_IDCOI > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_DCOI = 1;
		else
			XX_Pro.Cnt_IDCOI++;
	}
	else
		XX_Pro.Cnt_IDCOI = 0;
	//IAOI
	if (fabs(XX_UIIn.XIFt_IA) > XX_Pro.PI_IAOIL)
	{
		if (XX_Pro.Cnt_IAOI > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_IAOI = 1;
		else
			XX_Pro.Cnt_IAOI++;
	}
	else
		XX_Pro.Cnt_IAOI = 0;
	//IBOI
	if (fabs(XX_UIIn.XIFt_IB) > XX_Pro.PI_IBOIL)
	{
		if (XX_Pro.Cnt_IBOI > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_IBOI = 1;
		else
			XX_Pro.Cnt_IBOI++;
	}
	else
		XX_Pro.Cnt_IBOI = 0;
	//ICOI
	if (fabs(XX_UIIn.XIFt_IC) > XX_Pro.PI_ICOIL)
	{
		if (XX_Pro.Cnt_ICOI > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_ICOI = 1;
		else
			XX_Pro.Cnt_ICOI++;
	}
	else
		XX_Pro.Cnt_ICOI = 0;
	//IUB

	//PH

	//SpdO
	if (XX_SpdDrIn.XVFt_Spd1 > XX_Pro.PX_SpdOL)
	{
		if (XX_Pro.Cnt_SpdO > 3)
			os.ERR_DSPHandle->ERR_DSP3.bit.L_SpdO = 1;
		else
			XX_Pro.Cnt_SpdO++;
	}
	else
		XX_Pro.Cnt_SpdO = 0;

	//---------------------------------------------------------
	if ((os.ERR_DSPHandle->ERR_DSP1.all) || (os.ERR_DSPHandle->ERR_DSP2.all)
			|| (os.ERR_DSPHandle->ERR_DSP3.all))
		M_CvBc();

	if ((os.ERR_DSPHandle->ERR_DSP1.all))
		M_ChpBc();
}

/*
 *
 */
void HSTPDA(void)
{
	hstpda.NX_MtNo = os.CUST_MCU_PARHandle->MCUTxPar[0] & 0xff;
	hstpda.NX_Np = os.CUST_MCU_PARHandle->MCUTxPar[1] & 0xff;
	hstpda.PFt_Lm = (os.CUST_MCU_PARHandle->MCUTxPar[2] & 0xff + (os.CUST_MCU_PARHandle->MCUTxPar[3] & 0xff) >> 8) * 0.001;
	hstpda.PFt_Ls = (os.CUST_MCU_PARHandle->MCUTxPar[4] & 0xff + (os.CUST_MCU_PARHandle->MCUTxPar[5] & 0xff) >> 8) * 0.001;

	//参数合法性检查
	os.ERR_DSPHandle->ERR_DSP2.bit.L_Init = 0;		//初始化成功
	//
}

void HSTIDA(void)
{
	//1ms
	hstida.HMI_rsvd = (os.CUST_MCU_1msHandle->MCUTxVar[0] & 0x00FF) ;

	//2ms
	//	(os.CUST_MCU_2msHandle->MCUTxVar[0] & 0x00FF) * 10.0;

	//16ms
	//	(os.CUST_MCU_16msHandle->MCUTxVar[0] & 0x00FF) * 10.0;

	//64ms
	//	(os.CUST_MCU_64msHandle->MCUTxVar[0] & 0x00FF) * 10.0;

}

void HSTODA(void)
{
	//	Uint16 i;
	//	Uint16 *ptr;
	//
	hstoda.BankCh1 = (Uint16) os.STA_OUTHandle->DSPSt;
	hstoda.BankCh2 = (Uint16) os.ERR_DSPHandle->ERR_DSP1.all;
	hstoda.BankCh3 = (Uint16) os.ERR_DSPHandle->ERR_DSP2.all;
	hstoda.BankCh4 = (Uint16) os.ERR_DSPHandle->ERR_DSP3.all;
	hstoda.BankCh5 = (Uint16) os.PWM_OSHandle->YX_Pwm1AVv;
	hstoda.BankCh6 = (Uint16) frq;
	//		hstoda.BankCh4 = (Uint16)(AI[0]*10);
	//		hstoda.BankCh5 = (Uint16)(AI[1]*10);
	//		hstoda.BankCh4 = (Uint16)(XX_UIIn.XIFt_IA*100);
	//		hstoda.BankCh5 = (Uint16)(fabs(XX_UIIn.XIFt_IA)*100);
	//		hstoda.BankCh6 = (Uint16)(hstoda.XIFt_IA_Avg*100);
//	hstoda.BankCh2 = (Uint16) (hstoda.XIFt_IA_Rms*10);
//	hstoda.BankCh3 = (Uint16) (hstoda.XIFt_IB_Rms*10);
//	hstoda.BankCh4 = (Uint16) (hstoda.XIFt_IC_Rms*10);
		hstoda.BankCh2 = (Uint16) (os.AIHandle->XVFt_Spd1);
		hstoda.BankCh3 = (Uint16) (os.AIHandle->XVFt_Spd2);
		hstoda.BankCh4 = (Uint16) (os.AIHandle->NX_SpDir);
	//---------------------------CUST_DSP_1ms[20]-------------------------------------------
	//	ptr = (Uint16*)&hstoda;
	//	for(i=0;i<10;i++)
	//	{
	//		os.CUST_DSP_1msHandle->DSPTxVar[i] = *(ptr + i) & 0x00ff;
	//		os.CUST_DSP_1msHandle->DSPTxVar[i + 1] = *(ptr + i) & 0xff00>>8;
	//	}

	os.CUST_DSP_1msHandle->DSPTxVar[0] = (hstoda.BankCh1 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[1] = (hstoda.BankCh1 & 0xff00) >> 8;

	os.CUST_DSP_1msHandle->DSPTxVar[2] = (hstoda.BankCh2 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[3] = (hstoda.BankCh2 & 0xff00) >> 8;

	os.CUST_DSP_1msHandle->DSPTxVar[4] = (hstoda.BankCh3 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[5] = (hstoda.BankCh3 & 0xff00) >> 8;

	os.CUST_DSP_1msHandle->DSPTxVar[6] = (hstoda.BankCh4 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[7] = (hstoda.BankCh4 & 0xff00) >> 8;

	os.CUST_DSP_1msHandle->DSPTxVar[8] = (hstoda.BankCh5 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[9] = (hstoda.BankCh5 & 0xff00) >> 8;

	os.CUST_DSP_1msHandle->DSPTxVar[10] = (hstoda.BankCh6 & 0x00ff);
	os.CUST_DSP_1msHandle->DSPTxVar[11] = (hstoda.BankCh6 & 0xff00) >> 8;

	//---------------------------CUST_DSP_2ms[40]-------------------------------------------
	os.CUST_DSP_2msHandle->DSPTxVar[0] = (os.STA_OUTHandle->DSPSt & 0x00ff);
	os.CUST_DSP_2msHandle->DSPTxVar[1] = (os.STA_OUTHandle->DSPSt & 0xff00)	>> 8;

	//----------------------------CUST_DSP_16ms[40]------------------------------------------------------
	os.CUST_DSP_16msHandle->DSPTxVar[0] = (os.STA_OUTHandle->DSPSt & 0x00ff);
	os.CUST_DSP_16msHandle->DSPTxVar[1] = (os.STA_OUTHandle->DSPSt & 0xff00)>> 8;

	//----------------------------CUST_DSP_64ms[40]------------------------------------------------------
	os.CUST_DSP_64msHandle->DSPTxVar[0] = ((Uint16)(hstoda.XIFt_IA_Rms*10) & 0x00ff);
	os.CUST_DSP_64msHandle->DSPTxVar[1] = ((Uint16)(hstoda.XIFt_IA_Rms*10) & 0xff00)>> 8;
	os.CUST_DSP_64msHandle->DSPTxVar[2] = ((Uint16)(hstoda.XIFt_IB_Rms*10) & 0x00ff);
	os.CUST_DSP_64msHandle->DSPTxVar[3] = ((Uint16)(hstoda.XIFt_IB_Rms*10) & 0xff00)>> 8;
	os.CUST_DSP_64msHandle->DSPTxVar[4] = ((Uint16)(hstoda.XIFt_IC_Rms*10) & 0x00ff);
	os.CUST_DSP_64msHandle->DSPTxVar[5] = ((Uint16)(hstoda.XIFt_IC_Rms*10) & 0xff00)>> 8;

}

void CntCtrl(void)
{
	if(Counter.Cnt_ChpIni!=65535)Counter.Cnt_ChpIni++;
	if(Counter.Cnt_DspIni!=65535)Counter.Cnt_DspIni++;
	if(Counter.Cnt_OvpTst!=65535)Counter.Cnt_OvpTst++;
	if(Counter.Cnt_PreFlxSyn!=65535)Counter.Cnt_PreFlxSyn++;
	if(Counter.Cnt_DisChg!=65535)Counter.Cnt_DisChg++;
}

/*********************************************************
 * 测试态下，测试PWM脉冲死区
 *
 *
 *
 *
 * ******************************************************/
void OptoTest(void)
{
	if (21 == os.PWM_OSHandle->YX_PwmMo)
		os.PWM_OSHandle->YX_PwmMo = 0;
	else
		os.PWM_OSHandle->YX_PwmMo = 21;

	os.PWM_OSHandle->YTm_PwmPdVv = (Uint16) (0.0005 * (150.0e6 / 8.0));

	//A相正占空比 0.2 B相正占空比 0.3 C相正占空比 0.4
	if (os.PWM_OSHandle->YX_PwmMo == 21)
	{
		os.PWM_OSHandle->YX_Pwm1AVv = os.PWM_OSHandle->YTm_PwmPdVv	* (1.0 - 0.2);
		os.PWM_OSHandle->YX_Pwm2AVv = os.PWM_OSHandle->YTm_PwmPdVv	* (1.0 - 0.3);
		os.PWM_OSHandle->YX_Pwm3AVv = os.PWM_OSHandle->YTm_PwmPdVv	* (1.0 - 0.4);
	}
	else
	{
		os.PWM_OSHandle->YX_Pwm1AVv = os.PWM_OSHandle->YTm_PwmPdVv * 0.2;
		os.PWM_OSHandle->YX_Pwm2AVv = os.PWM_OSHandle->YTm_PwmPdVv * 0.3;
		os.PWM_OSHandle->YX_Pwm3AVv = os.PWM_OSHandle->YTm_PwmPdVv * 0.4;
	}

	//-----------------------------------------------------------
	os.PWM_OSHandle->YTm_Pwm4PdVv = 0.00025 * (150.0e6 / 8.0);;
	os.PWM_OSHandle->YX_Pwm4AVv = os.PWM_OSHandle->YTm_Pwm4PdVv *0.5;
}

