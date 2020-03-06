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
#define  ChpIni			0x00    	//оƬ��ʼ��״̬
#define  DSPIni			0x01		//DSP���Ʋ�����ʼ��״̬
#define  DSPIniFn		0x02		//ϵͳ��ʼ�����
#define  OVPTst			0x03		//OVP����״̬
#define  OVPTstFn		0x04		//OVP�������
#define  PreFlx			0x05		//Ԥ����״̬
#define  PreFlxFn		0x06		//Ԥ�������
#define  TqOut			0x07		//ת�����״̬
#define  TqOutFn		0x08		//ת���������״̬
#define  DisChg			0x09		//�ŵ�״̬
#define  DisChgFn		0x0A		//�ŵ����״̬
#define  FltStt			0x0B		//����״̬
#define  NX_DSPSt       (os.STA_OUTHandle->DSPSt)

//======================== MCU request ==========================
#define  DspRst			(1<<0) 			//1    bit0:  reset DSP   ����DSP�������DSP reset
#define  OvpTsEn		(1<<1)			//2    bit1:  Ovp test enabled  ���󼤻�OVP OVP test disabled
#define  Br 			(1<<2) 		    //4    bit2:  brake �ƶ�״̬   brake
#define  DspClr			(1<<3) 			//8    bit3:  clear DSP  DSP���ϸ�λ DSP clear
#define  CtOp			(1<<4)			//16   bit4:  contact open  ���󼤻�ն��   contactor open
#define  CtOpHL			(1<<5)			//32   bit5:  contact open HIGH LEVEL ���󼤻�ߵȼ�ն����ն������ʱ��ֱ������ն�����
#define  PrEtEn			(1<<6)			//64   bit6:  Pre-excitation enable ���󼤻�Ԥ����
#define  TqOutEn		(1<<7)			//128  bit7:  torque output enable ���󼤻�ת�����
#define  mTqOutFn		(1<<8)			//256  bit8:  torque output finish ���󼤻�ת���������
#define  CvSpFg			(1<<9)			//512  bit9:  covertor stop flag ������ֹͣ��־
#define  CvSpFn			(1<<10)			//1024 bit10: covertor stop completed ������ֹͣ��ɱ�־
#define  CvStAg			(1<<11)			//2048 bit11: covertor start again ������������������
#define  CvReSt			(1<<12)			//4096 bit12: covertor Re-start  ��������������
#define  MCUHwIniFn		(1<<13)			//8192 bit13: MCU hardware initial finish MCUӲ����ʼ�����
#define  NX_MCUCmd		(os.STA_INHandle->MCUCmd.all)

#define  M_CvSa()		(os.DO_OSHandle->DO1.bit.L_InvEn = 1)
#define	 M_CvSo()       (os.DO_OSHandle->DO1.bit.L_InvEn = 0)

// **************************************************************************
// the typedefs
typedef struct _AI_Obj_ {
	float32 XTpFt_MdAm_Flt; //����  AI05
	float32 XTpFt_HCOT_Flt; //ɢ�����¶� AI06
	float32 XTpFt_Mot1_Flt; //1�ŵ�������¶� AI07
	float32 XTpFt_Mot2_Flt; //2�ŵ�������¶� AI08
	float32 XTpFt_Mot3_Flt; //3�ŵ�������¶� AI21 IOE

	float32 XTpFt_Mot4_Flt; //4�ŵ�������¶� AI22 IOE
	float32 XTpFt_Brd_Flt; //�忨�¶�
	float32 XVFt_Spd1; //1�ŵ��ת��  SP01
	float32 XVFt_Spd2; //2�ŵ��ת�� SP02
	float32 XVFt_Spd3; //3�ŵ��ת�� SP03

	float32 XVFt_Spd4; //4�ŵ��ת�� SP04
	float32 XIFt_IA; //A����� AI01
	float32 XIFt_IB; //B����� AI02
	float32 XIFt_IacHwPrPos_Lim; //��������Ӳ������ֵ
	float32 XIFt_IacHwPrNeg_Lim; //��������Ӳ������ֵ

	float32 XIFt_IDC; //ֱ������ AI04
	float32 XUFt_UDC1; //ֱ����ѹ AI03
	float32 XUFt_UDCHwPr_Lim; //ֱ����ѹӲ����ֵ
	float32 XIFt_EFLekPr_Lim; //�ӵ�©����Ӳ����ֵ
	float32 XUFt_UPh; //������ѹ������ AI23

	float32 XIFt_IPh1; //�������������� AI24
	float32 XIFt_IBt; //���ص��� AI25
	float32 XUFt_UDC2; //ֱ��ĸ�ߵ�ѹ������ AI26
	float32 XIFt_IBtCg; //���س����� AI28
	float32 XIFt_IC; //C����� AI29

	float32 XUFt_UDC3; //ֱ��ĸ�ߵ�ѹ������ AI30
	float32 XIFt_IPh2; //�������������� AI31
	float32 XUFt_Bt; //���ص�ѹ AI27
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
	Uint16 L_DspRst :1;				//1    bit0:  reset DSP   ����DSP�������DSP reset
	Uint16 L_OvpTsEn :1;//2    bit1:  Ovp test enabled  ���󼤻�OVP OVP test disabled
	Uint16 L_Br :1; 			 		    //4    bit2:  brake �ƶ�״̬   brake
	Uint16 L_DspClr :1;			 	//8    bit3:  clear DSP  DSP���ϸ�λ DSP clear
	Uint16 L_CtOp :1;		//16   bit4:  contact open  ���󼤻�ն��   contactor open

	Uint16 L_CtOpHL :1;	//32   bit5:  contact open HIGH LEVEL ���󼤻�ߵȼ�ն����ն������ʱ��ֱ������ն�����
	Uint16 L_PrEtEn :1;				//64   bit6:  Pre-excitation enable ���󼤻�Ԥ����
	Uint16 L_TqOutEn :1;			//128  bit7:  torque output enable ���󼤻�ת�����
	Uint16 L_TqOutFn :1;		//256  bit8:  torque output finish ���󼤻�ת���������
	Uint16 L_CvSpFg :1;					//512  bit9:  covertor stop flag ������ֹͣ��־

	Uint16 L_CvSpFn :1;			//1024 bit10: covertor stop completed ������ֹͣ��ɱ�־
	Uint16 L_CvStAg :1;				//2048 bit11: covertor start again ������������������
	Uint16 L_CvReSt :1;					//4096 bit12: covertor Re-start  ��������������
	Uint16 L_MCUHwIniFn :1;	//8192 bit13: MCU hardware initial finish MCUӲ����ʼ�����
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
	Uint16 rsvd3;
	Uint16 rsvd4;
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
	float32 NX_SpdOrTq;//�������ģʽ
	float32 WX_CpDrt;//ն��ռ�ձ�

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
	Uint16 TA0 :1;		// Udc��ѹ/Ƿѹ
	Uint16 TA1 :1;		// Idc����
	Uint16 TA2 :1;		// Idc����ʧЧ
	Uint16 TA3 :1;		// Udc����ʧЧ
	Uint16 TA4 :1;		// DSP����
	Uint16 TA5 :1;		// Ia����
	Uint16 TA6 :1;		// Ib����
	Uint16 TA7 :1;		// Ic����
	Uint16 TA8 :1;		// Iac��������������ƽ��
	Uint16 TA9 :1;		// Iaʧ��
	Uint16 TA10 :1;		// Ibʧ��
	Uint16 TA11 :1;		// Icʧ��
	Uint16 TA12 :1;		// Ia�۲����
	Uint16 TA13 :1;		// Ib�۲����
	Uint16 TA14 :1;		// Ic�۲����
	Uint16 TA15 :1;		// Iac����������������ʧЧ
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
	Uint16 L_Stt :1;		//״̬������
	Uint16 L_Init :1;		//��ʼ������
	Uint16 L_OvpTst :1;	//ovp���Թ���
	Uint16 L_PreFlxSyn :1;	//Ԥ����ͬ��ʧ��
	Uint16 L_DisChg :1;	//�ŵ����
	Uint16 rsvd :11;
};
union DSPAPPFLT1_REG {
	Uint16 all;
	struct DSPAPPFLT1_BITS bit;
};

struct DSPAPPFLT2_BITS {
	Uint16 L_DCOV :1;		//ֱ����ѹ
	Uint16 L_DCLV :1;		//ֱ��Ƿѹ
	Uint16 L_DCOI :1;		//ֱ������
	Uint16 L_IAOI :1;		//A�����
	Uint16 L_IBOI :1;		//B�����
	Uint16 L_ICOI :1;		//C�����
	Uint16 L_IUB :1;		//�������಻ƽ��
	Uint16 L_PH :1;		//ȱ��
	Uint16 L_SpdO :1;		//����
	Uint16 rsvd :7;
};
union DSPAPPFLT2_REG {
	Uint16 all;
	struct DSPAPPFLT2_BITS bit;
};

struct DSPAPPFLT3_BITS {
	Uint16 L_UABOV :1;		//UV�ߵ�ѹ����ѹ
	Uint16 L_UBCOV :1;		//VW�ߵ�ѹ����ѹ
	Uint16 L_UCAOV :1;		//WU�ߵ�ѹ����ѹ
	Uint16 L_MdOT :1;		//ģ�����
	Uint16 L_MtOT :1;		//�������
	Uint16 L_Frq :1;		//Ƶ�ʴ���
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
	os.ERR_DSPHandle = (ERR_DSP_Handle) &ERR_DSP_OS[0];

	os.ERR_EXTRHandle = (ERR_EXTR_Handle) &ERR_EXTR[0];
	os.PWM_OSHandle = (PWM_OS_Handle) &PWM_OS[0];
	os.DO_OSHandle = (DO_OS_Handle) &DO_OS[0];
	os.CFG_OUTHandle = (CFG_OUT_Handle) &CFG_OUT[0];
	os.STA_OUTHandle = (STA_OUT_Handle) &STA_OUT[0];

}

void Cycle_OS(void) {
	state_machine();

	if (NX_DSPSt == DSPIni) {

		Nt_WarnFn = 1;			//��ʼ�����
		SX_NtFlt = 0;			//��ʼ������
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
	} else if (NX_DSPSt == DSPIniFn)	// DSPϵͳ��ʼ�����
	{
		if ((NX_MCUCmd & CtOp)||(NX_MCUCmd&CtOpHL))		//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
{		NX_DSPSt = DisChg;	//�ŵ�״̬
	}
	else if(NX_MCUCmd&CvReSt)	//�յ�MCUӲ����ʼ��
	{
		NX_DSPSt=ChpIni;	//DSP����оƬ��ʼ��״̬
	}
	else if(NX_MCUCmd&OvpTsEn)		//�յ�OVP��������
	{
		NX_DSPSt = OVPTst;	//OVP����״̬
	}
}
else if(NX_DSPSt==OVPTst)	//OVP����״̬
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//�ŵ�״̬
	}
	else if(SX_OvpTsOk==1)			//����ͨ��
	{
		NX_DSPSt=OVPTstFn;		//OVP���
	}
	else if(SX_OvpTsOk==0)		//����δͨ��
	{
		NX_DSPSt = FltStt;
		S_FltSt0.bit.TA2 = 1;	//
	}
}
else if(NX_DSPSt==OVPTstFn)	//OVP�������״̬
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//�ŵ�״̬
	}
	else if(NX_MCUCmd&PrEtEn)	//�յ�MCUӲ����ʼ����������              //ֱ����Դ������
	{
		NX_DSPSt=PreFlx;		//Ԥ����״̬
	}
}
else if(NX_DSPSt==PreFlx) //Ԥ����״̬
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//OVP����״̬
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
		NX_DSPSt = PreFlxFn;	//Ԥ�������
	}
	else if(S_PreFlxFlg==0.0)
	{
		NX_DSPSt = FltStt;
		S_FltSt0.bit.TA3 = 1;
	}
}
else if(NX_DSPSt==PreFlxFn)					//ϵͳ����
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//OVP����״̬
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
else if(NX_DSPSt==TqOut)					//ת�����
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//OVP����״̬
	}
	else if(NX_MCUCmd&mTqOutFn)
	{
		if(fabs(WM_TqCmd)<15)
		{
			NX_DSPSt=TqOutFn;
		}
	}
}
else if(NX_DSPSt==TqOutFn)		//ת���������
{
	if((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))	//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChg;	//OVP����״̬
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
	if((S_FltSt0.bit.TA4==1)&&(NX_MCUCmd&CtOpHL))//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
	{
		NX_DSPSt = DisChgFn;	//
		S_FltSt0.bit.TA4=0;
	}
	else if(((NX_MCUCmd&CtOp)||(NX_MCUCmd&CtOpHL))&&(S_FltSt0.bit.TA4==0))//�յ�MCU�ŵ����󣬽Ӵ����Ͽ���
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
	if(NX_MCUCmd&MCUHwIniFn)					//MCUӲ����ʼ�����
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
