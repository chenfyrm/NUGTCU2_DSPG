// ------------------------------------------------------------------------
//          Copyright (C) 2018 NUG, Incorporated.
//                          All Rights Reserved.
// ==========================================================================
//
// FILE:   DSP2833x_NUG_Infra.h
//
// TITLE:  Prototypes and Definitions for the user system
//
//###########################################################################
// $TI Release: C28x Floating Point Unit Library V1.30 $
// $Release Date: January 01, 2018 $
//###########################################################################

#ifndef DSP2833x_NUG_INFRA_H
#define DSP2833x_NUG_INFRA_H

#ifdef __cplusplus
extern "C" {
#endif

//===========================================================================
#define F_150M 150.0e6
#define _150M 6.66666667e-9
//===========================================================================
#define	C_SIN45	0.707214
#define	C_COS60	0.5                 //cos60
#define	C_SIN60	0.866025      	//sin60
#define	C_32COEF	0.816497      	//sqrt(2/3)
#define	C_23COEF	1.224745      	//sqrt(3/2)
#define	C_2SQRT	1.414214      	//sqrt(2)
#define	C_13SQRT	0.577350      	//sqrt(1/3)
//===========================================================================
#define	UDC_DELT	0.00025	//对应4KHz delt_t , 每隔0.00025s 采样一次
#define	PI2	6.2831853
#define	PI	3.1415927 
//===========================================================================
#define	RFFT_STAGES	8
#define	RFFT_SIZE	(1 << RFFT_STAGES)

struct  flagsci_BITS {        // bit    
   Uint16 Recv_byt:1;         // 0           
   Uint16 Overtime:1;   // 1     
   Uint16 Keyrunorstp:1;       // 2   
   Uint16 rsvd1:13;       // 15:3        
};
union flagsci_REG {
   Uint16              all;
   struct flagsci_BITS  bit;
};

struct s_adc_BITS {
		Uint16 res    :8;
		Uint16 BIP    :1;
		Uint16 LSBF   :1;
		Uint16 LEN    :2;
		Uint16 ADDR   :4;
 };
union s_adc {
   Uint16              all;
   struct s_adc_BITS  bit;
};

struct s_dac_REG {
		Uint16 Addr   :4;
		Uint16 Date   :12;
 };
union s_dac {
   Uint16              all;
   struct s_dac_REG  bit;
};

extern volatile struct flagsci_BITS flagsci;
extern volatile struct s_adc_BITS adc_low;
extern volatile struct s_dac_REG dac_low;
//extern int16 paramet[170];
extern Uint16 scitx_sta;
extern Uint16 SW_ID;
extern Uint16 Cnt_250us;
extern Uint16 Cnt_ms,Cnt_Sec,Cnt_Min,Cnt_Hour,Cnt_Day,Cnt_Year;

// standard C99 include
#include <stdint.h>
#include <stdio.h>

// system-tasks initialized flag
typedef struct
{
	uint16_t tms	:1;		// ecan-b
	uint16_t term 	:1;		// sci-b
	uint16_t stor	:1; 	// i2c-a
	uint16_t res	:13;	// reserved...
} tasks_ini_t;

extern tasks_ini_t tasks_ini;

extern void TaskECan_ini(void);
extern void TaskECan_run(void);
extern void TaskSci_ini(void);
extern void TaskSci_run(void);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of DSP2833x_USER_H

//===========================================================================
// End of file.
//===========================================================================
