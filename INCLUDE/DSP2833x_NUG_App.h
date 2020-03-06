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


#ifndef DSP2833x_NUG_APP_H
#define DSP2833x_NUG_APP_H
//----------------------------------------------------
extern float32 AI[30];
extern Uint16 DI_OS[1];
extern Uint16 CFG_IN[4];
extern Uint16 STA_IN[8];
extern Uint16 ERR_DSP[3];
extern Uint16 ERR_EXTR[3];

extern Uint16 PWM_OS[11];
extern Uint16 DO_OS[1];
extern Uint16 CFG_OUT[3];
extern Uint16 STA_OUT[4];

extern Uint16 CUST_MCU_PAR[100];
extern Uint16 CUST_MCU_1ms[20];
extern Uint16 CUST_MCU_2ms[40];
extern Uint16 CUST_MCU_16ms[40];
extern Uint16 CUST_MCU_64ms[40];
extern Uint16 CUST_DSP_1ms[20];
extern Uint16 CUST_DSP_2ms[40];
extern Uint16 CUST_DSP_16ms[40];
extern Uint16 CUST_DSP_64ms[40];


#ifdef __cplusplus
extern "C" {
#endif

extern void INIT_EV(void);
extern void Cycle_OS(void);
extern void INT_RTOS(void);
extern void INT_PWM(void);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of DSP2833x_NUG_APP_H

//===========================================================================
// End of file.//===========================================================================

