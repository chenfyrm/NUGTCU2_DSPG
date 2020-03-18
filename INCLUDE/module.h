/*
 * module.h
 *
 *  Created on: 2020-3-13
 *      Author: 700363
 */

#ifndef MODULE_H_
#define MODULE_H_

// **************************************************************************
// the includes

#define	TWObyTHREE	  	0.66666666666667       /* 2/3 */
#define ONEbySQRT3		0.57735026918963    /* 1/sqrt(3) */
#define SQRT3byTWO   	0.86602540378444    /* sqrt(3)/2 */
#define SQRT3   		1.73205080756888    /* sqrt(3)/2 */
#define	SQRT2			1.41421356237310
#define SQRT2bySQRT3    0.816397228637413   /*sqrt(2/3)*/
#define	PI  		  	3.14159265358979
#define	PI2  		  	6.28318530717959
#define	PIby2SQRT2			1.110720735
#define FALSE			0
#define	TRUE			1
#define	NULL			0

// **************************************************************************
// the typedefs

#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned int Uint16;
typedef unsigned long Uint32;
typedef unsigned long long Uint64;
typedef float float32;
#endif

//! \brief Defines the portable data type for 32 bit, signed floating-point data
//!
typedef float           float_t;


//! \brief Defines the portable data type for 64 bit, signed floating-point data
//!
typedef long double     double_t;

typedef struct
{
	float32 re;
	float32 im;
} cfloat32;


// **************************************************************************
// the function prototypes

#ifdef __cplusplus
extern "C" {
#endif

extern void Delay(volatile float32 *Dy, float32 Src);
extern void LowPass(volatile float32 *Flt, float32 Src, float32 TsPerT1);
extern void RmsClc(volatile float32 *rms, float32 Src, Uint16 N,
		volatile float32 *Square, volatile Uint16 *cnt);
extern void RAMP2(volatile float32 *Y, float32 X, float32 Dr, float32 Df,
		float32 Init, Uint16 Set, Uint16 Hold);
extern void RAMP(volatile float32 *Y, float32 X, float32 Tr, float32 Tf,
		float32 Init, Uint16 Set, Uint16 Hold, float32 Max);
extern float32 Cycle(void);
extern void INTEGR(volatile float32 *Y, float32 X, float32 T, float32 Init,
		float32 Max, float32 Min, Uint16 Set, Uint16 Hold);
extern float32 FKG4(float32 X, float32 X1, float32 Y1, float32 X2, float32 Y2,
		float32 X3, float32 Y3, float32 X4, float32 Y4);
extern float32 PIREG();

extern float32 Min(float32 a, float32 b);
extern float32 Max(float32 a, float32 b);

extern float32 Limit(float32 x, float32 low, float32 up);

/*compelx math*/
extern void CPLX2FRAC(volatile float32 *Re, volatile float32 *Im, cfloat32 Z);
extern cfloat32 FRAC2CPLX(float32 Re, float32 Im);
extern cfloat32 CPLXCONJ(cfloat32 Z);
extern cfloat32 CPLXMULT(cfloat32 Z1, cfloat32 Z2);
extern cfloat32 CPLXMULT_SHFT(cfloat32 Z1, cfloat32 Z2, int32 m);
extern float32 CPLXNORM(cfloat32 Z);
extern cfloat32 CPLXSCA(cfloat32 Z1, float32 a);
extern cfloat32 CPLXSCA_SHFT(cfloat32 Z1, float32 a, int32 m);
extern cfloat32 CPLXSHFT(cfloat32 Z, int32 m);
extern cfloat32 CPLXSUB(cfloat32 Z1, cfloat32 Z2);
extern cfloat32 CPLXADD(cfloat32 Z1, cfloat32 Z2);
extern cfloat32 _PREVCPLX(cfloat32 Z);
extern cfloat32 CPLXDIVSCA(cfloat32 Z1, float32 F, int32 m);
extern cfloat32 CPLXDIV(cfloat32 Z1, cfloat32 Z2);
extern void CPLX2POL(volatile float32 *r, volatile float32 *fi,
		volatile cfloat32 Z);
extern cfloat32 PH3TOCPLX(float32 a, float32 b, float32 c);
extern void CPLXTO3PH(volatile float32 *a, volatile float32 *b,
		volatile float32 *c, cfloat32 Z);
extern cfloat32 POL2CPLX(float32 r, float32 fi);
extern void CplxLowPass(volatile cfloat32 *Flt, cfloat32 Src, float32 TsPerT1);

/**/
extern float32 OvMd(float32 M1);
extern void SVPWM(volatile float32 *DutyA, volatile float32 *DutyB,
		volatile float32 *DutyC, cfloat32 _3PhAB);

#ifdef __cplusplus
}
#endif // extern "C"

//=======================SOGIOSGFLL===================================
typedef struct
{
	float32 phase;	//input
	float32 alpha;	//output
	float32 beta;
	float32 Ts;	//param
	float32 w0;
	float32 K;
	float32 Ki;
	float32 oldPhase1;	//state
	float32 oldPhase2;
	float32 oldAlpha1;
	float32 oldAlpha2;
	float32 oldBeta1;
	float32 oldBeta2;
	float32 a;	//local
	float32 b;
	float32 w;
	float32 peak;
	float32 ErrF;
	float32 ComW;
} TYPE_SOGIOSGMA;

#define SOGIOSGMA_DEFAULTS {\
	0.0,\
	0.0,\
	0.0,\
	1.0/1350.0/2.0,\
	100*3.1415926,\
	1.4142135,\
	10000,\
	0.0,\
	0.0,\
	0.0,\
	0.0,\
	0.0,\
	0.0,\
	0.0,\
	0.0,\
	100*3.1415926,\
	0.0,\
	0.0,\
	0.0,\
	}

#ifdef __cplusplus
extern "C"
{
#endif /* extern "C" */

extern void SOGIOSGFLL(TYPE_SOGIOSGMA *interface);

#ifdef __cplusplus
}
#endif

//========================================PI_CONTROLLER==============================
typedef struct
{
	float32 Ref;   			// Input: reference set-point
	float32 Fbk;   			// Input: feedback
	float32 Out;   			// Output: controller output
	float32 Kp;				// Parameter: proportional loop gain
	float32 Ki;			    // Parameter: integral gain
	float32 Umax;			// Parameter: upper saturation limit
	float32 Umin;			// Parameter: lower saturation limit
	float32 up;				// Data: proportional term
	float32 ui;				// Data: integral term
	float32 v1;				// Data: pre-saturated controller output
	float32 i1;				// Data: integrator storage: ui(k-1)
	float32 w1;				// Data: saturation record: [u(k-1) - v(k-1)]
} TYPE_PI_CONTROLLER;

/*-----------------------------------------------------------------------------
 Default initalisation values for the PI_GRANDO objects
 -----------------------------------------------------------------------------*/

#define PI_CONTROLLER_DEFAULTS {		\
	0, 		\
	0, 		\
	0, 		\
	1.0,	\
	0.0,	\
	1.0,	\
	-1.0, 	\
	0.0,	\
	0.0, 	\
	0.0,	\
	0.0,	\
	1.0 	\
	}

#ifdef __cplusplus
extern "C"
{
#endif /* extern "C" */

extern void PI_CONTROLLER(TYPE_PI_CONTROLLER *data);

#ifdef __cplusplus
}
#endif

typedef struct
{
	float32 In;				//input
	float32 Out;				//output
	float32 a1;				//param
	float32 a2;
	float32 b0;
	float32 b1;
	float32 b2;
	float32 oldIn1;				//state
	float32 oldIn2;
	float32 oldOut1;
	float32 oldOut2;
} TYPE_IIRFILTER_2ND;

#ifdef __cplusplus
extern "C"
{
#endif /* extern "C" */

extern void IIRFilter_2nd(TYPE_IIRFILTER_2ND *data);
extern void AdaptIIRNotchFilter(TYPE_IIRFILTER_2ND *data, float32 W0,
		float32 Ts);

#ifdef __cplusplus
}
#endif


#endif /* MODULE_H_ */
