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


/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   modules/types/src/types.h
//! \brief  Contains the public interface to the
//!         types definitions
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

#ifndef _TYPES_H_
#define _TYPES_H_

// system
#include "stdbool.h"  // needed for bool type, true/false
#if !defined(__TMS320C28XX_CLA__)
#include "string.h"   // needed for size_t typedef
#endif
#include "stdint.h"   // needed for C99 data types


//!
//!
//! \defgroup TYPES TYPES
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup TYPES_OVERVIEW


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines high
//!
#define   HIGH          1


//! \brief Defines low
//!
#define   LOW          0


//! \brief Defines off
//!
#define   OFF           0


//! \brief Defines ok
//!
#define   OK            0


//! \brief Defines on
//!
#define   ON            1


//! \brief Defines generic error
//!
#define   ERROR         1

//! \brief Defines pass
//!
#define   PASS          1


//! \brief Defines fails
//!
#define   FAIL          0


// **************************************************************************
// the typedefs

//! \brief Defines the portable data type for a status result
//!
typedef unsigned int    status;


//! \brief Defines the portable data type for 32 bit, signed floating-point data
//!
typedef float           float_t;


//! \brief Defines the portable data type for 64 bit, signed floating-point data
//!
typedef long double     double_t;


#ifdef __TMS320C28XX_CLA__
#ifndef NULL
/*LDRA_INSPECTED 218 S MR12 21.2 "NULL is defined in string.h but this header is not supported by CLA compiler, so defining NULL"*/
/*LDRA_INSPECTED 626 S MR12 20.4 "NULL is defined in string.h but this header is not supported by CLA compiler, so defining NULL"*/
#define NULL 0
#endif


typedef uint16_t  _Bool;


typedef unsigned int  size_t;
#endif


//! \brief Define the complex data type for at least 8 bit signed real and imaginary components
//!
typedef struct _cplx_int_least8_t
{
  int_least8_t  imag;
  int_least8_t  real;
} cplx_int_least8_t;


//! \brief Define the complex data type for at least 8 bit unsigned real and imaginary components
//!
typedef struct _cplx_uint_least8_t
{
  uint_least8_t  imag;
  uint_least8_t  real;
} cplx_uint_least8_t;


//! \brief Define the complex data type for at least 16 bit signed real and imaginary components
//!
typedef struct _cplx_least16_t
{
  int_least16_t  imag;
  int_least16_t  real;
} cplx_int_least16_t;


//! \brief Define the complex data type for at least 16 bit unsigned real and imaginary components
//!
typedef struct _cplx_uleast16_t
{
  uint_least16_t  imag;
  uint_least16_t  real;
} cplx_uint_least16_t;


//! \brief Define the complex data type for at least 32 bit signed real and imaginary components
//!
typedef struct _cplx_int_least32_t_
{
  int_least32_t  imag;
  int_least32_t  real;
} cplx_int_least32_t;


//! \brief Define the complex data type for at least 32 bit unsigned real and imaginary components
//!
typedef struct _cplx_uint_least32_t_
{
  uint_least32_t  imag;
  uint_least32_t  real;
} cplx_uint_least32_t;


//! \brief Define the complex data type for 16 bit signed real and imaginary components
//!
typedef struct _cplx_int16_t_
{
  int16_t  imag;
  int16_t  real;
} cplx_int16_t;


//! \brief Define the complex data type for 16 bit unsigned real and imaginary components
//!
typedef struct _cplx_uint16_t_
{
  uint16_t  imag;
  uint16_t  real;
} cplx_uint16_t;


//! \brief Define the complex data type for 32 bit signed real and imaginary components
//!
typedef struct _cplx_int32_t
{
  int32_t  imag;
  int32_t  real;
} cplx_int32_t;


//! \brief Define the complex data type for 32 bit unsigned real and imaginary components
//!
typedef struct _cplx_uint32_t
{
  uint32_t  imag;
  uint32_t  real;
} cplx_uint32_t;


#ifdef __cplusplus
}
#endif /* extern "C" */

//@} // ingroup
#endif  // end of _TYPES_H_ definition




/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _MATH_H_
#define _MATH_H_

//! \file   modules/math/src/float/math.h
//! \brief  Contains the public interface to the
//!         math (MATH) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes


//!
//!
//! \defgroup MATH MATH
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup MATH_OVERVIEW


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines conversion scale factor from N*m to lb*in
//!
#define MATH_Nm_TO_lbin_SF        ((float_t)(8.8507457913))

//! \brief Defines 2/3
//!
#define MATH_TWO_OVER_THREE       ((float_t)(0.66666666666666666666666666666667))

//! \brief Defines 1/3
//!
#define MATH_ONE_OVER_THREE       ((float_t)(0.33333333333333333333333333333333))

//! \brief Defines 1/(pi)
//!
#define MATH_ONE_OVER_PI          ((float_t)(0.318309886183791))

//! \brief Defines 1/sqrt(3)
//!
#define MATH_ONE_OVER_SQRT_THREE  ((float_t)(0.57735026918962576450914878050196))

//! \brief Defines 1/(4*pi)
//!
#define MATH_ONE_OVER_FOUR_PI     ((float_t)(0.07957747154594767))

//! \brief Defines 1/(2*pi)
//!
#define MATH_ONE_OVER_TWO_PI     ((float_t) (0.1591549430918954))

//! \brief Defines pi
//!
#define	MATH_PI                   ((float_t)(3.1415926535897932384626433832795))

//! \brief Defines pi per unit
//!
#define	MATH_PI_PU                ((float_t)(0.5))

//! \brief Defines 2*pi
//!
#define	MATH_TWO_PI               ((float_t)(6.283185307179586))

//! \brief Defines 2*pi per unit
//!
#define	MATH_TWO_PI_PU            ((float_t)(1.0))

//! \brief Defines 4*pi
//!
#define	MATH_FOUR_PI               ((float_t)(12.56637061435917))

//! \brief Defines 4*pi per unit
//!
#define	MATH_FOUR_PI_PU            ((float_t)(2.0))

//! \brief Defines pi/2
//!
#define	MATH_PI_OVER_TWO           ((float_t)(1.570796326794897))

//! \brief Defines pi/2 per unit
//!
#define	MATH_PI_OVER_TWO_PU        ((float_t)(0.25))

//! \brief Defines pi/4
//!
#define	MATH_PI_OVER_FOUR          ((float_t)(0.785398163397448))

//! \brief Defines pi/4 per unit
//!
#define	MATH_PI_OVER_FOUR_PU        ((float_t)(0.125))


// **************************************************************************
// the typedefs

//! \brief Defines a two element vector
//!
typedef struct _MATH_vec2_
{

  float_t  value[2];

} MATH_vec2;


//! \brief Defines a three element vector
//!
typedef struct _MATH_vec3_
{

  float_t  value[3];

} MATH_vec3;


// **************************************************************************
// the function prototypes


//! \brief     Finds the absolute value
//! \param[in] in   The input value
//! \return    The absolute value
static inline float_t MATH_abs(const float_t in)
{
  float_t out = in;


  if(in < (float_t)0.0)
    {
      out = -in;
    }

  return(out);
} // end of MATH_abs() function


//! \brief     Increments an angle value and handles wrap-around
//! \param[in] angle_rad       The angle value, rad
//! \param[in] angleDelta_rad  The angle increment value, rad
//! \return    The incremented angle value, rad
#ifdef __TMS320C28XX_CLA__
#pragma FUNC_ALWAYS_INLINE(MATH_incrAngle)
#endif
static inline float_t MATH_incrAngle(const float_t angle_rad,const float_t angleDelta_rad)
{
  float_t angleNew_rad;


  // increment the angle
  angleNew_rad = angle_rad + angleDelta_rad;


  // check for limits
  if(angleNew_rad > MATH_PI)
    {
      angleNew_rad -= MATH_TWO_PI;
    }
  else if(angleNew_rad < (-MATH_PI))
    {
      angleNew_rad += MATH_TWO_PI;
    }
  else
    {
      // doing nothing as of now
      ;
    }

  return(angleNew_rad);
} // end of MATH_incrAngle() function


//! \brief     Saturates the input value between the minimum and maximum values
//! \param[in] in   The input value
//! \param[in] max  The maximum value allowed
//! \param[in] min  The minimum value allowed
//! \return    The saturated value
static inline float_t MATH_sat(const float_t in,const float_t max,const float_t min)
{
  float_t out = in;


  if(in < min)
    {
      out = min;
    }
  else if(in > max)
    {
      out = max;
    }
  else
    {
      // do nothing as of now
      ;
    }

  return(out);
} // end of MATH_sat() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _MATH_H_ definition




/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _CLARKE_H_
#define _CLARKE_H_

//! \file   modules/clarke/src/float/clarke.h
//! \brief  Contains the public interface to the
//!         Clarke transform (CLARKE) module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
//#include "sw/modules/math/src/float/math.h"


//!
//!
//! \defgroup CLARKE CLARKE
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup CLARKE_OVERVIEW


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the CLARKE object
//!
typedef struct _CLARKE_Obj_
{
  float_t        alpha_sf;              //!< the scale factor for the alpha component
  float_t        beta_sf;               //!< the scale factor for the beta component

  uint_least8_t  numSensors;            //!< the number of sensors

} CLARKE_Obj;


//! \brief Defines the CLARKE handle
//!
typedef struct _CLARKE_Obj_ *CLARKE_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Gets the number of sensors
//! \param[in] handle  The Clarke transform handle
//! \return    The number of sensors
static inline uint_least8_t CLARKE_getNumSensors(CLARKE_Handle handle)
{
  CLARKE_Obj *obj = (CLARKE_Obj *)handle;

  return(obj->numSensors);
} // end of CLARKE_getNumSensors() function


//! \brief      Initializes the Clarke transform module
//! \param[in]  pMemory	  A pointer to the memory for the Clarke object
//! \param[in]  numBytes  The number of bytes allocated for the Clarke object, bytes
//! \return The Clarke (CLARKE) object handle
extern CLARKE_Handle CLARKE_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the Clarke transform module for three inputs
//! \param[in] handle  The Clarke transform handle
//! \param[in] pInVec        The pointer to the input vector
//! \param[in] pOutVec       The pointer to the output vector
static inline void CLARKE_run(CLARKE_Handle handle,const MATH_vec3 *pInVec,MATH_vec2 *pOutVec)
{
  CLARKE_Obj *obj = (CLARKE_Obj *)handle;

  uint_least8_t numSensors = obj->numSensors;

  float_t alpha_sf = obj->alpha_sf;
  float_t beta_sf = obj->beta_sf;


  if(numSensors == 3)
    {
      pOutVec->value[0] = ( (pInVec->value[0] * (float_t)2.0) - (pInVec->value[1] + pInVec->value[2]) ) * alpha_sf;
      pOutVec->value[1] = (pInVec->value[1] - pInVec->value[2]) * beta_sf;
    }

  else if(numSensors == 2)
    {
      pOutVec->value[0] = pInVec->value[0] * alpha_sf;
      pOutVec->value[1] = (pInVec->value[0] + (pInVec->value[1] * (float_t)2.0)) * beta_sf;
    }
  else
    {
      // assign value 0 if all conditions fail
      pOutVec->value[0] = (float_t)0;
      pOutVec->value[1] = (float_t)0;
    }

  return;
} // end of CLARKE_run() function


//! \brief     Runs the Clarke transform module for two inputs
//! \param[in] handle  The Clarke transform handle
//! \param[in] pInVec        The pointer to the input vector
//! \param[in] pOutVec       The pointer to the output vector
static inline void CLARKE_run_twoInput(CLARKE_Handle handle,const MATH_vec2 *pInVec,MATH_vec2 *pOutVec)
{
  CLARKE_Obj *obj = (CLARKE_Obj *)handle;

  float_t beta_sf = obj->beta_sf;


  pOutVec->value[0] = pInVec->value[0];

  pOutVec->value[1] = (pInVec->value[0] + (pInVec->value[1] * (float_t)2.0)) * beta_sf;

  return;
} // end of CLARKE_run_twoInput() function


//! \brief     Sets the number of sensors
//! \param[in] handle  The Clarke transform handle
//! \param[in] numSensors    The number of sensors
static inline void CLARKE_setNumSensors(CLARKE_Handle handle,const uint_least8_t numSensors)
{
  CLARKE_Obj *obj = (CLARKE_Obj *)handle;

  obj->numSensors = numSensors;

  return;
} // end of CLARKE_setNumSensors() function


//! \brief     Sets the scale factors
//! \param[in] handle  The Clarke transform handle
//! \param[in] alpha_sf      The scale factor for the alpha voltage
//! \param[in] beta_sf       The scale factor for the beta voltage
static inline void CLARKE_setScaleFactors(CLARKE_Handle handle,const float_t alpha_sf,const float_t beta_sf)
{
  CLARKE_Obj *obj = (CLARKE_Obj *)handle;


  obj->alpha_sf = alpha_sf;
  obj->beta_sf = beta_sf;

  return;
} // end of CLARKE_setScaleFactors() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _CLARKE_H_ definition






#endif /* MODULE_H_ */
