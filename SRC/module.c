/*
 * module.c
 */

// **************************************************************************
// the includes
#include <math.h>
#include "module.h"

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

// **************************************************************************
// the functions
/**/
void LowPass(volatile float32 *Flt, float32 Src, float32 TsPerT1)
{
	*Flt = (*Flt + Src * TsPerT1) / (1.0 + TsPerT1);
}

void CplxLowPass(volatile cfloat32 *Flt, cfloat32 Src, float32 TsPerT1)
{
	Flt->re = (Flt->re + Src.re * TsPerT1) / (1.0 + TsPerT1);
	Flt->im = (Flt->im + Src.im * TsPerT1) / (1.0 + TsPerT1);
}

void RmsClc(volatile float32 *rms, float32 Src, Uint16 N,
		volatile float32 *Square, volatile Uint16 *cnt)
{
	*Square += Src * Src / N;
	*cnt += 1;
	if (*cnt >= N)
	{
		*rms = sqrt(*Square);
		*Square = 0;
		*cnt = 0;
	}
}

float32 FKG4(float32 X, float32 X1, float32 Y1, float32 X2, float32 Y2,
		float32 X3, float32 Y3, float32 X4, float32 Y4)
{
	float32 Y;
	if (X < X1)
		Y = Y1;
	else if (X < X2)
		Y = Y1 + (Y2 - Y1) * (X - X1) / (X2 - X1);
	else if (X < X3)
		Y = Y2 + (Y3 - Y2) * (X - X2) / (X3 - X2);
	else if (X < X4)
		Y = Y3 + (Y4 - Y3) * (X - X3) / (X4 - X3);
	else
		Y = Y4;

	return Y;
}

/*
 * *Y-当前值
 * X-指令
 * Dr-上升率
 * Df-下降率
 * Init-初始值
 * Set-设为初始值
 * Hold-保持
 *
 * */
void RAMP2(volatile float32 *Y, float32 X, float32 Dr, float32 Df, float32 Init,
		Uint16 Set, Uint16 Hold)
{
	float32 Yinc, Ydec;
	if (!Hold)
	{
		if (Set)
		{
			*Y = Init;
		}
		else
		{
			Yinc = *Y + fabs(Dr);
			Ydec = *Y - fabs(Df);

			if (X > Yinc)
			{
				*Y = Yinc;
			}
			else if (X < Ydec)
			{
				*Y = Ydec;
			}
			else
			{
				*Y = X;
			}
		}
	}
}

/*
 * Y = X*Ki/s;
 * Ki =1/T1 ;
 * Y(k) = Y(k-1) + X(k)*TsPerT1;
 * */
void INTEGR(volatile float32 *Y, float32 X, float32 TsPerT1, float32 Init,
		float32 Max, float32 Min, Uint16 Set, Uint16 Hold)
{
	if (!Hold)
	{
		if (Set)
		{
			*Y = Init;
		}
		else
		{
			*Y = *Y + X * TsPerT1;
			*Y = Limit(*Y, Min, Max);
		}
	}
	else
	{
//		*Y = *Y;
	}
}

float32 Min(float32 a, float32 b)
{
	if (a <= b)
		return a;
	else
		return b;
}

float32 Max(float32 a, float32 b)
{
	if (a <= b)
		return b;
	else
		return a;
}

float32 Limit(float32 x, float32 low, float32 up)
{
	return Max(low, Min(x, up));
}

void CPLX2FRAC(volatile float32 *Re, volatile float32 *Im, cfloat32 Z)
{
	*Re = Z.re;
	*Im = Z.im;
}

cfloat32 FRAC2CPLX(float32 Re, float32 Im)
{
	cfloat32 Z;
	Z.re = Re;
	Z.im = Im;
	return Z;
}

cfloat32 CPLXCONJ(cfloat32 Z)
{
	cfloat32 Z_conj;
	Z_conj.re = Z.re;
	Z_conj.im = -Z.im;
	return Z_conj;
}

cfloat32 CPLXMULT(cfloat32 Z1, cfloat32 Z2)
{
	cfloat32 Z;
	Z.re = Z1.re * Z2.re - Z1.im * Z2.im;
	Z.im = Z1.re * Z2.im + Z1.im * Z2.re;
	return Z;
}

cfloat32 CPLXMULT_SHFT(cfloat32 Z1, cfloat32 Z2, int32 m)
{
	cfloat32 Z =
	{ 0, 0 };
	return Z;
}

float32 CPLXNORM(cfloat32 Z)
{
	return Z.re * Z.re + Z.im * Z.im;
}

cfloat32 CPLXSCA(cfloat32 Z1, float32 a)
{
	cfloat32 Z;
	Z.re = Z1.re * a;
	Z.im = Z1.im * a;
	return Z;
}

cfloat32 CPLXSCA_SHFT(cfloat32 Z1, float32 a, int32 m)
{
	cfloat32 Z =
	{ 0, 0 };
	return Z;
}

cfloat32 CPLXSHFT(cfloat32 Z1, int32 m)
{
	cfloat32 Z =
	{ 0, 0 };
	return Z;
}

cfloat32 CPLXSUB(cfloat32 Z1, cfloat32 Z2)
{
	cfloat32 Z;
	Z.re = Z1.re - Z2.re;
	Z.im = Z1.im - Z2.im;
	return Z;
}

cfloat32 CPLXADD(cfloat32 Z1, cfloat32 Z2)
{
	cfloat32 Z;
	Z.re = Z1.re + Z2.re;
	Z.im = Z1.im + Z2.im;
	return Z;
}

cfloat32 _PREVCPLX(cfloat32 Z)
{

	return Z;
}

cfloat32 CPLXDIVSCA(cfloat32 Z1, float32 F, int32 m)
{
	cfloat32 Z =
	{ 0, 0 };
	return Z;
}

cfloat32 CPLXDIV(cfloat32 Z1, cfloat32 Z2)
{
	cfloat32 Z;
	Z.re = (Z1.re * Z2.re + Z1.im * Z2.im) / (Z2.re * Z2.re + Z2.im * Z2.im);
	Z.im = (-Z1.re * Z2.im + Z1.im * Z2.re) / (Z2.re * Z2.re + Z2.im * Z2.im);
	return Z;
}

void CPLX2POL(volatile float32 *r, volatile float32 *fi, cfloat32 Z)
{
	*r = sqrt(Z.re * Z.re + Z.im * Z.im);
	*fi = fmod(atan2(Z.im, Z.re), PI2);
}

cfloat32 PH3TOCPLX(float32 a, float32 b, float32 c)
{
	cfloat32 Z;
	Z.re = (a - 0.5 * (b + c)) * 2.0 / 3.0;
	Z.im = (b - c) * ONEbySQRT3;
	return Z;
}

void CPLXTO3PH(volatile float32 *a, volatile float32 *b, volatile float32 *c,
		cfloat32 Z)
{
	*a = Z.re;
	*b = -Z.re * 0.5 + Z.im * SQRT3byTWO;
	*c = -Z.re * 0.5 - Z.im * SQRT3byTWO;
}

cfloat32 POL2CPLX(float32 r, float32 fi)
{
	cfloat32 Z;
	Z.re = r * cos(fi);
	Z.im = r * sin(fi);
	return Z;
}

/**/
void SOGIOSGFLL(TYPE_SOGIOSGMA *data)
{

	//	data->Ts = 1.0/2700.0;
	//	data->w0 = 100*3.1415926;
	//	data->K = sqrt(2);
	//	data->Ki = 10000;

	/**/
	data->a = data->Ts * data->w / 2.0 + 2.0 / data->Ts / data->w;
	data->b = data->Ts * data->w / 2.0 - 2.0 / data->Ts / data->w;

	data->alpha = data->K / (data->a + data->K)\

			* (data->phase - data->oldPhase2)\

			- 2.0 * data->b / (data->a + data->K) * data->oldAlpha1\

			- (data->a - data->K) / (data->a + data->K) * data->oldAlpha2;
	data->beta = data->K / (data->a + data->K) * (data->a + data->b) / 2.0\

			* (data->phase + 2.0 * data->oldPhase1 + data->oldPhase2)\

			- 2.0 * data->b / (data->a + data->K) * data->oldBeta1\

			- (data->a - data->K) / (data->a + data->K) * data->oldBeta2;

	data->peak = sqrt(data->alpha * data->alpha + data->beta * data->beta);
	if (data->peak <= 0.001)
		data->peak = 0.001;
	/**/
	data->ErrF = (data->phase - data->alpha) * data->beta
			/ (data->peak * data->peak);
	data->ComW += data->ErrF * (-1.0) * data->Ki * data->Ts;
	if (data->ComW > 30.0)
		data->ComW = 30.0;
	if (data->ComW < -30.0)
		data->ComW = -30.0;

	/*update*/
	data->w = data->w0 + data->ComW;

	data->oldPhase2 = data->oldPhase1;
	data->oldPhase1 = data->phase;
	data->oldAlpha2 = data->oldAlpha1;
	data->oldAlpha1 = data->alpha;
	data->oldBeta2 = data->oldBeta1;
	data->oldBeta1 = data->beta;
}

/**/
void PI_CONTROLLER(TYPE_PI_CONTROLLER *data)
{
	/* proportional term */
	data->up = data->Kp * (data->Ref - data->Fbk);

	/* integral term */
	data->ui =
			(data->Out == data->v1) ?
					(data->Ki * (data->Ref - data->Fbk) + data->i1) : data->i1;
	data->i1 = data->ui;

	/* control output */
	data->v1 = data->up + data->ui;
	data->Out = (data->v1 > data->Umax) ? data->Umax : data->v1;
	data->Out = (data->Out < data->Umin) ? data->Umin : data->Out;
}

/**/
void IIRFilter_2nd(TYPE_IIRFILTER_2ND *data)
{
	data->Out = data->b0 * data->In + data->b1 * data->oldIn1
			+ data->b2 * data->oldIn2 - data->a1 * data->oldOut1
			- data->a2 * data->oldOut2;
	/***********************************/
	data->oldIn2 = data->oldIn1;
	data->oldIn1 = data->In;
	data->oldOut2 = data->oldOut1;
	data->oldOut1 = data->Out;
}

/***************************
 *
 *
 *
 *
 ****************************/
void AdaptIIRNotchFilter(TYPE_IIRFILTER_2ND *data, float32 W0, float32 Ts)
{
	data->b0 = 1.0;
	data->b1 = -2.0 * cos(W0 * Ts);
	data->b2 = 1.0;
	data->a1 = (1 - W0 * Ts / 4) * data->b1;
	data->a2 = pow((1 - W0 * Ts / 4), 2.0);

	if (data->b1 == -2.0)
	{
		data->In *= 1.0;
	}
	else
	{
		data->In *= (1.0 + data->a1 + data->a2) / (2.0 + data->b1);
	}

	data->Out = data->b0 * data->In + data->b1 * data->oldIn1
			+ data->b2 * data->oldIn2 - data->a1 * data->oldOut1
			- data->a2 * data->oldOut2;

	/***********************************/
	data->oldIn2 = data->oldIn1;
	data->oldIn1 = data->In;
	data->oldOut2 = data->oldOut1;
	data->oldOut1 = data->Out;
}

/*
 * 	[0 1/sqrt(3) 0.579 0.6038 0.6057]
 * 	[0 1/sqrt(3) 0.58  0.6389 0.6667]
 * */
float32 OvMd(float32 M1)
{
	float32 M;

	if (M1 < 0)
		M = 0;
	else if (M1 < 1.0 / sqrt(3))
		M = M1;
	else if (M1 < 0.579)
		M = 1.0 / sqrt(3)
				+ (0.58 - 1.0 / sqrt(3)) / (0.579 - 1.0 / sqrt(3))
						* (M1 - 1.0 / sqrt(3));
	else if (M1 < 0.6038)
		M = 0.58 + (0.6389 - 0.58) / (0.6038 - 0.579) * (M1 - 0.579);
	else if (M1 < 0.6057)
		M = 0.6389 + (0.6667 - 0.6389) / (0.6057 - 0.6038) * (M1 - 0.6038);
	else
		M = 0.6667;

	return M;
}

/*******************************************************************************
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ******************************************************************************/
void SVPWM(volatile float32 *DutyA, volatile float32 *DutyB,
		volatile float32 *DutyC, cfloat32 _3PhAB)
{
	float32 a, b, c, min, max, NrmFa, Cml;

	CPLXTO3PH(&a, &b, &c, _3PhAB);

	min = Min(a, Min(b, c));
	max = Max(a, Max(b, c));

	NrmFa = Max(1.0, max - min);
	Cml = (max + min) * (-0.5);

	*DutyA = (a + Cml) / NrmFa + 0.5;
	*DutyB = (b + Cml) / NrmFa + 0.5;
	*DutyC = (c + Cml) / NrmFa + 0.5;
}
