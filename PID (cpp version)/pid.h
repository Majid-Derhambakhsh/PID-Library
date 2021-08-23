 /*
------------------------------------------------------------------------------
~ File   : pid.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 06/12/2021 05:00:00 PM
~ Brief  :
~ Support:
		   E-Mail : Majid.do16@gmail.com (subject : Embedded Library Support)

		   Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:      CPP version

~ Attention  :

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __PID_H_
#define __PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */

#ifdef __CODEVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

	#include "main.h"

	/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	#else
	#endif /* STM32F1 */

	/* ------------------------------------------------------- */

	#if defined ( __ICCARM__ ) /* ICCARM Compiler */

	#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */

	#elif defined   (  __GNUC__  ) /* GNU Compiler */

	#pragma diag_suppress 177     /* Disable 'unused function' warning */

	#endif /* __ICCARM__ */

/* ------------------------------------------------------------------ */

#else                     /* Compiler not found */

#error Chip or Library not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/* ------------------------------------------------------------------ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ------------------------ Library ------------------------ */
#define _PID_LIBRARY_VERSION    1.0.0

/* ------------------------ Public ------------------------- */
#define _PID_8BIT_PWM_MAX       UINT8_MAX
#define _PID_SAMPLE_TIME_MS_DEF 100

#ifndef _FALSE

	#define _FALSE 0

#endif

#ifndef _TRUE

	#define _TRUE 1

#endif

/* ---------------------- By compiler ---------------------- */
#ifndef GetTime

	/* ---------------------- By compiler ---------------------- */

	#ifdef __CODEVISIONAVR__  /* Check compiler */

		#define GetTime()   0

	/* ------------------------------------------------------------------ */

	#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

		#define GetTime()   0

	/* ------------------------------------------------------------------ */

	#elif defined(USE_HAL_DRIVER)  /* Check driver */

		#define GetTime()   HAL_GetTick()

	/* ------------------------------------------------------------------ */

	#else
	#endif /* __CODEVISIONAVR__ */
	/* ------------------------------------------------------------------ */

#endif

/* --------------------------------------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PID Mode */
typedef enum
{
	
	_PID_MODE_MANUAL    = 0,
	_PID_MODE_AUTOMATIC = 1
	
}PIDMode_TypeDef;

/* PID P On x */
typedef enum
{
	
	_PID_P_ON_M = 0, /* Proportional on Measurement */
	_PID_P_ON_E = 1
	
}PIDPON_TypeDef;

/* PID Control direction */
typedef enum
{
	
	_PID_CD_DIRECT  = 0,
	_PID_CD_REVERSE = 1
	
}PIDCD_TypeDef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
class PID
{
	
	private:
	
	PIDPON_TypeDef  _pOnE;
	PIDMode_TypeDef _inAuto;

	PIDPON_TypeDef  _pOn;
	PIDCD_TypeDef   _controllerDirection;

	uint32_t        _lastTime;
	uint32_t        _sampleTime;

	double          _dispKp;
	double          _dispKi;
	double          _dispKd;

	double          _kp;
	double          _ki;
	double          _kd;

	double          *_myInput;
	double          *_myOutput;
	double          *_mySetpoint;

	double          _outputSum;
	double          _lastInput;

	double          _outMin;
	double          _outMax;
	
	public:
	
	/* :::::::::: Constructor :::::::::: */
	PID();
	PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
	PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection);

	/* :::::::::::::: Init ::::::::::::: */
	void Init(void);
	void Init(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
	void Init(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection);

	/* ::::::::::: Computing ::::::::::: */
	uint8_t Compute(void);

	/* ::::::::::: PID Mode :::::::::::: */
	void            SetMode(PIDMode_TypeDef Mode);
	PIDMode_TypeDef GetMode(void);

	/* :::::::::: PID Limits ::::::::::: */
	void SetOutputLimits(double Min, double Max);

	/* :::::::::: PID Tunings :::::::::: */
	void SetTunings(double Kp, double Ki, double Kd);
	void SetTunings(double Kp, double Ki, double Kd, PIDPON_TypeDef POn);

	/* ::::::::: PID Direction ::::::::: */
	void          SetControllerDirection(PIDCD_TypeDef Direction);
	PIDCD_TypeDef GetDirection(void);

	/* ::::::::: PID Sampling :::::::::: */
	void SetSampleTime(int32_t NewSampleTime);

	/* ::::::: Get Tunings Param ::::::: */
	double GetKp(void);
	double GetKi(void);
	double GetKd(void);
	
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* __PID_H_ */
