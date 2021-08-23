/*
------------------------------------------------------------------------------
~ File   : pid.cpp
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

#include "pid.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~ Constructor ~~~~~~~~~~~~~~~~ */
PID::PID() { }

PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	_myOutput   = Output;
	_myInput    = Input;
	_mySetpoint = Setpoint;
	_inAuto     = (PIDMode_TypeDef)_FALSE;
	
	PID::SetOutputLimits(0, _PID_8BIT_PWM_MAX);
	
	_sampleTime = _PID_SAMPLE_TIME_MS_DEF; /* default Controller Sample Time is 0.1 seconds */
	
	PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd, POn);
	
	_lastTime = GetTime() - _sampleTime;
	
}

PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection) : PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, _PID_P_ON_E, ControllerDirection){ }

/* ~~~~~~~~~~~~~~~~~ Initialize ~~~~~~~~~~~~~~~~ */
void PID::Init(void)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	_outputSum = *_myOutput;
	_lastInput = *_myInput;
	
	if (_outputSum > _outMax)
	{
		_outputSum = _outMax;
	}
	else if (_outputSum < _outMin)
	{
		_outputSum = _outMin;
	}
	else { }
	
}

void PID::Init(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection)
{
	/* ~~~~~~~~~~ Set parameter ~~~~~~~~~~ */
	_myOutput   = Output;
	_myInput    = Input;
	_mySetpoint = Setpoint;
	_inAuto     = (PIDMode_TypeDef)_FALSE;
	
	PID::SetOutputLimits(0, _PID_8BIT_PWM_MAX);
	
	_sampleTime = _PID_SAMPLE_TIME_MS_DEF; /* default Controller Sample Time is 0.1 seconds */
	
	PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd, POn);
	
	_lastTime = GetTime() - _sampleTime;
	
}

void PID::Init(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection)
{
	PID::Init(Input, Output, Setpoint, Kp, Ki, Kd, _PID_P_ON_E, ControllerDirection);
}

/* ~~~~~~~~~~~~~~~~~ Computing ~~~~~~~~~~~~~~~~~ */
uint8_t PID::Compute(void)
{
	
	uint32_t now;
	uint32_t timeChange;
	
	double input;
	double error;
	double dInput;
	double output;
	
	/* ~~~~~~~~~~ Check PID mode ~~~~~~~~~~ */
	if (!_inAuto)
	{
		return _FALSE;
	}
	
	/* ~~~~~~~~~~ Calculate time ~~~~~~~~~~ */
	now        = GetTime();
	timeChange = (now - _lastTime);
	
	if (timeChange >= _sampleTime)
	{
		/* ..... Compute all the working error variables ..... */
		input   = *_myInput;
		error   = *_mySetpoint - input;
		dInput  = (input - _lastInput);
		
		_outputSum += (_ki * error);
		
		/* ..... Add Proportional on Measurement, if P_ON_M is specified ..... */
		if (!_pOnE)
		{
			_outputSum -= _kp * dInput;
		}
		
		if (_outputSum > _outMax)
		{
			_outputSum = _outMax;
		}
		else if (_outputSum < _outMin)
		{
			_outputSum = _outMin;
		}
		else { }
		
		/* ..... Add Proportional on Error, if P_ON_E is specified ..... */
		if (_pOnE)
		{
			output = _kp * error;
		}
		else
		{
			output = 0;
		}
		
		/* ..... Compute Rest of PID Output ..... */
		output += _outputSum - _kd * dInput;
		
		if (output > _outMax)
		{
			output = _outMax;
		}
		else if (output < _outMin)
		{
			output = _outMin;
		}
		else { }
		
		*_myOutput = output;
		
		/* ..... Remember some variables for next time ..... */
		_lastInput = input;
		_lastTime  = now;
		
		return _TRUE;
		
	}
	else
	{
		return _FALSE;
	}
	
}

/* ~~~~~~~~~~~~~~~~~ PID Mode ~~~~~~~~~~~~~~~~~~ */
void            PID::SetMode(PIDMode_TypeDef Mode)
{
	
	uint8_t newAuto = (Mode == _PID_MODE_AUTOMATIC);
	
	/* ~~~~~~~~~~ Initialize the PID ~~~~~~~~~~ */
	if (newAuto && !_inAuto)
	{
		Init();
	}
	
	_inAuto = (PIDMode_TypeDef)newAuto;
	
}
PIDMode_TypeDef PID::GetMode(void)
{
	return _inAuto ? _PID_MODE_AUTOMATIC : _PID_MODE_MANUAL;
}

/* ~~~~~~~~~~~~~~~~ PID Limits ~~~~~~~~~~~~~~~~~ */
void PID::SetOutputLimits(double Min, double Max)
{
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Min >= Max)
	{
		return;
	}
	
	_outMin = Min;
	_outMax = Max;
	
	/* ~~~~~~~~~~ Check PID Mode ~~~~~~~~~~ */
	if (_inAuto)
	{
		
		/* ..... Check out value ..... */
		if (*_myOutput > _outMax)
		{
			*_myOutput = _outMax;
		}
		else if (*_myOutput < _outMin)
		{
			*_myOutput = _outMin;
		}
		else { }
		
		/* ..... Check out value ..... */
		if (_outputSum > _outMax)
		{
			_outputSum = _outMax;
		}
		else if (_outputSum < _outMin)
		{
			_outputSum = _outMin;
		}
		else { }
		
	}
	
}

/* ~~~~~~~~~~~~~~~~ PID Tunings ~~~~~~~~~~~~~~~~ */
void PID::SetTunings(double Kp, double Ki, double Kd)
{
	PID::SetTunings(Kp, Ki, Kd, _pOn);
}
void PID::SetTunings(double Kp, double Ki, double Kd, PIDPON_TypeDef POn)
{
	
	double SampleTimeInSec;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (Kp < 0 || Ki < 0 || Kd < 0)
	{
		return;
	}
	
	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	_pOn    = POn;
	_pOnE   = (PIDPON_TypeDef)(POn == _PID_P_ON_E);

	_dispKp = Kp;
	_dispKi = Ki;
	_dispKd = Kd;
	
	/* ~~~~~~~~~ Calculate time ~~~~~~~~ */
	SampleTimeInSec = ((double)_sampleTime) / 1000;
	
	_kp = Kp;
	_ki = Ki * SampleTimeInSec;
	_kd = Kd / SampleTimeInSec;
	
	/* ~~~~~~~~ Check direction ~~~~~~~~ */
	if (_controllerDirection == _PID_CD_REVERSE)
	{
		
		_kp = (0 - _kp);
		_ki = (0 - _ki);
		_kd = (0 - _kd);
		
	}
	
}

/* ~~~~~~~~~~~~~~~ PID Direction ~~~~~~~~~~~~~~~ */
void          PID::SetControllerDirection(PIDCD_TypeDef Direction)
{
	/* ~~~~~~~~~~ Check parameters ~~~~~~~~~~ */
	if ((_inAuto) && (Direction != _controllerDirection))
	{
		
		_kp = (0 - _kp);
		_ki = (0 - _ki);
		_kd = (0 - _kd);
		
	}
	
	_controllerDirection = Direction;
	
}
PIDCD_TypeDef PID::GetDirection(void)
{
	return _controllerDirection;
}

/* ~~~~~~~~~~~~~~~ PID Sampling ~~~~~~~~~~~~~~~~ */
void PID::SetSampleTime(int32_t NewSampleTime)
{
	
	double ratio;
	
	/* ~~~~~~~~~~ Check value ~~~~~~~~~~ */
	if (NewSampleTime > 0)
	{
		
		ratio = (double)NewSampleTime / (double)_sampleTime;
		
		_ki *= ratio;
		_kd /= ratio;
		_sampleTime = (uint32_t)NewSampleTime;
		
	}
	
}

/* ~~~~~~~~~~~~~ Get Tunings Param ~~~~~~~~~~~~~ */
double PID::GetKp(void)
{
	return _dispKp;
}
double PID::GetKi(void)
{
	return _dispKi;
}
double PID::GetKd(void)
{
	return _dispKd;
}
