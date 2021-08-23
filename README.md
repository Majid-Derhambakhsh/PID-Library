![Banner](Banner.png)

# PID-Library (C Version)
PID Controller library for ARM Cortex M (STM32)

> #### Download Arduino Library : [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)  

## Release
- #### Version : 1.0.0

- #### Type : Embedded Software.

- #### Support :  
               - ARM STM32 series  

- #### Program Language : C/C++

- #### Properties :

- #### Changes :  

- #### Required Library/Driver :


## Overview 
### Initialization and de-initialization functions:
```c++
void PID(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
void PID2(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection);
``` 

### Operation functions:
```c++
/* ::::::::::: Computing ::::::::::: */
uint8_t PID_Compute(PID_TypeDef *uPID);

/* ::::::::::: PID Mode :::::::::::: */
void            PID_SetMode(PID_TypeDef *uPID, PIDMode_TypeDef Mode);
PIDMode_TypeDef PID_GetMode(PID_TypeDef *uPID);

/* :::::::::: PID Limits ::::::::::: */
void PID_SetOutputLimits(PID_TypeDef *uPID, double Min, double Max);

/* :::::::::: PID Tunings :::::::::: */
void PID_SetTunings(PID_TypeDef *uPID, double Kp, double Ki, double Kd);
void PID_SetTunings2(PID_TypeDef *uPID, double Kp, double Ki, double Kd, PIDPON_TypeDef POn);

/* ::::::::: PID Direction ::::::::: */
void          PID_SetControllerDirection(PID_TypeDef *uPID, PIDCD_TypeDef Direction);
PIDCD_TypeDef PID_GetDirection(PID_TypeDef *uPID);

/* ::::::::: PID Sampling :::::::::: */
void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime);

/* ::::::: Get Tunings Param ::::::: */
double PID_GetKp(PID_TypeDef *uPID);
double PID_GetKi(PID_TypeDef *uPID);
double PID_GetKd(PID_TypeDef *uPID);
``` 

### Macros:
```diff  
non   
```

## Guide

#### This library can be used as follows:
#### 1.  Add pid.h header  
#### 2.  Create PID struct and initialize it, for example:      
* Initializer:
  ```c++
  PID(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
  ``` 
* Parameters:  
     * uPID : Pointer to pid struct 
     * Input : The variable we're trying to control (double) 
     * Output : The variable that will be adjusted by the pid (double) 
     * Setpoint : The value we want to Input to maintain (double) 
     * Kp,Ki,Kd : Tuning Parameters. these affect how the pid will change the output (double>=0) 
     * POn : Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified.  
     * ControllerDirection : Either DIRECT or REVERSE. determines which direction the output will move when faced with a given error. DIRECT is most common  
          
          
* Example:
  ```c++  
  PID_TypeDef TPID;

  double Temp, PIDOut, TempSetpoint;

  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
  ``` 
#### 3.  Set 'mode', 'sample time' and 'output limit', for example:  
* Functions:
  ```c++
  void PID_SetMode(PID_TypeDef *uPID, PIDMode_TypeDef Mode);
  void PID_SetOutputLimits(PID_TypeDef *uPID, double Min, double Max);
  void PID_SetSampleTime(PID_TypeDef *uPID, int32_t NewSampleTime);
  ``` 
* Parameters:  
     * uPID : Pointer to pid struct 
     * Mode : _PID_MODE_AUTOMATIC or _PID_MODE_MANUAL
     * Min : Low end of the range. must be < max (double) 
     * Max : High end of the range. must be > min (double) 
     * NewSampleTime : How often, in milliseconds, the PID will be evaluated. (int>0)  
          
* Example:
  ```c++  
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 500);
  PID_SetOutputLimits(&TPID, 1, 100);
  ``` 
  
#### 4.  Using Compute function, for example:  
        
```c++
PID_Compute(&TPID);
```  
      
## Examples  

#### Example 1: PID Compute for temperature
```c++  
#include "main.h"
#include "pid.h"

PID_TypeDef TPID;

char OutBuf[50];
double Temp, PIDOut, TempSetpoint;

int main(void)
{
	
  HW_Init();
  
  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 500);
  PID_SetOutputLimits(&TPID, 1, 100);
  
    while (1) 
    {
  
      Temp = GetTemp();
      PID_Compute(&TPID);
      
      sprintf(OutBuf, "Temp%3.2f : %u\n", Temp, (uint16_t)PIDOut);
      UART_Transmit((uint8_t *)OutBuf, strlen(OutBuf));
      
      Delay_ms(500);
    
    }
}
   
``` 

## Tests performed:
- [x] Run on STM32 Fx cores 

## Developers: 
- ### Majid Derhambakhsh

