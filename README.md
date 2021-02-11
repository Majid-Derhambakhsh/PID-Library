# PID-Library
PID Controller library for ARM Cortex M (STM32)

> #### Download Arduino Library : [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)  

### Version : 1.0.0

- #### Type : Embedded Software.

- #### Support :  
               - ARM STM32 series  

- #### Program Language : C/C++

- #### Properties :

- #### Changes :  

- #### Required Library/Driver :


### Initialization and de-initialization functions:
```c++
void PID(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
void PID2(PID_TypeDef *uPID, double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, PIDCD_TypeDef ControllerDirection);
``` 

### PID operation functions:
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

## How to use this library

### The PID library can be used as follows:
1.1  Add pid.h header  
      
2.1  Initialize:  
        
```c++
PID_TypeDef TPID;

double Temp, PIDOut, TempSetpoint;

PID(&TPID, &Temp, &PIDOut, &TempSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	
PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
PID_SetSampleTime(&TPID, 500);
PID_SetOutputLimits(&TPID, 1, 100);
```  
      
3.1  Using Compute function, for example:  
        
```c++
PID_Compute(&TPID);
```  
      
##### Example 1:  
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
- [ ] Run on AVR
- [x] Run on STM32 Fx cores 

#### Developer: Majid Derhambakhsh

