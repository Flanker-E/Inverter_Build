/*
 * Settings.h
 *
 *  Created on: 2021Äê3ÔÂ11ÈÕ
 *      Author: Flanker
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "Settings.h"
//#include "stdint.h"

#include "PMSM_struct.h"
#include "state_machine.h"

#include "endat.h"

#include "inc/hw_can.h"
#include "driverlib/can.h"

#include "inc/hw_memmap.h"//UARTA_BASE
#include "driverlib/uart.h"//UARTEnable
#include "SciStdio.h"
#define   MATH_TYPE      1
#include "IQmathLib.h"

#include "park.h"       		// Include header for the PARK object
#include "ipark.h"       		// Include header for the IPARK object
#include "pi.h"       			// Include header for the PIDREG3 object
#include "clarke.h"         	// Include header for the CLARKE object
#include "svgen.h"		       	// Include header for the SVGENDQ object
#include "rampgen.h"        	// Include header for the RAMPGEN object
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object

#include "DLOG_4CH_F.h"
#include "MESSAGE_F.h"

#include "speed_fr.h"			// Include header for the SPEED_MEAS_QEP object

//#include "Resolver_Float.h"
#include "F2837x_QEP_Module.h"

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out (do not connect the motors)
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset and speed measurement
#define LEVEL3  3           // Verify closed current(torque) loop and its PIs
#define LEVEL4  4           // Verify speed loop and speed PID
#define LEVEL5  5           // Verify position loop

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL3

// Select Position Feedback Option
#define QEP_POS_ENCODER 1
#define POSITION_ENCODER QEP_POS_ENCODER

#define LAUNCHPAD 0
#define PYMBOARD 1
#define   BUILDTYPE LAUNCHPAD
// #define   BUILDTYPE PYMBOARD

#define FL 0
#define FR 1
#define RL 2
#define RR 3
#define MOTOR_NUM FL //build for FL motor

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (F28_2837xD==1)
#define SYSTEM_FREQUENCY 200
#endif

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10
#define INV_PWM_TICKS  ((SYSTEM_FREQUENCY/2.0)/ISR_FREQUENCY)*1000 //10000
#define INV_PWM_TBPRD INV_PWM_TICKS/2
#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4

//TODO
#define ENCODER_TYPE	21

// ****************************************************************************
// Variables for CPU control
// ****************************************************************************
// adc static cal
int *adc_cal;
// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] = {
		&EPwm1Regs,			//intentional: (ePWM[0] not used)
		&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
		&EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
_iq offset_lemU,         // offset in LEM current V fbk channel @ 0A
	offset_lemV,         // offset in LEM current V fbk channel @ 0A
	offset_lemW;         // offset in LEM current W fbk channel @ 0A

_iq K1 = _IQ(0.998),		  // Offset filter coefficient K1: 0.05/(T+0.05);
    K2 = _IQ(0.001999);	      // Offset filter coefficient K2: T/(T+0.05);

typedef struct {
	float As;      // phase A
	float Bs;      // phase B
	float Cs;      // phase C
} CURRENT_SENSOR;

CURRENT_SENSOR current_sensor;
/*------------------------------------------------------------------------------
Current sensors scaling
------------------------------------------------------------------------------*/
#define IFB_LEMV AdcaResultRegs.ADCRESULT0
#define IFB_LEMW AdcbResultRegs.ADCRESULT0
#define IFB_LEMU AdccResultRegs.ADCRESULT0
#define IFB_LEMV_PPB ((signed int)AdcaResultRegs.ADCPPB1RESULT.all)
#define IFB_LEMW_PPB ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)
#define IFB_LEMU_PPB ((signed int)AdccResultRegs.ADCPPB1RESULT.all)


// ************************************************************************
// Scaling factors to bring all current feedbacks to normal scale
//   matching with shunt based measurement
//  With shunt, 1.0pu current  == 9.945A
//       LEM,   1.0pu current  == 12A
//       SDFM,  0.8906pu current  == 12.5A
// ************************************************************************
#define  LEM_TO_SHUNT    12.5//100/8 DEVKIT //1.206637   // (12.0/9.945)
#define BASE_SHUNT_CURRENT    9.95    // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_LEM_CURRENT     12.0     //  ----- do -----
/*------------------------------------------------------------------------------
Current sensors scaling
------------------------------------------------------------------------------*/
// LEM    1.0pu current ==> 12.0A -> 2048 counts ==> 8A -> 1365
// SHUNT  1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
#define LEM(A)     2048*A/BASE_LEM_CURRENT
#define SHUNT(A)   2048*A/BASE_SHUNT_CURRENT

Uint16  clkPrescale = 20,
		sampwin     = 30,
		thresh      = 18,
		LEM_curHi   = 2048 + LEM(8),    //1365,  //3000
		LEM_curLo   = 2048 - LEM(8);   //1365,  // 200

// ADC Configuration
//definitions for selecting ADC resolution
#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

//definitions for selecting ADC signal mode
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

#define ADC_PU_SCALE_FACTOR        0.000244140625     //1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1

// ****************************************************************************
// Variables for Field Oriented Control
// ****************************************************************************
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
_iq VdTesting = _IQ(0.0),			// Vd reference (pu)
    VqTesting = _IQ(0.8),			// Vq reference (pu)
    IdRef     = _IQ(0.0),			// Id reference (pu)
    IqRef     = _IQ(0.0),			// Iq reference (pu)
    SpeedRef  = _IQ(0.0);           // For Closed Loop tests

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;

// Instance PI(D) regulators to regulate the d and q  axis currents, speed and position
//PIDREG3         pid_pos = PIDREG3_DEFAULTS;          // (optional - for eval)
//PI_CONTROLLER   pi_pos  = PI_CONTROLLER_DEFAULTS;
//PID_CONTROLLER	pid_spd = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PI_CONTROLLER   pi_id   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   pi_iq   = PI_CONTROLLER_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS; // for speed

//	Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

// Define the base quantites
//#define BASE_VOLTAGE        236.14    // Base peak phase voltage (volt), Vdc/sqrt(3)
//#define BASE_SHUNT_CURRENT    9.95    // Base peak phase current (amp), Max. measurable peak curr.
//#define BASE_LEM_CURRENT     12.0     //  ----- do -----
//#define BASE_TORQUE     		      // Base torque (N.m)
//#define BASE_FLUX       			  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	200           // Base electrical frequency (Hz)

// ****************************************************************************
// Variables for Datalog module
// ****************************************************************************
float DBUFF_4CH1[150],
      DBUFF_4CH2[150],
      DBUFF_4CH3[150],
      DBUFF_4CH4[150],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;


//unsigned char motor[2]={'F','L'},
//		AddressSetPoints=0x02,
//		AddressActualValues1=0x00,
//		AddressActualValues2=0x04,
//		Enabled=0;

SETPOINTS SetPoints;
ACTUALVALUES1 ActualValues1;
ACTUALVALUES2 ActualValues2;

// ****************************************************************************
// Extern functions and variables referred from resolver.c
// ****************************************************************************
extern void baseParamsInit(void);
extern void derivParamsCal(void);

//// Resolver Related defines
//#define  TWO_PI                  (2*PI)
//#define  DELAY_LENGTH            16
//
//#define  FIR32_SEL    0   /* 1 - Implement 33 tap FIR, samples  over 2 cycles
//                             0 - Implement 17 tap FIR, samples  over 1 cycle   */
//
//#define  TUNING_SEL   0   /* 1 - bypass atan value - to tune PI coefficients
//                             0 - use atan value*/
// Variables for Position Sensor Suite
_iq posEncElecTheta[6],
    posEncMechTheta[6];

_iq  cntr=0,
	 alignCnt = 20000;
_iq  IdRef_start = _IQ(0.1),
	 IdRef_run   = _IQ(0.0);

// Used to indirectly access eQEP module
volatile struct EQEP_REGS *eQEP[] =
 				  { &EQep1Regs,
 				  	&EQep1Regs,
					&EQep2Regs,
				  };

// Instance a speed calculator based on Encoder position
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

//// RESOLVER specs
//#define RESOLVER_STEPS_PER_TURN         4096       // Resolver's discrete steps/turn
//#define RESOLVER_STEPS_PER_POLEPAIR    (RESOLVER_STEPS_PER_TURN/(POLES/2))
// Define the electrical motor parametes (Estun Servomotor)
#define RS 		2.26		    	    // Stator resistance (ohm)
#define RR   			               	// Rotor resistance (ohm)
#define LS   	0.00131					// Stator inductance (H)
#define LR   			  				// Rotor inductance (H)
#define LM   			   				// Magnatizing inductance (H)
#define POLES  	8						// Number of poles

#endif /* SETTINGS_H_ */
