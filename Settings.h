/*
 * Settings.h
 *
 *  Created on: 2021Äê3ÔÂ11ÈÕ
 *      Author: Flanker
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "F28x_Project.h"
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
#define   BUILDLEVEL LEVEL1

#define LAUNCHPAD 0
#define PYMBOARD 1
#define   BUILDTYPE LAUNCHPAD
// #define   BUILDTYPE PYMBOARD

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
#define INV_PWM_TICKS  ((SYSTEM_FREQUENCY/2.0)/ISR_FREQUENCY)*1000
#define INV_PWM_TBPRD INV_PWM_TICKS/2
#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4

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

// ADC Configuration
//definitions for selecting ADC resolution
#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

//definitions for selecting ADC signal mode
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1

// ****************************************************************************
// Variables for Field Oriented Control
// ****************************************************************************
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
_iq VdTesting = _IQ(0.0),			// Vd reference (pu)
    VqTesting = _IQ(0.10),			// Vq reference (pu)
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

#endif /* SETTINGS_H_ */
