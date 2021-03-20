/******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
***************************************************************************//*
***************************************************************************//*!
*
* @file:	PMSM_struct.h
*
* @author:	B34195
*
* @date: 	Aug 9, 2016
*
* @brief: 	Header file containing definitions of the PMSM FOC control approach.
* 			It summarizes the individual Sensor/Actuator modules from BSP
* 			repository and creates a global pmsmFOC_t structure.
*
* Last Updated By: Yiming PENG
* Date of Last Update: 8/5/2020
****************************************************************************/
#ifndef PMSM_STRUCT_H_
#define PMSM_STRUCT_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "stdint.h"
#include "F28x_Project.h"
//#include "SWLIBS_Typedefs.h"
//#include "current_meas.h"
//#include "dcVoltage_meas.h"
//#include "temp_meas.h"
//#include "safety_logic.h"
//
//#include "voltage_gener.h"

//#include "MPC5744P_CB.h"

//#include "gflib.h"
//#include "gmclib.h"
//#include "gdflib.h"

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/
typedef enum {
	resolver		= 0,
	endat			= 1
}sensorOptions_t;         /* Application position/speed feedback type*/

typedef enum {
	torqueCtrl		= 0,
	speedCtrl		= 1
}focCascadeStruc;    	/* Application cascade control type - aligned with MCAT*/

typedef enum {
    reset           = 0,
	init            = 1,
	charge			= 2,
    ready           = 3,
    run           	= 4,
    fault           = 5
}AppStates;         	/* Application state identification user type*/

typedef enum {
	e_reset			= 0,
	e_reset_done 	= 1,
	e_init			= 2,
	e_init_done		= 3,
	e_charge		= 4,
	e_charge_done	= 5,
	e_ready			= 6,
	e_driver_enable	= 7,
	e_driver_disable= 8,
	e_run			= 9,
	e_fault			= 10,
	e_fault_clear	= 11
}AppEvents;         	/* Application event identification user type*/

/*! Application fault status user type */
typedef union
{
    volatile uint32_t R;
    struct
    {
    	volatile uint32_t CTU_Error              : 1;   /* 0 Error in CTU h/w initialization*/
    	volatile uint32_t ADC_Error              : 1;   /* 1 Error in ADC h/w initialization*/
    	volatile uint32_t FLEXPWM_Error          : 1;   /* 2 Error in FlexPWM h/w initialization */
    	volatile uint32_t MazetError          	 : 1;	/* 3 Error in Mazet initialization */
    	volatile uint32_t EndatCommError         : 1;	/* 4 Error in endat2.2 communication with heidenhain encoder*/
    	volatile uint32_t EncoderError           : 1;	/* 5 Error in encoder */
    	volatile uint32_t UnderLowVoltage        : 1;	/* 6 GLV voltage is lower than threshold */
    	volatile uint32_t ShutdownCircuitOFF     : 1;	/* 7 shutdown circuit is not closed = no current flow */
    	volatile uint32_t CANCommError       	 : 1;	/* 8 Error in CAN communication */
    	volatile uint32_t SafeLogicError       	 : 1;	/* 9 Safety logic cicuit report error */
    	volatile uint32_t GateDriverNotReady     : 1;	/* 10 gate driver not all ready */
    	volatile uint32_t DesaturationError      : 1;	/* 11 any desaturation error occurs in 6 drivers  */

    	volatile uint32_t InitError              : 1;   /* 12 Error during app. initialization */
    	volatile uint32_t CalibError             : 1;   /* 13 Error during calibration */
    	volatile uint32_t FOCError             	 : 1;   /* 14 Error during FOC calculation */

    	volatile uint32_t : 4;                          /* 15 RESERVED */
    												/* 16 RESERVED */
    												/* 17 RESERVED */
    												/* 18 RESERVED */
    	volatile uint32_t OffCancError           : 1;   /* 19 Offset Cancellation Error flag */
    	volatile uint32_t OverPhaseCCurrent      : 1;   /* 20 OverCurrent C fault flag */
    	volatile uint32_t OverPhaseBCurrent      : 1;   /* 21 OverCurrent B fault flag */
    	volatile uint32_t OverPhaseACurrent      : 1;   /* 22 OverCurrent A fault flag */
    	volatile uint32_t OverHeatDriver         : 1;	/* 23 Overheating driver fault flag */
    	volatile uint32_t OverHeatHeatsink       : 1;	/* 24 Overheating heatsink fault flag */
    	volatile uint32_t OverHeatMotor          : 1;   /* 25 Overheating motor fault flag */
    	volatile uint32_t WrongHardware          : 1;   /* 26 Wrong hardware fault flag */
    	volatile uint32_t MainsFault             : 1;   /* 27 Mains out of range */
    	volatile uint32_t OverLoad               : 1;   /* 28 Overload Flag */
    	volatile uint32_t OverDCBusCurrent       : 1;   /* 29 OverCurrent DC fault flag */
    	volatile uint32_t UnderDCBusVoltage      : 1;   /* 30 Undervoltage fault flag */
    	volatile uint32_t OverDCBusVoltage       : 1;   /* 31 Overvoltage fault flag */
    } B;
}AppFaultStatus;    /* Application fault status user type*/

/*------------------------------------------------------------------------*//*!
@brief  Structure containing raw ADC results
*//*-------------------------------------------------------------------------*/
typedef struct
{
    AppStates       state;                  // Application states of a State Machine
    AppEvents       event;                  // Application events of a State Machine
    bool			loadDefSetting;			// load default setting of FOC algorithm
    sensorOptions_t switchSensor;   		// Choose encoder/resolver/sensorless position/speed feedback
    bool           readFault;				// Read error status
  //tS16            ledFlashCounter;		// counter for LED toggle period
    focCascadeStruc	controlMode;			// defines the control mode within a control structure; range(0 - 3)
    signed short			speedLoopCtrl;			// rate between speed and current loop
}driveStates_t;

typedef struct{
	bool           enable;         //
	bool           faultClear;     //
	float			targetTorque;
	float			torqueLimitP;
	float			torqueLimitN;

}CANtorqueCtrlSetpoints_t;

/*------------------------------------------------------------------------*//*!
@brief  Structure containing raw ADC results
*//*-------------------------------------------------------------------------*/
//typedef struct
//{
//	tFrac32		thScalarEl;		// El. position entering the scalar control
//    float		wRotElReq;		// Required frequency [in rad/s] of supply voltage for scalar control
//    float		wRotElReqRamp;  // Required frequency [in rad/s] as a ramp
//    float		wElMax; 		// Required frequency as a ramp
//    float		UmReq;			// Required magnitude of supply voltage for scalar control
//    float		VHzRatioReq;	// V/f ratio
//    GFLIB_INTEGRATOR_TR_T_F32       integ;		 // F32 integrator -> position wrapping around +-1
//    GFLIB_RAMP_T				wRotReqRamp;
//} scalarControl_t;

/*------------------------------------------------------------------------*//*!
@brief  Structure containing position/speed module variables
*//*-------------------------------------------------------------------------*/
//typedef struct
//{
//	float          thRotEl;		// El. position entering to the control loop
//	float          wRotEl;			// El. speed entering to the control loop
//	float          wRotElReq;		// Required el. speed
//	float          wRotElReqRamp;	// Required el. speed converted to the ramp shape
//	float          wRotElErr;		// Error of the el. speed entering to speed controller
//    tS16			speedLoopCtrl;	// rate between speed and current loop
//    GFLIB_RAMP_T	speedRamp;		// Speed ramp function
//} speedLoop_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing control board for h/w initialization
*//*-------------------------------------------------------------------------*/
//typedef struct{
//	//resolver_hw_cfg_t			Resolver;		// resolver peripheral h/w initialization
//    ph_current_meas_hw_cfg_t	phCurrents;		// phase currents meas. h/w initialization
//    dcb_voltage_meas_hw_cfg_t	uDcb;			// DC-bus voltage meas. h/w initialization
//    temp_meas_hw_cfg_t          temp;           // temperatures meas. h/w initialization
//    flexPWM_hw_cfg_t			flexPWM;		// FlexPWM module h/w initialization
//}controlBoardHardConfig_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing encoder data and rotor status
*//*-------------------------------------------------------------------------*/
typedef struct{

	uint32_t 	position;  				// rotor position value 18 bit
	float		radianMec;				// rotor mechanical radian
	float		degreeMec;				// rotor mechanical degree
	float		degreeElec;				// rotor electrical degree
	float		RPM;					// rotor mechanical speed RPM
	float		wRotorElec;				// rotor electrical speed rad/s

	uint32_t   status;					// encoder status

}encoder_data_t;





/*------------------------------------------------------------------------*//*!
@brief  Structure containing the FOC structure, sensors, actuator, control variables
*//*-------------------------------------------------------------------------*/
/*! General structure for  */
typedef struct{
    AppFaultStatus				faultID;        // Application faults
    AppFaultStatus				faultIDp;       // Application fault flags
	uint32_t						svmSector;      // Space Vector Modulation sector
//    SWLIBS_2Syst                iDQFbck;        // dq - axis feedback currents
//    SWLIBS_2Syst                iDQReq;         // dq - axis required currents, given by speed PI
//    SWLIBS_2Syst                iDQReqZC;       // Transfer function zeros cancellation in current branch
//    SWLIBS_2Syst                iDQErr;         // Error between the reference and feedback signal
//    SWLIBS_2Syst                uDQReq;         // dq - axis required voltages given by current PIs
//    SWLIBS_2Syst                thTransform;    // Transformation angle - for Park transformation
//    SWLIBS_2Syst                iAlBeFbck;      // Alpha/Beta - axis feedback currents
//    SWLIBS_2Syst                uAlBeReq;       // Required dq currents transformed into the Alpha/Beta orth. system
//    SWLIBS_2Syst                uAlBeReqDCB;    // Alpha/Beta required voltages after DC Bus ripple elimination
//
//    GMCLIB_ELIMDCBUSRIP_T		elimDcbRip;     // Predefined structure related to DC Bus voltage ripple elimination
//
//    GDFLIB_FILTER_IIR1_T		dAxisZC;		// d-axis current zero cancellation
//    GDFLIB_FILTER_IIR1_T		qAxisZC;		// q-axis current zero cancellation
//    GFLIB_CONTROLLER_PIAW_P_T	dAxisPIp;		// d-axis current PI controller
//    GFLIB_CONTROLLER_PIAW_P_T	qAxisPIp;		// q-axis current PI controller
//    GFLIB_CONTROLLER_PIAW_P_T	speedPIp;       // Speed Loop PI controller

    driveStates_t				ctrlState;		// Control states variables
//    speedLoop_t					speedLoop;		// Position/Speed variables in Speed control loop
    CANtorqueCtrlSetpoints_t	setpoint;       // CAN setpoint for torque control
//    ph_current_meas_data_t		iAbcFbck;       // SENSOR - three phases current feedback
//    dcb_voltage_meas_data_t		uDcbFbck;		// SENSOR - raw/filtered value of the DCbus voltage feedback
//    temp_meas_data_t        	tempFbck;		// SENSOR - three temperature feedback
    encoder_data_t				encoder;		//SENSOR - encoder position/speed feedback
//    safety_logic_state_t		safetyLogicFbck;// safety logic state feedback;

//    flexPWM_sw_t				pwm;			// ACTUATOR - flexPWM s/w data type

//    controlBoardHardConfig_t	MPC5744P_HW;	// HW initialization of control board related modules

    volatile unsigned char			CmdDriverEnable; 	//CAN command driver enable
    volatile unsigned char			CmdErrorReset; 		//CAN command error reset


    //uint8_t						motorID; //unique motor identifier

}pmsmFOC_t;


#endif /* PMSM_STRUCT_H_ */
