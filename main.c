/******************************************************************************
* Project:	PMSM Field Oriented Control
* MCU:		MPC5744P
* Sensor:	Heidenhan/Absolute Encoder ECI-1118 with EnDat 2.2 communication
* Control:	Torque/Speed FOC control
*
*
* Last Updated By: Zhewei YE
* Date of Last Update: 3/11/2021
*
*
***************************************************************************/

#include "Settings.h"



/******************************************************************************
* Defines and macros
******************************************************************************/
#define DISABLE_IRQ()                   __asm__("wrteei 0")
#define ENABLE_IRQ()                    __asm__("wrteei 1")

#define CAN_TX_COUNTER					50   // fast ISR 100us, 50*100 us=5ms
#define CAN_OFFLINE_COUNTER				500   // fast ISR 100us, 500*100 us=50ms

#define BLINKY_LED_GPIO    31
#define DELAY (CPU_RATE/1000*6*510)  //Qual period at 6 samples

/******************************************************************************
* Local function prototypes
******************************************************************************/
extern void xcptn_xmpl(void);
static bool focCurrentLoop(pmsmFOC_t *ptr);
static bool faultDetect(pmsmFOC_t *ptr);

void InitLED();
void CAN_A_init();
void epwm_init();
void InitEPwm1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
void InitEPWM_1ch_UpCnt_CNF(int16 n, Uint16 period);
void HVDMC_Protection(void);
void cmpssConfig(volatile struct CMPSS_REGS *v, int16 Hi, int16 Lo);

//__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
//__interrupt void epwm3_isr(void);

void adc_init();
void adc_dac_configure();
void uart_init();
void enDat_init();
void param_init();

void mc34_init();
/* Interrupt functions */
//#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (FOC_Fast_ISR, LPI)

// Prototype statements for functions found within this file.
interrupt void FOC_Fast_ISR(void);// 10kHz interrupt.
interrupt void Message_Transmit(void);//200Hz

//void scia_init();  //init the scia and uart param

/******************************************************************************
* Local variables
******************************************************************************/
static volatile unsigned char runState, runState_last;

static volatile uint32_t canTxCounter =0 ;
static volatile uint32_t canOfflineCounter =0 ;
static volatile bool canOfflineDetectOn	= false;    // flag for CAN offline detect on
static unsigned char count_state=0;
int16 OffsetCalCounter;

unsigned char ucTXMsgData[4] = {1,2,3,4};
unsigned char ucRXMsgData[4];
uint16_t crc5_self;
/*------------------------------------
 * FOC variables
 * ----------------------------------*/
pmsmFOC_t 			FOC;
Uint32 IsrTicker = 0;
Uint32 IsrTxTicker = 1; //avoid being triggered with DATALOG in the same loop

Uint16
//BackTicker = 0,
       lsw = 0,
//       TripFlagDMC = 0,				//PWM trip status
//       clearTripFlagDMC = 0,
       RunMotor = 0;
// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

int test=0;

interrupt void adca1_isr(void)
{
//    AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
//    if(RESULTS_BUFFER_SIZE <= resultsIndex)
//    {
//        resultsIndex = 0;
//        bufferFull = 1;
//    }
	//gpio29
	EALLOW;
    GpioDataRegs.GPATOGGLE.bit.GPIO29=1;
	EDIS;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void main(void)
{

//    unsigned int DCVoltage =0xF00F;


    volatile int status = 0;

//    volatile FILE *fid;

	// Initializing of clock, reset, core mode etc.
    InitSysCtrl();

    InitLED();


    // Disable interrupts
    DINT;

    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    //
    //user specific code:
    //
    //TODO MC_ME.PCTL78.B.RUN_CFG = 0x0;


    //TODO GPIO CAN initializing
//    CAN_A_init();

    // init gd3000
    mc34_init();

    // GPIO&Understand PWM initializing
    epwm_init();


    //TODO can be replaced by Epwm CTU initializing
//    ctu0_init();

    //TODO understand&GPIO ADC DAC initializing
//    InitGpio();
    EALLOW;
//    GpioCtrlRegs.GPALOCK.bit.GPIO29=0;
        GpioCtrlRegs.GPAMUX2.bit.GPIO29=0;
        GpioCtrlRegs.GPADIR.bit.GPIO29=1;
//    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);  // 方向为输出
//	GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
	GpioDataRegs.GPATOGGLE.bit.GPIO29=1;


    EDIS;
    adc_init();
    adc_dac_configure();

	// GPIO Initialize UART
	uart_init();

    // SPI initializing
//    spi_init();

    //TODO GPIO reset encoder
//    enDat_init();

	//TODO  Initialize CTU modules including the CTU interrupt priority
    //INTC_0.PSR[713].B.PRIN = 10;	// Set CTU0-ADC Interrupt Priority

//    INTC_0.PSR[702].B.PRIN = 10;	// Set trigger1 Interrupt Priority

     //INTC_0.PSR[709].B.PRIN = 10;	// Set FIFO0 overflow Interrupt Priority

	//TODO init parameter
	param_init();

	// ****************************************************************************
	// ****************************************************************************
	//TODO ISR Mapping
	// ****************************************************************************
	// ****************************************************************************
	EALLOW;
	// ADC C EOC of SOC0 is used to trigger Resolver Interrupt
	AdccRegs.ADCINTSEL1N2.bit.INT1SEL  = 0;
	AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdccRegs.ADCINTSEL1N2.bit.INT1E    = 1;

	//PWM11 INT is used to trigger Motor Control ISR
	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

//	PieVectTable.ADCC1_INT = &ResolverISR;
	PieVectTable.EPWM11_INT = &FOC_Fast_ISR;
	PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1

	PieCtrlRegs.PIEIER3.bit.INTx11 = 1;  // Enable PWM11INT in PIE group 3 priority 11
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   //priority 1
//	PieCtrlRegs.PIEIER1.bit.INTx3  = 1;  // Enable ADCC1INT in PIE group 1

	EPwm11Regs.ETCLR.bit.INT=1;
	EDIS;


	// set initial state for the state machines
	FOC.ctrlState.state   = reset;
	FOC.ctrlState.event   = e_reset;

	// first call to state machines
	SCIPuts("\r\n ============Test Start===========.\r\n", -1);
	state_table[FOC.ctrlState.event][FOC.ctrlState.state](&FOC);
//	state_LED[FOC.ctrlState.state]();


	//TODO PIEgroup Enable interrupts
	IER |= M_INT3; // Enable group 3 interrupts
	IER |= M_INT1; // Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    while(1)
    {
        GPIO_WritePin(BLINKY_LED_GPIO, 0);
//        SCIPuts("\r\n ============Message Start===========.\r\n", -1);

        DELAY_US(50*500);

//        state_table[FOC.ctrlState.event][FOC.ctrlState.state](&FOC);

        GPIO_WritePin(BLINKY_LED_GPIO, 1);

        DELAY_US(50*500);

//        i++;if(i==200)i=0;
    }
}

/**************************************************************************//*!
@brief          interrupt service routine

@param[in,out]  void
@param[in]      void

@return         void

@details        The ISR runs the same frequency as defined in CTU (10KHz).
				ISR periodically performs: all ADC quantities measurement,
				state machine calling, user button/switch and LEDs control,
				FreeMASTER recorder

@note           none

@warning		none
******************************************************************************/
interrupt void FOC_Fast_ISR(void){
	//	//FOC.encoder.position=enDat_getPosition();
	//
	//	// get encoder position
	//	encoder_calc_mec_radian(&FOC);
	//	encoder_calc_mec_degree(&FOC);
	//	encoder_calc_elec_degree(&FOC);
	//	encoder_calc_RPM(&FOC, 10000);
	EINT;

	// Verifying the ISR
    IsrTicker++;


// =============================== LEVEL 1 ======================================
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level
// ==============================================================================
// =============================== LEVEL 2 ======================================
//	  Level 2 verifies
//	     - all current sense schems
//         - analog-to-digital conversion (shunt and LEM)
//         - SDFM function
//       - Current Limit Settings for over current protection
//       - clarke/park transformations (CLARKE/PARK)
//       - Position sensor interface
//         - speed estimation
// ==============================================================================

//TODO BUILD 2
#if (BUILDLEVEL == LEVEL2)
	// ------------------------------------------------------------------------------
	// Alignment Routine: this routine aligns the motor to zero electrical angle
	// and in case of QEP also finds the index location and initializes the angle
	// w.r.t. the index location
	// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

#if POSITION_ENCODER==QEP_POS_ENCODER
		lsw = 1;   // for QEP, spin the motor to find the index pulse
//#else
//		lsw  = 2;  // for absolute encoders no need for lsw=1

#endif
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)
//	if(rg1.Out<_IQ(1.0))
//		rg1.Out+=_IQ(_IQdiv(1,360));
//	else
//		rg1.Out=_IQ(0.0);
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
//	currentSensorSuite();
	current_sensor.As   = (float)IFB_LEMV_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor.Bs   = (float)IFB_LEMW_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor.Cs   = (float)IFB_LEMU_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
			//-current_sensor[LEM_CURRENT_SENSE-1].Cs-current_sensor[LEM_CURRENT_SENSE-1].Bs;

	clarke1.As = current_sensor.As; // Phase A curr.
	clarke1.Bs = current_sensor.Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha  = clarke1.Alpha;
	park1.Beta   = clarke1.Beta;
	park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//	There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

//    park1.Angle  = rg1.Out;
//	park1.Sine   = __sinpuf32(park1.Angle);
//	park1.Cosine = __cospuf32(park1.Angle);

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
//	posEncoderSuite();  // if needed reverse the sense of position in this module

// ----------------------------------
// lsw = 0 ---> Alignment Routine
// ----------------------------------
#if (POSITION_ENCODER == QEP_POS_ENCODER)
	if (lsw == 0)
	{
		// during alignment, assign the current shaft position as initial position
		EQep1Regs.QPOSCNT = 0;
		EQep1Regs.QCLR.bit.IEL = 1;  // Reset position cnt for QEP
	} // end if (lsw=0)

// ******************************************************************************
//    Detect calibration angle and call the QEP module
// ******************************************************************************
	// for once the QEP index pulse is found, go to lsw=2
	if(lsw==1)
	{
		if (EQep1Regs.QFLG.bit.IEL == 1)			// Check the index occurrence
		{
			qep1.CalibratedAngle=EQep1Regs.QPOSILAT;
//			EQep1Regs.QPOSINIT = EQep1Regs.QPOSILAT; //new
//			EQep1Regs.QEPCTL.bit.IEI = IEI_RISING;   // new
			lsw=2;
		}   // Keep the latched pos. at the first index
	}

	if (lsw!=0){
		QEP_MACRO(1,qep1);
	}

	// Reverse the sense of position if needed - comment / uncomment accordingly
	// Position Sense as is
	posEncElecTheta[QEP_POS_ENCODER] = qep1.ElecTheta;
	posEncMechTheta[QEP_POS_ENCODER] = qep1.MechTheta;

//	// Position Sense Reversal
//	posEncElecTheta[QEP_POS_ENCODER] = 1.0 - qep1.ElecTheta;
//	posEncMechTheta[QEP_POS_ENCODER] = 1.0 - qep1.MechTheta;
#endif
// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
 	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc*10)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta*10)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb*10)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
//	DlogCh3 = svgen1.Tb;
//	DlogCh4 = svgen1.Tc;
	DlogCh3 = clarke1.Alpha;
	DlogCh4 = clarke1.Beta;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
//	if (rslvrIn.TUNING)
//	{
//		DacbRegs.DACVALS.bit.DACVALS = rslvrOut.angleRaw*4096;
//		DaccRegs.DACVALS.bit.DACVALS = rslvrOut.angleOut*4096;
//	}
//	else
//	{
//	if(test<4096)
//		test++;
//	else
//		test=0;
//	DacbRegs.DACVALS.bit.DACVALS = (svgen1.Tb*0.5+0.5)*4096;
//	DacaRegs.DACVALS.bit.DACVALS = (svgen1.Tc*0.5+0.5)*4096;
//	DacbRegs.DACVALS.bit.DACVALS = (unsigned int)((clarke1.Alpha+4.0)*512.0);
//	DacaRegs.DACVALS.bit.DACVALS = (unsigned int)((clarke1.Beta+4.0)*512.0);
	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
	DacaRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;
//	}

#endif // (BUILDLEVEL==LEVEL2)

// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
	DLOG_4CH_F_FUNC(&dlog_4ch1);

//	if(IsrTxTicker==49){
//		IsrTxTicker=0;
//		ActualValues1.DCVoltage=AdcaResultRegs.ADCRESULT0;
//		ActualValues1.ActualTorque=AdcbResultRegs.ADCRESULT0;
//		ActualValues1.ActualVelocity=AdccResultRegs.ADCRESULT0;
//		ACTUALVALUE1_uart_TX(&ActualValues1, UARTA_BASE);
//		ACTUALVALUE2_uart_TX(&ActualValues2, UARTA_BASE);
//	}
//	else
//		IsrTxTicker++;

	EPwm11Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;
    EDIS;

//
//
//	//wait for ADC converted done
//	while(!ADC_1.ISR.B.EOCTU);
//
//	if(ADC_1.ISR.B.EOCTU == 1)
//	{
//		ADC_1.ISR.B.EOCTU = 1;
//
//
//		//get phase current
//		PhCurrent3Ph_get_data(&FOC.MPC5744P_HW.phCurrents,&FOC.iAbcFbck, I_MAX);
//		//PhCurrent2Ph_get_data(&FOC.MPC5744P_HW.phCurrents,&FOC.iAbcFbck, I_MAX);
//
//		//get dc bus voltage
//		DcbVoltage_get_data(&FOC.MPC5744P_HW.uDcb,&FOC.uDcbFbck,U_DCB_MAX);
//
//		//get temperatures
//		 temp_get_data(&FOC.MPC5744P_HW.temp,&FOC.tempFbck);
//
//		//get LV power source voltage
//
//	}
//
//	//get safety logic states
//	 getSafetyLogicState(&FOC.safetyLogicFbck);
//
//	// first received CAN setpoint - CAN1 buffer4
//	if(CAN_1.IFLAG1.B.BUF4TO1I == 8)
//		canOfflineDetectOn =TRUE;
//
//	if(canOfflineDetectOn){
//		//get CAN setpoint
//		if(CAN_Unpack_Rx_Setpoints(&FOC))
//			canOfflineCounter =0;
//
//		if(canOfflineCounter>=CAN_OFFLINE_COUNTER){
//			FOC.faultID.B.CANCommError = 1;
//			FOC.faultIDp.B.CANCommError = 1;
//			FOC.ctrlState.event 			= e_fault;
//		}
//		else
//			canOfflineCounter++;
//	}
//
//	//fault detect
//	//if(faultDetect(&FOC)) 	FOC.ctrlState.event = e_fault;
//
//	// Application state machine calling including LED signal
//	state_table[FOC.ctrlState.event][FOC.ctrlState.state](&FOC);
//	state_LED[FOC.ctrlState.state]();
////
//	// DC-bus voltage ripple elimination
//    FOC.elimDcbRip.fltArgDcBusMsr  = FOC.uDcbFbck.raw;
//	GMCLIB_ElimDcBusRip(&FOC.uAlBeReqDCB,&FOC.uAlBeReq,&FOC.elimDcbRip);
//
//	//Space Vector Modulation
//	FOC.svmSector = GMCLIB_SvmStd(&FOC.pwm.PhABC,&FOC.uAlBeReqDCB);
//
//	// Recalculate FOC voltages to DutyCycle values and apply to FlexPWM
//	apply_voltages(&FOC.MPC5744P_HW.flexPWM, &FOC.pwm);
//	FLEXPWM_set_LDOK(&FOC.MPC5744P_HW.flexPWM);
//
//	// if canTxCounter >= CAN_TX_COUNTER （5ms）, send CAN messages.
//	if(++canTxCounter > CAN_TX_COUNTER){
//		canTxCounter =0;
//		CAN_Pack_Tx(&FOC);
//	}
//
////	SPI_1.PUSHR.PUSHR.R  = 0x08015678;     /* Transmit data from master to slave SPI with EOQ */
////	read_data_DSPI_1();                     /* Read data on master DSPI */
//
////	uint16_t tx_buffer[6] = {0};
////	tx_buffer[0]=0x9ABC;
////	tx_buffer[1]=0x1234;
////	tx_buffer[2]=0x8543;
////
////	SPI1_transmit_SPI_16((uint16_t *)tx_buffer, NULL, 3, ASSERT_CTAR0, ASSERT_CS0);
//
//	mazet_status=mazet_getStatus();
//
////	//LED_STATE =0;
//	//LED_WARN_FLASH_FAST();
//
//	// Interrupt Flag Register - clear ADC command interrupt flag
//	//CTU_0.IFR.B.ADC_I = 0x1;
//
//	 CTU_0.IFR.B.T1_I =1;	// clear trigger1 interrupt flag

}


/***************************************************************************//*!
*
* @brief   RESET state
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
* @details	clear all application state variables and load default configuration
* 			of all parameters of the control algorithm (PI controllers, etc)
******************************************************************************/
void stateReset(pmsmFOC_t *ptr){

	SCIPuts("\r\n ============State Reset===========.\r\n", -1);
	// Resolve the RESET state sequence
//	if(statusResetPass)
	ptr->ctrlState.event   		= e_reset_done;
//	else					ptr->ctrlState.event   		= e_reset;

}
/***************************************************************************//*!
*
* @brief   INIT state - clear state variables
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
* @details	clear all application state variables, while algorithm parameters
* 			remain unchanged. It means if user changed the parameters during last
* 			run state, those parameters will not be set to default.
******************************************************************************/
void stateInit(pmsmFOC_t *ptr){
// Application State Machine - state identification
	ptr->ctrlState.state   = init;
	SCIPuts("\r\n ============State Init===========.\r\n", -1);
    /*------------------------------------
     * FOC general
     ------------------------------------*/

    ptr->ctrlState.event   				= e_init_done;
}
/***************************************************************************//*!
*
* @brief   Charge state
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
******************************************************************************/
void stateCharge(pmsmFOC_t *ptr){
	static int n=0;
	// Application State Machine - state identification
	ptr->ctrlState.state   = charge;
	ptr->ctrlState.event   = e_charge;

	SCIPuts("\r\n ============State Charge===========.\r\n", -1);
	n++;
	//DcbVoltage_get_data(&ptr->MPC5744P_HW.uDcb,&ptr->uDcbFbck,U_DCB_MAX);

	// Wait for dc bus voltage becomes greater than threshold
//	if (ptr->uDcbFbck.filt >= U_DCB_TRIP )
//
	if(n>5)
	{
	ptr->ctrlState.event   = e_charge_done;
	n=0;
	}

	if(ptr->ctrlState.loadDefSetting) ptr->ctrlState.event   = e_reset;
}

/***************************************************************************//*!
*
* @brief   READY state
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
******************************************************************************/
void stateReady(pmsmFOC_t *ptr){
	// Application State Machine - state identification
	ptr->ctrlState.state   = ready;
	ptr->ctrlState.event   = e_ready;

	SCIPuts("\r\n ============State Ready===========.\r\n", -1);

	// Wait for CAN command enable
//	if (ptr->setpoint.enable == TRUE)
//		ptr->ctrlState.event   = e_driver_enable;

	if(ptr->ctrlState.loadDefSetting) ptr->ctrlState.event   = e_reset;
}

/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
******************************************************************************/
void stateRun(pmsmFOC_t *ptr){
	static bool stateRunStatus = false;

	// Application State Machine - state identification
    ptr->ctrlState.state	= run;

}
/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   pointer to structure of pmsmFOC_t type (defined in PMSM_struct.h)
*
* @return  none
*
******************************************************************************/
void stateFault(pmsmFOC_t *ptr){
}
// configure LED
void InitLED()
{
#if (BUILDTYPE==LAUNCHPAD)
	GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);  // 方向为输出
	GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);                // 选择引脚功能为GPIO功能

	GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);

#elif (BUILDTYPE==PYMBOARD)
	GPIO_SetupPinOptions(105, GPIO_OUTPUT, GPIO_PUSHPULL);  // 方向为输出
	GPIO_SetupPinMux(105, GPIO_MUX_CPU1, 0);                // 选择引脚功能为GPIO功能

	GPIO_SetupPinOptions(107, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(107, GPIO_MUX_CPU1, 0);

	GPIO_SetupPinOptions(106, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(106, GPIO_MUX_CPU1, 0);
#endif
//	GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_ASYNC);
//	GPIO_SetupPinMux(34,0,0);
//
//	GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_ASYNC);
//	GPIO_SetupPinMux(31,0,0);
//
//	GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);
//	GPIO_SetupPinMux(43,0,0);
}
void CAN_A_init()
{
//	CANInit(CANA_BASE);
//	// 选择CAN模块的时钟源
//	CANClkSourceSelect(CANA_BASE, 0);
//	//配置CAN的比特率，时钟CAN bus is set to 500 kHz
//	CANBitRateSet(CANA_BASE, 200000000, 500000);
//	// Enable test mode and select external loopback
//	HWREG(CANA_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
//	HWREG(CANA_BASE + CAN_O_TEST) = CAN_TEST_EXL;
//	//使能CAN
//	CANEnable(CANA_BASE);
//	*(unsigned long *)ucTXMsgData = 0;
//
//	sTXCANMessage.ui32MsgID = 1;                      // CAN消息ID
//	sTXCANMessage.ui32MsgIDMask = 0;                  // 无屏蔽
//	sTXCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;  // 使能中断
//	sTXCANMessage.ui32MsgLen = sizeof(ucTXMsgData);   // 消息长度
//	sTXCANMessage.pucMsgData = ucTXMsgData;           // 发送内容的指针地址
//
//	// Initialize the message object that will be used for recieving CAN
//	// messages.
//	*(unsigned long *)ucRXMsgData = 0;
//	sRXCANMessage.ui32MsgID = 1;                      // CAN消息ID
//	sRXCANMessage.ui32MsgIDMask = 0;                  //无屏蔽
//	sRXCANMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
//	sRXCANMessage.ui32MsgLen = sizeof(ucRXMsgData);   //消息长度
//	sRXCANMessage.pucMsgData = ucRXMsgData;           //接受内容的指针地址
//
//	//配置message object用于接受
//	CANMessageSet(CANA_BASE, 2, &sRXCANMessage, MSG_OBJ_TYPE_RX);
}
void mc34_init(){

	EALLOW;
	//rst_gpio
	GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);  // 方向为输出
	GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);                // 选择引脚功能为GPIO功能
	//en1&en2
	GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
	//initialize outputs
	GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
	EDIS;

	GPIO_WritePin(24, 1);// RST置1
	DELAY_US(100);
	GPIO_WritePin(16, 1);// EN置1
	DELAY_US(100);
	//Px_HS&LS->0
	GPIO_WritePin(0, 0);
	GPIO_WritePin(1, 0);
	GPIO_WritePin(2, 0);
	GPIO_WritePin(3, 0);
	GPIO_WritePin(4, 0);
	GPIO_WritePin(5, 0);
	//Px_HS->1
	GPIO_WritePin(0, 1);
	GPIO_WritePin(2, 1);
	GPIO_WritePin(4, 1);
	//Px_LS->1
	GPIO_WritePin(1, 1);
	GPIO_WritePin(3, 1);
	GPIO_WritePin(5, 1);
	DELAY_US(100);//wait capacitors to charge
	//Px_LS->0
	GPIO_WritePin(1, 0);
	GPIO_WritePin(3, 0);
	GPIO_WritePin(5, 0);
	//Px_HS->0
	GPIO_WritePin(0, 0);
	GPIO_WritePin(2, 0);
	GPIO_WritePin(4, 0);
	//Px_HS->1
	GPIO_WritePin(0, 1);
	GPIO_WritePin(2, 1);
	GPIO_WritePin(4, 1);
}
void epwm_init()
{
    EALLOW;
    //
    // enable PWM1, PWM2 and PWM3
    //
	CpuSysRegs.PCLKCR2.bit.EPWM1=1;
	CpuSysRegs.PCLKCR2.bit.EPWM2=1;
	CpuSysRegs.PCLKCR2.bit.EPWM3=1;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // *****************************************
	// Inverter PWM configuration
	// ****************************************
	/* By default on soprano the PWM clock is divided by 2
	 * ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1
	 * Deadband needs to be 2.0us => 10ns*200=2us
	 */
    InitEPwm1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,200);
    InitEPwm1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,200);
    InitEPwm1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,200);

	// ********************************************************************
	//PWM 11 for syncing up the SD filter windows with motor control PWMs
	// ********************************************************************
	InitEPWM_1ch_UpCnt_CNF(11,INV_PWM_TICKS);



    // configure 2 and 3 as slaves
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm2Regs.TBPHS.bit.TBPHS    = 2;
	EPwm2Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm3Regs.TBPHS.bit.TBPHS    = 2;
	EPwm3Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm11Regs.TBCTL.bit.PHSEN  = TB_ENABLE;
	EPwm11Regs.TBPHS.bit.TBPHS  = 2;
	EPwm11Regs.TBCTL.bit.PHSDIR = TB_UP;

	EPwm11Regs.CMPC = (EPwm11Regs.TBPRD-640-640*0.5);
	EPwm11Regs.CMPA.bit.CMPA = 640+320+500; // 640+320;
	EPwm11Regs.CMPD = 0;
	// ***********************************
	// Set up GPIOs for PWM functions
	// **************************************
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	EDIS;

// Feedbacks OFFSET Calibration Routine
// ****************************************************************************
// ****************************************************************************
	EALLOW;
	  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

}
//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db)
{

	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period / 2; // PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0; // Phase is 0
	(*ePWM[n]).TBCTR = 0;         // Clear counter
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; //Up-down count mode
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT is 1
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;       //TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV)

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;  // Disable phase loading
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Load registers every ZERO
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

//	// Action Qualifier SubModule Registers
//	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;   //output low
//	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;     //output high
//
//	// Active high complementary PWMs - Set up the deadband
//	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
//	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;   //output low
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;     //output high

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_LO;
	(*ePWM[n]).DBRED.bit.DBRED = db;
	(*ePWM[n]).DBFED.bit.DBFED = db;

//    // Interrupt where we will change the Deadband
//    //
//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
//    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
//    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}
void InitEPWM_1ch_UpCnt_CNF(int16 n, Uint16 period) {
	EALLOW;
	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period-1; // PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UP;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA        = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.ZRO = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	(*ePWM[n]).DBRED.bit.DBRED = 0;
	(*ePWM[n]).DBRED.bit.DBRED = 0;
	EDIS;
}

void adc_init()
{
	//Write ADC configurations and power up the ADC for both ADC A
	Uint16 i;

	EALLOW;

	//write configurations for ADC-A
	// External REFERENCE must be provided
	AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;  //配置ADCA为12位转换精度
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;   //单端模式
	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;  //ADC中断脉冲位置发生在在转换结束时，在ADC结果锁存到结果寄存器之前的一个周期
	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//write configurations for ADC-B
	// External REFERENCE must be provided
	AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
	//Set pulse positions to late
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//write configurations for ADC-C
	// External REFERENCE must be provided
	AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
	//Set pulse positions to late
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//write configurations for ADC-D
	// External REFERENCE must be provided
	AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;
	//Set pulse positions to late
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for > 1ms to allow ADC time to power up
    DELAY_US(1000);
    EDIS;

    // 设置 ADCs 转换序列
    Uint16 acqps;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14; //12位单端模式下采样时间最快75ns,ACQPS>=14,
    }
    else { //resolution is 16-bit
        acqps = 63; //16位差分模式下采样时间最快320ns,ACQPS>=63
    }
    //Select the channels to convert and end of conversion flag
    //ADCA
//    EALLOW;
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin ADCINA0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin A1
//    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
//
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//    EDIS;
}
void adc_dac_configure()
{
	EALLOW;

	// Analog signals that are sampled
	//  CUR-U-MCU ADC-IN-C2
	//	CUR-V-MCU ADC-IN-A2
	//	CUR-W-MCU ADC-IN-B2
	//	TEMP-H-MCU ADC-IN-D3
	//	TEMP-D-MCU ADC-IN-D2
	//	TEMP-M-MCU ADC-IN-D1
	//	V24-MCU ADC-IN-D4
	//	V-DC-MCU ADC-IN-D0

	// On piccolo 133ns for ACQPS
	// hencce ACQPS on soprano is 133/5~30

	// Configure the SOC0 on ADC a-d
	//  CUR-V-MCU ADC-IN-A2
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 2;    // SOC0 will convert pin A2
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	// Configure the post processing block (PPB) to eliminate subtraction related calculation
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	CUR-W-MCU ADC-IN-B2
	// ********************************
	AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 2;    // SOC0 will convert pin B2
	AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	CUR-U-MCU ADC-IN-C2
	// ********************************
	AdccRegs.ADCSOC0CTL.bit.CHSEL     = 2;   // SOC0 will convert pin C2
	AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;   // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	//TODO trigger to be specify
	//	V-DC-MCU ADC-IN-D0
	// ********************************
	AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 0;    // SOC0 will convert pin D0
	AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C

	//	V24-MCU ADC-IN-D4
	// ********************************
	AdcdRegs.ADCSOC1CTL.bit.CHSEL     = 4;    // SOC1 will convert pin D4
	AdcdRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C

	//	TEMP-H-MCU ADC-IN-D3
	// ********************************
	AdcdRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin D3
	AdcdRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC2CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcdRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	TEMP-D-MCU ADC-IN-D2
	// ********************************
	AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 2;    // SOC3 will convert pin D2
	AdcdRegs.ADCSOC3CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC3CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB2CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcdRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	TEMP-M-MCU ADC-IN-D1
	// ********************************
	AdcdRegs.ADCSOC4CTL.bit.CHSEL     = 1;    // SOC4 will convert pin D1
	AdcdRegs.ADCSOC4CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC4CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB3CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcdRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// ******************************************************
	// static analog trim for all ADCs (A, B, C and D)
	// *******************************************************
	adc_cal=(int*)0x0000743F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000074BF;
	*adc_cal=0x7000;
	adc_cal=(int*)0x0000753F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000075BF;
	*adc_cal=0x7000;

	// Setting up link from EPWM to ADC
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
	EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

//	EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
//	EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
//	EPwm6Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

	// SETUP DACS
	DacaRegs.DACCTL.bit.DACREFSEL = REFERENCE_VREF;
//	DacaRegs.DACCTL.bit.LOADMODE  = 1;      // enable value change only on sync signal
	//Enable DAC output
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
//	DacaRegs.DACCTL.bit.SYNCSEL    = 5;     // sync sel 5 meanse sync from pwm 6
	DacaRegs.DACVALS.bit.DACVALS   = 1024;

	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//Enable DAC output
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacbRegs.DACVALS.bit.DACVALS   = 1024;

	DaccRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//Enable DAC output
	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
	DaccRegs.DACVALS.bit.DACVALS   = 1024;
	EDIS;
}

void enDat_init()
{
	//Initialization routine for endat operation - defined in endat.c
	//Configures the peripherals and enables clocks for required modules
	//Configures GPIO and XBar as needed for EnDat operation
	//Sets up the SPI peripheral in endat data structure and enables interrupt
		EnDat_Init();

	//A template for running all the EnDat21 commands
	//This is optional in real applications.
	//Function defined in endat_commands.c
		endat21_runCommandSet();

		if (ENCODER_TYPE == 22)
		{
	//A template for running all the EnDat22 commands
	//This is optional in real applications.
	//Function defined in endat_commands.c

			endat22_runCommandSet();
		}

	//Enables 2 additional datas in endat22 operation
	//This is also optional in real applications. Function defined in endat.c

		endat22_setupAddlData();

	//Peforms cable propagation delay calculation.
	//This is required for long cable lengths and higher EnDat Clock frequencies
	//Function defined in endat.c

		EnDat_initDelayComp();

	//Switch to high frequency - 8.3MHz	(=200/4*ENDAT_RUNTIME_FREQ_DIVIDER)
		PM_endat22_setFreq(ENDAT_RUNTIME_FREQ_DIVIDER);
		DELAY_US(800L); 	//Delay 800us
}

void uart_init()
{
	// Initialize GPIO
	EALLOW;
#if (BUILDTYPE==PYMBOARD)
	GpioCtrlRegs.GPEMUX1.bit.GPIO135 = 2;
	GpioCtrlRegs.GPEMUX1.bit.GPIO136 = 2;
	GpioCtrlRegs.GPEGMUX1.bit.GPIO135 = 1;
	GpioCtrlRegs.GPEGMUX1.bit.GPIO136 = 1;
	//launchpad
#elif (BUILDTYPE==LAUNCHPAD)
	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;
	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;
	GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 3;
	GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 3;
#endif
	EDIS;
//test fifo
	SciaRegs.SCIFFTX.all=0xE040;
	SciaRegs.SCIFFRX.all=0x2044;
	SciaRegs.SCIFFCT.all=0x0;
//end test
    // Initialize SCIA
 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.TXINTENA =1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	SciaRegs.SCIHBAUD.all    =0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
    SciaRegs.SCILBAUD.all    =53;

	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    // Enable the UART.
    UARTEnable(UARTA_BASE);

}
void param_init(){
// ****************************************************************************
// ****************************************************************************
// Initialize QEP module
// ****************************************************************************
// ****************************************************************************
	// Setup GPIO for QEP operation
	GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(20,0,1);

	GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(21,0,1);

	GPIO_SetupPinOptions(22, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(22,0,1);

	GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(23,0,1);

// ****************************************************************************
// ****************************************************************************
// Paramaeter Initialisation
// ****************************************************************************
// ****************************************************************************

	// Init QEP parameters
	qep1.LineEncoder = 1000; // these are the number of slots in the QEP encoder
	qep1.MechScaler  = _IQ30(0.25/qep1.LineEncoder);
	qep1.PolePairs   = POLES/2;
	qep1.CalibratedAngle = 0;
	QEP_INIT_MACRO(1,qep1)
	EQep1Regs.QEPCTL.bit.IEI = 0;        // disable POSCNT=POSINIT @ Index

//	// Init RESOLVER parameters
//	resolver1.StepsPerTurn = RESOLVER_STEPS_PER_TURN;
//	resolver1.MechScaler   =  1.0;       //_IQ30(1.0/resolver1.StepsPerTurn);
//	resolver1.PolePairs    = POLES/2;

//	baseParamsInit();                    // initialise all parameters
//	derivParamsCal();                    // set up derivative loop parameters
//	init_resolver_Float();

	// Initialize the Speed module for speed calculation from QEP/RESOLVER
	speed1.K1 = _IQ21(1/(BASE_FREQ*T));
	speed1.K2 = _IQ(1/(1+T*2*PI*5));      // Low-pass cut-off frequency
	speed1.K3 = _IQ(1)-speed1.K2;
	speed1.BaseRpm = 120*(BASE_FREQ/POLES);


    // Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

	// Init PI module for ID loop
	pi_id.Kp   = _IQ(1.0);//_IQ(3.0);
	pi_id.Ki   = _IQ(T/0.04);//0.0075);
	pi_id.Umax = _IQ(0.5);
	pi_id.Umin = _IQ(-0.5);

	// Init PI module for IQ loop
	pi_iq.Kp   = _IQ(1.0);//_IQ(4.0);
	pi_iq.Ki   = _IQ(T/0.04);//_IQ(0.015);
	pi_iq.Umax = _IQ(0.8);
	pi_iq.Umin = _IQ(-0.8);

	// Set mock REFERENCES for Speed and Iq loops
	SpeedRef = 0.05;

	//DEVKIT offset:1.65V
	offset_lemU  = 1.65;
	offset_lemW  = 1.65;
	offset_lemV  = 1.65;

//	for (OffsetCalCounter=0; OffsetCalCounter<20000; )
//	{
//		if(EPwm11Regs.ETFLG.bit.INT==1)
//		{
//			if(OffsetCalCounter>1000)
//			{
//				offset_lemU  = K1*offset_lemU + K2*(IFB_LEMU)*ADC_PU_SCALE_FACTOR;
//				offset_lemV  = K1*offset_lemV + K2*(IFB_LEMV)*ADC_PU_SCALE_FACTOR;
//				offset_lemW  = K1*offset_lemW + K2*(IFB_LEMW)*ADC_PU_SCALE_FACTOR;
//			}
//			EPwm11Regs.ETCLR.bit.INT=1;
//			OffsetCalCounter++;
//		}
//	}
	// Init FLAGS
	RunMotor = 0;
	// ********************************************
	// Init OFFSET regs with identified values
	// ********************************************
	EALLOW;
	AdcaRegs.ADCPPB1OFFREF = (unsigned int)(offset_lemV/3.0*4096.0);  // setting LEM Iv offset
	AdcbRegs.ADCPPB1OFFREF = (unsigned int)(offset_lemW/3.0*4096.0);  // setting LEM Iw offset
	AdccRegs.ADCPPB1OFFREF = (unsigned int)(offset_lemU/3.0*4096.0);
	EDIS;

// ****************************************************
// Initialize DATALOG module
// ****************************************************
	DLOG_4CH_F_init(&dlog_4ch1);
	dlog_4ch1.input_ptr1 = &DlogCh1;	//data value
	dlog_4ch1.input_ptr2 = &DlogCh2;
	dlog_4ch1.input_ptr3 = &DlogCh3;
	dlog_4ch1.input_ptr4 = &DlogCh4;
	dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
	dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
	dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
	dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
	dlog_4ch1.size = 150;
	dlog_4ch1.pre_scalar = 10;
	dlog_4ch1.trig_value = 0.01;
	dlog_4ch1.status = 2;

// ****************************************************
// Initialize MESSAGEs
// ****************************************************
	SETPOINTS_init(&SetPoints);
	ACTUALVALUE1_init(&ActualValues1);
	ACTUALVALUE2_init(&ActualValues2);
}

void HVDMC_Protection(void)
{
	EALLOW;

	// Configure GPIO used for Trip Mechanism

	//GPIO input for reading the status of the LEM-overcurrent macro block (active low), GPIO40
	//could trip PWM based on this, if desired
	// Configure as Input
	GpioCtrlRegs.GPBPUD.bit.GPIO40  = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO40  = 0; // set as input
	GpioCtrlRegs.GPBINV.bit.GPIO40  = 1;  //invert the input such that '0' is good and '1' is bad after inversion


	InputXbarRegs.INPUT1SELECT = 40; //Select GPIO40 as INPUTXBAR1

	//Clearing the Fault(active low), GPIO41,
	// Configure as Output
	GpioCtrlRegs.GPBPUD.bit.GPIO41  = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO41  = 1; // set as output
	GpioDataRegs.GPBSET.bit.GPIO41  = 1;

	//Forcing IPM Shutdown (Trip) using GPIO58 (Active high)
	// Configure as Output
	GpioCtrlRegs.GPBPUD.bit.GPIO58   = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX2.bit.GPIO58  = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO58   = 1; // set as output
	GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;

	// LEM Current phase V(ADC A2, COMP1) and W(ADC B2, COMP3), High Low Compare event trips
	cmpssConfig(&Cmpss1Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS1 - LEM V
	cmpssConfig(&Cmpss3Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS3 - LEM W

	// Shunt Current phase V(ADC A4, COMP2) and W(ADC C2, COMP6), High Low Compare event trips
//	cmpssConfig(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS2 - Shunt V
//	cmpssConfig(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS6 - Shunt U


	// Configure TRIP 4 to OR the High and Low trips from both comparator 1 & 3
	// Clear everything first
	EPwmXbarRegs.TRIP4MUX0TO15CFG.all  = 0x0000;
	EPwmXbarRegs.TRIP4MUX16TO31CFG.all = 0x0000;
	// Enable Muxes for ored input of CMPSS1H and 1L, i.e. .1 mux for Mux0
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0  = 1;  //cmpss1
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4  = 1;  //cmpss3
//    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2  = 1;  //cmpss2
//	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX10 = 1;  //cmpss6

	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX1 = 1;  //inputxbar1

	// Disable all the muxes first
	EPwmXbarRegs.TRIP4MUXENABLE.all = 0x0000;
	// Enable Mux 0  OR Mux 4 to generate TRIP4
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4  = 1;
//	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1;
//	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX10 = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1  = 1;

	EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm1Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm1Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm1Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm2Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm2Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm2Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm3Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm3Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm3Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm1Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm2Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm3Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop

	// What do we want the OST/CBC events to do?
	// TZA events can force EPWMxA
	// TZB events can force EPWMxB

	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	// Clear any spurious OV trip
	EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	EDIS;

//************************** End of Prot. Conf. ***************************//
}
// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************

//definitions for selecting DACH reference
#define REFERENCE_VDDA     0

//definitions for COMPH input selection
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//definitions for CTRIPH/CTRIPOUTH output selection
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

void cmpssConfig(volatile struct CMPSS_REGS *v, int16 Hi, int16 Lo)
{
	//Enable CMPSS
	v->COMPCTL.bit.COMPDACE = 1;
	//NEG signal comes from DAC for the low comparator
	v->COMPCTL.bit.COMPLSOURCE = NEGIN_DAC;
	//NEG signal comes from DAC for the high comparator
	v->COMPCTL.bit.COMPHSOURCE = NEGIN_DAC;
	//Use VDDA as the reference for comparator DACs
	v->COMPDACCTL.bit.SELREF = REFERENCE_VDDA;
	//Set DAC to H~75% and L ~25% values
	v->DACHVALS.bit.DACVAL = Hi;
	v->DACLVALS.bit.DACVAL = Lo;
	// comparator oputput is "not" inverted for high compare event
	v->COMPCTL.bit.COMPHINV = 0;
	// Comparator output is inverted for for low compare event
	v->COMPCTL.bit.COMPLINV = 1;

	// Configure Digital Filter
	//Maximum CLKPRESCALE value provides the most time between samples
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale;  //30;   /* Max count of 1023 */
	//Maximum SAMPWIN value provides largest number of samples
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;  //0x1F;
	//Maximum THRESH value requires static value for entire window
	//  THRESH should be GREATER than half of SAMPWIN
	v->CTRIPHFILCTL.bit.THRESH         = thresh;  //0x1F;
	//Reset filter logic & start filtering
	v->CTRIPHFILCTL.bit.FILINIT        = 1;

	// Configure CTRIPOUT path
	//Digital filter output feeds CTRIPH and CTRIPOUTH
	v->COMPCTL.bit.CTRIPHSEL           = CTRIP_FILTER;
	v->COMPCTL.bit.CTRIPOUTHSEL        = CTRIP_FILTER;

    // Make sure the asynchronous path compare high and low event
	// does not go to the OR gate with latched digital filter output
	v->COMPCTL.bit.ASYNCHEN = 0;
	v->COMPCTL.bit.ASYNCLEN = 0;
	//TODO Comparator hysteresis control , set to 2x typical value
	v->COMPHYSCTL.bit.COMPHYS = 2;
	// Dac value is updated on sysclock
	v->COMPDACCTL.bit.SWLOADSEL = 0;
	// ramp is bypassed
	v->COMPDACCTL.bit.DACSOURCE = 0;
	// Clear the latched comparator events
	v->COMPSTSCLR.bit.HLATCHCLR = 1;
	v->COMPSTSCLR.bit.LLATCHCLR = 1;

	return;
}
