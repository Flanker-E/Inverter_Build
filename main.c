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



/******************************************************************************
* Defines and macros
******************************************************************************/
#define DISABLE_IRQ()                   __asm__("wrteei 0")
#define ENABLE_IRQ()                    __asm__("wrteei 1")

#define CAN_TX_COUNTER					50   // fast ISR 100us, 50*100 us=5ms
#define CAN_OFFLINE_COUNTER				500   // fast ISR 100us, 500*100 us=50ms

#define BLINKY_LED_GPIO    31
#define DELAY (CPU_RATE/1000*6*510)  //Qual period at 6 samples

//TODO
#define ENCODER_TYPE	21
/******************************************************************************
* Local function prototypes
******************************************************************************/
extern void xcptn_xmpl(void);
static bool focCurrentLoop(pmsmFOC_t *ptr);
static bool faultDetect(pmsmFOC_t *ptr);

void InitLED();
void CAN_A_init();
//void InitEPwm1Example(void);
void epwm_init();
void InitEPwm1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
void InitEPWM_1ch_UpCnt_CNF(int16 n, Uint16 period);
//void InitEPwm2Example(void);
//void InitEPwm3Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);

void adc_init();
void adc_dac_configure();
void uart_init();
void enDat_init();
void param_init();
/* Interrupt functions */
//#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (FOC_Fast_ISR, LPI)

// Prototype statements for functions found within this file.
interrupt void FOC_Fast_ISR(void);// 10kHz interrupt.
//interrupt void ResolverISR(void);

//void scia_init();  //init the scia and uart param

/******************************************************************************
* Local variables
******************************************************************************/
static volatile unsigned char runState, runState_last;

static volatile uint32_t canTxCounter =0 ;
static volatile uint32_t canOfflineCounter =0 ;
static volatile bool canOfflineDetectOn	= false;    // flag for CAN offline detect on
static unsigned char count_state=0;

unsigned char ucTXMsgData[4] = {1,2,3,4};
unsigned char ucRXMsgData[4];
uint16_t crc5_self;
/*------------------------------------
 * FOC variables
 * ----------------------------------*/
pmsmFOC_t 			FOC;
Uint32 IsrTicker = 0;
int test=0;

void main(void)
{

//    unsigned int DCVoltage =0xF00F;
    unsigned char Byte1;
    unsigned char Byte2;

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

    //TODO GPIO&Understand PWM initializing
    epwm_init();


    //TODO can be replaced by Epwm CTU initializing
//    ctu0_init();

    //TODO understand&GPIO ADC DAC initializing
    adc_init();
    adc_dac_configure();

	//TODO GPIO Initialize UART
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

	PieCtrlRegs.PIEIER3.bit.INTx11 = 1;  // Enable PWM11INT in PIE group 3
//	PieCtrlRegs.PIEIER1.bit.INTx3  = 1;  // Enable ADCC1INT in PIE group 1

	EPwm11Regs.ETCLR.bit.INT=1;
	EDIS;


	//TODO set initial state for the state machines
	FOC.ctrlState.state   = reset;
	FOC.ctrlState.event   = e_reset;

	//TODO first call to state machines
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

        state_table[FOC.ctrlState.event][FOC.ctrlState.state](&FOC);

        GPIO_WritePin(BLINKY_LED_GPIO, 1);

        DELAY_US(50*500);

//        i++;if(i==200)i=0;
    }
}

/**************************************************************************//*!
@brief          CTU0-trigger0 interrupt service routine

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
	EINT;

	// Verifying the ISR
    IsrTicker++;


// =============================== LEVEL 1 ======================================
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level
// ==============================================================================

//TODO BUILD 1
#if (BUILDLEVEL == LEVEL1)

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
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//	There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

    park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
 	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
	DlogCh3 = svgen1.Tb;
	DlogCh4 = svgen1.Tc;

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
	DacbRegs.DACVALS.bit.DACVALS = (svgen1.Tb*0.5+0.5)*4096;
	DacaRegs.DACVALS.bit.DACVALS = (svgen1.Tc*0.5+0.5)*4096;
//	}

#endif // (BUILDLEVEL==LEVEL1)

// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
	DLOG_4CH_F_FUNC(&dlog_4ch1);

	EPwm11Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;
    EDIS;
//	//FOC.encoder.position=enDat_getPosition();
//
//	// get encoder position
//	encoder_calc_mec_radian(&FOC);
//	encoder_calc_mec_degree(&FOC);
//	encoder_calc_elec_degree(&FOC);
//	encoder_calc_RPM(&FOC, 10000);
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
//TODO configure LED
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

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;   //output low
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;     //output high

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
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
	//  CUR-U-MCU ADC-IN-A2
	//	CUR-V-MCU ADC-IN-B2
	//	CUR-W-MCU ADC-IN-C2
	//	TEMP-H-MCU ADC-IN-D3
	//	TEMP-D-MCU ADC-IN-D2
	//	TEMP-M-MCU ADC-IN-D1
	//	V24-MCU ADC-IN-D4
	//	V-DC-MCU ADC-IN-D0

	// On piccolo 133ns for ACQPS
	// hencce ACQPS on soprano is 133/5~30

	// Configure the SOC0 on ADC a-d
	//  CUR-U-MCU ADC-IN-A2
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 2;    // SOC0 will convert pin A4
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure the post processing block (PPB) to eliminate subtraction related calculation
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	CUR-V-MCU ADC-IN-B2
	// ********************************
	AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 2;    // SOC0 will convert pin B4
	AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	CUR-W-MCU ADC-IN-C2
	// ********************************
	AdccRegs.ADCSOC0CTL.bit.CHSEL     = 2;   // SOC0 will convert pin C15
	AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	//TODO trigger to be specify
	//	V-DC-MCU ADC-IN-D0
	// ********************************
	AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 0;    // SOC0 will convert pin D1
	AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C

	//	V24-MCU ADC-IN-D4
	// ********************************
	AdcdRegs.ADCSOC1CTL.bit.CHSEL     = 4;    // SOC1 will convert pin D1
	AdcdRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C

	//	TEMP-H-MCU ADC-IN-D3
	// ********************************
	AdcdRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin D1
	AdcdRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC2CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcdRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
	//	TEMP-D-MCU ADC-IN-D2
	// ********************************
	AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 2;    // SOC3 will convert pin D1
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
//    for(i=0;i<100;i++)
//    {
//    	voltage[i]=i;
//    }
//    for(i=0;i<100;i++)
//	{
//		voltage[i+100]=100-i;
//	}
//    i=0;
}
void param_init(){

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
}
