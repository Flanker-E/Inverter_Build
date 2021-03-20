/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2008 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************//*!
*
* @file      state_machine.h
*
* @author    b15651
* 
* @version   1.0.1.0
* 
* @date      Jul-22-2011
* 
* @brief     Header file for StateMachineFrame "c" project
*
*******************************************************************************
*
* Detailed Description of the file. If not used, remove the separator above.
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "state_machine.h"


extern void stateReset(pmsmFOC_t *ptr);
extern void stateInit(pmsmFOC_t *ptr);
extern void stateCharge(pmsmFOC_t *ptr);
extern void stateReady(pmsmFOC_t *ptr);
extern void stateRun(pmsmFOC_t *ptr);
extern void stateFault(pmsmFOC_t *ptr);

PFCN_VOID_PARAM_PMSM state_table[12][6]={
    /* Actual state ->       'Reset'       'Init'           'Charge'         'Ready'         'Run'         'Fault' */
    /* e_reset          */ { stateReset,    stateReset,     stateReset,     stateReset,     stateReset,     stateFault},
    /* e_reset_done     */ { stateInit,     stateFault,     stateFault,     stateFault,     stateFault,     stateFault},
    /* e_init      	    */ { stateFault,    stateInit,      stateInit,      stateInit,      stateInit,      stateFault},
    /* e_init_done      */ { stateFault,    stateCharge,    stateFault,     stateFault,     stateFault,     stateFault},
    /* e_charge         */ { stateFault,    stateFault,     stateCharge,    stateFault,     stateFault,     stateFault},
    /* e_charge_done    */ { stateFault,    stateFault,     stateReady,     stateFault,     stateFault,     stateFault},
    /* e_ready          */ { stateFault,    stateFault,     stateFault,     stateReady,     stateFault,     stateFault},
    /* e_driver_enable  */ { stateFault,    stateFault,     stateFault,     stateRun,       stateFault,     stateFault},
    /* e_driver_disable */ { stateFault,    stateFault,     stateFault,     stateFault,     stateReady,     stateFault},
    /* e_run            */ { stateFault,    stateFault,     stateFault,     stateFault,     stateRun,     	stateFault},
    /* e_fault          */ { stateFault,    stateFault,     stateFault,     stateFault,     stateFault,     stateFault},
    /* e_fault_clear    */ { stateFault,    stateFault,     stateFault,     stateFault,     stateFault,     stateInit }
};



