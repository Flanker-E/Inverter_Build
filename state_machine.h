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
* @author    r63172
* 
* @version   1.0.1.0
* 
* @date      Mar-31-2009
* 
* @brief     Header file for StateMachineFrame "c" project
*
*******************************************************************************
*
* Detailed Description of the file. If not used, remove the separator above.
*
******************************************************************************/

#ifndef _STATE_MACHINE_FRAME_H
#define _STATE_MACHINE_FRAME_H

/******************************************************************************
* Includes
******************************************************************************/
//#include "motor_structure.h"
#include "PMSM_struct.h"

/******************************************************************************
* Constants
******************************************************************************/
/*#define	TURNED_ON	0x1
#define	TURNED_OFF	0x0

#ifndef true
#define true  ((tBool)1)
#endif

#ifndef false
#define false ((tBool)0)
#endif*/

typedef void (*PFCN_VOID_PARAM_PMSM)(pmsmFOC_t *ptr); /* pointer to function with parameter*/

extern PFCN_VOID_PARAM_PMSM state_table[12][6];

#endif //_STATE_MACHINE_FRAME_H
