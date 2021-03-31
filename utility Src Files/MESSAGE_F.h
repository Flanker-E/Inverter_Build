/*
 * MESSAGE_F.h
 *
 *  Created on: 2021Äê3ÔÂ21ÈÕ
 *      Author: Flanker
 */

#ifndef UTILITY_SRC_FILES_MESSAGE_F_H_
#define UTILITY_SRC_FILES_MESSAGE_F_H_

//Name    Setpoints        ActualValues1      ActualValues2
//Byte0   address          address            address
//Byte1   6En7ErrReset     6Enabled7Err       TempMotor0
//Byte2   TargetTorque0    DCVoltage0         TempMotor1
//Byte3   TargetTorque1    DCVoltage1         TempInverter0
//Byte4   TorqueLimitP0    ActualTorque0      TempInverter1
//Byte5   TorqueLimitP1    ActualTorque1      TempIGBT0
//Byte6   TorqueLimitN0    ActualVelocity0    TempIGBT1
//Byte7   TorqueLimitN1    ActualVelocity1    Reserved0
//Byte8   Reserved         Diagnostic         Reserved1
//Byte9   \r               \r                 \r
//Byte10  \n               \n                 \n

typedef struct{
	unsigned char Address;
	unsigned char Enable;
	unsigned char ErrorReset;
	short TargetTorque;
	short TorqueLimitP;
	short TorqueLimitN;
	unsigned char Reserved;
}SETPOINTS;

typedef struct{
	unsigned char Address;
	unsigned char Enabled;
	unsigned char Error;
	unsigned short DCVoltage;
	short ActualTorque;
	short ActualVelocity;
	unsigned char Diagnostic;
}ACTUALVALUES1;

typedef struct{
	unsigned char Address;
	unsigned short TempMotor;
	unsigned short TempInverter;
	unsigned short TempIGBT;
	unsigned char Reserved0;
	unsigned char Reserved1;
}ACTUALVALUES2;

typedef struct{
	unsigned char Byte0_address; //address
	unsigned char Byte1;
	unsigned char Byte2;
	unsigned char Byte3;
	unsigned char Byte4;
	unsigned char Byte5;
	unsigned char Byte6;
	unsigned char Byte7;
	unsigned char Byte8;
	unsigned char Byte9_r; // \r
	unsigned char Byte10_n; // \n
}MESSAGE;
//*********** Function Declarations *******//
void SETPOINTS_init(SETPOINTS *v);
void ACTUALVALUE1_init(ACTUALVALUES1 *v);
void ACTUALVALUE2_init(ACTUALVALUES2 *v);
void ACTUALVALUE1_uart_TX(ACTUALVALUES1 *v,uint32_t UART_BASE);
void ACTUALVALUE2_uart_TX(ACTUALVALUES2 *v,uint32_t UART_BASE);
void ACTUALVALUE1_can_TX(ACTUALVALUES1 *v);
void ACTUALVALUE2_can_TX(ACTUALVALUES2 *v);


#endif /* UTILITY_SRC_FILES_MESSAGE_F_H_ */
