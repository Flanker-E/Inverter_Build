/*
 * MESSAGE_F.c
 *
 *  Created on: 2021Äê3ÔÂ21ÈÕ
 *      Author: Flanker
 */
#include "F28x_Project.h"
#include "MESSAGE_F.h"
#include "driverlib/uart.h"//UARTEnable

void SETPOINTS_init(SETPOINTS *v){
#if (MOTOR_NUM==FL)
	v->Address=0x02;
#elif (MOTOR_NUM==FR)
	v->Address=0x02;
#elif (MOTOR_NUM==FL)
	v->Address=0x02;
#else
	v->Address=0x02;
#endif
	v->Enable=0;
	v->ErrorReset=0;
	v->TargetTorque=0;
	v->TorqueLimitN=0;
	v->TorqueLimitP=0;
	v->Reserved=0;
}

void ACTUALVALUE1_init(ACTUALVALUES1 *v){
#if (MOTOR_NUM==FL)
	v->Address=0x00;
#elif (MOTOR_NUM==FR)
	v->Address=0x02;
#elif (MOTOR_NUM==FL)
	v->Address=0x02;
#else
	v->Address=0x02;
#endif
	v->ActualTorque=0;
	v->ActualVelocity=0;

	v->Diagnostic=0;
	v->Enabled=0;
	v->Error=0;
	v->DCVoltage=0x0;
}
void ACTUALVALUE2_init(ACTUALVALUES2 *v){
#if (MOTOR_NUM==FL)
	v->Address=0x04;
#elif (MOTOR_NUM==FR)
	v->Address=0x02;
#elif (MOTOR_NUM==FL)
	v->Address=0x02;
#else
	v->Address=0x02;
#endif
	v->TempIGBT=0;
	v->TempInverter=0;
	v->TempMotor=0;
	v->Reserved0=0;
	v->Reserved1=0;
}
void ACTUALVALUE_uart_TX(unsigned char *av,uint32_t UART_BASE, unsigned char address){
    int i;
	//address
	UARTCharPut(UART_BASE, address);
	//actual values
	for (i=0;i<8;i++){
		UARTCharPut(UART_BASE, av[i]);
	}
	UARTCharPut(UART_BASE, '\r');
	UARTCharPut(UART_BASE, '\n');
}
void ACTUALVALUE2_uart_TX(ACTUALVALUES2 *v,uint32_t UART_BASE){
    //address
	UARTCharPut(UART_BASE, v->Address);
	//TempMotor
	UARTCharPut(UART_BASE, (v->TempMotor&0xFF00)>>8);
	UARTCharPut(UART_BASE, v->TempMotor&0x00FF);
	//TempInverter
	UARTCharPut(UART_BASE, (v->TempInverter&0xFF00)>>8);
	UARTCharPut(UART_BASE, v->TempInverter&0x00FF);
	//TempIGBT
	UARTCharPut(UART_BASE, (v->TempIGBT&0xFF00)>>8);
	UARTCharPut(UART_BASE, v->TempIGBT&0x00FF);
	//Reserved
	UARTCharPut(UART_BASE, v->Reserved0);
	UARTCharPut(UART_BASE, v->Reserved1);
	UARTCharPut(UART_BASE, '\r');
	UARTCharPut(UART_BASE, '\n');
}
void ACTUALVALUE1_can_TX(ACTUALVALUES1 *v){

}
void ACTUALVALUE2_can_TX(ACTUALVALUES2 *v){

}



