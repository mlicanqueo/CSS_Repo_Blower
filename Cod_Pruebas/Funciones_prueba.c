/*
* Funciones_prueba.c
*
*Archivo que contiene las funciones principales usadas en el código.
*
*  Created on: 30-12-2024
*      Author: Matias_L
*/
//
// Included Files
//

#include "F28x_Project.h"


void ADC_initAdcABCD(void);



//
// ADC_initAdc - Initialize ADC A,B,C,D configurations and power it up
// Configure the ADC A,B,C,D to start sampling on an EPWM period
//
void ADC_initAdcABCD(void)
{
    EALLOW;
    // encender adca
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // Enciende el circuito análogo del ADC
    //
    // encender adcB
    //
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    // encender adcC
    //
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    // encender adcD
    //
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC

    //delay for > 1ms to allow ADC time to power up
    DELAY_US(1000);
    EDIS;
}


