/*
* ACDC_drive.c
*
*  Created on: 20-07-2023
*      Author: Matias_L
*/
//
// Included Files
//
#include "task_file.h"
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include <CLAmath.h>
#include <math.h>
//
// Defines
//
#define socA0 0
#define socA1 1
#define socA2 2
#define socA3 3
#define socA4 4
#define socA5 5

#define socB0 6
#define socB1 7
#define socB2 8
#define socB3 9
#define socB4 10
#define socB5 11

#define socC0 12
#define socC1 13
#define socC2 14
#define socC3 15
#define socC4 16
#define socC5 17

#define socD0 17


//#define socB0 0
//#define socB1 1
//#define socB2 2
//#define socB3 3
//#define socB4 4
//#define socB5 5
//
//#define socC0 0
//#define socC1 1
//#define socC2 2
//#define socC3 3
//#define socC4 4
//#define socC5 5

//
//    SetupSOC(socA0, 0, acqps, 5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA2, 2, acqps, 5); // SOC2 convierte ADC-A2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA3, 3, acqps, 5); // SOC3 convierte ADC-A3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA5, 5, acqps, 5); // SOC4 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB0, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB1, 1, acqps, 5); // SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB2, 2, acqps, 5); // SOC8 convierte ADC-B2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB3, 3, acqps, 5); // SOC8 convierte ADC-B3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB4, 4, acqps, 5); // SOC8 convierte ADC-B4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB5, 5, acqps, 5); // SOC8 convierte ADC-B5 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC0, 2, acqps, 5); // SOC8 convierte ADC-C2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC1, 3, acqps, 5); // SOC8 convierte ADC-C3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC2, 4, acqps, 5); // SOC8 convierte ADC-C4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC3, 5, acqps, 5); // SOC8 convierte ADC-C5 y hace trigger desde ePWM1(5) con una ventana de acqps




//#define PWM_TBPRD 1562 // Frecuencia de 2khz
//#define PWM_CMPA 781   // Frecuencia 2khz

#define PWM_TBPRD 97656 // Frecuencia de 2khz
#define PWM_CMPA 48828  // Frecuencia 2khz

//   Globales
//------------maquina-de-estados-------------//
uint16_t estado   = 0;   // Estado inicial de la máquina de estados
uint16_t boton_1  = 0;   // Condición de salto desde estado iddle a estado sync_pll
uint16_t boton_2  = 0;   // Condición de salto desde estado sync_pll a estado precarga
uint16_t fault    = 0;

float vg_d, vg_q, i_out; // Variables máquina de estados
float va, vb, vc;        // Variables máquina de estados
float vdc;               // Variables máquina de estados
//-------------------------------------------//

uint16_t test_1    = 0;


//
// Defines
//
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF
#define EPWM1_MIN_DB   0
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   0
#define DB_UP          1
#define DB_DOWN        0


#define sw1_output         39 // Salida para precarga |  Pin en RTBOX es DI14
#define sw2_output         44 // Salida para precarga |  Pin en RTBOX es DI15


//
// Globals
//
Uint32 EPwm1TimerIntCount;
Uint32 EPwm2TimerIntCount;
Uint32 EPwm3TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint16 EPwm2_DB_Direction;
Uint16 EPwm3_DB_Direction;

extern uint16_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
extern uint16_t CLA1mathTablesRunStart, CLA1mathTablesLoadStart;
extern uint16_t CLA1mathTablesLoadSize;
//
// Function Prototypes
//
void CLA_configClaMemory(void);
void configCLAMemory2(void);
void CLA_initCpu1Cla1(void);
void EPWM_initEpwm(void);
void ADC_initAdcABCD(void);
void SetupADCEpwm2(void);
void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel);

__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();

//-----------ejemplos------------------//
void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCePWM(void);
//-----------ejemplos------------------//


//__interrupt void epwm1_isr(void);
//__interrupt void epwm1_isr();
__interrupt void adca1_isr2(void);
__interrupt void adca1_isr2();
uint16_t ADCread0;
uint16_t ADCread1;
uint16_t ADCread2;
uint16_t ADCread3;
uint16_t ADCread4;
uint16_t ADCread5;
uint16_t ADCread6;
uint16_t ADCread7;
uint16_t ADCread8;
uint16_t ADCread9;
uint16_t ADCread10;
uint16_t ADCread11;
uint16_t ADCread12;
uint16_t ADCread13;
uint16_t ADCread14;
uint16_t ADCread15;


void InitEPwm1Example(void);
Uint32 EPwm1TimerIntCount;
Uint16 EPwm1_DB_Direction;

//
// Start of main
//
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
     InitSysCtrl();
//
// Step 2. enable PWM1 and PWM2 and their GPIOs
//
     InitGpio();
     CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; // PWM1A y PWM1B se utilizan de forma interna para hacer trigger al adc
     CpuSysRegs.PCLKCR2.bit.EPWM3 = 1; // PWM3A y PWM3B se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM4 = 1; // PWM4A y PWM4B se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM7 = 1; // PWM7A y PWM7B se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM8 = 1; // PWM8A y PWM8B se utilizan de forma Externa, salidas hacia inversor
     CpuSysRegs.PCLKCR2.bit.EPWM11 = 1; // PWM11A y PWM11B se utilizan de forma Externa, salidas hacia inversor
     CpuSysRegs.PCLKCR2.bit.EPWM12 = 1; // PWM12A y PWM12B se utilizan de forma Externa, salidas hacia inversor
//
// For this case just init GPIO pins
// These functions are in the F2837xD_EPwm.c file, do not work for ePWM9-12.
//
    InitEPwm1Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    // Inicializacion de las PWM usando registros directos, ya que a través de la función
    // InitEPwm11Gpio y InitEPwm12Gpio, no funciono por problema de la librería.
EALLOW;
// Inicialización ePWM11
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EPWM11A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EPWM11B)
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EPWM11A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EPWM11B
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EPWM11A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EPWM11B
// Inicialización ePWM12
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;    // Disable pull-up on GPIO22 (EPWM12A)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;    // Disable pull-up on GPIO23 (EPWM12B)
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EPWM12A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EPWM12B
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EPWM12A
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EPWM12B
EDIS;

//  Encendido y apagado de un pin
    GPIO_SetupPinMux(sw1_output, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(sw1_output, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(sw2_output, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(sw2_output, GPIO_OUTPUT, GPIO_PUSHPULL);
//
// Encendido y apagado de 2 leds, para comprobar que programa se cargó de forma correcta.
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
//

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;
//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();
//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;
//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();
//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
//    EALLOW; // This is needed to write to EALLOW protected registers
//    PieVectTable.EPWM1_INT = &epwm1_isr;
//    EDIS;   // This is needed to disable write to EALLOW protected registers
//
    EALLOW;
//    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.ADCA1_INT = &adca1_isr2; //function for ADCD interrupt 1

    EDIS;
////
// Step 5. Configure the ADC A,B,C,D to start sampling on an EPWM period
//
    ADC_initAdcABCD();
//
// Step 6. Configure EPWM to trigger ADC every 20us
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;
    EPWM_initEpwm();
//
// Configurar los ADC para ser disparados con el ePWM
//
    SetupADCEpwm2();
//
// Step 7. Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
//
// enable PIE interrupt
//
//    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;


//
// Step 8. Turn on the EPWM. Se configura para que el ePWM1A haga trigger a la conversión de los ADC.
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
//    EPwm1Regs.ETSEL.bit.SOCBEN  = 1; // Habilita pulso SOCB  (EPWMxSOCB)
    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode

    EPwm2Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode

    EPwm3Regs.TBCTL.bit.CTRMODE = 2;

    EPwm4Regs.TBCTL.bit.CTRMODE = 2;

    EPwm5Regs.TBCTL.bit.CTRMODE = 2;

    EPwm6Regs.TBCTL.bit.CTRMODE = 2;

    EPwm7Regs.TBCTL.bit.CTRMODE = 2;
    EDIS;

    for(;;)
    {
        asm ("          NOP");
    }


{
    }
{

    //        GPIO_WritePin(sw1_output, 1); // encendido del pin
    //        DELAY_US(1000*500);
    ////        Cla1ForceTask6();
    //        GPIO_WritePin(sw1_output, 0); // apagado del pin
    //        DELAY_US(1000*500);


//            switch (estado)
//            {
//            case idle:
//            {
//
//
//            }
//            break;
//
//            case pre_charge:
//            {
//
//            }
//            break;
//
//            case boost_ON:
//            {
//
//            }
//            break;
//
//            case normal_mode:
//            {
//
//            }
//            break;
//
//            case apagado:
//            {
//
//            }
//            break;
//
//            case fault_mode:
//            {
//
//            }
//            break;
//            }
}
}


//
// ADC_initAdc - Initialize ADC A,B,C,D configurations and power it up
//
void ADC_initAdcABCD(void)
{
    EALLOW;
    //
    //write configurations
    //
    // encender adca
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //
    //Set pulse positions to late
    //
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión
    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    // encender adcB
    //
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //
    //Set pulse positions to late
    //
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //
    // encender adcC
    //
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //
    //Set pulse positions to late
    //
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //
    // encender adcD
    //
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //
    //Set pulse positions to late
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión

    //
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //delay for > 1ms to allow ADC time to power up
    DELAY_US(1000);
    EDIS;
}

//
// EPWM_initEpwm - Initialize EPWM1, EPWM2, EPWM3, EPWM4 settings
//
// PWM1A y PWM1B se utilizan de forma interna para hacer trigger al adc
// PWM2A y PWM2B se utilizan de forma Externa, salidas hacia boost converter
// PWM3A y PWM3B se utilizan de forma Externa, salidas hacia boost converter
// PWM4A y PWM4B se utilizan de forma Externa, salidas hacia boost converter
// PWM5A y PWM5B se utilizan de forma Externa, salidas hacia inversor
// PWM6A y PWM6B se utilizan de forma Externa, salidas hacia inversor
// PWM7A y PWM7B se utilizan de forma Externa, salidas hacia inversor
//

void EPWM_initEpwm(void)
{
    EALLOW;
//    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
//    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
//    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento

    EPwm1Regs.TBPRD               = PWM_TBPRD;  // Frecuencia de 5khz
    EPwm1Regs.TBPHS.bit.TBPHS     = 0;           // Phase is 0
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 4
    EPwm1Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module, master module

    EPwm1Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//
//// Configuracion Deadband - 500ns de FED y RED
//    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//    EPwm1Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
//    EPwm1Regs.DBFED.bit.DBFED = 0x19; // 25 decimal

//     Configuración eWPM2
//    EPwm2Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
//    EPwm2Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
//    EPwm2Regs.TBPRD               = PWM_TBPRD;  // Frecuencia de 5khz
//    EPwm2Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
//    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x2;  // Divide el reloj en 4
//    EPwm2Regs.TBCTL.bit.CLKDIV    = 0x2;  // Divide el reloj en 4
//
//    EPwm2Regs.CMPA.bit.CMPA = PWM_CMPA;
//    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
//    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
//    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

    // Configuración eWPM3
    //
    EPwm3Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0;           // Phase is 0

    EPwm3Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm3Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm3Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm3Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm3Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm3Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm3Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
    //
    // Setup compare
    //
    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;

    EDIS;
}

void SetupADCEpwm2(void)
{
    Uint16 acqps;
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
//    acqps = 19; //1us -> (100e-9)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
    acqps = 500;


    EALLOW;
    SetupSOC(socA0, 0, acqps, 5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socA1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socA2, 2, acqps, 5); // SOC2 convierte ADC-A2 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socA3, 3, acqps, 5); // SOC3 convierte ADC-A3 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socA4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socA5, 5, acqps, 5); // SOC4 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socB0, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socB1, 1, acqps, 5); // SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socB2, 2, acqps, 5); // SOC8 convierte ADC-B2 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socB3, 3, acqps, 5); // SOC8 convierte ADC-B3 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socB4, 4, acqps, 5); // SOC8 convierte ADC-B4 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socC0, 2, acqps, 5); // SOC8 convierte ADC-C2 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socC1, 3, acqps, 5); // SOC8 convierte ADC-C3 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socC2, 4, acqps, 5); // SOC8 convierte ADC-C4 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socC3, 5, acqps, 5); // SOC8 convierte ADC-C5 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(socD0, 0, acqps, 5); // SOC0 convierte ADC-D0 y hace trigger desde ePWM1(5) con una ventana de acqps

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5; //fin de SOCA5 genera INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag
    EDIS;
}

void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel)
{
    switch(soc)
        {
            case socA0:
            {
                AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
                AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA1:
            {
                AdcaRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
                AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA2:
            {
                AdcaRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC2 will convert pin A2
                AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA3:
            {
                AdcaRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC3 will convert pin A3
                AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA4:
            {
                AdcaRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC4 will convert pin A4
                AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA5:
            {
                AdcaRegs.ADCSOC5CTL.bit.CHSEL = channel;  //SOC5 will convert pin A5
                AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB0:
            {
                AdcbRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC6 will convert pin B0
                AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB1:
            {
                AdcbRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC7 will convert pin B1
                AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB2:
            {
                AdcbRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC8 will convert pin B2
                AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB3:
            {
                AdcbRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC8 will convert pin B3
                AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB4:
            {
                AdcbRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC8 will convert pin B4
                AdcbRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socB5:
            {
                AdcbRegs.ADCSOC5CTL.bit.CHSEL = channel;  //SOC8 will convert pin B5
                AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socC0:
            {
                AdccRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC8 will convert pin C2
                AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
                AdccRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socC1:
            {
                AdccRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC8 will convert pin C3
                AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
                AdccRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socC2:
            {
                AdccRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC8 will convert pin C4
                AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
                AdccRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socC3:
            {
                AdccRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC8 will convert pin C5
                AdccRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
                AdccRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socD0:
            {
                AdcdRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin D0
                AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
                AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }

        }
    }

//---------------------------------------------------------------//


//void ConfigureADC(void)
//{
//    EALLOW;
//    //
//    //write configurations
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    //
//    //Set pulse positions to late
//    AdcaRegs.ADCCTL1.bit. = 1; // Interrupción se genera al final de la conversión
//    //
//    //power up the ADC
//    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
//    //
//    //delay for > 1ms to allow ADC time to power up
//    DELAY_US(1000);
//    EDIS;
//}
//
//void ConfigureEPWM(void)
//{
//    EALLOW;
//
//    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCASEL   = 0x4;  // Enable event time-base counter equal to CMPA when the timer is incrementing or CMPC when the timer is incrementing
//    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
//
////    EPwm1Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
////    EPwm1Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
////    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
//
//    EPwm1Regs.CMPA.bit.CMPA = 3125/2;            // Contador q genera un evento
//    EPwm1Regs.TBPRD = 3125;                      // configura la frecuencia
//    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
//
//    EDIS;
//}
//
//void SetupADCePWM(void)
//{
//    EALLOW;
//    Uint16 acqps;
//
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;//trigger on ePWM1 SOCA/C
//
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//
//    EDIS;
//}
//
//
// adca1_isr --
//
interrupt void adca1_isr2(void)
{

    test_1    = 2;
    ADCread0 = (AdcaResultRegs.ADCRESULT0);
    ADCread1 = (AdcaResultRegs.ADCRESULT1);
    ADCread2 = (AdcaResultRegs.ADCRESULT2);
    ADCread3 = (AdcaResultRegs.ADCRESULT3);
    ADCread4 = (AdcaResultRegs.ADCRESULT4);
    ADCread5 = (AdcaResultRegs.ADCRESULT5);
    ADCread6 = (AdcbResultRegs.ADCRESULT0);
    ADCread7 = (AdcbResultRegs.ADCRESULT1);
    ADCread8 = (AdcbResultRegs.ADCRESULT2);
    ADCread9 = (AdcbResultRegs.ADCRESULT3);
    ADCread10 =(AdcbResultRegs.ADCRESULT4);
    ADCread11 =(AdcbResultRegs.ADCRESULT5);
    ADCread12 =(AdccResultRegs.ADCRESULT2);
    ADCread13 =(AdccResultRegs.ADCRESULT3);
    ADCread14 =(AdccResultRegs.ADCRESULT4);
    ADCread15 =(AdccResultRegs.ADCRESULT5);


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
//        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

//SetupSOC(soc0, 0, acqps, 5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc2, 2, acqps, 5); // SOC2 convierte ADC-A2 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc3, 3, acqps, 5); // SOC3 convierte ADC-A3 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc5, 5, acqps, 5); // SOC4 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc6, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc7, 1, acqps, 5); // SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc8, 2, acqps, 5); // SOC8 convierte ADC-B2 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc9, 3, acqps, 5); // SOC8 convierte ADC-B3 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc10, 4, acqps, 5); // SOC8 convierte ADC-B4 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc11, 5, acqps, 5); // SOC8 convierte ADC-B5 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc12, 2, acqps, 5); // SOC8 convierte ADC-C2 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc13, 3, acqps, 5); // SOC8 convierte ADC-C3 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc14, 4, acqps, 5); // SOC8 convierte ADC-C4 y hace trigger desde ePWM1(5) con una ventana de acqps
//SetupSOC(soc15, 5, acqps, 5); // SOC8 convierte ADC-C5 y hace trigger desde ePWM1(5) con una ventana de acqps
//
//// End of File
//
