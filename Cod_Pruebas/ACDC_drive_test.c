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
//#include "F2837xD_Cla_defines.h"
#include <CLAmath.h>
//#include <math.h>
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




#define PWM_TBPRD 625 // Frecuencia de 5000 hz
#define PWM_CMPA 313  // Frecuencia 5000 hz
//#define PWM_TBPRD 31250 // Frecuencia de 100 hz
//#define PWM_CMPA  15625 // Frecuencia 100 hz
//#define PWM_TBPRD 3125 // Frecuencia de 1000 hz
//#define PWM_CMPA  1562 // Frecuencia 1000 hz
//#define PWM_TBPRD 312 // Frecuencia de 10000 hz
//#define PWM_CMPA  156 // Frecuencia 10000 hz
//uint16_t PWM_CMPA_mod = 0;  // Frecuencia 100 hz

//-------------Modo-cuenta-Down--------------//
#define PWM_TBPRD_2 1249 // Frecuencia de 5000 hz
#define PWM_CMPA_2  550 // Frecuencia 5000 hz
//-------------------------------------------//

#define PHASE2 PWM_TBPRD/3  // Frecuencia 100 hz
#define PHASE3 (PWM_TBPRD/3)*2 // Frecuencia 100 hz

//#define PHASE3 20834  // Frecuencia 100 hz

//#define PWM_TBPRD 97656 // Frecuencia de 2khz
//#define PWM_CMPA 48828  // Frecuencia 2khz

//-------------------------------------------//
// Variables tareas CLA
//
//------------Variables-de-prueba------------//
#pragma DATA_SECTION(aaa,"CpuToCla1MsgRAM");
float aaa;
#pragma DATA_SECTION(resultado,"Cla1ToCpuMsgRAM");
float resultado[15];
#pragma DATA_SECTION(resultado2,"Cla1ToCpuMsgRAM");
double resultado2;
#pragma DATA_SECTION(resultado3,"Cla1ToCpuMsgRAM");
double resultado3;
#pragma DATA_SECTION(PWM_CMPA_mod,"Cla1ToCpuMsgRAM");
int16 PWM_CMPA_mod = 0;
#pragma DATA_SECTION(V_low,"CpuToCla1MsgRAM");
float V_low;
#pragma DATA_SECTION(V_high,"CpuToCla1MsgRAM");
float V_high;
long double asdf = 2;

int16 a1;
int16 b1;
float c;
float d;

//-------------------------------------------//
//----------Controlador-de-voltaje-----------//
#pragma DATA_SECTION(x_ant_v_l,"CpuToCla1MsgRAM");
float x_ant_v_l;
#pragma DATA_SECTION(x_ant_v_g,"Cla1ToCpuMsgRAM");
float x_ant_v_g;
#pragma DATA_SECTION(x_v_l,"CpuToCla1MsgRAM");
float x_v_l;
#pragma DATA_SECTION(x_v_g,"Cla1ToCpuMsgRAM");
float x_v_g;
//-------------------------------------------//
//---------Controlador-de-corriente----------//
//#pragma DATA_SECTION(adc_read,"Cla1ToCpuMsgRAM");
//float adc_read [8];
#pragma DATA_SECTION(i_count_g,"Cla1ToCpuMsgRAM");
uint16_t i_count_g;
#pragma DATA_SECTION(i_count_l,"CpuToCla1MsgRAM");
uint16_t i_count_l;

#pragma DATA_SECTION(x_ant_i_l,"CpuToCla1MsgRAM");
float x_ant_i_l [3];
#pragma DATA_SECTION(x_ant_i_g,"Cla1ToCpuMsgRAM");
float x_ant_i_g [3];

#pragma DATA_SECTION(x_i_l,"CpuToCla1MsgRAM");
float x_i_l [3];
#pragma DATA_SECTION(x_i_g,"Cla1ToCpuMsgRAM");
float x_i_g [3];
//-------------------------------------------//
//-------vector-guardado-variables-----------//
void corrimiento_izq(float v1);
#define ADC_BUF_PREV 1000
float AdcBuf_ant_fall[ADC_BUF_PREV];
float AdcBuf_ant_fall2[ADC_BUF_PREV];


//-------------------------------------------//

int16 CMPA_Boost_a;     //CMPA Boost pierna a
int16 CMPA_Boost_b;     //CMPA Boost pierna b
int16 CMPA_Boost_c;     //CMPA Boost pierna c

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
//---------Controlador-de-corriente----------//

uint16_t i = 0;
uint16_t ii = 0;
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
//     InitGpio();
     CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; //  PWM1A y PWM1B   se utilizan de forma interna para hacer trigger al adc
     CpuSysRegs.PCLKCR2.bit.EPWM3 = 1; //  PWM3A y PWM3B   se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM4 = 1; //  PWM4A y PWM4B   se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;  // PWM7A y PWM7B   se utilizan de forma Externa, salidas hacia boost converter
     CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;  // PWM8A y PWM8B   se utilizan de forma Externa, salidas hacia inversor
     CpuSysRegs.PCLKCR2.bit.EPWM11 = 1; // PWM11A y PWM11B se utilizan de forma Externa, salidas hacia inversor
     CpuSysRegs.PCLKCR2.bit.EPWM12 = 1; // PWM12A y PWM12B se utilizan de forma Externa, salidas hacia inversor
//
// For this case just init GPIO pins for ePWM1, ePWM2
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
EALLOW;
// Estos registros se modifican de forma directa ya que la función macro no permite inicializarlos de esa forma.
//// Inicialización ePWM8
//    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up on GPIO14 (EPWM8A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    // Disable pull-up on GPIO15 (EPWM8B)
//    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
//    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
//    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;   // Configure GPIO15 as EPWM8B
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

////  Encendido y apagado de los 2 leds y 2 GPIO
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(35, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(35, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

////  Encendido y apagado de un pin
//    GPIO_SetupPinMux(sw1_output, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(sw1_output, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(sw2_output, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(sw2_output, GPIO_OUTPUT, GPIO_PUSHPULL);
////
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
//    PieVectTable.ADCA1_INT = &adca1_isr2; //function for ADCD interrupt 1
    PieVectTable.ADCA1_INT = &adca1_isr2; //function for ADCA interrupt 3

    EDIS;
////
// Step 5. Configure the ADC A,B,C,D to start sampling on an EPWM period
//
    ADC_initAdcABCD();
//
// Step 6. Configure EPWM to trigger ADC every 20us
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
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
//    IER |= M_INT10; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
//
// enable PIE interrupt
//
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//
// Step 8. Turn on the EPWM. Se configura para que el ePWM1A haga trigger a la conversión de los ADC.
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EPwm3Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)

    //    EPwm1Regs.ETSEL.bit.SOCBEN  = 1; // Habilita pulso SOCB  (EPWMxSOCB)

    EPwm1Regs.TBCTL.bit.CTRMODE  = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
    EPwm3Regs.TBCTL.bit.CTRMODE  = TB_COUNT_DOWN;
//    EPwm4Regs.TBCTL.bit.CTRMODE  = TB_COUNT_DOWN; // Contador entra en up-down count mode
//    EPwm7Regs.TBCTL.bit.CTRMODE  = TB_COUNT_DOWN; // Contador entra en up-down count mode
//    EPwm8Regs.TBCTL.bit.CTRMODE  = TB_COUNT_DOWN; // Contador entra en up-down count mode
//    EPwm11Regs.TBCTL.bit.CTRMODE = TB_COUNT_DOWN; // Contador entra en up-down count mode
//    EPwm12Regs.TBCTL.bit.CTRMODE = TB_COUNT_DOWN; // Contador entra en up-down count mode

//    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
//    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
//    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
//    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
//    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
//    EPwm11Regs.TBCTL.bit.CTRMODE= TB_COUNT_UPDOWN; // Contador entra en up-down count mode
//    EPwm12Regs.TBCTL.bit.CTRMODE= TB_COUNT_UPDOWN; // Contador entra en up-down count mode
    EDIS;

// Configure the CLA memory spaces first followed by
// the CLA task vectors.
    configCLAMemory2();
    CLA_initCpu1Cla1();
//
//  Initial parameters
    estado = 1;
//----------Controlador-de-voltaje-----------//
    x_v_l       = 0;
    x_ant_v_l   = 0;
//------------------------------------------//
//---------Controlador-de-corriente----------//
    i_count_g = 0;
    i_count_l = 0;

    for (i = 0; i < 2; i++)
        {
            x_ant_i_l[i] = 0;
            x_ant_i_g[i] = 0;
            x_i_l[i] = 0;
            x_i_g[i] = 0;
        }
    for (ii = 0; ii<(ADC_BUF_PREV); ii++)
    {
        AdcBuf_ant_fall[ii]=0;
        AdcBuf_ant_fall2[ii]=0;
    }
ii = 0;
//------------------------------------------//
    //
//    for(;;)
//    {
//    }
//        asm ("          NOP");

        if (GpioDataRegs.GPBDAT.bit.GPIO35)
        {
            GPIO_WritePin(31, 1);
        }
        else
        {
            GPIO_WritePin(31, 0);
        }

        if (GpioDataRegs.GPBDAT.bit.GPIO60)
        {
            GPIO_WritePin(34, 1);
        }
        else
        {
            GPIO_WritePin(34, 0);
        }

        //
        // Turn on LED
        //
//        GPIO_WritePin(31, 1);
//        GPIO_WritePin(34, 0);
        //
        // Delay for a bit.
        //
//        DELAY_US(1000*500);
        //
        // Turn off LED
        //
//        GPIO_WritePin(31, 0);
//        GPIO_WritePin(34, 1);
        //
        // Delay for a bit.
        //
//        DELAY_US(1000*500);

//{

    //        GPIO_WritePin(sw1_output, 1); // encendido del pin
    //        DELAY_US(1000*500);
    ////        Cla1ForceTask6();
    //        GPIO_WritePin(sw1_output, 0); // apagado del pin
    //        DELAY_US(1000*500);

//        double Imax[3], Ik_ref[3], Dk[3], Xk[3], Imax_ref[3], Imax_corregido[3];
//        int N = 0;
//
//        double V_low, V_high, Iavg_ref, Von, Voff, numerador, denominador, raiz_arg, Idc_inv_est, D_dcm, D_ccm, D_off_dcm;
//

//        V_high = 500;
//        V_low =350;

//        ////////////////////////////////////////
//        // Variables intermedias
//        ////////////////////////////////////////
//        double E;
//        double F;
//        double G;
//        double Z;
//
//        double R;
//        double S;
//        double T;
//
//        double a;
//        double b;
//        double delta;
//        float  a0 = 0;
//        double a2;
//        double a3;
//        double Vss;
//        double Imax_ref_ccm;
//        double Imax_ref_dcm;
//        double Imax_bound;
//
////        double Tpwm_dcdc = 0.0002;
////        float alpha = 0.0000014179699;
//        float alpha = 0.0000014179699;
//
//        double L0 = 0.000522393871688478;
//        V_low = 300;
//        V_high = 600;
//
//        Von = V_low;
//        Voff = V_low - V_high;
//        Vss = Von*Voff / ( Voff - Von);
        ////////////////////////////////////////////////////
        // Conversión de corriente promedio estacionaria
        // de referencia a corriente máxima
        ////////////////////////////////////////////////////

//        // Prealimentación
//        Iavg_ref = Ik_ref[N]*V_high/V_low + Idc_inv_est/3;
//        Iavg_ref = (Iavg_ref > 0)? Iavg_ref: 0;

        ////////////////////////////////////////////////////
        // Conversión de corriente promedio estacionaria
        // de referencia a corriente máxima
        ////////////////////////////////////////////////////

        // Cálculo de referencia de corriente CCM
//        a0 = alpha * Tpwm_dcdc * Vss * ((4.0/3.0) * alpha * Tpwm_dcdc * Vss - (3/2) * pow(L0 - alpha * Iavg_ref, 2));
//        a0 = alpha * Tpwm_dcdc * Vss* ((4.0/3.0)* alpha * Tpwm_dcdc * Vss - (3.0/2.0) * (L0 - alpha * Iavg_ref)*(L0 - alpha * Iavg_ref));
//        a2 = 2*alpha*Tpwm_dcdc*Vss;
//        a3 = 1/Tpwm_dcdc;
//        R = Tpwm_dcdc/1;
//        a3 = alpha * Iavg_ref - L0;
//        E = (1/3.0) * (-12*a0 - a2*a2);
//        F = (1/27.0) * (-2*a2*a2*a2 + 72*a2*a0-27*a3*a3*a0);
//        G = F*F/4.0 + E*E*E/27.0;
//        Z = pow(sqrt(G)-F/2,1/3.0)-pow(sqrt(G)+F/2.0,1/3.0)+a2/3.0;
////        Z=0;
//        R = sqrt(a3*a3/4 - a2 + Z);
//        S = 3*a3*a3/4 - R*R - 2*a2;
//        T = (4*a3*a2 - a3*a3*a3) / (4*R);

//-----------Maquina-estados-prueba-boost-1-fase------------------//
        while(1)
        {

            // Turn on LED
            //
            GPIO_WritePin(31, 1);
            GPIO_WritePin(34, 0);
            //
            // Delay for a bit.
            //
            DELAY_US(1000*500);
            //
            // Turn off LED
            //
            GPIO_WritePin(31, 0);
            GPIO_WritePin(34, 1);
            //
            // Delay for a bit.
            //
            DELAY_US(1000*500);

//        switch (estado)
//        {
//            case idle:
//                if (GpioDataRegs.GPBDAT.bit.GPIO35)
//                {
//                    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on Zero
//                    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;            // Set PWM1A on Zero
//                    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
//                    EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM1B on Zero
//
//                    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
//
//                }
//                else
//                {
//                    estado = normal_mode;
//                    DELAY_US(1000*500);
//                }
//            break;
//            case normal_mode:
//                if (GpioDataRegs.GPBDAT.bit.GPIO60 == 0x0)
//                {
//                    estado = apagado;
//                }
//                if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)    // Revisa si ocurrió overflow
//                {
//                    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
//                    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
//                }
//                PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);
//            break;
//            case apagado:
//                EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on Zero
//                EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Set PWM1A on Zero
//                EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
//                EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;            // Set PWM1B on Zero
//
////                EPwm3Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
//                if (GpioDataRegs.GPBDAT.bit.GPIO35 == 0)
//                {
//                    estado = idle;
//                }
//            break;
//        }
}
}






//-----------Maquina-estados-prueba-boost-3-fases-----------------//



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
//}

//
// ADC_initAdc - Initialize ADC A,B,C,D configurations and power it up
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
    EPwm1Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm1Regs.TBPHS.bit.TBPHS = 0;          // Set Phase register to zero

    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Enable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm1Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module, master module

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    //
    // Setup compare
    //
    EPwm1Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
//    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    //
    // Configuración eWPM2
    //
    EPwm2Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
//    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
//    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm2Regs.TBPHS.bit.TBPHS = PHASE2;          // Set Phase register to zero

    EPwm2Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Enable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm2Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm2Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    //
    // Setup compare
    //
    EPwm2Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM3
    //
    EPwm3Regs.TBPRD           = PWM_TBPRD_2;  // Frecuencia de 2khz
//    EPwm3Regs.TBPRD           = 1200;  // Frecuencia de 2khz
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0;           // Phase is 0
//    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm3Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Enable phase loading
//    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 4
    EPwm3Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // sync flow-through, slave mode
//    EPwm3Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event. up-down count
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    //
    // Setup compare
    //
//    EPwm3Regs.CMPA.bit.CMPA = CMPA_Boost_a;
//    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA_2;
    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA_2;
//    EPwm3Regs.CMPB.bit.CMPB = PWM_CMPA;

    EPwm3Regs.AQCTLA.bit.PRD = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

////    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on ONE
//    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Set PWM1A on Zero
//    EPwm3Regs.AQCTLA.bit.CBD = AQ_CLEAR;          // Set PWM1A on Zero
////    EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;            // Set PWM1B on ONE
//    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;            // Set PWM1B on ONE

//    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA;
//    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on ONE
//    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Set PWM1A on Zero
//    EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;            // Set PWM1B on ONE
//    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;            // Set PWM1B on ONE

    EPwm3Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm3Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm3Regs.ETSEL.bit.SOCASEL   = 0x2;    // Genera Evento cuando TBCTR = TBPRD (inicio conversión) down-count
    EPwm3Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    //
    // Configuración eWPM4
    //
    EPwm4Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm4Regs.TBPHS.bit.TBPHS = PHASE2;           // Phase is 0
    //    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm4Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm4Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm4Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm4Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    //
    // Setup compare
    //
    EPwm4Regs.CMPA.bit.CMPA = CMPA_Boost_b;
    EPwm4Regs.AQCTLA.bit.PRD = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
//    EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//    EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
//    EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM7
    //
    EPwm7Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm7Regs.TBPHS.bit.TBPHS = PHASE3;           // Phase is 0
    //    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm7Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm7Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm7Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm7Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    //
    // Setup compare
    //
    EPwm7Regs.CMPA.bit.CMPA = CMPA_Boost_c;
    EPwm7Regs.AQCTLA.bit.PRD = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
//    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//    EPwm7Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
//    EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM8
    //
    EPwm8Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0;           // Phase is 0

    EPwm8Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm8Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm8Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm8Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    //
    // Setup compare
    //
    EPwm8Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm8Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm8Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM10
    //
    EPwm10Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm10Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0

    EPwm10Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm10Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // Enable phase loading
    EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm10Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm10Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm10Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm10Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    //
    // Setup compare
    //
    EPwm10Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm10Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm10Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm10Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm10Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM11
    //
    EPwm11Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm11Regs.TBPHS.bit.TBPHS = 0;           // Phase is 0

    EPwm11Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm11Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
    EPwm11Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm11Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    EPwm11Regs.TBCTL.bit.PRDLD    = TB_SHADOW;  // Habilita el shadow mode.
    EPwm11Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through, slave mode
    EPwm11Regs.TBCTL.bit.PHSDIR = 1;         // Count up after the synchronization event.

    EPwm11Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm11Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm11Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm11Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    //
    // Setup compare
    //
    EPwm11Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm11Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm11Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm11Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm11Regs.AQCTLB.bit.CAD = AQ_SET;
    //
    // Configuración eWPM12
    //
    EPwm12Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm12Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm12Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm12Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;  // Divide el reloj en 4
    EPwm12Regs.TBCTL.bit.CLKDIV    = TB_DIV4;  // Divide el reloj en 2
    //
    // Setup compare
    //
    EPwm12Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm12Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm12Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm12Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm12Regs.AQCTLB.bit.CAD = AQ_SET;
}
void SetupADCEpwm2(void)
{
    Uint16 acqps;
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
    acqps = 20;

    EALLOW;
    SetupSOC(socA0, 0, acqps, 9); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps

    SetupSOC(socA2, 2, acqps, 9); // SOC2 convierte ADC-A2 y hace trigger desde ePWM3, ADCSOCA con una ventana de acqps
    SetupSOC(socA3, 3, acqps, 9); // SOC3 convierte ADC-A3 y hace trigger desde ePWM4, ADCSOCA con una ventana de acqps

//    SetupSOC(socA4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA5, 5, acqps, 5); // SOC4 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB0, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB1, 1, acqps, 5); // SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB2, 2, acqps, 5); // SOC8 convierte ADC-B2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB3, 3, acqps, 5); // SOC8 convierte ADC-B3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socB4, 4, acqps, 5); // SOC8 convierte ADC-B4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC0, 2, acqps, 5); // SOC8 convierte ADC-C2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC1, 3, acqps, 5); // SOC8 convierte ADC-C3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC2, 4, acqps, 5); // SOC8 convierte ADC-C4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socC3, 5, acqps, 5); // SOC8 convierte ADC-C5 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socD0, 0, acqps, 5); // SOC0 convierte ADC-D0 y hace trigger desde ePWM1(5) con una ventana de acqps

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //fin de SOCA3 genera INT1 flag
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

void configCLAMemory2(void)
{
#ifdef _FLASH
    //
    // Copy over code and tables from FLASH to RAM
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t)&RamfuncsLoadSize);
    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
            (uint32_t)&Cla1ProgLoadSize);

#if !(CLA_MATH_TABLES_IN_ROM)
    //
    // Copy over CLA Math tables from FLASH to RAM
    //
    memcpy((uint32_t *)&CLA1mathTablesRunStart, (uint32_t *)&CLA1mathTablesLoadStart,
            (uint32_t)&CLA1mathTablesLoadSize);
#endif
#endif

    EALLOW;
    //
    // Perform RAM initialization on the message RAMs
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1)
    {
    }
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1)
    {
    }
    //
    // Select LS0RAM and LS1RAM to be program space for the CLA
    // First, configure the CLA to be the master for LS0 and LS1
    // Second, set the space to be a program block
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;
    //
    // Configure RAM blocks LS2-LS5 as data spaces for the CLA
    // First, configure the CLA to be the master for LSx
    // Second, set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 0;

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);
    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
//    Cla1Regs.MIER.all = (M_INT8 | M_INT7| M_INT6| M_INT5| M_INT4| M_INT3| M_INT2| M_INT1);
    Cla1Regs.MIER.bit.INT1 = 1;     // Permite que interrupciones o cpu inicien la tarea 1
    Cla1Regs.MIER.bit.INT2 = 1;     // Permite que interrupciones o cpu inicien la tarea 2
    Cla1Regs.MIER.bit.INT6 = 1;     // Permite que interrupciones o cpu inicien la tarea 6
    Cla1Regs.MIER.bit.INT7 = 1;     // Permite que interrupciones o cpu inicien la tarea 7

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;
    //
    // Set the adca.1 as the trigger for task 7
    //
//    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;
//    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT

//--- Select Task interrupt source                     /******** TRIGGER SOURCE FOR EACH TASK (unlisted numbers are reserved) ********/
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 1;    // 0=none       8=ADCBINT3  16=ADCDINT1  32=XINT4     42=EPWM7INT   70=TINT2     78=ECAP4INT   95=SD1INT     114=SPIRXINTC
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK3 = 0;    // 2=ADCAINT2  10=ADCBEVT   18=ADCDINT3  36=EPWM1INT  44=EPWM9INT   72=MREVTA    80=ECAP6INT  107=UPP1INT
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK4 = 0;    // 3=ADCAINT3  11=ADCCINT1  19=ADCDINT4  37=EPWM2INT  45=EPWM10INT  73=MXEVTB    83=EQEP1INT  109=SPITXINTA
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK5 = 0;    // 4=ADCAINT4  12=ADCCINT2  20=ADCDEVT   38=EPWM3INT  46=EPWM11INT  74=MREVTB    84=EQEP2INT  110=SPIRXINTA
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK6 = 0;    // 5=ADCAEVT   13=ADCCINT3  29=XINT1     39=EPWM4INT  47=EPWM12INT  75=ECAP1INT  85=EQEP3INT  111=SPITXINTB
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;    // 6=ADCBINT1  14=ADCCINT4  30=XINT2     40=EPWM5INT  48=TINT0      76=ECAP2INT  87=HRCAP1INT 112=SPIRXINTB
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8 = 0;    // 7=ADCBINT2  15=ADCCEVT   31=XINT3     41=EPWM6INT  69=TINT1      77=ECAP3INT  88=HRCAP2INT 113=SPITXINTC

    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 0;     // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)

    //
    // Enable all CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
    EDIS;
}

void corrimiento_izq(float v1){
    int i;
    for (i = 0; i < ADC_BUF_PREV; i++){
        AdcBuf_ant_fall[i] = AdcBuf_ant_fall[i + 1];
    }
    AdcBuf_ant_fall[(ADC_BUF_PREV-1)] = v1;
    return;
}

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
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

//    PWM_CMPA_mod = (((int16)(AdcaResultRegs.ADCRESULT2)*625.0/(4096.0)));
//    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA_mod;
//    EPwm4Regs.CMPA.bit.CMPA = PWM_CMPA_mod;
//    EPwm7Regs.CMPA.bit.CMPA = PWM_CMPA_mod;

//    GPIO_WritePin(64, 0);// se usa para medir el tiempo de ejecución del programa
//    GPIO_WritePin(64, 1);// se usa para medir el tiempo de ejecución del programa
//    GPIO_WritePin(64, 0);// se usa para medir el tiempo de ejecución del programa

    x_ant_i_l[1] = 0;
//    Cla1ForceTask1();
//    Cla1ForceTask3andWait();
    x_ant_i_l[1] = 1;
//a1 = CpuTimer1Regs.TIM;
//    a = alpha;
//    b = 1 + alpha;
//    a = 2*a*3+1;
//    d = a/alpha;
//    c = 3.0*2.0*alpha+1;
//    d = alpha/2;
//    alpha

}

//
// cla1Isr1 - CLA1 ISR 1 - Fault mode
//
__interrupt void cla1Isr1 () // Fault mode
{
//    GPIO_WritePin(64, 1);// se usa para medir el tiempo de ejecución del programa

//    EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA_mod;

//----------Controlador-de-voltaje-----------//
//    if (i_count_l == 0)
//    {
//        x_ant_v_l = x_ant_v_g;
//        x_v_l     = x_v_g;
//    }
//-------------------------------------------//
//b1 = a1 + (float)CpuTimer1Regs.TIM;
//---------Controlador-de-corriente----------//
    x_ant_i_l[i_count_l] = x_ant_i_g[i_count_l];
    x_i_l[i_count_l]     = x_i_g[i_count_l];

    switch (i_count_l)
        {
            case 0:
            {
                EPwm3Regs.CMPA.bit.CMPA = 0.2*PWM_TBPRD;
                EPwm4Regs.CMPA.bit.CMPA = 0.5*PWM_TBPRD;
                EPwm7Regs.CMPA.bit.CMPA = 0.8*PWM_TBPRD;
                x_ant_v_l = x_ant_v_g;  // Guardado de variables controlador de voltaje
                x_v_l     = x_v_g;      // Guardado de variables controlador de voltaje
//                EPwm4Regs.CMPA.bit.CMPA = x_i_g[i_count_l]*PWM_TBPRD;
//                EPwm3Regs.CMPA.bit.CMPA = x_i_g[i_count_l]*PWM_TBPRD;
            }
            break;
            case 1:
            {
//                EPwm4Regs.CMPA.bit.CMPA = x_i_g[i_count_l]*PWM_TBPRD;
            }
            break;
            case 2:
            {
//                EPwm7Regs.CMPA.bit.CMPA = x_i_g[i_count_l]*PWM_TBPRD;
            }
            break;
        }

//        PWM_CMPA_mod = (((int16)(AdcaResultRegs.ADCRESULT2)*625.0/(4096.0)));
//        PWM_CMPA_mod = x_i_g[i_count_l]*PWM_TBPRD;
//        EPwm3Regs.CMPA.bit.CMPA = PWM_CMPA_mod;

// activar cuando se ocupe el interleaved
    i_count_l = 0;
//    if (i_count_g < 0x2)
//    {
//        i_count_l = i_count_l+1;
//    }
//    else
//    {
//        i_count_l = 0;
//    }

    GPIO_WritePin(64, 1);// se usa para medir el tiempo de ejecución del programa


//    int i;

    if (ii<ADC_BUF_PREV)
    {
        AdcBuf_ant_fall[ii] = (float)(AdcaResultRegs.ADCRESULT2);
        AdcBuf_ant_fall2[ii] = (float)(AdcaResultRegs.ADCRESULT0);
        ii = ii+1;
    }
    else
    {
        ii = 0;
    }

//    for (i = 0; i < ADC_BUF_PREV; i++){
//        AdcBuf_ant_fall[i] = AdcBuf_ant_fall[i + 1];
//    }
//    AdcBuf_ant_fall[(ADC_BUF_PREV-1)] = v1;
//    corrimiento_izq ((float)(AdcaResultRegs.ADCRESULT2));
    GPIO_WritePin(64, 0);// se usa para medir el tiempo de ejecución del programa

//
//-------------------------------------------//

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);

}

//
// cla1Isr2 - CLA1 ISR 2 - Sync_PLL
//
__interrupt void cla1Isr2 () // Sync_PLL
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // limpia el bit de la INT1 flag
    // Revisa si ocurrió overflow
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);
}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr4 - CLA1 ISR 4
//
__interrupt void cla1Isr4 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr5 - CLA1 ISR 5
//
__interrupt void cla1Isr5 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr6 - CLA1 ISR 6 - Modo de Fallas

//
__interrupt void cla1Isr6 ()
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // limpia el bit de la INT1 flag
    // Revisa si ocurrió overflow
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);

//    GPIO_WritePin(BLINKY_LED_GPIO, 0);

}

//
// cla1Isr7 - CLA1 ISR 7 - Normal mode
//
__interrupt void cla1Isr7 () // Normal mode
{

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // limpia el bit de la INT1 flag
    // Revisa si ocurrió overflow
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);




}

//
// cla1Isr8 - CLA1 ISR 8
//
__interrupt void cla1Isr8 ()
{
    PieCtrlRegs.PIEACK.all = M_INT11;
}



//
//// End of File
//
