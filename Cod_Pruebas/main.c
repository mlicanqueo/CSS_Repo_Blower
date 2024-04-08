/*
 * main.c
 *
 *  Created on: 06-07-2020
 *      Author: Matias_L
 */
//
// Included Files
//
#include "task_file.h"
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include <CLAmath.h>
//#include <math.h>
//
// Defines
//
#define soc0 0
#define soc1 1
#define soc2 2
#define soc3 3
#define soc4 4
#define soc5 5
#define soc6 6
#define soc7 7
#define soc8 8

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
// Globales
//------------maquina-de-estados-------------//
uint16_t estado   = 0;   // Estado inicial de la máquina de estados
uint16_t boton_1  = 0;   // Condición de salto desde estado iddle a estado sync_pll
uint16_t boton_2  = 0;   // Condición de salto desde estado sync_pll a estado precarga
uint16_t fault    = 0;
float vg_d, vg_q, i_out; // Variables máquina de estados
float va, vb, vc;        // Variables máquina de estados
float vdc;               // Variables máquina de estados
//-------------------------------------------//
//-------------Variables-fallas--------------//
// estado sincronizacion del pll
int cont_sinc;           // Contador interno para estado de fallas
// estado precarga
int cont_precarg;
// estado normal mode
int cont_normal, cont_norm_fase;
//-------------------------------------------//
// Variables tareas CLA
//
//------------Variables-de-prueba------------//
#pragma DATA_SECTION(Num,"CpuToCla1MsgRAM");
float Num;
#pragma DATA_SECTION(Den,"CpuToCla1MsgRAM");
float Den;
#pragma DATA_SECTION(Res,"Cla1ToCpuMsgRAM");
float Res;
#pragma DATA_SECTION(salida_prueba,"Cla1ToCpuMsgRAM");
float salida_prueba;
#pragma DATA_SECTION(voltFilt,"Cla1ToCpuMsgRAM")
float voltFilt;
#pragma DATA_SECTION(a1,"Cla1ToCpuMsgRAM");
float a1;
#pragma DATA_SECTION(a2,"Cla1ToCpuMsgRAM");
float a2;
#pragma DATA_SECTION(a3,"Cla1ToCpuMsgRAM");
float a3;
#pragma DATA_SECTION(a4,"Cla1ToCpuMsgRAM");
float a4;
#pragma DATA_SECTION(a5,"Cla1ToCpuMsgRAM");
float a5;
#pragma DATA_SECTION(a6,"Cla1ToCpuMsgRAM");
float a6;
#pragma DATA_SECTION(a7,"Cla1ToCpuMsgRAM");
float a7;
#pragma DATA_SECTION(a8,"Cla1ToCpuMsgRAM");
float a8;
#pragma DATA_SECTION(a9,"Cla1ToCpuMsgRAM");
float a9;
#pragma DATA_SECTION(a10,"Cla1ToCpuMsgRAM");
float a10;
#define BLINKY_LED_GPIO    31

#define sw1_output         39 // Salida para precarga |  Pin en RTBOX es DI14
#define sw2_output         44 // Salida para precarga |  Pin en RTBOX es DI15
uint16_t aaa = 0;
int bbb = 0;
int ccc = 0;
int vg_q1 = 10;

#pragma DATA_SECTION(vect,"CpuToCla1MsgRAM");
float vect [6];

//-------------------------------------------//
//---------------Variables-FSM---------------//
#pragma DATA_SECTION(adc_read,"Cla1ToCpuMsgRAM");
float adc_read [8];
//-------------------------------------------//
//------------variables-srf-pll--------------//
#pragma DATA_SECTION(theta_l,"CpuToCla1MsgRAM");
float theta_l;
#pragma DATA_SECTION(theta_ant_l,"CpuToCla1MsgRAM");
float theta_ant_l;
#pragma DATA_SECTION(integrador_ant_l,"CpuToCla1MsgRAM");
float integrador_ant_l;
#pragma DATA_SECTION(integrador_l,"CpuToCla1MsgRAM");
float integrador_l;

#pragma DATA_SECTION(integrador_g,"Cla1ToCpuMsgRAM");
float integrador_g;
#pragma DATA_SECTION(theta_g,"Cla1ToCpuMsgRAM");
float theta_g;
#pragma DATA_SECTION(theta_ant_g,"Cla1ToCpuMsgRAM");
float theta_ant_g;
#pragma DATA_SECTION(integrador_ant_g,"Cla1ToCpuMsgRAM");
float integrador_ant_g;
//-------------------------------------------//
//-----------variables-DDSRFpll--------------//
#pragma DATA_SECTION(vg_d_men_prom_l,"CpuToCla1MsgRAM");
float vg_d_men_prom_l;
#pragma DATA_SECTION(vg_q_men_prom_l,"CpuToCla1MsgRAM");
float vg_q_men_prom_l;
#pragma DATA_SECTION(vg_d_mas_prom_l,"CpuToCla1MsgRAM");
float vg_d_mas_prom_l;
#pragma DATA_SECTION(vg_q_mas_prom_l,"CpuToCla1MsgRAM");
float vg_q_mas_prom_l;

#pragma DATA_SECTION(vg_d_men_ref_ant_l,"CpuToCla1MsgRAM");
float vg_d_men_ref_ant_l;
#pragma DATA_SECTION(vg_d_men_prom_ant_l,"CpuToCla1MsgRAM");
float vg_d_men_prom_ant_l;
#pragma DATA_SECTION(vg_q_men_ref_ant_l,"CpuToCla1MsgRAM");
float vg_q_men_ref_ant_l;
#pragma DATA_SECTION(vg_q_men_prom_ant_l,"CpuToCla1MsgRAM");
float vg_q_men_prom_ant_l;

#pragma DATA_SECTION(vg_d_mas_ref_ant_l,"CpuToCla1MsgRAM");
float vg_d_mas_ref_ant_l;
#pragma DATA_SECTION(vg_d_mas_prom_ant_l,"CpuToCla1MsgRAM");
float vg_d_mas_prom_ant_l;
#pragma DATA_SECTION(vg_q_mas_ref_ant_l,"CpuToCla1MsgRAM");
float vg_q_mas_ref_ant_l;
#pragma DATA_SECTION(vg_q_mas_prom_ant_l,"CpuToCla1MsgRAM");
float vg_q_mas_prom_ant_l;

#pragma DATA_SECTION(integr_ddsrf_l,"CpuToCla1MsgRAM");
float integr_ddsrf_l;
#pragma DATA_SECTION(integr_ddsrf_ant_l,"CpuToCla1MsgRAM");
float integr_ddsrf_ant_l;
#pragma DATA_SECTION(theta_ddsrf_ant_l,"CpuToCla1MsgRAM");
float theta_ddsrf_ant_l;
#pragma DATA_SECTION(theta_ddsrf_l,"CpuToCla1MsgRAM");
float theta_ddsrf_l;


#pragma DATA_SECTION(vg_d_men_prom_g,"Cla1ToCpuMsgRAM");
float vg_d_men_prom_g;
#pragma DATA_SECTION(vg_q_men_prom_g,"Cla1ToCpuMsgRAM");
float vg_q_men_prom_g;
#pragma DATA_SECTION(vg_d_mas_prom_g,"Cla1ToCpuMsgRAM");
float vg_d_mas_prom_g;
#pragma DATA_SECTION(vg_q_mas_prom_g,"Cla1ToCpuMsgRAM");
float vg_q_mas_prom_g;

#pragma DATA_SECTION(vg_d_men_ref_ant_g,"Cla1ToCpuMsgRAM");
float vg_d_men_ref_ant_g;
#pragma DATA_SECTION(vg_d_men_prom_ant_g,"Cla1ToCpuMsgRAM");
float vg_d_men_prom_ant_g;
#pragma DATA_SECTION(vg_q_men_ref_ant_g,"Cla1ToCpuMsgRAM");
float vg_q_men_ref_ant_g;
#pragma DATA_SECTION(vg_q_men_prom_ant_g,"Cla1ToCpuMsgRAM");
float vg_q_men_prom_ant_g;

#pragma DATA_SECTION(vg_d_mas_ref_ant_g,"Cla1ToCpuMsgRAM");
float vg_d_mas_ref_ant_g;
#pragma DATA_SECTION(vg_d_mas_prom_ant_g,"Cla1ToCpuMsgRAM");
float vg_d_mas_prom_ant_g;
#pragma DATA_SECTION(vg_q_mas_ref_ant_g,"Cla1ToCpuMsgRAM");
float vg_q_mas_ref_ant_g;
#pragma DATA_SECTION(vg_q_mas_prom_ant_g,"Cla1ToCpuMsgRAM");
float vg_q_mas_prom_ant_g;

#pragma DATA_SECTION(integr_ddsrf_g,"Cla1ToCpuMsgRAM");
float integr_ddsrf_g;
#pragma DATA_SECTION(integr_ddsrf_ant_g,"Cla1ToCpuMsgRAM");
float integr_ddsrf_ant_g;
#pragma DATA_SECTION(theta_ddsrf_ant_g,"Cla1ToCpuMsgRAM");
float theta_ddsrf_ant_g;
#pragma DATA_SECTION(theta_ddsrf_g,"Cla1ToCpuMsgRAM");
float theta_ddsrf_g;

//    vg_d_men_prom_l = vg_d_men_prom_g; // las l cpu->cla, las g cla->cpu
//    vg_q_men_prom_l = vg_q_men_prom_g;
//    vg_d_mas_prom_l = vg_d_mas_prom_g;
//    vg_q_mas_prom_l = vg_q_mas_prom_g;
//
//    vg_d_men_ref_ant_l = vg_d_men_ref_ant_g;
//    vg_d_men_prom_ant_l = vg_d_men_prom_ant_g;
//    vg_q_men_ref_ant_l = vg_q_men_ref_ant_g;
//    vg_q_men_prom_ant_l = vg_q_men_prom_ant_g;
//
//    vg_d_mas_ref_ant_l = vg_d_mas_ref_ant_g;
//    vg_d_mas_prom_ant_l = vg_d_mas_prom_ant_g;
//    vg_q_mas_ref_ant_l = vg_q_mas_ref_ant_g;
//    vg_q_mas_prom_ant_l = vg_q_mas_prom_ant_g;
////-------------------------------------------//
////------------variables-pi-pll---------------//
//    integr_ddsrf_l = integr_ddsrf_g
//    integr_ddsrf_ant_l = integr_ddsrf_ant_g;
//    theta_ddsrf_ant_l = theta_ddsrf_ant_g;
//    theta_ddsrf_l = theta_ddsrf_g;
////-------------------------------------------//

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

//---------Controlador-de-corriente-d--------//
#pragma DATA_SECTION(x_ant_id_l,"CpuToCla1MsgRAM");
float x_ant_id_l;
#pragma DATA_SECTION(x_ant_id_g,"Cla1ToCpuMsgRAM");
float x_ant_id_g;

#pragma DATA_SECTION(x_id_l,"CpuToCla1MsgRAM");
float x_id_l;
#pragma DATA_SECTION(x_id_g,"Cla1ToCpuMsgRAM");
float x_id_g;
//-------------------------------------------//

//---------Controlador-de-corriente-q--------//
#pragma DATA_SECTION(x_ant_iq_l,"CpuToCla1MsgRAM");
float x_ant_iq_l;
#pragma DATA_SECTION(x_ant_iq_g,"Cla1ToCpuMsgRAM");
float x_ant_iq_g;

#pragma DATA_SECTION(x_iq_l,"CpuToCla1MsgRAM");
float x_iq_l;
#pragma DATA_SECTION(x_iq_g,"Cla1ToCpuMsgRAM");
float x_iq_g;
//-------------------------------------------//
//--------------dq-a-alpha_beta--------------//

//-------------------------------------------//
//--------------alpha_beta-a-abc-------------//
#pragma DATA_SECTION(v_conv_a,"Cla1ToCpuMsgRAM");
float v_conv_a;
#pragma DATA_SECTION(v_conv_b,"Cla1ToCpuMsgRAM");
float v_conv_b;
#pragma DATA_SECTION(v_conv_c,"Cla1ToCpuMsgRAM");
float v_conv_c;
//-------------------------------------------//
//--------------Buffer-ant-falla-------------//
//uint16_t largo_theta = 1000;
float AdcBuf_ant_fall[ADC_BUF_PREV];
//uint16_t k = 0;
//-------------------------------------------//
//--------------Buffer-dsp-falla-------------//
uint16_t SampleCount;
uint16_t AdcBuf_dsp_fall[ADC_BUF_LEN];
uint16_t i = 0;
//-------------------------------------------//
//----------Variables-guardado-datos---------//
#define largo_theta 1300
float theta_buff[largo_theta];
float alfa_buff[largo_theta];
float beta_buff[largo_theta];
//-------------------------------------------//


//
// Function Prototypes
//
void CLA_configClaMemory(void);
void configCLAMemory2(void);
void CLA_initCpu1Cla1(void);
void EPWM_initEpwm(void);
void ADC_initAdcA(void);
void SetupADCEpwm2(void);
void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel);

//-----------ejemplos------------------//
void ConfigureADC(void);
void ConfigureEPWM(void);
void SetupADCePWM(void);
//-----------ejemplos------------------//



//__interrupt void epwm1_isr(void);
//__interrupt void epwm1_isr();
__interrupt void adca1_isr(void);
__interrupt void adca1_isr();
uint16_t ADCread;
uint16_t ADCread2;


void InitEPwm1Example(void);
Uint32 EPwm1TimerIntCount;
Uint16 EPwm1_DB_Direction;



/*void estado_fallas(void);*/
/*void corrimiento_izq(float v1, float  v2, float v3, float v4, float v5, float v6, float v7, float v8);*/

/*__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();*/

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
//    InitGpio();


    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; // PWM1A y PWM1B se utilizan de forma interna para hacer trigger al adc
//    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1; // PWM2A y PWM2B se utilizan de forma Externa, salidas en RTBOX son DI2 y DI3
//    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1; // PWM3A y PWM3B se utilizan de forma Externa, salidas en RTBOX son DI4 y DI5
//    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1; // PWM4A y PWM4B se utilizan de forma Externa, salidas en RTBOX son DI6 y DI7
//
// For this case just init GPIO pins for ePWM1, ePWM2
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();
//    InitEPwm2Gpio();
//    InitEPwm3Gpio();
//    InitEPwm4Gpio();
//
//  Encendido y apagado de un led
//
/*
    GPIO_SetupPinMux(BLINKY_LED_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
//  Encendido y apagado de un pin
    GPIO_SetupPinMux(sw1_output, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(sw1_output, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(sw2_output, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(sw2_output, GPIO_OUTPUT, GPIO_PUSHPULL);
//
*/
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
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;
//
// Step 5. Configure the ADC to start sampling on an EPWM period
//
//    ADC_initAdcA();
//
// Step 6. Configure EPWM to trigger ADC every 20us
//
//    EPWM_initEpwm();
//
// Configurar los ADC para ser disparados con el ePWM
//
/*    SetupADCEpwm2();*/
//


//
// Step 8. Turn on the EPWM
//

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

//    InitEPwm1Example();
//    InitEPwm2Example();
//    InitEPwm3Example();

//  Inicialiar los modulos y sincronizarlos.

    ConfigureADC();
    ConfigureEPWM();
    SetupADCePWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

    // Initialize counters:
    //
//        EPwm1TimerIntCount = 0;
//        EPwm2TimerIntCount = 0;
//        EPwm3TimerIntCount = 0;

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    //
//    IER |= M_INT3;
    IER |= M_INT1; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
// enable PIE interrupt
//
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;


//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
//    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Step 7. Enable global Interrupts and higher priority real-time debug events:
//
//    EINT;  // Enable Global interrupt INTM
//    ERTM;  // Enable Global realtime interrupt DBGM

// start ePWM
//
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

//
// Step 6. IDLE loop. Just sit and loop forever (optional):
//
    for(;;)
    {
        asm ("          NOP");
    }

//    EALLOW;
//    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
//    EPwm1Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
////    EPwm1Regs.ETSEL.bit.SOCBEN  = 1; // Habilita pulso SOCB  (EPWMxSOCB)
//    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode
//
//    EPwm2Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode
//    EPwm3Regs.TBCTL.bit.CTRMODE = 2;
//    EPwm4Regs.TBCTL.bit.CTRMODE = 2;
//    EDIS;


//// Configure the CLA memory spaces first followed by
//// the CLA task vectors.
//    configCLAMemory2();
//    CLA_initCpu1Cla1();
////  Initial parameters

//    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

//EALLOW;
//            GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pullup on GPIO31
//            GpioDataRegs.GPASET.bit.GPIO31 = 1;   // Load output latch
//            GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // GPIO31 = GPIO31
//            GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;   // GPIO31 = output
//
//            GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;   // Enable pullup on GPIO34
//            GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // Load output latch
//            GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  // GPIO34 = GPIO6
//            GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   // GPIO34 = output
//EDIS;

//    while (1)
//    {
//                   GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;   // GPIO6 = 0
//                   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;   // GPIO31 = TOGGGLE
//                   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;   // GPIO34 = TOGGLE
//                   DELAY_US(1000*100);
//
//                   GpioDataRegs.GPASET.bit.GPIO6 = 1;   // GPIO6 = 1
//                   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;   // GPIO31 = TOGGGLE
//                   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;   // GPIO34 = TOGGLE
//                   DELAY_US(1000*100);
//    }

}
//
////
//// epwm1_isr - EPWM1 ISR
////
//__interrupt void epwm1_isr(void)
//{
//    if(EPwm1_DB_Direction == DB_UP)
//    {
//        if(EPwm1Regs.DBFED.bit.DBFED < EPWM1_MAX_DB)
//        {
//            EPwm1Regs.DBFED.bit.DBFED++;
//            EPwm1Regs.DBRED.bit.DBRED++;
//        }
//        else
//        {
//            EPwm1_DB_Direction = DB_DOWN;
//            EPwm1Regs.DBFED.bit.DBFED--;
//            EPwm1Regs.DBRED.bit.DBRED--;
//        }
//    }
//    else
//    {
//        if(EPwm1Regs.DBFED.bit.DBFED == EPWM1_MIN_DB)
//        {
//            EPwm1_DB_Direction = DB_UP;
//            EPwm1Regs.DBFED.bit.DBFED++;
//            EPwm1Regs.DBRED.bit.DBRED++;
//        }
//        else
//        {
//            EPwm1Regs.DBFED.bit.DBFED--;
//            EPwm1Regs.DBRED.bit.DBRED--;
//        }
//    }
//    EPwm1TimerIntCount++;
//
//    //
//    // Clear INT flag for this timer
//    //
//    EPwm1Regs.ETCLR.bit.INT = 1;
//
//    //
//    // Acknowledge this interrupt to receive more interrupts from group 3
//    //
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//}

void ConfigureADC(void)
{
    EALLOW;
    //
    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión
    //
    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //delay for > 1ms to allow ADC time to power up
    DELAY_US(1000);
    EDIS;
}

void ConfigureEPWM(void)
{
    EALLOW;

    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm1Regs.ETSEL.bit.SOCASEL   = 0x4;  // Enable event time-base counter equal to CMPA when the timer is incrementing or CMPC when the timer is incrementing
    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento

//    EPwm1Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
//    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento

    EPwm1Regs.CMPA.bit.CMPA = 3125/2;            // Contador q genera un evento
    EPwm1Regs.TBPRD = 3125;                      // configura la frecuencia
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter

    EDIS;
}

void SetupADCePWM(void)
{
    EALLOW;
    Uint16 acqps;

    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;//trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;
}

//
// adca1_isr - Read Temperature ISR
//
interrupt void adca1_isr(void)
{

    ADCread = (AdcaResultRegs.ADCRESULT0);
    ADCread2 = ADCread*(3/4096);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}



////
//// InitEPwm1Example - Initialize EPWM1 configuration
////
//void InitEPwm1Example()
//{
//    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
//    EPwm1Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
//    EPwm1Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
//    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
//    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
//    ////    EPwm1Regs.TBPRD               = 500;  // Frecuencia de 50khz
//    //////    EPwm1Regs.TBPRD               = 50000;  // Frecuencia de .5khz
//    ////    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
//    ////    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
//    ////    EPwm1Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1
//
//    //
//    // Setup TBCLK, from TBCTL (Time base control register)
//    //
//   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //Up Down Count Mode
//   EPwm1Regs.TBCTL.bit.PHSEN =   TB_DISABLE;      //Disable phase loading
//   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
//   EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
//   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Divide el reloj en 4
//   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Divide el reloj en 4
//
//   //
//   // Configurar frecuencia y comparador
//   //
//   EPwm1Regs.TBCTR = 0x0000;                     // Clear counter
////   EPwm1Regs.TBPRD = 31250;                      // configura la frecuencia
//   EPwm1Regs.TBPRD = 3125000;                      // configura la frecuencia
//   EPwm1Regs.CMPA.bit.CMPA = 3125000/2;            // Contador q genera un evento
//                                                 // cuando TBPRD llega al valor
//                                                 // escrito en CMPA
//
////   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
////   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
////   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
////   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;    // Load registers every ZERO
//
//   //
//   // Set Actions
//   //
//
//   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on 1
//   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//   EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on 0
//   EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//
//}



//
////
//// CLA_configClaMemory - Configure CLA memory sections
////
//void CLA_configClaMemory(void)
//{
//    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
//
//    EALLOW;
//#ifdef _FLASH
//    //
//    // Copy over code from FLASH to RAM
//    //
//    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
//           (uint32_t)&Cla1funcsLoadSize);
//#endif //_FLASH
//
//    //
//    // Initialize and wait for CLA1ToCPUMsgRAM
//    //
//    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
//    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};
//
//    //
//    // Initialize and wait for CPUToCLA1MsgRAM
//    //
//    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
//    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};
//
//    //
//    // Select LS5RAM to be the programming space for the CLA
//    // First configure the CLA to be the master for LS5 and then
//    // set the space to be a program block
//    //
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;
//
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;
//
//    //
//    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
//    // First configure the CLA to be the master for LS0(1) and then
//    // set the spaces to be code blocks
//    //
//MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
//MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;
//
//MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
//MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;
//
//    EDIS;
//}
//
//void configCLAMemory2(void)
//{
//#ifdef _FLASH
//    //
//    // Copy over code and tables from FLASH to RAM
//    //
//    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t)&RamfuncsLoadSize);
//    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
//            (uint32_t)&Cla1ProgLoadSize);
//
//#if !(CLA_MATH_TABLES_IN_ROM)
//    //
//    // Copy over CLA Math tables from FLASH to RAM
//    //
//    memcpy((uint32_t *)&CLA1mathTablesRunStart, (uint32_t *)&CLA1mathTablesLoadStart,
//            (uint32_t)&CLA1mathTablesLoadSize);
//#endif
//#endif
//
//    EALLOW;
//    //
//    // Perform RAM initialization on the message RAMs
//    //
//    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
//    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1)
//    {
//    }
//    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
//    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1)
//    {
//    }
//    //
//    // Select LS0RAM and LS1RAM to be program space for the CLA
//    // First, configure the CLA to be the master for LS0 and LS1
//    // Second, set the space to be a program block
//    //
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;
//
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;
//    //
//    // Configure RAM blocks LS2-LS5 as data spaces for the CLA
//    // First, configure the CLA to be the master for LSx
//    // Second, set the spaces to be code blocks
//    //
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 0;
//
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;
//
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;
//
//    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
//    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 0;
//
//    EDIS;
//}
//
////
//// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
////
////void CLA_initCpu1Cla1(void)
////{
////    //
////    // Compute all CLA task vectors
////    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
////    // opposed to offsets used on older Type-0 CLAs
////    //
////    EALLOW;
////    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
////    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
////    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
////    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
////    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
////    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
////    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
////    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);
////    //
////    // Enable the IACK instruction to start a task on CLA in software
////    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
////    // subset of tasks) by writing to their respective bits in the
////    // MIER register
////    //
////    Cla1Regs.MCTL.bit.IACKE = 1;
//////    Cla1Regs.MIER.all = (M_INT8 | M_INT7| M_INT6| M_INT5| M_INT4| M_INT3| M_INT2| M_INT1);
////    Cla1Regs.MIER.bit.INT1 = 1;     // Permite que interrupciones o cpu inicien la tarea 1
////    Cla1Regs.MIER.bit.INT2 = 1;     // Permite que interrupciones o cpu inicien la tarea 2
////    Cla1Regs.MIER.bit.INT6 = 1;     // Permite que interrupciones o cpu inicien la tarea 6
////    Cla1Regs.MIER.bit.INT7 = 1;     // Permite que interrupciones o cpu inicien la tarea 7
////
////    //
////    // Configure the vectors for the end-of-task interrupt for all
////    // 8 tasks
////    //
////    PieVectTable.CLA1_1_INT = &cla1Isr1;
////    PieVectTable.CLA1_2_INT = &cla1Isr2;
////    PieVectTable.CLA1_3_INT = &cla1Isr3;
////    PieVectTable.CLA1_4_INT = &cla1Isr4;
////    PieVectTable.CLA1_5_INT = &cla1Isr5;
////    PieVectTable.CLA1_6_INT = &cla1Isr6;
////    PieVectTable.CLA1_7_INT = &cla1Isr7;
////    PieVectTable.CLA1_8_INT = &cla1Isr8;
////    //
////    // Set the adca.1 as the trigger for task 7
////    //
//////    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;
//////    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
////
//////--- Select Task interrupt source                     /******** TRIGGER SOURCE FOR EACH TASK (unlisted numbers are reserved) *******
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 0;    // 0=none       8=ADCBINT3  16=ADCDINT1  32=XINT4     42=EPWM7INT   70=TINT2     78=ECAP4INT   95=SD1INT     114=SPIRXINTC
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK3 = 0;    // 2=ADCAINT2  10=ADCBEVT   18=ADCDINT3  36=EPWM1INT  44=EPWM9INT   72=MREVTA    80=ECAP6INT  107=UPP1INT
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK4 = 0;    // 3=ADCAINT3  11=ADCCINT1  19=ADCDINT4  37=EPWM2INT  45=EPWM10INT  73=MXEVTB    83=EQEP1INT  109=SPITXINTA
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK5 = 0;    // 4=ADCAINT4  12=ADCCINT2  20=ADCDEVT   38=EPWM3INT  46=EPWM11INT  74=MREVTB    84=EQEP2INT  110=SPIRXINTA
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK6 = 0;    // 5=ADCAEVT   13=ADCCINT3  29=XINT1     39=EPWM4INT  47=EPWM12INT  75=ECAP1INT  85=EQEP3INT  111=SPITXINTB
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;    // 6=ADCBINT1  14=ADCCINT4  30=XINT2     40=EPWM5INT  48=TINT0      76=ECAP2INT  87=HRCAP1INT 112=SPIRXINTB
////    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8 = 0;    // 7=ADCBINT2  15=ADCCEVT   31=XINT3     41=EPWM6INT  69=TINT1      77=ECAP3INT  88=HRCAP2INT 113=SPITXINTC
////
////    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 0;     // Write a 1 to lock (cannot be cleared once set)
////    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)
////
////    //
////    // Enable all CLA interrupts at the group and subgroup levels
////    //
////    PieCtrlRegs.PIEIER11.all = 0xFFFF;
////    IER |= (M_INT11 );
////    EDIS;
////}
////
//// EPWM_initEpwm - Initialize EPWM1, EPWM2, EPWM3, EPWM4 settings
////
//// PWM1A y PWM1B se utilizan de forma interna para hacer trigger al adc
//// PWM2A y PWM2B se utilizan de forma Externa, salidas en RTBOX son DI2 y DI3
//// PWM3A y PWM3B se utilizan de forma Externa, salidas en RTBOX son DI4 y DI5
//// PWM4A y PWM4B se utilizan de forma Externa, salidas en RTBOX son DI6 y DI7
////
////void EPWM_initEpwm(void)
////{
////    EALLOW;
////    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
////    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
////    EPwm1Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
////    EPwm1Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
////    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm1Regs.TBPRD               = 500;  // Frecuencia de 50khz
//////    EPwm1Regs.TBPRD               = 50000;  // Frecuencia de .5khz
////    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
////    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
////    EPwm1Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1
////
////    EPwm1Regs.CMPA.bit.CMPA = 250;
////    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
////    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
////    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
////    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//////
////// Configuracion Deadband - 500ns de FED y RED
////    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
////    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
////    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
////    EPwm1Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
////    EPwm1Regs.DBFED.bit.DBFED = 0x19; // 25 decimal
////
////    // Configuración eWPM2
////    EPwm2Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm2Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm2Regs.TBPRD               = 500;  // Frecuencia de 50khz
////    EPwm2Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
////    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
////    EPwm2Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1
////
////    EPwm2Regs.CMPA.bit.CMPA = 250;
////    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
////    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
////    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
////    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
//////
////// Configuracion Deadband - 500ns de FED y RED
////    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
////    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
////    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
////    EPwm3Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
////    EPwm3Regs.DBFED.bit.DBFED = 0x19; // 25 decimal
////
////    // Configuración eWPM3
////    EPwm3Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm3Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm3Regs.TBPRD               = 500;  // Frecuencia de 50khz
////    EPwm3Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
////    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
////    EPwm3Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1
////
////    EPwm3Regs.CMPA.bit.CMPA = 250;
////    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
////    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
////    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
////    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;
//////
////// Configuracion Deadband - 500ns de FED y RED
////    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
////    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
////    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
////    EPwm3Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
////    EPwm3Regs.DBFED.bit.DBFED = 0x19; // 25 decimal
////
////    // Configuración eWPM4
////    EPwm4Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm4Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
////    EPwm4Regs.TBPRD               = 500;  // Frecuencia de 50khz
////    EPwm4Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
////    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
////    EPwm4Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1
////
////    EPwm4Regs.CMPA.bit.CMPA = 250;
////    EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
////    EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
////    EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
////    EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;
//////
////// Configuracion Deadband - 500ns de FED y RED
////    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
////    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
////    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
////    EPwm4Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
////    EPwm4Regs.DBFED.bit.DBFED = 0x19; // 25 decimal
////    EDIS;
////}

//
//// ADC_initAdcA - Initialize ADCA configurations and power it up
//
//void ADC_initAdcA(void)
//{
//    EALLOW;
//    //
//    //write configurations
//    //
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    //
//    //Set pulse positions to late
//    //
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión
//    //
//    //power up the ADC
//    //
//    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
//    // encender adcB
//    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    //
//    //Set pulse positions to late
//    //
//    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
//    //
//    //delay for > 1ms to allow ADC time to power up
//    DELAY_US(1000);
//
//    Uint16 acqps;
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
////    acqps = 800; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
//
//        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
//        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
//        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;//trigger on ePWM1 SOCA/C
//
////        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //fin de SOC4 genera INT1 flag
////        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
////        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag
//
//    EDIS;
//}
//
///*void SetupADCEpwm2(void)
//{
//    Uint16 acqps;
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
////    acqps = 800; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
//
//    EALLOW;
//    SetupSOC(soc0, 0, acqps, 5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc2, 2, acqps, 5); // SOC2 convierte ADC-A2 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc3, 3, acqps, 5); // SOC3 convierte ADC-A3 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
////    SetupSOC(soc5, 5, acqps, 5); // SOC5 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc6, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc7, 1, acqps, 5); // SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc8, 2, acqps, 5); // SOC8 convierte ADC-B2 y hace trigger desde ePWM1(5) con una ventana de acqps
//
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 4; //fin de SOC4 genera INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag
//    EDIS;
//}*/
///*
//void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel)
//{
//    switch(soc)
//        {
//            case soc0:
//            {
//                AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
//                AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc1:
//            {
//                AdcaRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
//                AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc2:
//            {
//                AdcaRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC2 will convert pin A2
//                AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc3:
//            {
//                AdcaRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC3 will convert pin A3
//                AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc4:
//            {
//                AdcaRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC4 will convert pin A4
//                AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc5:
//            {
//                AdcaRegs.ADCSOC5CTL.bit.CHSEL = channel;  //SOC5 will convert pin A5
//                AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window
//                AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc6:
//            {
//                AdcbRegs.ADCSOC6CTL.bit.CHSEL = channel;  //SOC6 will convert pin B0
//                AdcbRegs.ADCSOC6CTL.bit.ACQPS = acqps;    //sample window
//                AdcbRegs.ADCSOC6CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc7:
//            {
//                AdcbRegs.ADCSOC7CTL.bit.CHSEL = channel;  //SOC7 will convert pin B1
//                AdcbRegs.ADCSOC7CTL.bit.ACQPS = acqps;    //sample window
//                AdcbRegs.ADCSOC7CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//            case soc8:
//            {
//                AdcbRegs.ADCSOC8CTL.bit.CHSEL = channel;  //SOC8 will convert pin B2
//                AdcbRegs.ADCSOC8CTL.bit.ACQPS = acqps;    //sample window
//                AdcbRegs.ADCSOC8CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//            break;
//            }
//        }
//    }*/
///*
//void estado_fallas(void)
//{
//    fault = 0;
//    switch (estado)
//    {
//        case sync_pll:
//            // sincronizacion del pll
//            // Pll no se engancha
//            // no hay alimentación
//            if ((vg_d < vg_d_min)||(vg_q < vg_q_min)||(vg_q > vg_q_max))
//            {
//                cont_sinc ++;
//            }
//            else
//            {
//                cont_sinc = 0;
//            }
//            if (cont_sinc >= 150)
//            {
//                fault = 1;
//            }
//        break;
//        case pre_charge:
//            // Estado Precarga
//            // No alcanza la precarga, o tarda más de lo necesario
//            // problemas de voltaje en alguna fase, alguna pegada en 0 por mas de 10 ciclos
//            // no hay alimentación
//            va = (float)(AdcaResultRegs.ADCRESULT1);
//            vb = (float)(AdcaResultRegs.ADCRESULT2);
//            vc = (float)(AdcaResultRegs.ADCRESULT3);
//            if (((va<v_phase_max) || (va>v_phase_min)) || ((vb<v_phase_max) || (vb>v_phase_min)) || ((vc<v_phase_max) || (vc>v_phase_min)))
//            {
//                cont_precarg++;
//            }
//            else
//            {
//                cont_precarg = 0;
//            }
//            if (cont_precarg >= 30)
//            {
//                fault = 1;
//            }
//        break;
//        case normal_mode:
//            // Estado trabajo normal
//            // sobrecorriente, no se como probarlo
//            // tensión elevada
//            // tensión 0
//            // problema en alguna fase
//            // no hay alimentación
//            // Medicion v*i out no debe superar los 5.5kw, 10% más de la potencia nominal.
//            if (vdc > vdc_max)
//            {
//                cont_normal ++;
//            }
//            else
//            {
//                cont_normal = 0;
//            }
//            if (((va<v_phase_max) || (va>v_phase_min)) || ((vb<v_phase_max) || (vb>v_phase_min)) || ((vc<v_phase_max) || (vc>v_phase_min)))
//            {
//                cont_norm_fase++;
//            }
//            else
//            {
//                cont_norm_fase = 0;
//            }
//            if ((cont_normal >= 150) || (vdc < vdc_charged) || (cont_norm_fase >= 30) || (vdc*i_out>pot_max))
//            {                                                       // cuando se enciende el estado, sube la tensión
//                fault = 1;                                          // en la carga a más  de 850, pero es un tiempo
//            }                                                       // breve, menor a 0.003[s], ese tiempo corresponde
//                                                                    // a 150 ciclos de ejecución del código cla.
//        break;
//        default:
//        break;
//    }
//
////    fault = 0;  // Definido para saltarse esta función. Para utilizar estado de fallas, comentar esta linea
//}
//*/
//
////#define idle            1
////#define sync_pll        2
////#define pre_charge      3
////#define normal_mode     4
////#define apagado         5
////#define fault_mode      6
//
///*void corrimiento_izq(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8){
//    int i;
//    for (i = 0; i < 40; i++){
//        AdcBuf_ant_fall[i] = AdcBuf_ant_fall[i + 8];
//    }
//    AdcBuf_ant_fall[40] = v1;
//    AdcBuf_ant_fall[41] = v2;
//    AdcBuf_ant_fall[42] = v3;
//    AdcBuf_ant_fall[43] = v4;
//    AdcBuf_ant_fall[44] = v5;
//    AdcBuf_ant_fall[45] = v6;
//    AdcBuf_ant_fall[46] = v7;
//    AdcBuf_ant_fall[47] = v8;
//    return;
//}*/
//
//// End of File
//
