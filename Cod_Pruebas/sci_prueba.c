/*
 * sci_prueba.c
 *
 *  Created on: 04-06-2024
 *      Author: U1311149
 */




//###########################################################################
//
// FILE:    Example_2837xDSci_Echoback.c
//
// TITLE:   SCI Echoback.
//
//! \addtogroup cpu01_example_list
//! <h1>SCI Echoback (sci_echoback)</h1>
//!
//!  This test receives and echo-backs data through the SCI-A port.
//!
//!  The PC application 'hyperterminal' or another terminal
//!  such as 'putty' can be used to view the data from the SCI and
//!  to send information to the SCI.  Characters received
//!  by the SCI port are sent back to the host.
//!
//!  \b Running \b the \b Application
//!  -# Configure hyperterminal or another terminal such as putty:
//!
//!  For hyperterminal you can use the included hyperterminal configuration
//!  file SCI_96.ht.
//!  To load this configuration in hyperterminal
//!    -# Open hyperterminal
//!    -# Go to file->open
//!    -# Browse to the location of the project and
//!       select the SCI_96.ht file.
//!  -# Check the COM port.
//!  The configuration file is currently setup for COM1.
//!  If this is not correct, disconnect (Call->Disconnect)
//!  Open the File-Properties dialogue and select the correct COM port.
//!  -# Connect hyperterminal Call->Call
//!  and then start the 2837xD SCI echoback program execution.
//!  -# The program will print out a greeting and then ask you to
//!  enter a character which it will echo back to hyperterminal.
//!
//!  \note If you are unable to open the .ht file, or you are using
//!  a different terminal, you can open a COM port with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!  - LoopCount - the number of characters sent
//!
//! \b External \b Connections \n
//!  Connect the SCI-A port to a PC via a transceiver and cable.
//!  - GPIO28 is SCI_A-RXD (Connect to Pin3, PC-TX, of serial DB9 cable)
//!  - GPIO29 is SCI_A-TXD (Connect to Pin2, PC-RX, of serial DB9 cable)
//!
//
//###########################################################################
//
// $Release Date:  $
// $Copyright:
// Copyright (C) 2013-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include <stdio.h>
#include <stdbool.h>
//#include "F28x_Project.h"
#include "F2837xD_device.h"        // F2837xD Headerfile Include File
#include <stdint.h>

//
// Globals
//
Uint16 LoopCount;

//
// Function Prototypes
//
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scib_xmit(int a);
void scia_msg(char *msg);
Uint16 prueba1;
Uint16 prueba2;
#define largo_prueba 100
Uint16 prueba3[largo_prueba];
Uint16 adc_promedio;

//
//----------Controlador-de-voltaje_dcdc------//
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
//----------Controlador-de-voltaje_dcdc------//
#pragma DATA_SECTION(contador_l,"CpuToCla1MsgRAM");
float contador_l;
#pragma DATA_SECTION(frec_vf,"CpuToCla1MsgRAM");
float frec_vf;
#pragma DATA_SECTION(voltaje_vf,"CpuToCla1MsgRAM");
float voltaje_vf;
#pragma DATA_SECTION(contador_g,"Cla1ToCpuMsgRAM");
float contador_g;
#pragma DATA_SECTION(duty_va,"Cla1ToCpuMsgRAM");
float duty_va;
#pragma DATA_SECTION(duty_vb,"Cla1ToCpuMsgRAM");
float duty_vb;
#pragma DATA_SECTION(duty_vc,"Cla1ToCpuMsgRAM");
float duty_vc;
#pragma DATA_SECTION(resultado,"Cla1ToCpuMsgRAM");
float resultado[15];
#pragma DATA_SECTION(resultado2,"Cla1ToCpuMsgRAM");
double resultado2;
#pragma DATA_SECTION(resultado3,"Cla1ToCpuMsgRAM");
double resultado3;
//
int i;
int ii;
#define PWM_TBPRD 2500 // Frecuencia de
#define PWM_CMPA   1250  // Frecuencia de X hz
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


#define m_semik_vdc_0 0.3218
#define y_semik_vdc_0 1.3263
uint16_t brr_2;


extern void InitEPwm1Gpio(void);
void ADC_initAdcABCD(void);
void EPWM_initEpwm(void);
void SetupADCEpwm2(void);
void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel);
__interrupt void adca1_isr2();
__interrupt void adcb1_isr2();
__interrupt void adcd1_isr2();
extern void InitEPwm1Gpio(void);

//#define __TI_COMPILER_VERSION__

//uint8_t asd;
//int8_t asdf;

//uint8 asdff;

//
// Main
//
void main(void)
{
    Uint16 ReceivedChar;
    char *msg;

//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
     InitSysCtrl();
     EALLOW;
     ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0x1;
     EDIS;

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
   InitGpio();

//
// For this example, only init the pins for the SCI-A port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
#ifdef _LAUNCHXL_F2837xD
   GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
   GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
   GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
#else
   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
#endif

//   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
//   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
//   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
//   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

//
// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();
//
// Step 2. enable PWM1 and PWM2 and their GPIOs
//
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; //  PWM1A y PWM1B   se utilizan de forma interna para hacer trigger al adc

//
// For this case just init GPIO pins for ePWM1, ePWM2
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();
EALLOW;
// Estos registros se modifican de forma directa ya que la función macro no permite inicializarlos de esa forma.
//// Inicialización ePWM1
//    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
//    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 1;  // Configure GPIO0 as EPWM1A
//    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 1;  // Configure GPIO1 as EPWM1B
//    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

//        GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO14 (EPWM8A)
//        GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO15 (EPWM8B)
//        GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 1;   // Configure GPIO14 as EPWM8A
//        GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 1;   // Configure GPIO15 as EPWM8B
//        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO14 as EPWM8A
//        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO15 as EPWM8B
EDIS;
//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;
//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
   InitPieVectTable();

   EALLOW;
//    PieVectTable.ADCA1_INT = &adca1_isr2; //function for ADCD interrupt 1
   PieVectTable.ADCA1_INT = &adca1_isr2; //function for ADCA interrupt 1
   PieVectTable.ADCB1_INT = &adcb1_isr2; //function for ADCB interrupt 1
   PieVectTable.ADCD1_INT = &adcd1_isr2; //function for ADCB interrupt 1

   EDIS;

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
   PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

//
// Step 8. Turn on the EPWM. Se configura para que el ePWM1A haga trigger a la conversión de los ADC.
//
   EALLOW;
   CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

   EPwm1Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
//   EPwm8Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
//    EPwm1Regs.ETSEL.bit.SOCBEN  = 1; // Habilita pulso SOCB  (EPWMxSOCB)
   EPwm1Regs.TBCTL.bit.CTRMODE  = TB_COUNT_UPDOWN; // Contador entra en up-down count mode
   EDIS;
//
// Step 4. User specific code:
//

   ii=0;

   LoopCount = 0;

   scia_fifo_init();       // Initialize the SCI FIFO
   scia_echoback_init();   // Initialize SCI for echoback

//   msg = "\r\n\n\nHello World!\0";
//   scia_msg(msg);
//   msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
//   scia_msg(msg);

   for(;;)
   {

//       adc_promedio = ((AdcbResultRegs.ADCRESULT0)+(AdcbResultRegs.ADCRESULT1)+(AdcbResultRegs.ADCRESULT2)+(AdcbResultRegs.ADCRESULT3)+(AdcbResultRegs.ADCRESULT4))/5;
//
//       scia_xmit((AdcaResultRegs.ADCRESULT0));
//       scia_xmit((AdcaResultRegs.ADCRESULT0)>>8);

   //    scia_xmit((adc_promedio));
   //    scia_xmit((adc_promedio)>>8);

//       adc_promedio = (AdcbResultRegs.ADCRESULT0)*m_semik_vdc_0 + y_semik_vdc_0;


//       adc_promedio = AdcbResultRegs.ADCRESULT0;
//       scia_xmit((adc_promedio));
//       scia_xmit((adc_promedio)>>8);




//       msg = "\r\nEnter a character: \0";
//       scia_msg(msg);
       //
       // Wait for inc character
       //
//       while(SciaRegs.SCIFFRX.bit.RXFFST == 0) { } // wait for empty state
       //
       // Get character
       //
//       ReceivedChar = SciaRegs.SCIRXBUF.all;
       //
       // Echo character back
       //
//       msg = "  You sent: \0";
//       scia_msg(msg);
//       scia_xmit(ReceivedChar);

//       for(i = 0; i < 64; i++){
//       for(i = 1000; i < 1500; i++){
//           scia_xmit(i);
////           DELAY_US(1000*1);
//       }

//       DELAY_US(1000*1000);
//      for(i = 0; i < 4096; i++){
//          scia_xmit(i);
//          scia_xmit(i>>8);
//          DELAY_US(1000*1);
//      }

//      scia_xmit(AdcaResultRegs.ADCRESULT0);
//      scia_xmit((AdcaResultRegs.ADCRESULT0)>>8);
//
//
//       DELAY_US(1000*5);
//       i=0;
//       scia_msg("300\n");
//       scia_msg("3000\n");


//       scia_xmit("151\n");
//       prueba1 = 5;
//       prueba2 = prueba1>>8;

//       scia_xmit(prueba1);
//       scia_xmit(prueba1>>8);
//       scia_xmit(303);
//       scia_xmit(607);
//       scia_msg("151\n");
//       scia_msg("151\n");



//       LoopCount++;
   }
}

//
//  scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
void scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //
////---------------------------------------//
////---------------------------------------//
////---------------------------------------//
//    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    SciaRegs.SCICCR.bit.SCICHAR = 0x7;   // largo cadena a enviar 8 bits
    SciaRegs.SCICCR.bit.PARITYENA = 0X0; // Parity bit disabled;
    SciaRegs.SCICCR.bit.STOPBITS = 0x0;  // 1 stop bit.
    SciaRegs.SCICCR.bit.LOOPBKENA = 0x0; // No loopback

//    SciaRegs.SCICTL1.bit.

    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
//    SciaRegs.SCIHBAUD.all = 0x0002;
//    SciaRegs.SCILBAUD.all = 0x008B;

    // Calculate BAUDRATE
    uint64_t LSPCLK, BRR, baud_rate;
//    baud_rate = 57600;
    baud_rate = 115200;
    LSPCLK = (200e6/2);     // SYSCLK/LOSPCP
    BRR = (LSPCLK)/(8*baud_rate)-1;

    brr_2 = BRR;
//    SciaRegs.SCIHBAUD.all = (1301>>8) & 0xFF;
//    SciaRegs.SCILBAUD.all = 1301 & 0xFF;

    SciaRegs.SCIHBAUD.all = (BRR>>8) & 0xFF;
    SciaRegs.SCILBAUD.all = BRR & 0xFF;

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
////---------------------------------------//
////---------------------------------------//
////---------------------------------------//
//    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
//                                    // No parity,8 char bits,
//                                    // async mode, idle-line protocol
//    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
//                                    // Disable RX ERR, SLEEP, TXWAKE
//    ScibRegs.SCICTL2.all = 0x0003;
//    ScibRegs.SCICTL2.bit.TXINTENA = 1;
//    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
//
//    //
//    // SCIA at 9600 baud
//    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
//    //
//    ScibRegs.SCIHBAUD.all = 0x0002;
//    ScibRegs.SCILBAUD.all = 0x008B;
//
//    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset

//    EALLOW;
//        GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // Configura GPIO28 como SCIRXDA
//        GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // Configura GPIO29 como SCITXDA
//
//        SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit, No loopback, No parity, 8 char bits, async mode, idle-line protocol
//        SciaRegs.SCICTL1.all = 0x0003;  // Enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
//        SciaRegs.SCICTL2.all = 0x0003;  // Enable TX, RX
//
//        // Configura el baud rate a 19200
//        SciaRegs.SCIHBAUD.all = 0x0003;  // Baud rate high (3 para 19200)
//        SciaRegs.SCILBAUD.all = 0x007F;  // Baud rate low (127 para 19200)
//
//        SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
//    EDIS;

}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all =a;
}
//
// scia_msg - Transmit message via SCIA
//
void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init()
{
//---------------------------------------//
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
//---------------------------------------//
}

//
// ADC_initAdc - Initialize ADC A,B,C,D configurations and power it up
//
void ADC_initAdcABCD(void)
{
    EALLOW;
    // encender adca
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK divider to /4
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0x4; //set ADCCLK divider to /4
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
//
void EPWM_initEpwm(void)
{
    EPwm1Regs.TBPRD           = PWM_TBPRD;  // Frecuencia de 2khz
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter
    EPwm1Regs.TBPHS.bit.TBPHS = 0;          // Set Phase register to zero

    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Enable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;  // Divide el reloj en 1
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV2;  // Divide el reloj en 1
    //
    // Setup compare
    //
    EPwm1Regs.CMPA.bit.CMPA = PWM_CMPA;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;

    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
}

void SetupADCEpwm2(void)
{
    Uint16 acqps;
    Uint16 trigsel;
//    acqps = 200; //1us -> (1e-6)/(5e-9), Mínimo son 75, set by ACQPS and  PERx.SYSCLK
//    acqps = 500;
    acqps = 50;
//    acqps = 50;
    trigsel = 5;

    EALLOW;
//    SetupSOC(socA0, 0, acqps, 0x5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA1, 1, acqps, 0x5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(socA0, 0, acqps, 0x5);
//    SetupSOC(socA1, 0, acqps, 0x5);
//    SetupSOC(socA2, 0, acqps, 0x5);
//    SetupSOC(socA3, 0, acqps, 0x5);
//    SetupSOC(socA4, 0, acqps, 0x5);

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x0;  //SOC0 will convert pin A1
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x2;  //SOC0 will convert pin A2

    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C


    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x0;  //SOC0 will convert pin A0
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0x1;  //SOC0 will convert pin A0
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0x2;  //SOC0 will convert pin A0
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0x3;  //SOC0 will convert pin A0
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x4;  //SOC0 will convert pin A0
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 0x5;  //SOC0 will convert pin A0

    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0x0;  //SOC0 will convert pin A0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0x1;  //SOC0 will convert pin A0
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 0x2;  //SOC0 will convert pin A0
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 0x3;  //SOC0 will convert pin A0

    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window

    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x5; //fin de EOC5 genera INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag

//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //fin de SOCA1 genera INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag


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
                AdcaRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
                AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA3:
            {
                AdcaRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
                AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case socA4:
            {
                AdcaRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
                AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
        }
    }

interrupt void adca1_isr2(void)
{
//    if (ii < largo_prueba){
//
//        prueba3[ii] = (AdcaResultRegs.ADCRESULT0);
//        ii=ii+1;
//    }
//    else
//    {
//        ii = 0;
//    }

    adc_promedio = ((AdcbResultRegs.ADCRESULT0)+(AdcbResultRegs.ADCRESULT1)+(AdcbResultRegs.ADCRESULT2)+(AdcbResultRegs.ADCRESULT3)+(AdcbResultRegs.ADCRESULT4))/5;

//    scia_xmit((AdcaResultRegs.ADCRESULT0));
//    scia_xmit((AdcaResultRegs.ADCRESULT0)>>8);


//    scia_xmit((adc_promedio));
//    scia_xmit((adc_promedio)>>8);


//     DELAY_US(1000*5);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP2);

}
interrupt void adcb1_isr2(void)
{
//    adc_promedio = ((AdcbResultRegs.ADCRESULT0)+(AdcbResultRegs.ADCRESULT1)+(AdcbResultRegs.ADCRESULT2)+(AdcbResultRegs.ADCRESULT3)+(AdcbResultRegs.ADCRESULT4)+(AdcdResultRegs.ADCRESULT0)+(AdcdResultRegs.ADCRESULT1)+(AdcdResultRegs.ADCRESULT2)+(AdcdResultRegs.ADCRESULT3)+(AdcdResultRegs.ADCRESULT4));
//    adc_promedio = ((AdcdResultRegs.ADCRESULT0)+(AdcdResultRegs.ADCRESULT1)+(AdcdResultRegs.ADCRESULT2)+(AdcdResultRegs.ADCRESULT3)+(AdcdResultRegs.ADCRESULT4))/5;

//    scia_xmit((adc_promedio));
//    scia_xmit((adc_promedio)>>8);

//
//-----Forma A de enviar datos-----//
//
//    scia_xmit((AdcbResultRegs.ADCRESULT0));
//    scia_xmit((AdcbResultRegs.ADCRESULT0)>>8);
//
//    scia_xmit((AdcbResultRegs.ADCRESULT1));
//    scia_xmit((AdcbResultRegs.ADCRESULT1)>>8);
//
//    scia_xmit((AdcbResultRegs.ADCRESULT2));
//    scia_xmit((AdcbResultRegs.ADCRESULT2)>>8);
//
//    scia_xmit((AdcbResultRegs.ADCRESULT3));
//    scia_xmit((AdcbResultRegs.ADCRESULT3)>>8);
//
//-----Forma B de enviar datos-----//
//
    Uint16 a,b,c,d, e, f, g, h;
    a = AdcbResultRegs.ADCRESULT0;
    b = AdcbResultRegs.ADCRESULT1;
    c = AdcbResultRegs.ADCRESULT2;
    d = AdcbResultRegs.ADCRESULT3;

    e = AdcaResultRegs.ADCRESULT0;
    f = AdcaResultRegs.ADCRESULT1;

    g = AdcbResultRegs.ADCRESULT3;
    h = AdcbResultRegs.ADCRESULT3;

//    a = AdcbResultRegs.ADCRESULT5;
//    b = AdcdResultRegs.ADCRESULT0;
    c = AdcdResultRegs.ADCRESULT1;
    d = AdcbResultRegs.ADCRESULT2;
    e = AdcbResultRegs.ADCRESULT3;

    b = AdcaResultRegs.ADCRESULT1;




    a = 1000;
    b = 2000;
    c = 3000;
    d = 4000;

    scia_xmit(((a>>6))|(0x80));
    scia_xmit(a&0x13F);
//    DELAY_US(1000*1);

    scia_xmit(((b>>6))&(0x13F));
    scia_xmit(b&0x13F);
//    DELAY_US(1000*1);

    scia_xmit(((c>>6))&(0x13F));
    scia_xmit(c&0x13F);
//    DELAY_US(1000*1);

    scia_xmit(((d>>6))&(0x13F));
    scia_xmit(d&0x13F);
//    DELAY_US(1000*1);

    scia_xmit(((e>>6))&(0x13F));
    scia_xmit(e&0x13F);
//    DELAY_US(1000*1);
    scia_xmit(((f>>6))&(0x13F));
    scia_xmit(f&0x13F);
//    DELAY_US(1000*1);



//    scia_xmit(a);
//     DELAY_US(1000*1);

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    //
    // Check if overflow has occurred
    //
    if(1 == AdcbRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1);
}
interrupt void adcd1_isr2(void)
{
    adc_promedio = ((AdcdResultRegs.ADCRESULT0)+(AdcdResultRegs.ADCRESULT1)+(AdcdResultRegs.ADCRESULT2)+(AdcdResultRegs.ADCRESULT3)+(AdcdResultRegs.ADCRESULT4))/5;



    scia_xmit((adc_promedio));
    scia_xmit((adc_promedio)>>8);

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    //
    // Check if overflow has occurred
    //
    if(1 == AdcdRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcdRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1);
}

//
// End of file
//
