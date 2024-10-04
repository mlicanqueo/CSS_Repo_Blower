//#include "F28x_Project.h"
//
//// Prototipos de funciones
//void InitEPwm(void);
//void InitADC(void);
//void InitSCIA(void);
//__interrupt void adc_isr(void);
//void scia_xmit(uint16_t a);
//
//void main(void) {
//    InitSysCtrl();  // Inicializa el controlador del sistema
//    InitGpio();     // Inicializa los GPIOs
//
//    DINT;           // Desactiva todas las interrupciones de la CPU
//
//    InitPieCtrl();  // Inicializa el controlador PIE
//    IER = 0x0000;   // Desactiva todas las interrupciones de la CPU
//    IFR = 0x0000;   // Borra todos los flags de las interrupciones
//
//    InitPieVectTable(); // Inicializa la tabla de vectores PIE
//
//    // Mapea la interrupción del ADC al ISR
//    EALLOW;
//    PieVectTable.ADCA1_INT = &adc_isr;
//    EDIS;
//
//    InitEPwm();     // Inicializa el módulo PWM
//    InitADC();      // Inicializa el módulo ADC
//    InitSCIA();     // Inicializa el módulo SCIA
//
//    // Habilita la interrupción del ADC en el PIE y la CPU
//    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//    IER |= M_INT1;
//    EINT;           // Habilita las interrupciones globales
//    ERTM;           // Habilita el modo de depuración en tiempo real
//
//    while(1) {
//        // Bucle principal
//    }
//}
//
//void InitEPwm(void) {
//    EALLOW;
//    // Configura EPWM1 para generar una señal PWM
//    EPwm1Regs.TBPRD = 500;  // Período del contador del temporizador
//    EPwm1Regs.TBPHS.bit.TBPHS = 0;
//    EPwm1Regs.TBCTR = 0;
//
//    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  // Modo de conteo
//    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
//    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
//    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//
//    EPwm1Regs.CMPA.bit.CMPA = 250;  // Valor de comparación
//    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//
//    // Configura el disparo del ADC
//    EPwm1Regs.ETSEL.bit.SOCAEN = 1; // Habilita el disparo SOCA
//    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA; // Dispara en CMPA al contar hacia arriba
//    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST; // Genera un disparo en el primer evento
//
//    EDIS;
//}
//
//void InitADC(void) {
//    EALLOW;
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Configura el prescaler del ADC
//    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Habilita el ADC
//    DELAY_US(1000); // Espera 1ms para que el ADC esté listo
//
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Genera la interrupción al final de la conversión
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0; // Selecciona el canal 0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; // Configura el tiempo de muestreo
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Selecciona el disparo de EPWM1 SOCA
//
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // Configura la interrupción ADCINT1 al final de la conversión de SOC0
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // Habilita la interrupción ADCINT1
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Borra el flag de la interrupción
//    EDIS;
//}
//
//void InitSCIA(void) {
//    EALLOW;
//    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // Configura GPIO28 como SCIRXDA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // Configura GPIO29 como SCITXDA
//
//    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback, No parity, 8 char bits, async mode, idle-line protocol
//    SciaRegs.SCICTL1.all = 0x0003;  // Enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
//    SciaRegs.SCICTL2.all = 0x0003;  // Enable TX, RX
//    SciaRegs.SCICTL2.bit.TXINTENA = 1;
//    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
//    SciaRegs.SCIHBAUD.all = 0x0002;
//    SciaRegs.SCILBAUD.all = 0x008B;
//    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
//    EDIS;
//}
//
//void scia_xmit(uint16_t a) {
////    while (SciaRegs.SCICTL2.bit.TXRDY == 0) {}
//    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
//    SciaRegs.SCITXBUF.all = a;
//}
//
//__interrupt void adc_isr(void) {
//    // Código a ejecutar al finalizar la conversión del ADC
//    uint16_t adc_result = AdcaResultRegs.ADCRESULT0;
//
//    // Transmitir el resultado por SCIA
//    scia_xmit(adc_result & 0xFF);         // Transmite el byte bajo
//    scia_xmit((adc_result >> 8) & 0xFF);  // Transmite el byte alto
//
////    scia_xmit((AdcaResultRegs.ADCRESULT0));
////    scia_xmit((AdcaResultRegs.ADCRESULT0)>>8);
//
//    // Borra el flag de la interrupción
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
//
//    // Avisa al PIE que la interrupción ha sido atendida
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}

//#include "F28x_Project.h"
//
//// Prototipos de funciones
//void InitEPwm(void);
//void InitADC(void);
//void InitSCIA(void);
//__interrupt void adc_isr(void);
//void scia_xmit(uint16_t a);
//
//#define NUM_SAMPLES 20
//int i;
//
//uint16_t adcResults[NUM_SAMPLES];
//uint16_t sampleIndex = 0;
//
//void main(void) {
//    InitSysCtrl();  // Inicializa el controlador del sistema
//    InitGpio();     // Inicializa los GPIOs
//
//    DINT;           // Desactiva todas las interrupciones de la CPU
//
//    InitPieCtrl();  // Inicializa el controlador PIE
//    IER = 0x0000;   // Desactiva todas las interrupciones de la CPU
//    IFR = 0x0000;   // Borra todos los flags de las interrupciones
//
//    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; //  PWM1A y PWM1B   se utilizan de forma interna para hacer trigger al adc
//    InitEPwm1Gpio();
//
//    InitPieVectTable(); // Inicializa la tabla de vectores PIE
//
//    // Mapea la interrupción del ADC al ISR
//    EALLOW;
//    PieVectTable.ADCA1_INT = &adc_isr;
//    EDIS;
//
//    InitEPwm();     // Inicializa el módulo PWM
//    InitADC();      // Inicializa el módulo ADC
//    InitSCIA();     // Inicializa el módulo SCIA
//
//    // Habilita la interrupción del ADC en el PIE y la CPU
//    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//    IER |= M_INT1;
//    EINT;           // Habilita las interrupciones globales
//    ERTM;           // Habilita el modo de depuración en tiempo real
//
//    EPwm1Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
//
//    while(1) {
//        // Bucle principal
//    }
//}
//
//void InitEPwm(void) {
//    EALLOW;
//    // Configura EPWM1 para generar una señal PWM con periodo de 1 ms
//    EPwm1Regs.TBPRD = 1000;  // Período del contador del temporizador (1 ms)
//    EPwm1Regs.TBPHS.bit.TBPHS = 0;
//    EPwm1Regs.TBCTR = 0;
//
//    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;  // Modo de conteo
//    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
//    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
//    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//
//    EPwm1Regs.CMPA.bit.CMPA = 500;  // Valor de comparación
//    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//
//    // Configura el disparo del ADC
//    EPwm1Regs.ETSEL.bit.SOCAEN = 1; // Habilita el disparo SOCA
//    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA; // Dispara en CMPA al contar hacia arriba
//    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST; // Genera un disparo en el primer evento
//
//    EDIS;
//}
//
//void InitADC(void) {
//    EALLOW;
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Configura el prescaler del ADC
//    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Habilita el ADC
//    DELAY_US(1000); // Espera 1ms para que el ADC esté listo
//
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Genera la interrupción al final de la conversión
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0; // Selecciona el canal 0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; // Configura el tiempo de muestreo
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Selecciona el disparo de EPWM1 SOCA
//
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // Configura la interrupción ADCINT1 al final de la conversión de SOC0
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // Habilita la interrupción ADCINT1
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Borra el flag de la interrupción
//    EDIS;
//}
//
//void InitSCIA(void) {
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
//        SciaRegs.SCIHBAUD.all = 0x0002;
//        SciaRegs.SCILBAUD.all = 0x008B;
//
//
//        SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
//        EDIS;
//
//}
//
//void scia_xmit(uint16_t a) {
////    while (SciaRegs.SCICTL2.bit.TXRDY == 0) {}
//    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
//    SciaRegs.SCITXBUF.all = a;
//}
//
//__interrupt void adc_isr(void) {
//    // Almacenar el resultado del ADC
//    adcResults[sampleIndex] = AdcaResultRegs.ADCRESULT0;
//    sampleIndex++;
//
//    // Si se han tomado todas las muestras
//    if (sampleIndex >= NUM_SAMPLES) {
//        sampleIndex = 0;
//        for (i = 0; i < NUM_SAMPLES; i++) {
//            // Transmitir el resultado por SCIA
//            scia_xmit(adcResults[i] & 0xFF);         // Transmite el byte bajo
//            scia_xmit((adcResults[i] >> 8) & 0xFF);  // Transmite el byte alto
//
//        }
//    }
//
//    // Borra el flag de la interrupción
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
//
//    // Avisa al PIE que la interrupción ha sido atendida
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}

#include "F28x_Project.h"

void InitADC(void);
void InitSCI(void);
void InitEPWM(void);
void SendADCData(void);

void main(void)
{
    InitSysCtrl();           // Inicializa el sistema
    InitGpio();              // Inicializa GPIO

    InitADC();               // Inicializa ADC
    InitSCI();               // Inicializa SCI
    InitEPWM();              // Inicializa ePWM para disparar ADC

    // Habilitar interrupciones globales
    EINT;
    ERTM;

    while(1)
    {
        SendADCData();
        DELAY_US(100000);  // Espera 100 ms antes de enviar los siguientes datos
    }
}

void InitADC(void)
{
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // Configuración de SOC
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  // Selecciona el canal A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14; // Establece el periodo de adquisición
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // Configura el disparador para ePWM1

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  // Selecciona el canal A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14; // Establece el periodo de adquisición
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // Configura el disparador para ePWM1

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  // Selecciona el canal A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14; // Establece el periodo de adquisición
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // Configura el disparador para ePWM1

    EDIS;
}

void InitSCI(void)
{
    EALLOW;
    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity, 8 char bits, async mode
    SciaRegs.SCICTL1.all = 0x0003;  // habilitar TX, RX, internal SCICLK,
                                    // Deshabilitar RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    SciaRegs.SCIHBAUD.all = 0x0001;  // Configurar velocidad de transmisión a 9600
    SciaRegs.SCILBAUD.all = 0x0044;
    SciaRegs.SCICTL1.all = 0x0023;  // Liberar SCI de Reset
    EDIS;
}

void InitEPWM(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; // Habilitar reloj ePWM1

    EPwm1Regs.TBCTL.bit.CTRMODE = 0;   // Up-count mode
    EPwm1Regs.TBPRD = 50000;           // Periodo del timer
    EPwm1Regs.TBCTL.bit.PHSEN = 0;     // Deshabilitar carga de fase
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000; // Fase inicial
    EPwm1Regs.TBCTR = 0x0000;          // Contador inicial

    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // Set en contador hasta periodo
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear en contador hasta cero

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Habilitar SOC en A
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // Seleccionar evento en contador hasta periodo
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;    // Generar pulsos en cada evento
    EDIS;
}

void SendADCData(void)
{
    uint16_t adcA0, adcA1, adcA2;
    char buffer[20];
    Uint16 i;
    i=0;
    // Leer los valores de los ADC
    adcA0 = AdcaResultRegs.ADCRESULT0;
    adcA1 = AdcaResultRegs.ADCRESULT1;
    adcA2 = AdcaResultRegs.ADCRESULT2;

    // Formatear los datos para enviar
    sprintf(buffer, "%u,%u,%u\n", adcA0, adcA1, adcA2);

    // Enviar datos por SCI
    for(i = 0; buffer[i] != '\0'; i++)
    {
//        while (SciaRegs.SCICTL2.bit.TXRDY == 0);
//        while (SciaRegs.SCICTL2.bit.TXRDY == 0) {}
        SciaRegs.SCITXBUF.all = buffer[i];
//        SciaRegs.SCITXBUF.all = a;

    }
}


//        for (i = 0; i < NUM_SAMPLES; i++) {
//            // Transmitir el resultado por SCIA
//            scia_xmit(adcResults[i] & 0xFF);         // Transmite el byte bajo
//            scia_xmit((adcResults[i] >> 8) & 0xFF);  // Transmite el byte alto
//
//        }

