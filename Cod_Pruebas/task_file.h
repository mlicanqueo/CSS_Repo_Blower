/*
 * task_file.h
 *
 *  Created on: 06-07-2020
 *      Author: Matias_L
 */
#ifndef TASK_FILE_H_
#define TASK_FILE_H_
//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
//
// Defines
//------------Variables-de-prueba------------//
extern float aaa;
//extern float V_high, V_low;
extern double resultado[15];
extern double resultado2;
extern double resultado3;
extern int16 PWM_CMPA_mod;
//extern float Num,Den; //A/B user input at the C28x side
//-------------------------------------------//
//----------Controlador-de-voltaje_dcdc------//
#define vdc_ref     100.0
//#define MAX_ACT_v   480.0 // es 160*3, ya que la corriente por pierna son 160A, pero el control de voltaje es para las 3 fases
#define MAX_ACT_v    40.0    // es 160*3, ya que la corriente por pierna son 160A, pero el control de voltaje es para las 3 fases
#define IMAX         40.0    // se usa para el control de corriente
#define MIN_ACT_v    0.0    //asdasdsad
#define IMAX_est   250.0    // ahora no se ocupa


//#define KNO_v        0.0267
//#define PIAWU_N1_v  -0.0092
//#define PIAWU_D1_v   1.0

//#define KNO_v        0.0501
//#define PIAWU_N1_v  -0.0398
//#define PIAWU_D1_v   0.9980

#define KNO_v        0.0100
#define PIAWU_N1_v  -0.0500
#define PIAWU_D1_v   0.9995


extern double x_v_l;
extern double x_v_g;
extern double x_ant_v_l;
extern double x_ant_v_g;
//-------------------------------------------//
//---------Controlador-de-corriente----------//
//#define KN0_i       10.364817
#define KN0_i         0.0010
#define MAX_ACT_i     0.99
#define MIN_ACT_i     0.01
#define PIAWU_D1_i    0.9980
#define PIAWU_N1_i   -1.9920

#define DMAX          0.98
#define DMIN          0.00
//#define IMAX          300

#define IMIN          0.00

#define Fpwm_dcdc     5000
#define Tpwm_dcdc     0.0002
#define Ts_dcdc       0.0002
#define Fpwm_vf       2500
//#define L0            1
#define L0            0.000522393871688478
#define alpha         0.0000014179699
//#define alpha         3

extern uint16_t i_count;
extern double x_i_l[3];
extern double x_i_g[3];

extern double x_ant_i_l[3];
extern double x_ant_i_g[3];

extern uint16_t i_count_g;
extern uint16_t i_count_l;
//-------------------------------------------//
//----------Controlador-de-voltaje_dcdc------//
extern float contador_l;
extern float contador_g;
extern float frec_vf;
extern float voltaje_vf;
extern float duty_va;
extern float duty_vb;
extern float duty_vc;
//-------------------------------------------//
//------------Variables-Lectura-ADC---------//
// Para interpretar la lectura del adc, se deben trazar una curva de 1° orden de la forma y = mx+n,
// donde y es la interpretación de lectura y x es el numero de cuentas de ADC.

#define m_vdc       0.4876
#define n_vdc    -995.12
//#define m_vg      0.170940171
#define m_i         0.1433
#define n_i      -314.92
//#define m_iout    0.003663004
#define m_i_2       0.1464
#define n_i_2    -303.89
#define N_vueltas   9.0
//#define n_vg        350
//#define n_iout      0.0

#define pi          3.14159265
#define sqrt_2      1.4142
#define sqrt_3      1.7321
#define f           50.0
//#define L           0.00005 // (500e-6)
#define h_sample    0.00002 // (1/50k)
#define raiz_de_3   1.73205
#define uno_div_3   0.33334
#define dos_div_3   0.66667
//-------------------------------------------//
//
// Globals
//
//------------maquina-de-estados-------------//
//estados
#define idle            1
#define pre_charge      2
#define boost_ON        3
#define normal_mode     4
#define apagado         5
#define fault_mode      6
extern float adc_read[8];
////cambio de estados
////#define boton_1         1
////#define boton_2         0
////-------------------------------------------//
//
////-------------Variables-fallas--------------//
//#define vg_d_min     280
//#define vg_q_max     10
//#define vg_q_min    -10
//#define vdc_max      850
//#define vdc_charged  530
//#define v_phase_max  5
//#define v_phase_min -5
//#define pot_max      5500
//
////-------------------------------------------//
//
////------------Variables-de-prueba------------//
//extern float voltFilt;
//extern float Num,Den; //A/B user input at the C28x side
//extern float Res;  //Final Result used in C28x code
//extern float salida_prueba;
//extern float a1;
//extern float a2;
//extern float a3;
//extern float a4;
//extern float a5;
//extern float a6;
//extern float a7;
//extern float a8;
//extern float a9;
//extern float a10;
//extern float vect[6];
////-------------------------------------------//
//
////-------------Variables-generales-----------//
//#define pi          3.141592
//#define f           50.0
//#define L           0.00005 // (500e-6)
//#define h_sample    0.00002 // (1/50k)
//#define raiz_de_3   1.73205
//#define uno_div_3   0.33334
//#define dos_div_3   0.66667
////-------------------------------------------//
//
////------------Variables-Lectura-ADC---------//
//// Para interpretar la lectura del adc, se deben trazar una curva de 1° orden de la forma y = mx+n,
//// donde y es la interpretación de lectura y x es el numero de cuentas de ADC.
//
//#define m_vdc       0.244200244
//#define m_vg        0.170940171
//#define m_ig        0.017094017
//#define m_iout      0.003663004
//
//#define n_vdc       0.0
//#define n_vg        350
//#define n_ig        20
//#define n_iout      0.0
//
//#define pi          3.141592
//#define f           50.0
//#define L           0.00005 // (500e-6)
//#define h_sample    0.00002 // (1/50k)
//#define raiz_de_3   1.73205
//#define uno_div_3   0.33334
//#define dos_div_3   0.66667
////-------------------------------------------//
//
////------------variables-SRF-pll--------------//
////#define kp_pll       8.78568
////#define ki_pll       78956.83
//#define kp_pll       3.1817    // bw 700, xi = 0.7071
//#define ki_pll       1574.9196 // bw 700, xi = 0.7071
//
//#define lim_up_pll   6.283184
//
//extern float theta_l;
//extern float theta_g;
//extern float theta_ant_l;
//extern float theta_ant_g;
//extern float integrador_ant_l;
//extern float integrador_ant_g;
//extern float integrador_l;
//extern float integrador_g;
////-------------------------------------------//
//
////------------Variables-DDSRF-PLL------------//
//#define omega_lpf 222.1441
//
//extern float vg_d_men_prom_l;
//extern float vg_q_men_prom_l;
//extern float vg_d_mas_prom_l;
//extern float vg_q_mas_prom_l;
//
//extern float vg_d_men_ref_ant_l;
//extern float vg_d_men_prom_ant_l;
//extern float vg_q_men_ref_ant_l;
//extern float vg_q_men_prom_ant_l;
//
//extern float vg_d_mas_ref_ant_l;
//extern float vg_d_mas_prom_ant_l;
//extern float vg_q_mas_ref_ant_l;
//extern float vg_q_mas_prom_ant_l;
//
//extern float integr_ddsrf_l;
//extern float integr_ddsrf_ant_l;
//extern float theta_ddsrf_ant_l;
//extern float theta_ddsrf_l;
//
//
//extern float vg_d_men_prom_g;
//extern float vg_q_men_prom_g;
//extern float vg_d_mas_prom_g;
//extern float vg_q_mas_prom_g;
//
//extern float vg_d_men_ref_ant_g;
//extern float vg_d_men_prom_ant_g;
//extern float vg_q_men_ref_ant_g;
//extern float vg_q_men_prom_ant_g;
//
//extern float vg_d_mas_ref_ant_g;
//extern float vg_d_mas_prom_ant_g;
//extern float vg_q_mas_ref_ant_g;
//extern float vg_q_mas_prom_ant_g;
//
//extern float integr_ddsrf_g;
//extern float integr_ddsrf_ant_g;
//extern float theta_ddsrf_ant_g;
//extern float theta_ddsrf_g;
////-------------------------------------------//
//
////----------Controlador-de-voltaje-----------//
//
//#define kpv1         0.000019
//
//#define vdc_ref      750.0
//#define MAX_ACT_v    22.71 // 3*irms(7.57)
//#define MIN_ACT_v   -22.71
//#define PIAWU_D1_v   0.9825
//#define PIAWU_N1_v  -901.0866
//#define KNO_v        0.00001938
//
//extern float x_v_l;
//extern float x_v_g;
//extern float x_ant_v_l;
//extern float x_ant_v_g;
////-------------------------------------------//
//
////---------Controlador-de-corriente-d--------//
//#define KN0_i       10.364817
//#define MAX_ACT_i   217.78889
//#define MIN_ACT_i  -217.78889
//#define PIAWU_N1_i -0.014699
//#define PIAWU_D1_i  0.847645
//
//extern float x_id_l;
//extern float x_id_g;
//
//extern float x_ant_id_l;
//extern float x_ant_id_g;
////-------------------------------------------//
//
////---------Controlador-de-corriente-q--------//
//#define iq_ref      0
//
//extern float x_ant_iq_l;
//extern float x_ant_iq_g;
//
//extern float x_iq_l;
//extern float x_iq_g;
////-------------------------------------------//
////--------------dq-a-alpha_beta--------------//
//
////-------------------------------------------//
////--------------alpha_beta-a-abc-------------//
//extern float v_conv_a;
//extern float v_conv_b;
//extern float v_conv_c;
////-------------------------------------------//
//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"

#endif /* TASK_FILE_H_ */
//
// End of file
//
