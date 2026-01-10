/***********************************************************************
ECE 4624 - Fall 2025 - Lab 4

Author: 

***********************************************************************/

// for fixed-width integers
#include <stdint.h>
#include <stdio.h>
// HAL includes
#include "hal_data.h"
#include "bsp_api.h"
#include <math.h>

// FSP/CMSIS includes for SIMD Intrinics
#include "core_cm4.h"

#define M 10
#define Ns 5 // number of sections
#define N_scales 6 // numbers of scales
#define Q15_SCALE 32768.0f
#define Q14_SCALE 16384.0f
#define DAC_OUT BSP_IO_PORT_00_PIN_14
#define PACK(op1, op2) ( ((int32_t)(op2) << 16) | ((int16_t)(op1)) )
#define PORT1_PODR (*(volatile uint16_t *)(0x40040020)) 
#define Q14_SHIFT(x)   ((x) >> 14)
#define DAC_SHIFT(x)   ((x) >> 2)
//----------------------------------------------------------------------
// Definition of Memory Addresses and Offsets
// See the "Renesas RA4M1 Group -- User’s Manual: Hardware"
//----------------------------------------------------------------------
// ICU Base Address
#define SYSTEM 0x40010000 

// Protect Register
#define SYSTEM_PRCR ((volatile uint16_t *)(SYSTEM + 0xE3FE)) 

// System Clock Division Control Register
#define SYSTEM_SCKDIVCR ((volatile uint32_t *)(SYSTEM + 0xE020)) 

// Module Registers Base Address
#define MSTP 0x40040000 

// Module Stop Control Register D
#define MSTP_MSTPCRD ((volatile uint32_t *)(MSTP + 0x7008)) 

// ADC140 - 14-Bit A/D Converter Module
#define MSTPD16 16

// DAC12  - 12-Bit D/A Converter Module
#define MSTPD20 20 

// 14-Bit A/D Converter (ADC) Addresses
//-------------------------------------
// ADC Base Address
#define ADCBASE 0x40050000 

// A/D Control Register
#define ADC140_ADCSR ((volatile uint16_t *)(ADCBASE + 0xC000)) 

// A/D Channel Select Register A0
#define ADC140_ADANSA0 ((volatile uint16_t *)(ADCBASE + 0xC004)) 

// A/D Control Extended Register
#define ADC140_ADCER ((volatile uint16_t *)(ADCBASE + 0xC00E))  

// A1 (P000 AN00 AMP+)
#define ADC140_ADDR00 ((volatile uint16_t *)(ADCBASE + 0xC020)) 

// A/D Conversion 
#define ADCSR_ADST 15     

// A/D start-conversion bit
#define ADST (1 << ADCSR_ADST) 

// 12-Bit D/A Converter (DAC) Addresses
//-------------------------------------

// DAC Base - DAC output on A0 (P014 AN09 DAC)
#define DACBASE 0x40050000 

// D/A Data Register 0 
#define DAC12_DADR0 ((volatile uint16_t *)(DACBASE + 0xE000)) 

// D/A Control Register
#define DAC12_DACR ((volatile uint8_t  *)(DACBASE + 0xE004))

// DADR0 Format Select Register
#define DAC12_DADPR ((volatile uint8_t  *)(DACBASE + 0xE005)) 

// D/A A/D Synchronous Start Control Register
#define DAC12_DAADSCR ((volatile uint8_t  *)(DACBASE + 0xE006)) 

// D/A VREF Control Register
#define DAC12_DAVREFCR ((volatile uint8_t  *)(DACBASE + 0xE007)) 

// Ports Addresses
//------------------

// Port Base Address 
#define PORTBASE 0x40040000 

// Port 0 Pin Function Select Register
#define P000PFS 0x0800  

// Port 0 Mode Control
#define PFS_P014PFS ((volatile uint32_t *)(PORTBASE + P000PFS + (14 * 4)))
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Adjustable Parameters
//----------------------------------------------------------------------
#define ADCLK_RATE 2 // rate divider power: 0,1,2,3,4,5,6
//----------------------------------------------------------------------

// current input 
volatile uint16_t current_input = 0;

// current output
volatile uint16_t current_output = 0;

// helper function to setup the ADC
void setup_adc(void){

// clear the ADST bit to stop scanning
*ADC140_ADCSR &= 0x7FFF; 

// Enable writing to the clock registers
*SYSTEM_PRCR = 0xA501;

// zero all PCKC bits
*SYSTEM_SCKDIVCR &= 0xFFFFFF8F;

// set PCKC value
*SYSTEM_SCKDIVCR |= (ADCLK_RATE << 4);

// Disable writing to the clock registers
*SYSTEM_PRCR = 0xA500; 

// Enable ADC140 module
*MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD16)); 

// 14-bit resolution, no self-diagnosis, flush right
*ADC140_ADCER = 6; 

// set ANSA00 = A1
*ADC140_ADANSA0 |= (0x01 << 0); 
}

// helper function to setup DAC
void setup_dac(void){

// Enable DAC12 module
*MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD20)); 

// DADR0 Format Select Register - Set right-justified format
*DAC12_DADPR = 0x00; 

// D/A A/D Synchronous Start Control Register - Default
*DAC12_DAADSCR = 0x00; 

// D/A VREF Control Register - Write 0x00 first
*DAC12_DAVREFCR = 0x00; 

// D/A Data Register 0
*DAC12_DADR0 = 0x0000;      

// required delay
R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);    

// D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
*DAC12_DAVREFCR = 0x01; 

// D/A Control Register
*DAC12_DACR = 0x5F; 

// required delay
R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS); 

// D/A Data Register 0 
*DAC12_DADR0 = 0x0800; 

// clear all bits for Port 0
*PFS_P014PFS = 0x00000000; 

// Port Mode Control - Used as an analog pin
*PFS_P014PFS |= (0x1 << 15); 
}


// DO NOT TOUCH THIS 
void DoublePrecisionToQ14(const double section[], int16_t out[]) {
for(int j = 0; j < 6; ++j) {
out[j] = (int16_t) llround(section[j] * Q14_SCALE);
}            
}         
void DoublePrecisionToQ14_Scales(const double index[], int16_t out[]) {
for(int j = 0; j < N_scales; ++j) {
out[j] = (int16_t) llround(index[j] * Q14_SCALE);
}            
}     

static inline int32_t PackQ14(int16_t op1, int16_t op2) {
int32_t packed = (int32_t)__PKHBT(op1,op2,16);
return packed;
}




static int16_t w[2][Ns] = {0}; // changed from w

int16_t BiquadSectionFilter(int32_t (*coeffs[]),const int16_t out_scales[],int16_t input, int16_t w_n[2][Ns]) {
int16_t temp16; 
//int32_t w_0 = ((int32_t)input) << 14;  // promote x[n] from Q14 to Q28
int32_t w_0 = (int32_t) input;  // promote x[n] from Q14 to Q28
w_0 <<= 14; // promote to Q28
int32_t temp32; // for operations requiring 32 bits 
//uint8_t k;
// k = (Ns*2); // (Ns*2)-1 when Ns is a power of 2

for(int i = 0; i < Ns; ++i) {
// [w_1(n-1) w_2(n-1)... w_k(n-1) w_1(n-2) w_2(n-2)... w_k(n-2)]




// (l+Ns) only works with powers of 2

// w_n[(l+Ns)%M] | w_n[l] = w(n-2) | w(n-1)
temp32 = PackQ14(-w_n[0][i], w_n[1][i]);

// 
// p2 = a2 * w_n[(l+Ns)%M], p1 = a1 * -w_n[l]  -> w_0 = w_0 - p2 - p1 
w_0 = (int32_t)__SMLSD(coeffs[i][2], temp32, w_0);  // w(n) = x[n] - a1*w(n-1) - a2*w(n-2)


temp16 = w_n[1][i]; // temporarily save w(n-2) for further operations

//w_n[l] = w_n[save_index]; // w(n-2) = w(n-1)

int16_t old = (int16_t) (Q14_SHIFT(w_0)); // save w(n) so that I can do w(n-1) = w(n) later
//old = (int16_t)__SSAT(old,15); //saturate just in case

// y = b0*w + b1*d1 + b2*d2


temp32 = PackQ14(0,temp16); // temp16 | 0 -> int32_t (temp16 << 16) = w(n-2) as 32 bit


int32_t y = (int32_t)__SMUAD(coeffs[i][1],temp32); // b2 * old w(n-2) -> y(n) = b2*w(n-2)


temp32 = PackQ14(w_n[0][i],old); // w(n) | w(n-1)  <-- I was using w_n[save_index] which is NOT w(n)


y = (int32_t)__SMLAD(coeffs[i][0],temp32,y); //y(n) = b0*w(n) + b2*w(n-2) + b1*w(n-1)


w_n[1][i] = w_n[0][i];//w(n-2) = w(n-1)
w_n[0][i] = old;// w(n-1) = w(n)

temp32 = PackQ14(0,__SSAT(Q14_SHIFT(y),15)); // convert to Q14 pack y, -> y | 0
//p1 = w_0 * scales[i] -> w_0 = p1
w_0 = (int32_t)__SMUAD(temp32,coeffs[i][3]);// scale final y(n)*scale

}


  temp32 = PackQ14(0,out_scales[Ns]); 
  w_0 = PackQ14(0,__SSAT(Q14_SHIFT(w_0),15));
  //temp32 = PackQ14(0,out_scales[N_scales - 1]); 
  temp32 = PackQ14(0,out_scales[N_scales - 1]); 
  w_0 = PackQ14(0,Q14_SHIFT(w_0));


int32_t y_out = (int32_t)__SMUAD(w_0,temp32); // scales[6] * w_0 -> y(n) = system gain * y_k(n)

int16_t y_out2 = (int16_t) (Q14_SHIFT(y_out)); // convert to Q14

// use SSAT instead
  y_out2 = (int16_t)__SSAT(y_out2, 15);   // saturate to ±(2^14 - 1)
 // y_out2 = (int16_t)__SSAT(y_out2, 15);   // saturate to ±(2^14 - 1)

return y_out2;
}

static double h_section1 [6] = 
{1.00000000000000, 0.00000000000000, -1.00000000000000, 1.00000000000000, -1.46978759765625, 0.97467041015625};
static double h_section2 [6] = 
{1.00000000000000, 0.00000000000000, -1.00000000000000, 1.00000000000000, -1.84881591796875, 0.98602294921875};
static double h_section3 [6] = 
{1.00000000000000, 0.00000000000000, -1.00000000000000, 1.00000000000000, -1.54632568359375, 0.93890380859375};
static double h_section4 [6] = 
{1.00000000000000, 0.00000000000000, -1.00000000000000, 1.00000000000000, -1.78363037109375, 0.95782470703125};
static double h_section5 [6] = 
{1.00000000000000, 0.00000000000000, -1.00000000000000, 1.00000000000000, -1.67431640625000, 0.93572998046875};



static double scales [N_scales] = {
0.17620849609375, 0.17620849609375, 0.11114501953125, 0.11114501953125, 0.03210449218750, 1.00000000000000
};


void pack_coefficients(int16_t coeff_array[],int16_t scale,int32_t out[]) {
// tf2sos = b0, b1, b2, 1, a1, a2

//b0b1
out[0] = PackQ14(coeff_array[1],coeff_array[0]);
//b2 | 0
out[1] = PackQ14(0,coeff_array[2]);
//a2a1 
out[2] = PackQ14(coeff_array[4],coeff_array[5]);

// scales | 0
out[3] = PackQ14(0,scale);
}




void hal_entry(void) {
setup_dac();
setup_adc();

*DAC12_DADR0 = 0; // reset the DAC
*ADC140_ADCSR |= ADST; // start ADC conversions

R_BSP_PinAccessEnable();
//R_IOPORT_PinCfg(&g_ioport_ctrl, DAC_OUT, IOPORT_CFG_ANALOG_ENABLE);
R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_06, IOPORT_CFG_PORT_DIRECTION_OUTPUT);

int16_t Q14_section1[6]; int16_t Q14_section2[6];int16_t Q14_section3[6];int16_t Q14_section4[6];
int16_t Q14_section5[6];
DoublePrecisionToQ14(h_section1,Q14_section1);
DoublePrecisionToQ14(h_section2,Q14_section2);
DoublePrecisionToQ14(h_section3,Q14_section3);
DoublePrecisionToQ14(h_section4,Q14_section4);
DoublePrecisionToQ14(h_section5,Q14_section5);


int16_t Q14_scales[N_scales];
DoublePrecisionToQ14_Scales(scales,Q14_scales);


int32_t packed_coeffs1[4];int32_t packed_coeffs2[4];int32_t packed_coeffs3[4];int32_t packed_coeffs4[4];
int32_t packed_coeffs5[4];

// pack coefficients up
pack_coefficients(Q14_section1,Q14_scales[0],packed_coeffs1);
pack_coefficients(Q14_section2,Q14_scales[1],packed_coeffs2);
pack_coefficients(Q14_section3,Q14_scales[2],packed_coeffs3);
pack_coefficients(Q14_section4,Q14_scales[3],packed_coeffs4);
pack_coefficients(Q14_section5,Q14_scales[4],packed_coeffs5);


int32_t (*SOS[Ns]) = {
packed_coeffs1, packed_coeffs2, packed_coeffs3, packed_coeffs4,packed_coeffs5
};

int16_t y = 0;
while(true){
while(*ADC140_ADCSR & ADST); // bit clears when conversion has ended

current_input = *ADC140_ADDR00; // read from ADC result register

// R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_06, BSP_IO_LEVEL_HIGH);
//PORT1_PODR |=  (1 << 6);   // HIGH
*ADC140_ADCSR |= ADST; // restart conversion
//PORT1_PODR &= ~(1 << 6);   // LOW
int16_t x_q14 = (int16_t)(current_input);

// obviously DAC cant output negative
y = BiquadSectionFilter(SOS,Q14_scales,x_q14,w); 

int32_t dac14 = y;  // optional attenuation

    
if (dac14 < 0) dac14 = 0;
if (dac14 > 16383) dac14 = 16383;
uint16_t dac12 = (uint16_t)(dac14 >> 2);

current_output = dac12;

//R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_01_PIN_06, BSP_IO_LEVEL_LOW);
*DAC12_DADR0 = current_output; // write to DAC register


}

}
