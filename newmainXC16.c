/*
 * File:   newmainXC16.c
 * Author: Josh
 *
 * Created on March 25, 2017, 6:19 PM
 */

#include <xc.h>
#include <p33FJ128MC802.h>
#include "timer.h"
#include "adc.h"
#include <libpic30.h>
//#include "config.h"

//configuration bits
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC	(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
_FWDT	(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS1);
_FPOR	(FPWRT_PWR1 & ALTI2C_ON);
_FICD	(ICS_PGD1 & JTAGEN_OFF);

//Global variables
double degrees=0;
unsigned int ONTime = 0;		// Variable used to control the RC Servo motor's Position			
unsigned int OFFTime = 0;		// Main variable used to control the period of the square wave	
unsigned int TmrState = 0;		// Variable used to store whether square wave is ON or OFF 		
unsigned int TmrVal = 0;		// Variable to store the value used to setup Timer1 which controls
unsigned int period =4096;
unsigned int temp=0;

void InitIO(void);
void ADC(void);
void InitTimer(void);

int main(void) {
    InitIO(); // Call InitIO which configures the input and output pins
    InitTimer(); // Call InitTimer which configures the timer and its interrupt

    while (1) // Infinite loop
    {
        
        //   LATBbits.LATB13 = 1; // Turn ON Output to set high signal for RB13
     //   Delay_1S_Cnt();
      //     LATBbits.LATB13 =0;
        //   delay(1000);
        
        ADC(); // Call ADC which configures and reads analog inputs 0 and 1 (AN0 and AN1)
    }
    return 0;
}


/*********************************************************************************************************/
void InitIO(void) {
    TRISAbits.TRISA0 = 1; // Set RA0 (AN0) as input
    TRISAbits.TRISA1 = 1; // Set RA1 (AN1) as input
    TRISBbits.TRISB13 = 0; // Set RB13 as output which is used to generate the square wave signal				
}

/*********************************************************************************************************/
// For more information on PIC24H Timers Peripheral Module Library refer to link below:
// file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.25/docs/periph_libs/dsPIC30F_dsPIC33F_PIC24H_dsPIC33E_PIC24E_Timers_Help.htm
//********************************************************************************************************/

void InitTimer(void) { // Prescaler = 1:1
    // Period = 0x0FFF
    OpenTimer1(T1_ON & T1_PS_1_1 & T1_SYNC_EXT_OFF & T1_SOURCE_INT & T1_GATE_OFF & T1_IDLE_STOP, 0x0FFF);
    // Turn Timer1 interrupt ON
    ConfigIntTimer1(T1_INT_PRIOR_7 & T1_INT_ON);
}

/*********************************************************************************************************/
// For more information on PIC24H ADC Peripheral Module Library refer to link below:
// file:///C:/Program%20Files%20%28x86%29/Microchip/xc16/v1.25/docs/periph_libs/dsPIC33F_PIC24H_dsPIC33E_PIC24E_ADC_Library_Help.htm
//********************************************************************************************************/

void ADC(void) { // 12-bit sampling
    // Use dedicated ADC RC oscillator
    // Automatically start new conversion after previous
    // Use AVdd and AVss as reference levels
    OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
            ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN4_ANA, //changed to an4 from an0, now uses photoresistor
            0,
            0,
            0);
    
    SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN4); //also changed to an4 from an0
    AD1CON1bits.ADON = 1; // Turn on ADC hardware module
    while (AD1CON1bits.DONE == 0); // Wait until conversion is done
    ONTime = (ReadADC1(0)-720)*4096.0/3376.0;
    if (ONTime<0) ONTime=0;
  //  ONTime = 1000;
    //degrees = ONTime * 180.0 / 4095.0; // ONTime = converted results
    AD1CON1bits.ADON = 0; // Turn off ADC hardware module


    OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
            ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN1_ANA,
            0,
            0,
            0);
    // Select AN1
    SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN1);
    AD1CON1bits.ADON = 1; // Turn on ADC hardware module
    while (AD1CON1bits.DONE == 0); // Wait until conversion is done
    temp = ReadADC1(0); // OFFTime = converted results
    AD1CON1bits.ADON = 0; // Turn off ADC hardware module 
 
}

/*********************************************************************************************************/
// For more information on PIC24H Interrupts, see the MPLAB® XC16 C Compiler User's Guide - Chapter 14. Interrupts
//
// For more information on PIC24H Timers Peripheral Module Library refer to link below:
// file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.25/docs/periph_libs/dsPIC30F_dsPIC33F_PIC24H_dsPIC33E_PIC24E_Timers_Help.htm

/*********************************************************************************************************/
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    DisableIntT1; // Disable Timer1 interrupt 

    // This IF statement will constantly switch in order to generate the square wave signal (ONTime and OFFTime)
    if (TmrState == 0) // If signal is low (OFF)
    {
        LATBbits.LATB13 = 1; // Turn ON Output to set high signal for RB13
        T1CONbits.TCKPS = 1; // Change prescaler to 1:8
      //  TmrVal = degrees * 858.0 / 180.0 + 3015; // Set TmrVal = ONTime
        TmrVal=ONTime;
        TmrState = 1; // Set signal state to be ON for next interrupt
    } else if (TmrState == 1) // If signal is HIGH (ON)
    {
        LATBbits.LATB13 = 0; // Turn OFF Output to set LOW signal for RB13
     //   TmrVal = OFFTime; // Set TmrVal = OFFTime
        OFFTime=period-ONTime;
        TmrVal=OFFTime;
        T1CONbits.TCKPS = 1; // Change prescaler to 1:8
        TmrState = 0; // Set Timer state to be OFF for next interrupt in order to repeat again
    }
    WriteTimer1(TmrVal); // Setup Timer1 with the appropriate value to set the interrupt time
    IFS0bits.T1IF = 0; // Reset Timer1 interrupt flag
    EnableIntT1; // Enable Timer1 interrupt
}