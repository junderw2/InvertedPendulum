/*
 * File:   newmainXC16.c
 * Author: Josh
 *
 * Created on March 25, 2017, 6:19 PM
 */

#include <xc.h>
#include "build/../p33FJ128MC802.h"
#include "timer.h"
#include "adc.h"

_FGS(GWRP_OFF & GCP_OFF);
_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);
//Global variables
enum microStepping {
    FULL,
    HALF,
    QUARTER,
    EIGHTH,
    SIXTEENTH
};
enum controls {
    POSITION,
    VELOCITY
};
double degrees = 0;
unsigned int ONTime = 0; // Variable used to control the RC Servo motor's Position			
unsigned int OFFTime = 0; // Main variable used to control the period of the square wave	
unsigned int TmrState = 0; // Variable used to store whether square wave is ON or OFF 		
unsigned int TmrVal = 0; // Variable to store the value used to setup Timer1 which controls
unsigned int period = 4096;
unsigned int temp = 0;

int steps_per_rev;
int actual_steps_per_rev;
unsigned dirPin;
unsigned stepPin;
double maxSpeed; //RPS
unsigned dir; // 0 - CW, 1 - CCW
enum controls controlType; //0 - position, 1 - velocity
double deltaT;
double goPosition;
double velocity; //RPS
unsigned long lastStep;
const int pulseLength = 1; //microseconds
enum microStepping curStepping;
unsigned MS0, MS1, MS2;
double curPosition; //ticks
int multiplier;
unsigned HIGH = 1;
unsigned LOW =0;
unsigned INPUT=1;
unsigned OUTPUT=0;
unsigned long count1us = 0;
unsigned long micros = 0;

int potPosition = 0;
int potPosition1 = 0;
/*These functions are standalone and are defined in the supporting files*/
double map(double value, float x_min, float x_max, float y_min, float y_max);
int min(int, int);
int abs(int);
void InitIO(void);
/************************************************************************/
/*These functions are defined below main*/
void ADC(void);
void InitTimer(void);
void InitMotorTimer();
void Stepper_motor(int, unsigned, unsigned, int, enum controls, enum microStepping, unsigned, unsigned, unsigned, unsigned);
void setStepping(enum microStepping);
void setPosition(long);
void setVelocity(int);
void setControl(enum controls);
void sendStep();
void sendDir(unsigned);
void digitalWrite(unsigned, unsigned);
void pinMode(unsigned, unsigned);
void runOverhead();
/*****************************************/

int main(void) {
    /* Configure Oscillator to operate the device at 40MHz.
     * Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
     * Fosc= 7.37M*40/(2*2)=80Mhz for 7.37M input clock */

    PLLFBD = 40; // M = 40    
    CLKDIVbits.PLLPOST = 0; // N2 = 2    
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    while (OSCCONbits.LOCK != 1) {
    };

    InitIO(); // Call InitIO which configures the input and output pins
    InitTimer(); // Call InitTimer which configures the timer and its interrupt
    Stepper_motor(200, 10, 11, 2, VELOCITY, FULL, 12, 13, 14, 15); //function sets up motor with specified pins
    while (1) // Infinite loop
    {
        ADC(); // Call ADC which configures and reads analog inputs 0 and 1 (AN0 and AN1)
        potPosition = (map(potPosition1, 2417, 4086, 0, 360 * 1.8) + potPosition * 9)/10;
      //  setPosition(potPosition);
        setVelocity(5);
        runOverhead();
    }
    return 0;
}

void ADC(void) { // 12-bit sampling
    // Use dedicated ADC RC oscillator
    // Automatically start new conversion after previous
    // Use AVdd and AVss as reference levels
    OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
            ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
            ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
            ADC_DMA_BUF_LOC_1,
            ENABLE_AN0_ANA, //changed to an4 from an0, now uses photoresistor
            0, 0, 0);

    SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN0); //also changed to an4 from an0
    AD1CON1bits.ADON = 1; // Turn on ADC hardware module
    while (AD1CON1bits.DONE == 0); // Wait until conversion is done
    potPosition1 = ReadADC1(0); // Read pot position
    AD1CON1bits.ADON = 0; // Turn off ADC hardware module 
}
void runOverhead() {

    if (controlType) {
        //velocity control
        curPosition = 0;
        switch (dir) {
            case 0:
                goPosition = -1;
                break;
            case 1:
                goPosition = 1;
                break;
        }
    }

    //runStepControl
    if (deltaT < (count1us - lastStep) && (goPosition - curPosition != 0)) { //limit velocity

        //set step direction
        if (goPosition - curPosition > 0) {
            sendDir(0);
            curPosition += 1 / multiplier;
        } else {
            curPosition -= 1 / multiplier;
            sendDir(1);
        }

        sendStep();

        lastStep = count1us;
    }
}
void InitTimer(void) { // Prescaler = 1:1
    // Period = 0x0FFF
    T1CON = 0x8000;
    IPC0 = IPC0 | 0x1000;
    PR1 = 38;

    OpenTimer1(T1_ON & T1_PS_1_1 & T1_SYNC_EXT_OFF & T1_SOURCE_INT & T1_GATE_OFF & T1_IDLE_STOP, PR1);
    // Turn Timer1 interrupt ON
    ConfigIntTimer1(T1_INT_PRIOR_7 & T1_INT_ON);
}
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    //DisableIntT1; // Disable Timer1 interrupt 
    //   TmrVal = OFFTime; // Set TmrVal = OFFTime
    // T1CONbits.TCKPS = 1; // Change prescaler to 1:8
    count1us = count1us + 1;
    //WriteTimer1(5); // Setup Timer1 with the appropriate value to set the interrupt time
    IFS0bits.T1IF = 0; // Reset Timer1 interrupt flag
    //EnableIntT1; // Enable Timer1 interrupt
}
void sendStep() { // needs converted to interrupts
    digitalWrite(stepPin, HIGH);
    unsigned long now = count1us;
    while (1) {
        if (count1us > (2 + now)) break;
    }// Still uses the DELAY
    digitalWrite(stepPin, LOW);
}
void sendDir(unsigned setD) { //
    digitalWrite(dirPin, setD);
}
void setControl(enum controls input) {
    controlType = input;
}
void setVelocity(int vel) {
    velocity = min(abs(vel), 30000);
    deltaT = 1000000.0 / (steps_per_rev * multiplier * velocity);
    if (vel < 0)
        dir = 0;
    else
        dir = 1;
}
void setPosition(long pos) {
    goPosition = pos;
}
void setStepping(enum microStepping stepChoice) {
    unsigned MSBPin = 0;
    unsigned midSBPin = 0;
    unsigned LSBPin = 0;
    switch (stepChoice) {
        case FULL:
            multiplier = 1;
            break;
        case HALF:
            multiplier = 2;
            LSBPin = 1;
            break;
        case QUARTER:
            multiplier = 4; //**************************needs implementation Kaspar's note
            break;
        case EIGHTH:
            multiplier = 8; //**************************needs implementation Kaspar's note
            break;
        case SIXTEENTH:
            multiplier = 16;
            MSBPin = 1;
            midSBPin = 1;
            LSBPin = 1;
            break;
            digitalWrite(MS0, MSBPin);
            digitalWrite(MS1, midSBPin);
            digitalWrite(MS2, LSBPin);
    }

}
void Stepper_motor(int step_to_rev, unsigned dPin, unsigned sPin, int mSpeed, enum controls con, enum microStepping stepChoice, unsigned ePin, unsigned inMS0, unsigned inMS1, unsigned inMS2) {
    steps_per_rev = step_to_rev;
    dirPin = dPin;
    stepPin = sPin;
    maxSpeed = mSpeed;
    dir = 0;
    controlType = con;
    setStepping(stepChoice);
    goPosition = 0;
    curPosition = 0;
    velocity = 0;
    lastStep = 0;
    deltaT = 1000000.0 / (steps_per_rev * velocity);
    MS0 = inMS0;
    MS1 = inMS1;
    MS2 = inMS2;
    
    pinMode(stepPin, OUTPUT);
    pinMode(ePin, OUTPUT);

    digitalWrite(ePin, LOW);

    pinMode(MS0, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
}
void digitalWrite(unsigned pin, unsigned highLow) {
    switch (pin) {
        case 0:
        
            LATBbits.LATB0 = highLow;
            break;
        
        case 1:
        
            LATBbits.LATB1 = highLow;
            break;
        
        case 2:
        
            LATBbits.LATB2 = highLow;
            break;
        
        case 3:
        
            LATBbits.LATB3 = highLow;
            break;
        
        case 4:
        
            LATBbits.LATB4 = highLow;
            break;
        
        case 5:
        
            LATBbits.LATB5 = highLow;
            break;
        
        case 6:
        
            LATBbits.LATB6 = highLow;
            break;
        
        case 7:
        
            LATBbits.LATB7 = highLow;
           break;
        
        case 8:
        
            LATBbits.LATB8 = highLow;
            break;
        
        case 9:
        
            LATBbits.LATB9 = highLow;
            break;
        
        case 10:
        
            LATBbits.LATB10 = highLow;
            break;
        
        case 11:
        
            LATBbits.LATB11 = highLow;
            break;
        
        case 12:
        
            LATBbits.LATB12 = highLow;
            break;
        
        case 13:
        
            LATBbits.LATB13 = highLow;
            break;
        
        case 14:
        
            LATBbits.LATB14 = highLow;
            break;
        
        case 15:
        
            LATBbits.LATB15 = highLow;
            break;
        

    }
}
void pinMode(unsigned pin, unsigned inOut) {
    switch (pin) {
        case 0:
        
            TRISBbits.TRISB0 = inOut;
            break;
        
        case 1:
        
            TRISBbits.TRISB1 = inOut;
            break;
        
        case 2:
        
            TRISBbits.TRISB2 = inOut;
            break;
        
        case 3:
        
            TRISBbits.TRISB3 = inOut;
            break;
        
        case 4:
        
            TRISBbits.TRISB4 = inOut;
            break;
        
        case 5:
        
            TRISBbits.TRISB5 = inOut;
            break;
        
        case 6:
        
            TRISBbits.TRISB6 = inOut;
            break;
        
        case 7:
        
            TRISBbits.TRISB7 = inOut;
            break;
        
        case 8:
        
            TRISBbits.TRISB8 = inOut;
            break;
        
        case 9:
        
            TRISBbits.TRISB9 = inOut;
            break;
        
        case 10:
        
            TRISBbits.TRISB10 = inOut;
            break;
        
        case 11:
        
            TRISBbits.TRISB11 = inOut;
            break;
        
        case 12:
        
            TRISBbits.TRISB12 = inOut;
            break;
        
        case 13:
        
            TRISBbits.TRISB13 = inOut;
            break;
        
        case 14:
        
            TRISBbits.TRISB14 = inOut;
            break;
        
        case 15:
        
            TRISBbits.TRISB15 = inOut;
            break;
        

    }
}