

/* 
 * File:   Motor.h
 * Author: Josh Underwood   
 * Comments: For controlling a stepper motor
 * Revision history: Mar 25 2017
 */

/*
 * NOTE - THIS CODE ONLY ALLOWS RB PINS TO BE USED AS THE OUTPUTS
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>


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

//Stepper Motor Class
//Stepper Motor Class
class Stepper_motor {
  private:
    int steps_per_rev;
    int actual_steps_per_rev;
    unsigned dirPin;
    unsigned stepPin;
    double maxSpeed; //RPS
    bool dir; // 0 - CW, 1 - CCW
    controls controlType; //0 - position, 1 - velocity
    double deltaT;
    double goPosition;
    double velocity; //RPS
    unsigned long lastStep;
    const int pulseLength = 1; //microseconds
    microStepping curStepping;
    unsigned MS0, MS1, MS2;
    double curPosition; //ticks
    int multiplier;
    bool HIGH;
    bool LOW;
    bool INPUT;
    bool OUTPUT;
  public:

    Stepper_motor(int step_to_rev, unsigned dPin, unsigned sPin, int mSpeed, controls con, microStepping stepChoice, unsigned ePin, unsigned inMS0, unsigned inMS1, unsigned inMS2) {
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
      LOW=0;
      HIGH=1;
      OUTPUT = 0;
      INPUT=1;
      
      pinMode(stepPin, OUTPUT);      
      pinMode(ePin, OUTPUT);
      
      digitalWrite(ePin, LOW);

      pinMode(MS0, OUTPUT);
      pinMode(MS1, OUTPUT);
      pinMode(MS2, OUTPUT);
    }

    void setStepping(microStepping stepChoice) {
      bool MSBPin = false;
      bool midSBPin = false;
      bool LSBPin = false;
      switch (stepChoice) {
        case FULL:
          multiplier = 1;
          break;
        case HALF:
          multiplier = 2;
          LSBPin = true;
          break;
        case QUARTER:
          multiplier = 4; //**************************needs implementation
          break;
        case EIGHTH:
          multiplier = 8; //**************************needs implementation
          break;
        case SIXTEENTH:
          multiplier = 16;
          MSBPin = true;
          midSBPin = true;
          LSBPin = true;
          break;
          digitalWrite(MS0, MSBPin);
          digitalWrite(MS1, midSBPin);
          digitalWrite(MS2, LSBPin);
      }

    }

    void setPosition(long pos) {
      goPosition = pos;
    }

    void setVelocity(int vel) {
      velocity = min(abs(vel), 30000);
      deltaT = 1000000.0 / (steps_per_rev * multiplier * velocity);
      if (vel < 0)
        dir = 0;
      else
        dir = 1;
    }
    int min (int i1, int i2)
    {
        if (i1<i2) return i1;
        else return i2;
    }
    int abs (int val)
    {
        if (val <0) return (val*-1);
        else return val;
    }
    void setControl(controls input) {
      controlType = input;
    }

    void sendStep() { // needs converted to interrupts
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(stepPin, LOW);
    }

    void sendDir(bool setD) { //
      digitalWrite(dirPin, setD);
    }

    void digitalWrite(unsigned pin, bool highLow)
    {
        switch(pin)
        {  
            case 0:
            { 
                LATBbits.LATB0=highLow ;
                break;
            }
            case 1:
            { 
                LATBbits.LATB1=highLow ;
                break;
            }
            case 2:
            { 
                LATBbits.LATB2=highLow ;
                break;
            }
            case 3:
            { 
                LATBbits.LATB3=highLow ;
                break;
            }
            case 4:
            { 
                LATBbits.LATB4=highLow ;
                break;
            }
            case 5:
            { 
                LATBbits.LATB5=highLow ;
                break;
            }
            case 6:
            { 
                LATBbits.LATB6=highLow ;
                break;
            }
            case 7:
            { 
                LATBbits.LATB7=highLow ;
                break;
            }
            case 8:
            { 
                LATBbits.LATB8=highLow ;
                break;
            }
            case 9:
            { 
                LATBbits.LATB9=highLow ;
                break;
            }
            case 10:
            { 
                LATBbits.LATB10=highLow ;
                break;
            }
            case 11:
            { 
                LATBbits.LATB11=highLow ;
                break;
            }
            case 12:
            { 
                LATBbits.LATB12=highLow ;
                break;
            }
            case 13:
            { 
                LATBbits.LATB13=highLow ;
                break;
            }
            case 14:
            { 
               LATBbits.LATB14=highLow ;
                break;
            }
            case 15:
            { 
                LATBbits.LATB15=highLow ;
                break;
            }
            
        }
    }
    void pinMode(unsigned pin, bool inOut)
    {
        switch(pin)
        {  
            case 0:
            { 
                TRISBbits.TRISB0=inOut ;
                break;
            }
            case 1:
            { 
                TRISBbits.TRISB1=inOut ;
                break;
            }
            case 2:
            { 
                TRISBbits.TRISB2=inOut ;
                break;
            }
            case 3:
            { 
                TRISBbits.TRISB3=inOut ;
                break;
            }
            case 4:
            { 
                TRISBbits.TRISB4=inOut ;
                break;
            }
            case 5:
            { 
                TRISBbits.TRISB5=inOut ;
                break;
            }
            case 6:
            { 
                TRISBbits.TRISB6=inOut ;
                break;
            }
            case 7:
            { 
                TRISBbits.TRISB7=inOut ;
                break;
            }
            case 8:
            { 
                TRISBbits.TRISB8=inOut ;
                break;
            }
            case 9:
            { 
                TRISBbits.TRISB9=inOut ;
                break;
            }
            case 10:
            { 
                TRISBbits.TRISB10=inOut ;
                break;
            }
            case 11:
            { 
                TRISBbits.TRISB11=inOut ;
                break;
            }
            case 12:
            { 
                TRISBbits.TRISB12=inOut ;
                break;
            }
            case 13:
            { 
                TRISBbits.TRISB13=inOut ;
                break;
            }
            case 14:
            { 
                TRISBbits.TRISB14=inOut ;
                break;
            }
            case 15:
            { 
                TRISBbits.TRISB15=inOut ;
                break;
            }
            
        }
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
      if (deltaT < (micros() - lastStep) && (goPosition - curPosition != 0)) { //limit velocity

        //set step direction
        if (goPosition - curPosition > 0) {
          sendDir(0);
          curPosition+= 1/multiplier;
        } else {
          curPosition-=1/multiplier;
          sendDir(1);
        }

        sendStep();

        lastStep = micros();
      }
    }

};

      
// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

