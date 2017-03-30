/*
 * File:   supportFiles.c
 * Author: Josh-Alien
 *
 * Created on March 29, 2017, 9:16 AM
 */
#if defined(__dsPIC33F__)
#include "p33fxxxx.h"
#elif defined(__PIC24H__)
#include "p24hxxxx.h"
#endif
double map(double value, float x_min, float x_max, float y_min, float y_max) {
    return (y_min + (((y_max - y_min) / (x_max - x_min)) * (value - x_min)));
}
int min(int i1, int i2) {
    if (i1 < i2) return i1;
    else return i2;
}
int abs(int val) {
    if (val < 0) return (val*-1);
    else return val;
}
void InitIO(void) {
    TRISAbits.TRISA0 = 1; // Set RA0 (AN0) as input for potentiometer
    //TRISAbits.TRISA1 = 1; // Set RA1 (AN1) as input
}



