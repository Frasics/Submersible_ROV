/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include "math.h"
#include "combinator.h"
//#include "motion.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    ADC_X_Start();
    ADC_Y_Start();
    ADC_X_StartConvert();
    ADC_Y_StartConvert();
    PWM_Start();
    uint32 freq = 100000;
    //uint32 halfFreq = freq/2;
    uint16 maxPot = 3299; // max potentiometer value
    uint16 Forwardthresh = 2390; // threshold where thrusters go forwards
    uint16 Backwardthresh = 2380; // backwards threshold
    //uint16 maxFor = 4000;// gussed maxforwards value
    //uint16 maxBack = 100;// guessed maxbackwards value
    uint16 Leftthresh = 2345; // joystick to the left 
    uint16 Rightthresh = 2362; // joystick to the right

    volatile uint32 ADC_X = ADC_X_CountsTo_mVolts(ADC_X_GetResult16()); // converts result to mV for X
    volatile uint32 ADC_Y = ADC_Y_CountsTo_mVolts(ADC_Y_GetResult16()); // onverts results to mV for Y
    
   // uint32 x = (((float)ADC_X/maxPot)*(float)freq); 
    uint32 y = (((float)ADC_Y/maxPot)*(float)freq); // Y threshold signal

    volatile uint8 speedX = (((float)ADC_X/3299)*(float)20); //Calculates the PWM for thrust magnitude
    volatile uint8 turn = (((float)ADC_Y/3299)*(float)20); //calculates the turning PWM
    
    char turnRatio = turn/2353.5; // so when not left or right this should be == 1
    
    // The two calculations below are for when you want to turn and move forwards atst
    
    double more = (double)(speedX^2) + (double)(turnRatio^2);
    double less = (double)(speedX^2) - (double)(turnRatio^2);
    double speedMag1 = sqrt(more); 
    double speedMag2 = sqrt(less);
    
    for(;;)
    //Maybe this should all be in a Case statement
    
    if (ADC_X > Forwardthresh) // when moving forwards but not max
    {
        if (y > Leftthresh) // if you want to turn left while moving forwards
        {
            PWM_WriteCompare1(speedMag2*y); //makes the left thruster slower
            PWM_WriteCompare2(speedMag1*y); // makes the right thruster faster
        }
        else if (y < Rightthresh) // if you want to go right while moving forwards
        {
            PWM_WriteCompare1(speedMag1*y); // makes the left thruster faster
            PWM_WriteCompare2(speedMag2*y); // makes the right thruster slower
        }
        else //neither left or right
        {
             PWM_WriteCompare1(speedMag1); // same speeds
             PWM_WriteCompare2(speedMag1); // same speeds
        }
    }
    
    else if (ADC_X <Backwardthresh) //for when you are moving backwards
    {
        if (y > Leftthresh) // if you want to turn left while moving backwards
        {
            PWM_WriteCompare1(speedMag2*y); //makes the left thruster slower
            PWM_WriteCompare2(speedMag1*y); // makes the right thruster faster
        }
        else if (y < Rightthresh) // if you want to go right while moving backwards
        {
            PWM_WriteCompare1(speedMag1*y); // makes the left thruster faster
            PWM_WriteCompare2(speedMag2*y); // makes the right thruster slower
        }
         else //neither left or right
        {
             PWM_WriteCompare1(speedMag1); // same speeds
             PWM_WriteCompare2(speedMag1); // same speeds
        }
    }
    
}

/* [] END OF FILE */
