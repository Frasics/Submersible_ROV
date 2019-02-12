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

/* this file should take the inputs from the controller, and the data from the imu and compare the two. 
If the imu is saying there is any change in z while moving, the planes should correct this.
If rising, the planes should turn up to aid in rising
if descending, the planes should turn down  to aid in descending
if w (roll) happens then we are in a pickle.
While moving straight forwards, if there is any change in y then the thrusters should correct properly.
*/
#include "project.h"
#include "combinator.h"

// below are place holders for the actual quaternion data
//uint8 temp_x_quat;
//uint8 temp_y_quat;
//uint8 temp_z_quat;



char balance_x(uint32 ADC_Y)
{
    /*idk how to do this yet
    this will have to take the input of the sideways pot of
    the joystick and compare that to the rms value of the temp_x_quat 
    and temp_y_quat. gotta co
    */
    return ADC_Y;
}

char balance_z(uint8 ballast, uint16 plane_angle)
{
    if (ballast == 0)
    {
        if (temp_z_quat>0)
        {
            //ewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin;
            plane_angle = temp_z_quat*90;
        }
        else if (temp_z_quat<0)
        {
            plane_angle = temp_z_quat*(-90);
        }
    }
    return plane_angle;
}
        
/* [] END OF FILE */
