/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Fraser Forbes.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include "bno055.h"
#inlcude "FrasersMagic.h"


typedef int bool;
#define TRUE 0x01
#define FALSE 0x00
 
int main()
{
    CyGlobalIntEnable; /* Enable global interrupts. */
	//LCD being
    LCD_Start();
    LCD_Position(0,0);
	CyDelay(1000);
	
	//i2c begin
    I2C_Start();
	I2C_DisableInt();
    LCD_PrintString("12cStart Done");
    LCD_Position(0,0);
	CyDelay(500);
	
	//Tests to see if chip is powered and communicating
	uint8 test = read8(BNO055_CHIP_ID_ADDR);
    LCD_PrintNumber(test);

    if (test != 0xA0)
    {
        LCD_PrintString("Could not read chip id");
    }
	power_set_mode();
	units_set_mode();
	sensor_set_mode();
	
	
    for(;;)
    {
       //quaternion data?
        LCD_PrintString("quaternion data?");
        LCD_Position(1,0);
        LCD_PrintString("w");
        LCD_PrintNumber(read_qw);
        LCD_PrintString("x");
        LCD_PrintNumber(read_qx);
        LCD_PrintString("y");
        LCD_PrintNumber(read_qy);
        LCD_PrintString("z");
        LCD_PrintNumber(read_qz);
        CyDelay(100);
    }
}

