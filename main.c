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

typedef int bool;
#define TRUE 0x01
#define FALSE 0x00
 
//write function
void write8(uint8 reg_addr, uint8 data);

//read8 function
uint8 read8(uint8 reg_addr);

//function callling. All these and the functions should be moved to a header/seperate fle
s8 read_qw();
s8 read_qx();
s8 read_qy();
s8 read_qz();
u8 read_quat_data();
s8 sensor_set_mode();
s8 set_data_type();
s8 power_set_mode();
s8 set_page_zero();
s8 units_set_mode();

//required address
u8 dev_addr = BNO055_I2C_ADDR1;

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

//write function
void write8(uint8 reg_addr, uint8 data)
{
    volatile uint8 status = 0;
    status = I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_WRITE_XFER_MODE);
    status = I2C_MasterWriteByte(reg_addr);
    status = I2C_MasterWriteByte(data);
    I2C_MasterSendStop();
}

//read function
uint8 read8(uint8 subAddr)
{
    volatile uint8 status = 0;
    volatile uint8 dataStored= 0;
    status = I2C_MasterSendStart(BNO055_I2C_ADDR1,I2C_WRITE_XFER_MODE);
    status = I2C_MasterWriteByte(reg_addr);
    status = I2C_MasterSendStop();
    status = I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_READ_XFER_MODE);
    dataStored = I2C_MasterReadByte(I2C_NAK_DATA);
    I2C_MasterSendStop();
    status = I2C_MasterStatus();
    if(status !=0)
    {
        LCD_PrintString("Read Error");
    }
    return dataStored;
}

//set page back to 0
s8 set_page_zero()
{
    write8(BNO055_PAGE_ID_REG,0);
}

//set the power mode
s8 power_set_mode()
{
	set_page_zero();
    write8(BNO055_PWR_MODE_ADDR,BNO055_POWER_MODE_NORMAL);
    CyDelay(10);
}    
    
// set the units
s8 units_set_mode()
{
    	    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
    */
    write8(BNO055_PAGE_ID_ADDR,0);
    write8(BNO055_UNIT_SEL_ADDR, 0<<7);
    write8(BNO055_SYS_TRIGGER_ADDR,0x00);
    CyDelay(10);
}

s8 sensor_set_mode()
{
	set_page_zero();
    write8(BNO055_OPERATION_MODE_REG, BNO055_OPERATION_MODE_NDOF);
}

u8 read_quat_data()
{
	set_page_zero();
	u8 data_u8[4] = {0, 0, 0, 0};
			/* Read the eight byte value
			of quaternion wxyz data*/
			/* Data W*/
            data_u8[0] = read_qw();
			/* Data X*/
            data_u8[1] =read_qx();
            /* Data Y*/
            data_u8[2] = read_qy();
			/* Data Z*/
            data_u8[3] = read_qz();
    return data_u8[0];
    return data_u8[1];
    return data_u8[2];
    return data_u8[3];    
}

//read from the quaternion registers:

//Reading quaternion W
s8 read_qw()
{
	set_page_zero();
    s8 qw1 = read8(BNO055_QUATERNION_DATA_W_LSB_ADDR);
    s8 qw2 = read8(BNO055_QUATERNION_DATA_W_MSB_ADDR);
    s16 qw = (s16)qw1;
    qw = qw<<8;
    qw |= qw2;
    return qw;
}
//Reading quaternion X
s8 read_qx()
{
	set_page_zero();
    s8 qx1 = read8(BNO055_QUATERNION_DATA_W_LSB_ADDR);
    s8 qx2 = read8(BNO055_QUATERNION_DATA_W_MSB_ADDR);
    s16 qx = (s16)qx1;
    qx = qx<<8;
    qx |= qx2;
    return qx;
}
//Reading quaternion Y
s8 read_qy()
{
	set_page_zero();
    s8 qy1 = read8(BNO055_QUATERNION_DATA_W_LSB_ADDR);
    s8 qy2 = read8(BNO055_QUATERNION_DATA_W_MSB_ADDR);
    s16 qy = (s16)qy1;
    qy = qy<<8;
    qy |= qy2;
    return qy;
}//Reading quaternion Z
s8 read_qz()
{
	set_page_zero();
    s8 qz1 = read8(BNO055_QUATERNION_DATA_W_LSB_ADDR);
    s8 qz2 = read8(BNO055_QUATERNION_DATA_W_MSB_ADDR);
    s16 qz = (s16)qz1;
    qz = qz<<8;
    qz |= qz2;
    return qz;
}
