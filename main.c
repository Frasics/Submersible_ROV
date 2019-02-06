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
typedef enum
    {
      
      OPERATION_MODE_CONFIG= 0X00,
      OPERATION_MODE_ACCONLY= 0X01,
      OPERATION_MODE_MAGONLY= 0X02,
      OPERATION_MODE_GYRONLY= 0X03,
      OPERATION_MODE_ACCMAG= 0X04,
      OPERATION_MODE_ACCGYRO= 0X05,
      OPERATION_MODE_MAGGYRO = 0X06,
      OPERATION_MODE_AMG     = 0X07,
      OPERATION_MODE_IMUPLUS = 0X08,
      OPERATION_MODE_COMPASS = 0X09,
      OPERATION_MODE_M4G  = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
      OPERATION_MODE_NDOF  = 0X0C
    } adafruit_bno055_opmode_t;
  
    
    //initialization function
bool bno055_init1(adafruit_bno055_opmode_t mode)
{
    I2C_Start();
    
    uint8_t id = read8(BNO055_CHIP_ID_ADDR); 
    while (id!= 0xA0) // checks if the chip is properly connected, 0xA0 is the proper return address
    {
        LCD_PrintString("Not connected");
        return FALSE;
    }
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    
    //reset
    write8(BNO055_SYS_TRIGGER_ADDR,0x20);
    
    while (read8(BNO055_CHIP_ID_ADDR)!= 0xA0) // check if id is still valid
    {
        LCD_Position(0,0);
        LCD_PrintString("                ");
        LCD_Position(0,0);
        LCD_PrintString("Disconnected");
        CyDelay(10);
    }
    CyDelay(50);
    
    write8(BNO055_PWR_MODE_ADDR,BNO055_POWER_MODE_NORMAL);
    CyDelay(10);
    write8(BNO055_PAGE_ID_ADDR,0);
   // set the units
	    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
    */
    write8(BNO055_UNIT_SEL_ADDR, 0<<7);
    write8(BNO055_SYS_TRIGGER_ADDR,0x00);
    CyDelay(10);
    bno055_set_operation_mode(mode);
    return TRUE;
}

//write function
void write8(uint8 reg_addr, uint8 data);
void write8(uint8 reg_addr, uint8 data)
{
    volatile uint8 status = 0;
    status = I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_WRITE_XFER_MODE);
    status = I2C_MasterWriteByte(reg_addr);
    status = I2C_MasterWriteByte(data);
    I2C_MasterSendStop();
}

//read8 function
uint8 read8(uint8 reg_addr);
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
	
    for(;;)
    {
        //Function to read the quaternion data
	    
    }
}
