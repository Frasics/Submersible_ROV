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
#include "bno055.c"
//#include "bno055_support.c"
//#define BNO055_API
#define I2C_BUFFER_LEN 8

struct bno055_t myBNO;

/*
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

uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  
    
    //initialization function
bool bno055_init1(adafruit_bno055_opmode_t mode)
{
    I2C_Start();
    
    uint8_t id = readByte(BNO055_CHIP_ID_ADDR); 
    while (id!= 0xA0) // checks if the chip is properly connected
    {
        LCD_PrintString("Not connected");
        return FALSE;
    }
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    
    //reset
    writeByte(BNO055_SYS_TRIGGER_ADDR,0x20);
    
    while (readByte(BNO055_CHIP_ID_ADDR)!= 0xA0) // check if id is still valid
    {
        LCD_Position(0,0);
        LCD_PrintString("                ");
        LCD_Position(0,0);
        LCD_PrintString("Disconnected");
        CyDelay(10);
    }
    CyDelay(50);
    
    writeByte(BNO055_PWR_MODE_ADDR,BNO055_POWER_MODE_NORMAL);
    CyDelay(10);
    writeByte(BNO055_PAGE_ID_ADDR,0);
    set the units
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
    
    writeByte(BNO055_UNIT_SEL_ADDR, 0<<7);
    writeByte(BNO055_SYS_TRIGGER_ADDR,0x00);
    CyDelay(10);
    bno055_set_operation_mode(mode);
    return TRUE;
}
void writeByte(uint8 subAddr, uint8 dataAdded)
{
    I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_WRITE_XFER_MODE);
    I2C_MasterWriteByte(subAddr);
    LCD_PrintString("Done");
    LCD_Position(0,0);
    I2C_MasterWriteByte(dataAdded);
    I2C_MasterSendStop();
}
uint8 readByte(uint8 subAddr)
{
    uint8 dataStored;
    I2C_MasterSendStart(BNO055_I2C_ADDR1,I2C_WRITE_XFER_MODE);
    LCD_PrintString("Done1");
    LCD_Position(0,0);
    LCD_PrintString("               ");
    LCD_Position(0,0);
    I2C_MasterWriteByte(subAddr);
    LCD_PrintString("Done2");
    LCD_Position(0,0);
    LCD_PrintString("               ");
    LCD_Position(0,0);
    I2C_MasterSendStop();
    I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_READ_XFER_MODE);
    dataStored = I2C_MasterReadByte(I2C_NAK_DATA);
    I2C_MasterSendStop();
    return dataStored;
}
s8 BNO055_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 dataStored;
    u8 stringpos = 0;
    u8 array[BNO055_INIT_VALUE];
    array[BNO055_INIT_VALUE] = reg_addr;
    u8 BNO055_BUS_ARRAY_INDEX = 0;
    for(stringpos = 0; stringpos<cnt; stringpos++)
    {
        array[stringpos+0]=*(reg_data +stringpos);  
    }
    I2C_MasterSendStart(dev_addr,I2C_READ_XFER_MODE);
    I2C_MasterWriteByte(reg_addr);
    I2C_MasterWriteByte(*reg_data);
    I2C_MasterSendStop();
    I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_READ_XFER_MODE);
    dataStored = I2C_MasterReadByte(I2C_NAK_DATA);
    I2C_MasterSendStop();
    return dataStored;
}
s8 BNO055_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    uint8 status = 0;
    cnt=1;
    status = I2C_MasterSendStart(dev_addr, I2C_WRITE_XFER_MODE);
    status = I2C_MasterWriteByte(reg_addr);
    status = I2C_MasterWriteByte(*reg_data);
    status = I2C_MasterSendStop();
    return status;
}

s8 bno055_read_cpp(u8 reg_addr)
{
    u8 val = 0;
    I2C_MasterSendStart(BNO055_I2C_ADDR1,I2C_WRITE_XFER_MODE);
    I2C_MasterWriteByte(reg_addr);
    I2C_MasterSendStop();
    I2C_MasterSendStart(BNO055_I2C_ADDR1,I2C_READ_XFER_MODE);
    val = I2C_MasterReadByte(reg_addr);
    I2C_MasterSendStop();
    return val;
}
s8 bno055_reads_cpp(u8 reg_addr, s8 *buffer, uint8 len)
{
    u8 dataStored = 0;
    I2C_MasterSendStart(BNO055_I2C_ADDR1,I2C_READ_XFER_MODE);
    I2C_MasterWriteByte(reg_addr);
    I2C_MasterWriteByte(*buffer);
    I2C_MasterSendStop();
    I2C_MasterSendStart(BNO055_I2C_ADDR1, I2C_READ_XFER_MODE);
    for (uint8 i = 0; i<len; i++)
    {
        buffer[i] = myBNO.bus_read;
    }
    dataStored = I2C_MasterReadByte(I2C_NAK_DATA);
    I2C_MasterSendStop();
}
*/

u8 dev_addr = BNO055_I2C_ADDR1;
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)
s8 bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


s8 bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    //u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
    //uint8 status = 0;
    cnt = 1;
    BNO055_iERROR = I2C_MasterSendStart(dev_addr,I2C_WRITE_XFER_MODE);
    BNO055_iERROR = I2C_MasterWriteByte(reg_addr);
    BNO055_iERROR = I2C_MasterWriteByte(*reg_data);
    I2C_MasterSendStop();
    //array[BNO055_INIT_VALUE] = reg_addr;
    //BNO055_iERROR = I2C_WRITE_STRING(dev_addr,array, cnt+1);
    return (s8)BNO055_iERROR;
    /*
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BNO055_INIT_VALUE;

	array[BNO055_INIT_VALUE] = reg_addr;
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++) 
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
	}
	
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	
	return (s8)BNO055_iERROR;
    */
}

s8 bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
   // volatile unsigned int i;
   // volatile unsigned char s_addr_internW , s_addr_internR;
   // s_addr_internW = (dev_addr<< 1) & 0xFE;     // shift der Adresse um 1 Bit nach links da für Adresse nur obersten 7 Bits verwendet werden + Write 0bit an stelle 0
   // s_addr_internR = (dev_addr << 1) | 0x01;     // shift der Adresse um 1 Bit nach links da für Adresse nur obersten 7 Bits verwendet werden + Read  1bit an stelle 0
    volatile s32 BNO055_iERROR = 0;
    //uint8 status = 0;
    BNO055_iERROR = I2C_MasterSendStart(dev_addr, I2C_WRITE_XFER_MODE);
    BNO055_iERROR = I2C_MasterWriteByte(reg_addr);
    BNO055_iERROR = I2C_MasterSendStop();
    CyDelay(100);
	//u8 array[I2C_BUFFER_LEN] = {0};
	volatile u8 stringpos = 0;
	//array[B0] = reg_addr;
    BNO055_iERROR = I2C_MasterSendStart(dev_addr, I2C_READ_XFER_MODE);
    BNO055_iERROR = I2C_MasterWriteByte(reg_addr);
    
	for (stringpos = 0; stringpos < cnt; stringpos++) 
    {	
        *(reg_data+stringpos) = I2C_MasterReadByte(I2C_ACK_DATA);
	}
    *reg_data = I2C_MasterReadByte(I2C_NAK_DATA);
    BNO055_iERROR = I2C_MasterSendStop();
	return (s8)BNO055_iERROR;
}
void BNO055_delay_msek(u32 msek);

void BNO055_delay_msek(u32 msek)
{
    u32 i = 0;
    for( i= 0; i<msek; msek++)
    {
        LCD_PrintString("Delaying          ");
        CyDelay(1);
    }
}

    
s8 I2C_routine(void){
    CyDelay(10);
    myBNO.bus_read = bus_read;
    myBNO.bus_write = bus_write;
    myBNO.delay_msec = BNO055_delay_msek;
    myBNO.dev_addr = BNO055_I2C_ADDR1;
    return BNO055_INIT_VALUE; 
}

int main()
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    LCD_Start();
    LCD_Position(0,0);
    I2C_Start();
    LCD_PrintString("12cStart Done");
    CyDelay(500);
    LCD_Position(0,0);
    I2C_routine();
    LCD_Position(0,0);
    LCD_PrintString("Routine'd                ");
    CyDelay(1000);
    LCD_Position(1,0);
    LCD_PrintInt8(bno055_init(&myBNO));
    CyDelay(1000);
    LCD_Position(0,0);
    LCD_PrintString("bno055 Init done");
    CyDelay(1000);
    LCD_Position(0,0);
    //bno055_get_selftest();
    LCD_Position(0,0);
    LCD_PrintString("                 ");
    LCD_Position(0,0);
    LCD_PrintInt8(bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF));
    CyDelay(1000);
    LCD_PrintString("bno055 starting...");
    CyDelay(1000);
    LCD_Position(0,0);
    LCD_PrintString("                ");
    LCD_Position(0,0);
    for(;;)
    {
        struct bno055_euler_float_t eulerData;
        s8 eul = bno055_convert_float_euler_hpr_deg(&eulerData);
        unsigned char accel_calib_status = 0;
        unsigned char gyro_calib_status = 0;
        unsigned char mag_calib_status = 0;
        unsigned char sys_calib_status = 0;
        LCD_Position(0,0);
        LCD_PrintString("                 ");
        LCD_Position(0,0);
        LCD_PrintInt8(bno055_get_accel_calib_stat(&accel_calib_status));
        CyDelay(1000);
        LCD_Position(0,0);
        LCD_PrintString("                 ");
        LCD_Position(0,0);
        LCD_PrintInt8(bno055_get_mag_calib_stat(&mag_calib_status));
        CyDelay(1000);
        LCD_Position(0,0);
        LCD_PrintString("                 ");
        LCD_Position(0,0);
        LCD_PrintInt8(bno055_get_gyro_calib_stat(&gyro_calib_status));
        CyDelay(1000);
        LCD_Position(0,0);
        LCD_PrintString("                 ");
        LCD_Position(0,0);
        LCD_PrintInt8(bno055_get_sys_calib_stat(&sys_calib_status));
        CyDelay(1000);
        LCD_Position(0,0);
        LCD_PrintString("                 ");
        LCD_Position(0,0);
        LCD_PrintInt8(eul);
        LCD_PrintString(" euler angles?");
        CyDelay(1000);
        LCD_Position(0,0);
        LCD_PrintString("                ");
        LCD_Position(0,0);
        CyDelay(100);
    }
}