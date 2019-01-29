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

//All data starting from 50 of https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
#define IMU_ADDR    (0X29)
#define IMU_COMMAND_BIT (0X80)
#define GYR_AM_SET (0X1F) //default value 0x0A
#define GYR_AM_THRES (0x1E) //0x04
#define GYR_DUR_Z (0x1D) //0x19HR_Z_Duration
#define GYR_HR_Z_SET (0x1C) //0x01
#define GYR_DUR_Y (0x1B) //0x19 HR_Y_Duration
#define GYR_HR_Y_SET (0x1A) //0x01
#define GYR_DUR_X (0x19) //0x19 HR_X_Duration
#define GYR_HR_X_SET (0x18) //0x01HR_X_THRES_HYST <1:0> HR_X_Threshold <4:0>
#define GYR_INT_SETING (0x17) //0x00 
#define ACC_NM_SET (0x16) // 0x0B NO/SLOW Motion Duration <5:0> SMNM
#define ACC_NM_THRE (0x15) //0x0A Accelerometer NO/SLOW motion threshold
#define ACC_HG_THRES (0x14) //0xC0 Accelerometer High G Threshold
#define ACC_HG_DURATION (0x13) //0x0F Accelerometer High G Duration
#define ACC_INT_Settings (0x12) //0x03
#define ACC_AM_THRES (0x11) //0x14 Accelerometer Any motion Threshold
#define INT_EN (0x10) //0x00
#define INT_MSK (0x0F) //0x00
#define GYR_SLeep_Config (0x0B) //0x00
#define ACC_Sleep_Config (0x0C) // 0x00
#define GYR_Config_1 (0x0B) // 0x00
#define GYR_Config_0 (0x0A) //0x00
#define MAG_Config (0x0A) //0x6D
#define ACC_Config (0x08) // 0x0D
#define PageID (0x07) //0x01

/* Page id register definition */
#define BNO055_PAGE_ID_ADDR  (0X07)

      /* PAGE0 REGISTER DEFINITION START*/
#define      BNO055_CHIP_ID_ADDR (0x00)
#define       BNO055_ACCEL_REV_ID_ADDR  (0x01)
#define       BNO055_MAG_REV_ID_ADDR (0x02)
#define       BNO055_GYRO_REV_ID_ADDR (0x03)
#define       BNO055_SW_REV_ID_LSB_ADDR  (0x04)
#define       BNO055_SW_REV_ID_MSB_ADDR   (0x05)
#define       BNO055_BL_REV_ID_ADDR (0X06)

      /* Accel data register */
#define       BNO055_ACCEL_DATA_X_LSB_ADDR (0X08)
#define       BNO055_ACCEL_DATA_X_MSB_ADDR  (0X09)
#define       BNO055_ACCEL_DATA_Y_LSB_ADDR  (0X0A)
#define       BNO055_ACCEL_DATA_Y_MSB_ADDR   (0X0B)
#define       BNO055_ACCEL_DATA_Z_LSB_ADDR   (0X0C)
#define       BNO055_ACCEL_DATA_Z_MSB_ADDR   (0X0D)
    
    /* Mag data register */
#define       BNO055_MAG_DATA_X_LSB_ADDR   (0X0E)
#define       BNO055_MAG_DATA_X_MSB_ADDR    (0X0F)
#define       BNO055_MAG_DATA_Y_LSB_ADDR    (0X10)
#define       BNO055_MAG_DATA_Y_MSB_ADDR    (0X11)
#define       BNO055_MAG_DATA_Z_LSB_ADDR   (0X12)
#define       BNO055_MAG_DATA_Z_MSB_ADDR   (0X13)

      /* Gyro data registers */
#define       BNO055_GYRO_DATA_X_LSB_ADDR  (0X14)
#define       BNO055_GYRO_DATA_X_MSB_ADDR  (0X15)
#define       BNO055_GYRO_DATA_Y_LSB_ADDR (0X16)
#define       BNO055_GYRO_DATA_Y_MSB_ADDR (0X17)
#define       BNO055_GYRO_DATA_Z_LSB_ADDR (0X18)
#define       BNO055_GYRO_DATA_Z_MSB_ADDR (0X19)

     /* Quaternion data registers */
#define       BNO055_QUATERNION_DATA_W_LSB_ADDR  (0X20)
#define       BNO055_QUATERNION_DATA_W_MSB_ADDR  (0X21)
#define       BNO055_QUATERNION_DATA_X_LSB_ADDR  (0X22)
#define       BNO055_QUATERNION_DATA_X_MSB_ADDR  (0X23)
#define       BNO055_QUATERNION_DATA_Y_LSB_ADDR  (0X24)
#define       BNO055_QUATERNION_DATA_Y_MSB_ADDR  (0X25)
#define       BNO055_QUATERNION_DATA_Z_LSB_ADDR  (0X26)
#define       BNO055_QUATERNION_DATA_Z_MSB_ADDR  (0X27)

     /* Gravity data registers */
#define       BNO055_GRAVITY_DATA_X_LSB_ADDR  (0X2E)
#define       BNO055_GRAVITY_DATA_X_MSB_ADDR  (0X2F)
#define       BNO055_GRAVITY_DATA_Y_LSB_ADDR  (0X30)
#define       BNO055_GRAVITY_DATA_Y_MSB_ADDR  (0X31)
#define       BNO055_GRAVITY_DATA_Z_LSB_ADDR  (0X32)
#define       BNO055_GRAVITY_DATA_Z_MSB_ADDR (0X33)

      /* Status registers */
#define       BNO055_CALIB_STAT_ADDR  (0X35)
#define       BNO055_SELFTEST_RESULT_ADDR  (0X36)
#define       BNO055_INTR_STAT_ADDR  (0X37)

#define       BNO055_SYS_CLK_STAT_ADDR  (0X38)
#define       BNO055_SYS_STAT_ADDR     (0X39)
#define       BNO055_SYS_ERR_ADDR      (0X3A)

      /* Unit selection register */
#define       BNO055_UNIT_SEL_ADDR   (0X3B)
#define       BNO055_DATA_SELECT_ADDR (0X3C)

      /* Mode registers */
#define       BNO055_OPR_MODE_ADDR   (0X3D)
#define       BNO055_PWR_MODE_ADDR   (0X3E)

#define       BNO055_SYS_TRIGGER_ADDR  (0X3F)
#define       BNO055_TEMP_SOURCE_ADDR  (0X40)

      /* Axis remap registers */
#define       BNO055_AXIS_MAP_CONFIG_ADDR   (0X41)
#define       BNO055_AXIS_MAP_SIGN_ADDR (0X42)

/* [] END OF FILE */
