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
#include "bno055.h"
#ifndef _FrasersMagic_h_included
    #define _FrasersMagic_h_included

    u8 dev_addr = BNO055_I2C_ADDR2;

    void write8(uint8 reg_addr, uint8 data);
    uint8 read8(uint8 reg_addr);
    s8 read_qw();
    s8 read_qx();
    s8 read_qy();
    s8 read_qz();
    u8 read_quat_data();

    void sensor_set_mode();
    void set_data_type();
    void power_set_mode();
    void set_page_zero();
    void units_set_mode();

#endif
