/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
    -- Coded by Victor Mayoral Vilches --    

    The MPU9150 is a sensor composed by a MPU6050 with a AK8975 on the auxiliary bus.
    Please check the following links for datasheets and documentation:
     - http://www.invensense.com/mems/gyro/documents/PS-MPU-9150A-00v4_3.pdf
     - http://www.invensense.com/mems/gyro/documents/RM-MPU-9150A-00v4_2.pdf

     Note that this is an experimental driver. It is not used by any
     actively maintained board and should be considered untested and
     unmaintained 
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include <AP_Math.h>
#include "AP_InertialSensor_MPU9150.h"
//#include <stdio.h>
//#include <unistd.h>
#include <time.h>     

const extern AP_HAL::HAL& hal;

///////
/* Hardware registers needed by driver. */
struct gyro_reg_s {
    uint8_t who_am_i;
    uint8_t rate_div;
    uint8_t lpf;
    uint8_t prod_id;
    uint8_t user_ctrl;
    uint8_t fifo_en;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;
    uint8_t motion_thr;
    uint8_t motion_dur;
    uint8_t fifo_count_h;
    uint8_t fifo_r_w;
    uint8_t raw_gyro;
    uint8_t raw_accel;
    uint8_t temp;
    uint8_t int_enable;
    uint8_t dmp_int_status;
    uint8_t int_status;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    uint8_t int_pin_cfg;
    uint8_t mem_r_w;
    uint8_t accel_offs;
    uint8_t i2c_mst;
    uint8_t bank_sel;
    uint8_t mem_start_addr;
    uint8_t prgm_start_h;
    uint8_t raw_compass;    
    uint8_t yg_offs_tc;
    uint8_t s0_addr;
    uint8_t s0_reg;
    uint8_t s0_ctrl;
    uint8_t s1_addr;
    uint8_t s1_reg;
    uint8_t s1_ctrl;
    uint8_t s4_ctrl;
    uint8_t s0_do;
    uint8_t s1_do;
    uint8_t i2c_delay_ctrl;
};

/* Information specific to a particular device. */
struct hw_s {
    uint8_t addr;
    uint16_t max_fifo;
    uint8_t num_reg;
    uint16_t temp_sens;
    int16_t temp_offset;
    uint16_t bank_size;
    uint16_t compass_fsr;
};

/* Information for self-test. */
struct test_s {
    uint32_t gyro_sens;
    uint32_t accel_sens;
    uint8_t reg_rate_div;
    uint8_t reg_lpf;
    uint8_t reg_gyro_fsr;
    uint8_t reg_accel_fsr;
    uint16_t wait_ms;
    uint8_t packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
};

/* Gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    const struct test_s *test;
};

/* Filter configurations. The values correspond to the DLPF_CFG register.
Note that the gyro and accel frequencies are slightly different.
 (DLPF_CFG register, RM-MPU-9150A00.pdf, pg. 13)
*/
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ = 1,
    INV_FILTER_98HZ = 2,
    INV_FILTER_42HZ = 3,
    INV_FILTER_20HZ = 4,
    INV_FILTER_10HZ = 5,
    INV_FILTER_5HZ = 6,
    INV_FILTER_2100HZ_NOLPF = 7,
    NUM_FILTER
};

/* Full scale ranges. FS_SEL register.
 (RM-MPU-9150A00.pdf, pg. 14)
 */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS = 1,
    INV_FSR_1000DPS = 2,
    INV_FSR_2000DPS = 3,
    NUM_GYRO_FSR
};

/* Full scale ranges. AFS_SEL register.
 (RM-MPU-9150A00.pdf, pg. 15)
*/
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G = 1,
    INV_FSR_8G = 2,
    INV_FSR_16G = 4,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

#define MPU9150_ADDRESS                  0x68

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
// AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)
#define MAX_COMPASS_SAMPLE_RATE (100)

// Gyroscope scale (uncertain where the 0.01745 value comes from)
#define MPU9150_GYRO_SCALE_2000  (0.0174532f / 16.4f)
#define MPU9150_GYRO_SCALE_1000  (0.0174532f / 32.8f)
#define MPU9150_GYRO_SCALE_500  (0.0174532f / 65.5f)
#define MPU9150_GYRO_SCALE_250  (0.0174532f / 131f)

// Accelerometer scale adjustment
#define MPU9150_ACCEL_SCALE_16G    (GRAVITY_MSS / 2048.0f)
#define MPU9150_ACCEL_SCALE_8G    (GRAVITY_MSS / 4096.0f)
#define MPU9150_ACCEL_SCALE_4G    (GRAVITY_MSS / 8192.0f)
#define MPU9150_ACCEL_SCALE_2G    (GRAVITY_MSS / 16384.0f)

#define MPUREG_USER_CTRL         0x6A
#define MPUREG_INT_PIN_CFG       0x37
#define BIT_I2C_BYPASS_EN        0x02 


#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

// MPU 6000 I2C Address
#define MPU6000_ADDR 0x68

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
	#define MPUREG_SMPLRT_1000HZ                             0x00
	#define MPUREG_SMPLRT_500HZ                              0x01
	#define MPUREG_SMPLRT_250HZ                              0x03
	#define MPUREG_SMPLRT_200HZ                              0x04
	#define MPUREG_SMPLRT_100HZ                              0x09
	#define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG 0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#       define BIT_I2C_BYPASS_EN                                0x02 
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin 
#define MPUREG_INT_ENABLE 0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS 0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07

											// Product ID Description for MPU6000
											// high 4 bits 	low 4 bits
											// Product Name	Product Revision
#define MPU6000_REV_A4				0x04 	// 0000			0100
#define MPU6000ES_REV_C4 			0x14 	// 0001			0100
#define MPU6000ES_REV_C5 			0x15 	// 0001			0101
#define MPU6000ES_REV_D6 			0x16	// 0001			0110
#define MPU6000ES_REV_D7 			0x17	// 0001			0111
#define MPU6000ES_REV_D8 			0x18	// 0001			1000	
#define MPU6000_REV_C4 				0x54	// 0101			0100 
#define MPU6000_REV_C5 				0x55	// 0101			0101
#define MPU6000_REV_D6 				0x56	// 0101			0110	
#define MPU6000_REV_D7 				0x57	// 0101			0111
#define MPU6000_REV_D8 				0x58	// 0101			1000
#define MPU6000_REV_D9 				0x59	// 0101			1001

static volatile uint32_t _ins_timer = 0;
static volatile bool _sens_stage = 0;


const struct gyro_reg_s reg = {
/*    .who_am_i      */ 0x75,
/*    .rate_div      */ 0x19,
/*    .lpf           */ 0x1A,
/*    .prod_id       */ 0x0C,
/*    .user_ctrl     */ 0x6A,
/*    .fifo_en       */ 0x23,
/*    .gyro_cfg      */ 0x1B,
/*    .accel_cfg     */ 0x1C,
/*    .motion_thr    */ 0x1F,
/*    .motion_dur    */ 0x20,
/*    .fifo_count_h  */ 0x72,
/*    .fifo_r_w      */ 0x74,
/*    .raw_gyro      */ 0x43,
/*    .raw_accel     */ 0x3B,
/*    .temp          */ 0x41,
/*    .int_enable    */ 0x38,
/*    .dmp_int_status*/ 0x39,
/*    .int_status    */ 0x3A,
/*    .pwr_mgmt_1    */ 0x6B,
/*    .pwr_mgmt_2    */ 0x6C,
/*    .int_pin_cfg   */ 0x37,
/*    .mem_r_w       */ 0x6F,
/*    .accel_offs    */ 0x06,
/*    .i2c_mst       */ 0x24,
/*    .bank_sel      */ 0x6D,
/*    .mem_start_addr*/ 0x6E,
/*    .prgm_start_h  */ 0x70,
/*    .raw_compass   */ 0x49,
/*    .yg_offs_tc    */ 0x01,
/*    .s0_addr       */ 0x25,
/*    .s0_reg        */ 0x26,
/*    .s0_ctrl       */ 0x27,
/*    .s1_addr       */ 0x28,
/*    .s1_reg        */ 0x29,
/*    .s1_ctrl       */ 0x2A,
/*    .s4_ctrl       */ 0x34,
/*    .s0_do         */ 0x63,
/*    .s1_do         */ 0x64,
/*    .i2c_delay_ctrl*/ 0x67
};

const struct hw_s hw = {
/*    .addr          */ 0x68,
/*    .max_fifo      */ 1024,
/*    .num_reg       */ 118,
/*    .temp_sens     */ 340,
/*    .temp_offset   */ -521,
/*    .bank_size     */ 256,
/*    .compass_fsr   */ AK89xx_FSR

};

const struct test_s test = {
/*    .gyro_sens     */ 32768/250,
/*    .accel_sens    */ 32768/16,
/*    .reg_rate_div  */ 0,    /* 1kHz. */
/*    .reg_lpf       */ 1,    /* 188Hz. */
/*    .reg_gyro_fsr  */ 0,    /* 250dps. */
/*    .reg_accel_fsr */ 0x18, /* 16g. */
/*    .wait_ms       */ 50,
/*    .packet_thresh */ 5,    /* 5% */
/*    .min_dps       */ 10.f,
/*    .max_dps       */ 105.f,
/*    .max_gyro_var  */ 0.14f,
/*    .min_g         */ 0.3f,
/*    .max_g         */ 0.95f,
/*    .max_accel_var */ 0.14f
};

static struct gyro_state_s st = {
/*    .reg  */ &reg,
/*    .hw   */ &hw,    
/*    .test */ &test
   
};

/**
 *  @brief      Constructor
 */
AP_InertialSensor_MPU9150::AP_InertialSensor_MPU9150(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _have_sample_available(false),
    _accel_filter_x(100, 20),
    _accel_filter_y(100, 20),
    _accel_filter_z(100, 20),
    _gyro_filter_x(100, 20),
    _gyro_filter_y(100, 20),
    _gyro_filter_z(100, 20)
{
}


/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_MPU9150::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_MPU9150 *sensor = new AP_InertialSensor_MPU9150(_imu);
    if (sensor == NULL) {
	    hal.console->printf("\n NULL!\n");
        return NULL;
    }
    if (!sensor->_init_sensor()) {
	    hal.console->printf("\n DELETE!\n");
        delete sensor;
        return NULL;
    }
    return sensor;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_MPU9150::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0)
        filter_hz = _default_filter_hz;

    _accel_filter_x.set_cutoff_frequency(100, filter_hz);
    _accel_filter_y.set_cutoff_frequency(100, filter_hz);
    _accel_filter_z.set_cutoff_frequency(100, filter_hz);
    _gyro_filter_x.set_cutoff_frequency(100, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(100, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(100, filter_hz);
}

/**
 *  Init method
 */
bool AP_InertialSensor_MPU9150::_init_sensor(void) 
{
    // Sensors pushed to the FIFO.
	hal.console->printf("\n Iniciando!\n");
    uint8_t sensors;

    _default_filter_hz = _default_filter();

    // get pointer to i2c bus semaphore
    //AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();
    i2c_sem = hal.i2c->get_semaphore();


    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;
    do {
        bool success = hardware_init(AP_InertialSensor::RATE_100HZ);
        if (success) {
		    hal.console->printf("\n succes!\n");
        	break;
        } else {
            hal.scheduler->delay(50); // delay for 50ms
                hal.console->println_P(
                        PSTR("MPU6000 startup failed: no data ready"));
        }
        if (tries++ > 5) {
            hal.scheduler->panic(PSTR("PANIC: failed to boot MPU6000 5 times")); 
        }
    } while (1);

    hal.scheduler->resume_timer_procs();

    _ins_timer = hal.scheduler->micros();

    //_accumulate();
    hal.console->printf("\n Acumul!\n");
	/*

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){
        return false;
    }        

	
	hal.console->printf("\n PASO!\n");
	
    // Init the sensor
    //  Reset the device
    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->pwr_mgmt_1, 
                           BIT_RESET);
    hal.scheduler->delay(100);
    
    //  Wake up the chip
    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->pwr_mgmt_1, 
                           0x03);
    hal.scheduler->delay(5);
    // Check product revision
    //  This registers are not documented in the register map.
    uint8_t buff[6];
    if (hal.i2c->readRegisters(st.hw->addr, st.reg->accel_offs, 6, buff) != 0) {
        hal.console->printf("AP_InertialSensor_MPU9150: couldn't read the registers to determine revision");
        goto failed;
    }    
    uint8_t rev;
    rev = ((buff[5] & 0x01) << 2) | ((buff[3] & 0x01) << 1) |
        (buff[1] & 0x01);
	*/
    // Do not do the checking, for some reason the MPU-9150 returns 0
    // if (rev) {        
    //     if (rev == 1){
    //         /* Congrats, these parts are better. */
    //         ;
    //     }
    //     else if (rev == 2){
    //         ;
    //     }
    //     else {
    //         hal.scheduler->panic(PSTR("AP_InertialSensor_MPU9150: Unsupported software product rev.\n"));
    //         goto failed;
    //     }
    // } else {
    //         hal.scheduler->panic(PSTR("Product ID read as 0 indicates device is either incompatible or an MPU3050.\n"));
    //         goto failed;            
    // }

    // Set gyro full-scale range [250, 500, 1000, 2000]
	/*
    if (mpu_set_gyro_fsr(2000)){
        hal.console->printf("AP_InertialSensor_MPU9150: mpu_set_gyro_fsr.\n");
        goto failed;
    }
	 hal.scheduler->delay(1);
    // Set the accel full-scale range
    if (mpu_set_accel_fsr(2)){
        hal.console->printf("AP_InertialSensor_MPU9150: mpu_set_accel_fsr.\n");
        goto failed;    
    }
	 hal.scheduler->delay(1);
    // Set digital low pass filter (using _default_filter_hz, 20 for 100 Hz of sample rate)
    if (mpu_set_lpf(20)){
        hal.console->printf("AP_InertialSensor_MPU9150: mpu_set_lpf.\n");
        goto failed;    
    }
	 hal.scheduler->delay(1);
    // Set sampling rate (value must be between 4Hz and 1KHz)
    if (mpu_set_sample_rate(10)){
        hal.console->printf("AP_InertialSensor_MPU9150: mpu_set_sample_rate.\n");
        goto failed;    
    }
	 hal.scheduler->delay(1);
    */
	// Select which sensors are pushed to FIFO.
	
    sensors = INV_XYZ_ACCEL| INV_XYZ_GYRO;
    if (mpu_configure_fifo(sensors)){
        hal.console->printf("AP_InertialSensor_MPU9150: mpu_configure_fifo.\n");
        goto failed;    
    }
	
     hal.scheduler->delay(1);
	 /*
    // For now the compass is not used.
    // TODO adjust the functions to the ArduPilot API

    // setup_compass();
    // if (mpu_set_compass_sample_rate(10, 400))
    //     return -1;
	hal.console->printf("\n PASO1!\n");
    mpu_set_sensors(sensors);

    // Set the filter frecuency (_mpu6000_filter configured to the default value, check AP_InertialSensor.cpp)
    _set_filter_frequency(_imu.get_filter());


    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    hal.i2c->writeRegister(st.hw->addr, st.reg->pwr_mgmt_2, 0);
    hal.scheduler->delay(1);

	hal.console->printf("\n Registrando!\n");
    hal.i2c->writeRegister(st.hw->addr, MPUREG_USER_CTRL, 0);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(st.hw->addr, MPUREG_INT_PIN_CFG, BIT_I2C_BYPASS_EN);
    hal.console->printf("\n Registrado!\n");
    hal.scheduler->delay(100);

    hal.console->println_P(
                     PSTR("I2C BYPASS MODE"));
    */
    // give back i2c semaphore
    i2c_sem->give();
	
	//hal.scheduler->resume_timer_procs();

    // start the timer process to read samples    
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU9150::_accumulate));

    return true;

failed:
    // give back i2c semaphore
	hal.console->printf("\n FAILED!\n");
    i2c_sem->give();
    return false;
}


void AP_InertialSensor_MPU9150::_poll_data(void)
{
	hal.gpio->write(30,1); 

	uint32_t now = hal.scheduler->micros();
	
	if ( (now - _ins_timer > _micros_per_sample) || (_sens_stage == 1)) {
		if (hal.scheduler->in_timerprocess()) {
			if (i2c_sem->take_nonblocking()) {
				_accumulate();
				if (_sens_stage == 0) {
					_ins_timer = now;
				}
				i2c_sem->give();
			}
	  } else {
	      /* Synchronous read - take semaphore */
	      if (i2c_sem->take(10)) {
					  if (_sens_stage == 0) {
							_ins_timer = now;
						}
						_accumulate(); 
						i2c_sem->give();
	      } else {
	          hal.scheduler->panic(
	                  PSTR("PANIC: AP_InertialSensor_MPU6000::_poll_data "
	                       "failed to take I2C semaphore synchronously"));
	      }
	  }
	}
  hal.gpio->write(30,0); 
}


bool AP_InertialSensor_MPU9150::hardware_init(AP_InertialSensor::Sample_rate sample_rate)
{
    if (!i2c_sem->take(100)) {
        hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
    }

    // Chip reset
    uint8_t tries;
    uint8_t reg_val;
    for (tries = 0; tries<5; tries++) {
			hal.i2c->writeRegister(st.hw->addr, st.reg->pwr_mgmt_1, BIT_RESET);
			hal.scheduler->delay(100);
			
			// Wake up device and select GyroZ clock. Note that the
			// MPU6000 starts up in sleep mode, and it can take some time
			// for it to come out of sleep
			hal.i2c->writeRegister(st.hw->addr, st.reg->pwr_mgmt_1, 0x03);
			hal.scheduler->delay(5);
			
			// check it has woken up
			hal.i2c->readRegister(st.hw->addr, st.reg->pwr_mgmt_1, &reg_val);
			if (reg_val == 0x03) {
				break;
			}
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU6000 5 times"));
        i2c_sem->give();
        return false;
    }

    // only used for wake-up in accelerometer only low power mode
    hal.i2c->writeRegister(st.hw->addr, st.reg->pwr_mgmt_2, 0);
    hal.scheduler->delay(1);
    
    uint8_t default_filter, rate;

    // sample rate and filtering
    // to minimise the effects of aliasing we choose a filter
    // that is less than half of the sample rate
    switch (sample_rate) {
    case RATE_50HZ:
        // this is used for plane and rover, where noise resistance is
        // more important than update rate. Tests on an aerobatic plane
        // show that 10Hz is fine, and makes it very noise resistant
				_micros_per_sample = 19000;
				_delta_time = 0.02;
    	  rate = MPUREG_SMPLRT_50HZ;
        default_filter = BITS_DLPF_CFG_10HZ;
        break;
    case RATE_100HZ:
				_micros_per_sample = 9900;
				_delta_time = 0.01;
    	  rate = MPUREG_SMPLRT_100HZ;
        default_filter = BITS_DLPF_CFG_20HZ;
		hal.console->printf("\n 100!\n");
        break;
    case RATE_200HZ:
    default:
				_micros_per_sample = 4900;
				_delta_time = 0.005;
    	  rate = MPUREG_SMPLRT_200HZ;
        default_filter = BITS_DLPF_CFG_20HZ;
        break;
    }

    _set_filter_register(_imu.get_filter(), default_filter);

    // set sample rate to 200Hz, and use _sample_divider to give
    // the requested rate to the application
    hal.i2c->writeRegister(mpu_addr, MPUREG_SMPLRT_DIV, rate);
    hal.scheduler->delay(1);

    hal.i2c->writeRegister(mpu_addr, MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS); // Gyro scale 2000?/s
    hal.scheduler->delay(1);

		// Get chip revision
    hal.i2c->readRegister(mpu_addr, MPUREG_PRODUCT_ID, &reg_val);
    _mpu6000_product_id = reg_val;

		// Select Accel scale
		if ( (_mpu6000_product_id == MPU6000_REV_A4) || (_mpu6000_product_id == MPU6000ES_REV_C4) || (_mpu6000_product_id == MPU6000ES_REV_C5) ||
			(_mpu6000_product_id == MPU6000_REV_C4)   || (_mpu6000_product_id == MPU6000_REV_C5)){
			// Accel scale 8g (4096 LSB/g)
			// Rev C has different scaling than rev D
			hal.i2c->writeRegister(mpu_addr, MPUREG_ACCEL_CONFIG, 1<<3);
		} else {
			// Accel scale 8g (4096 LSB/g)
			hal.i2c->writeRegister(mpu_addr, MPUREG_ACCEL_CONFIG, 2<<3);
		}
			
    hal.scheduler->delay(1);


    // Enable I2C bypass mode, to work with Magnetometer 5883L
    // Disable I2C Master mode
    hal.i2c->writeRegister(mpu_addr, MPUREG_USER_CTRL, 0);
	hal.scheduler->delay(5);
    hal.i2c->writeRegister(mpu_addr, MPUREG_INT_PIN_CFG, BIT_I2C_BYPASS_EN);
	hal.scheduler->delay(100);

    
/*  Dump MPU6050 registers  
    hal.console->println_P(PSTR("MPU6000 registers"));
    for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
    	  hal.i2c->readRegister(mpu_addr, reg, &reg_val);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)reg_val);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();*/
    
    i2c_sem->give();
    return true;
}




/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU9150::_set_filter_register(uint8_t filter_hz, uint8_t default_filter)
{
    uint8_t filter = default_filter;
    // choose filtering frequency
    switch (filter_hz) {
    case 5:
        filter = BITS_DLPF_CFG_5HZ;
        break;
    case 10:
        filter = BITS_DLPF_CFG_10HZ;
        break;
    case 20:
        filter = BITS_DLPF_CFG_20HZ;
        break;
    case 42:
        filter = BITS_DLPF_CFG_42HZ;
        break;
    case 98:
        filter = BITS_DLPF_CFG_98HZ;
        break;
    }

    if (filter != 0) {
        _last_filter_hz = filter_hz;
        hal.i2c->writeRegister(mpu_addr, MPUREG_CONFIG, filter);
    }
}



/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_gyro_fsr(uint16_t fsr)
{
    uint8_t data;

    switch (fsr) {
    case 250:
        data = INV_FSR_250DPS << 3;
        break;
    case 500:
        data = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        return -1;
    }
	
	
    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->gyro_cfg,
                           data);
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_accel_fsr(uint8_t fsr)
{
    uint8_t data;

    switch (fsr) {
    case 2:
        data = INV_FSR_2G << 3;
        break;
    case 4:
        data = INV_FSR_4G << 3;
        break;
    case 8:
        data = INV_FSR_8G << 3;
        break;
    case 16:
        data = INV_FSR_16G << 3;
        break;
    default:
        return -1;
    }

    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->accel_cfg,
                           data);
    return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_lpf(uint16_t lpf)
{
    uint8_t data;

    if (lpf >= 188){
        data = INV_FILTER_188HZ;
    }
    else if (lpf >= 98){
        data = INV_FILTER_98HZ;
    }
    else if (lpf >= 42){
        data = INV_FILTER_42HZ;
    }
    else if (lpf >= 20){
        data = INV_FILTER_20HZ;
    }
    else if (lpf >= 10){
        data = INV_FILTER_10HZ;
    }
    else {
        data = INV_FILTER_5HZ;
    }

    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->lpf,
                           data);
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_sample_rate(uint16_t rate)
{
    uint8_t data;
    // uint16_t sample_rate;

    if (rate < 4){
        rate = 4;
    }
    else if (rate > 1000){
        rate = 1000;
    }

    //data = 1000 / (rate + 1);
	data = rate;
    hal.i2c->writeRegister(st.hw->addr, 
                               st.reg->rate_div,
                               data);
    
    // sample_rate = 1000 / (1 + data);
    // mpu_set_compass_sample_rate(min(sample_rate, MAX_COMPASS_SAMPLE_RATE), rate);

    return 0;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_compass_sample_rate(uint16_t rate, uint16_t chip_sample_rate)
{
    uint8_t div;
    if (!rate || rate > MAX_COMPASS_SAMPLE_RATE){
        return -1;
    }

    div = chip_sample_rate / rate - 1;
    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->s4_ctrl,
                           div);
    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_configure_fifo(uint8_t sensors)
{
    int16_t result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    //sensors &= ~INV_XYZ_COMPASS;

    // Enable or disable the interrupts
     //set_int_enable(1);
    set_int_enable(0);
    if (sensors) {
        if (mpu_reset_fifo(sensors)) {            
            return -1;
        }
    }
    return result;
}

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::set_int_enable(uint8_t enable)
{
    uint8_t tmp;

    if (enable){
        tmp = BIT_DATA_RDY_EN;
    }
    else {
        tmp = 0x00;
    }
    hal.i2c->writeRegister(st.hw->addr, 
                           st.reg->int_enable,
                           tmp);
    return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_reset_fifo(uint8_t sensors)
{
    uint8_t data;

    data = 0;
    hal.i2c->writeRegister(st.hw->addr,st.reg->int_enable, data);
    hal.i2c->writeRegister(st.hw->addr,st.reg->fifo_en, data);
    hal.i2c->writeRegister(st.hw->addr,st.reg->user_ctrl, data);
    data = BIT_FIFO_RST;
    hal.i2c->writeRegister(st.hw->addr,st.reg->user_ctrl, data);

    data = BIT_FIFO_EN;
    // data = BIT_FIFO_EN | BIT_AUX_IF_EN;
    hal.i2c->writeRegister(st.hw->addr,st.reg->user_ctrl, data);
    hal.scheduler->delay(50);

    // interrupts for the DMP
    // data = BIT_DATA_RDY_EN;
    data = 0;
    hal.i2c->writeRegister(st.hw->addr,st.reg->int_enable, data);
    // enable FIFO
    hal.i2c->writeRegister(st.hw->addr,st.reg->fifo_en, sensors);
    return 0;
}


#if 0
/* This initialization is similar to the one in ak8975.c. 
        TODO: Use the ArduPilot APIs (write, read, ...), remove the st.chip_cfg cache vars
*/
static int AP_InertialSensor_MPU9150::setup_compass(void)
{
    uint8_t data[4], akm_addr;

    mpu_set_bypass(1);

    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
        int result;
        result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data);
        if (!result && (data[0] == AKM_WHOAMI))
            break;
    }

    if (akm_addr > 0x0F) {
        /* TODO: Handle this case in all compass-related functions. */
        log_e("Compass not found.\n");
        return -1;
    }

    st.chip_cfg.compass_addr = akm_addr;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    data[0] = AKM_FUSE_ROM_ACCESS;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    /* Get sensitivity adjustment data from fuse ROM. */
    if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ASAX, 3, data))
        return -1;
    st.chip_cfg.mag_sens_adj[0] = (long)data[0] + 128;
    st.chip_cfg.mag_sens_adj[1] = (long)data[1] + 128;
    st.chip_cfg.mag_sens_adj[2] = (long)data[2] + 128;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    mpu_set_bypass(0);

    /* Set up master mode, master clock, and ES bit. */
    data[0] = 0x40;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;

    /* Slave 0 reads from AKM data registers. */
    data[0] = BIT_I2C_READ | st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s0_addr, 1, data))
        return -1;

    /* Compass reads start at this register. */
    data[0] = AKM_REG_ST1;
    if (i2c_write(st.hw->addr, st.reg->s0_reg, 1, data))
        return -1;

    /* Enable slave 0, 8-byte reads. */
    data[0] = BIT_SLAVE_EN | 8;
    if (i2c_write(st.hw->addr, st.reg->s0_ctrl, 1, data))
        return -1;

    /* Slave 1 changes AKM measurement mode. */
    data[0] = st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s1_addr, 1, data))
        return -1;

    /* AKM measurement mode register. */
    data[0] = AKM_REG_CNTL;
    if (i2c_write(st.hw->addr, st.reg->s1_reg, 1, data))
        return -1;

    /* Enable slave 1, 1-byte writes. */
    data[0] = BIT_SLAVE_EN | 1;
    if (i2c_write(st.hw->addr, st.reg->s1_ctrl, 1, data))
        return -1;

    /* Set slave 1 data. */
    data[0] = AKM_SINGLE_MEASUREMENT;
    if (i2c_write(st.hw->addr, st.reg->s1_do, 1, data))
        return -1;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data[0] = 0x03;
    if (i2c_write(st.hw->addr, st.reg->i2c_delay_ctrl, 1, data))
        return -1;

    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data[0] = BIT_I2C_MST_VDDIO;
    if (i2c_write(st.hw->addr, st.reg->yg_offs_tc, 1, data))
        return -1;

    return 0;
}
#endif

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_sensors(uint8_t sensors)
{
    uint8_t data;
    // uint8_t user_ctrl;

    if (sensors & INV_XYZ_GYRO){
        data = INV_CLK_PLL;
    }
    else if (sensors){
        data = 0;
    }
    else {
        data = BIT_SLEEP;
    }
    hal.i2c->writeRegister(st.hw->addr,st.reg->pwr_mgmt_1, data);
    data = 0;
    if (!(sensors & INV_X_GYRO)){
        data |= BIT_STBY_XG;
    }
    if (!(sensors & INV_Y_GYRO)){
        data |= BIT_STBY_YG;
    }
    if (!(sensors & INV_Z_GYRO)){
        data |= BIT_STBY_ZG;
    }
    if (!(sensors & INV_XYZ_ACCEL)){
        data |= BIT_STBY_XYZA;
    }
    hal.i2c->writeRegister(st.hw->addr,st.reg->pwr_mgmt_2, data);

#if 0
    if (sensors && (sensors != INV_XYZ_ACCEL)){
        /* Latched interrupts only used in LP accel mode. */
        mpu_set_int_latched(0);
    }
#endif

// // handle the compass, not implemented for now
// #ifdef AK89xx_SECONDARY
// #ifdef AK89xx_BYPASS
//     if (sensors & INV_XYZ_COMPASS)
//         mpu_set_bypass(1);
//     else
//         mpu_set_bypass(0);
// #else
//     if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
//         return -1;
//     /* Handle AKM power management. */
//     if (sensors & INV_XYZ_COMPASS) {
//         data = AKM_SINGLE_MEASUREMENT;
//         user_ctrl |= BIT_AUX_IF_EN;
//     } else {
//         data = AKM_POWER_DOWN;
//         user_ctrl &= ~BIT_AUX_IF_EN;
//     }
//     if (st.chip_cfg.dmp_on)
//         user_ctrl |= BIT_DMP_EN;
//     else
//         user_ctrl &= ~BIT_DMP_EN;
//     if (i2c_write(st.hw->addr, st.reg->s1_do, 1, &data))
//         return -1;
//     /* Enable/disable I2C master mode. */
//     if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
//         return -1;
// #endif

    hal.scheduler->delay(50);
    return 0;
}

#if 0
/**
 *          TODO: Remove the st.chip_cfg cache variables
 *
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_set_int_latched(uint8_t enable)
{
    uint8_t tmp;
    if (st.chip_cfg.latched_int == enable){
        return 0;
    }

    if (enable){
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    }
    else {
        tmp = 0;
    }
    if (st.chip_cfg.bypass_mode){
        tmp |= BIT_BYPASS_EN;
    }
    if (st.chip_cfg.active_low_int){
        tmp |= BIT_ACTL;
    }
    hal.i2c->writeRegister(st.hw->addr,st.reg->int_pin_cfg, tmp);
    st.chip_cfg.latched_int = enable;
    return 0;
}
#endif



/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int16_t AP_InertialSensor_MPU9150::mpu_read_fifo(int16_t *gyro, int16_t *accel, uint32_t timestamp,
        uint8_t *sensors, uint8_t *more)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    uint8_t data[12];
    uint8_t packet_size = 0;
    uint16_t fifo_count, index = 0;
    uint8_t sensors_aux = INV_XYZ_ACCEL| INV_XYZ_GYRO;

    sensors[0] = 0;
    // We assume we want gyro and accel values
    packet_size = 12;

    // fifo_count_h register contains the number of samples in the FIFO
    hal.i2c->readRegisters(st.hw->addr, st.reg->fifo_count_h, 2, data);

    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size){
        return 0;
    }
    // hal.console->printf_P(PTR("FIFO count: %hd\n", fifo_count));
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */        
        hal.i2c->readRegister(st.hw->addr, st.reg->int_status, data);
        if (data[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo(sensors_aux);
            return -2;
        }
    }

    timestamp = hal.scheduler->millis();    
    // read the data
    hal.i2c->readRegisters(st.hw->addr, st.reg->fifo_r_w, packet_size, data);
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if (index != packet_size) {
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= INV_XYZ_ACCEL;
        index += 6;
    }
    if (index != packet_size) {
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_X_GYRO;
        index += 2;
    }
    if (index != packet_size) {
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Y_GYRO;
        index += 2;
    }
    if (index != packet_size) {
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Z_GYRO;
        index += 2;
    }
    return 0;
}

/**
 *  @brief      Accumulate values from accels and gyros. 
 *
 *     This method is called periodically by the scheduler.
 */
void AP_InertialSensor_MPU9150::_accumulate(void)
{
    // get pointer to i2c bus semaphore
	//hal.gpio->write(45,1); 
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take_nonblocking()){
        return;
    }

    // Read accelerometer FIFO to find out how many samples are available
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    uint8_t data[12];
    uint8_t packet_size = 12;
    uint16_t fifo_count, index = 0;
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    // fifo_count_h register contains the number of samples in the FIFO
    hal.i2c->readRegisters(st.hw->addr, st.reg->fifo_count_h, 2, data);
    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size){
        // give back i2c semaphore
        i2c_sem->give();
        return;
    }

    // hal.console->printf_P(PTR("FIFO count: %hd\n", fifo_count));
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */        
        hal.i2c->readRegister(st.hw->addr, st.reg->int_status, data);
        if (data[0] & BIT_FIFO_OVERFLOW) {            
            mpu_reset_fifo(INV_XYZ_ACCEL| INV_XYZ_GYRO);
            i2c_sem->give();
            return;
        }
    }    

    // read the samples
    for (uint16_t i=0; i< fifo_count; i++) {        
        // read the data
        // TODO check whether it's possible to read all the packages in a single call
        hal.i2c->readRegisters(st.hw->addr, st.reg->fifo_r_w, packet_size, data);
        // TODO, remove all the checking since it's being configured this way.
        if (index != packet_size) {
            accel_x = (int16_t) (data[index+0] << 8) | data[index+1];
            accel_y = (int16_t) (data[index+2] << 8) | data[index+3];
            accel_z = (int16_t) (data[index+4] << 8) | data[index+5];
            index += 6;
        }
        if (index != packet_size) {
            gyro_x = (int16_t) (data[index+0] << 8) | data[index+1];
            index += 2;
        }
        if (index != packet_size) {
            gyro_y = (int16_t) (data[index+0] << 8) | data[index+1];
            index += 2;
        }
        if (index != packet_size) {
            gyro_z = (int16_t) (data[index+0] << 8) | data[index+1];
            index += 2;
        }        
        // reset the index
        index = 0;

        // TODO Revisit why AP_InertialSensor_L3G4200D uses a minus sign in the y and z component. Maybe this
        //  is because the sensor is placed in the bottom side of the board?
        _accel_filtered = Vector3f(
            _accel_filter_x.apply(accel_x), 
            _accel_filter_y.apply(accel_y), 
            _accel_filter_z.apply(accel_z));
        
        _gyro_filtered = Vector3f(
            _gyro_filter_x.apply(gyro_x), 
            _gyro_filter_y.apply(gyro_y), 
            _gyro_filter_z.apply(gyro_z));

        _have_sample_available = true;
    }

    //hal.gpio->write(45,0); 
    // give back i2c semaphore
    i2c_sem->give();
}

bool AP_InertialSensor_MPU9150::update(void) 
{
    Vector3f accel, gyro;

    hal.scheduler->suspend_timer_procs();
    accel = _accel_filtered;
    gyro = _gyro_filtered;
    _have_sample_available = false;
    hal.scheduler->resume_timer_procs();

    accel *= MPU9150_ACCEL_SCALE_2G;
    _rotate_and_offset_accel(_accel_instance, accel);

    gyro *= MPU9150_GYRO_SCALE_2000;
    _rotate_and_offset_gyro(_gyro_instance, gyro);

    if (_last_filter_hz != _imu.get_filter()) {
        _set_filter_frequency(_imu.get_filter());
		//mpu_set_lpf(20);
        _last_filter_hz = _imu.get_filter();
    }

    return true;
}


#endif // CONFIG_HAL_BOARD
