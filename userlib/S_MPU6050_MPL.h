/**
 * @file S_MPU6050_MPL.h
 * @author Foriver 
 * @brief 配套用于32F4HAL库以及6050的MPL库
 * @version 0.1
 * @date 2022-01-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "stm32f4xx_hal.h"
#include "i2c.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"

#include "S_UART.h"
#include "S_SYSTICK.h"

#define MPU_I2C_ADDR hi2c1

//下面四个函数实现的是给motion_driver库的底层驱动，需要在inv_mpu_dmp_motion_driver.c和inv_mpu.c中设置
uint8_t MPU_i2cWrite(uint8_t slave_addr,uint8_t reg_addr,uint8_t length,uint8_t *data);
uint8_t MPU_i2cRead(uint8_t slave_addr,uint8_t reg_addr,uint8_t length,uint8_t *data);
void MPL_getms(uint32_t *num);
void MPL_printf(const char *fmt, ...);

/**
 * @brief 初始化函数没什么好说的
 * 
 */
void MPU6050_Init(void);

/**
 * @brief 直接获取解算出来的欧拉角数据
 * 
 * @param pitch 
 * @param roll 
 * @param yaw 
 */
uint8_t MPUGetData(float *pitch,float *roll,float *yaw);