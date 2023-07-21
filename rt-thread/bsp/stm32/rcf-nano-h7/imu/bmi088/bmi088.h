/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\imu\bmi088\bmi088.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <rtthread.h>
#include "sensor.h"
// #include "drv_gpio.h"
// #include "drv_spi.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum 
{
    ACC_CHIP_ID     = 0x00,
    ACC_ERR_REG     = 0x02,
    ACC_STATUS      = 0x03,
    ACC_X_LSB       = 0x12,
    ACC_X_MSB       = 0x13,
    ACC_Y_LSB       = 0x14,
    ACC_Y_MSB       = 0x15,
    ACC_Z_LSB       = 0x16,
    ACC_Z_MSB       = 0x17,
    SENSORTIME_0    = 0x18,
    SENSORTIME_1    = 0x19,
    SENSORTIME_2    = 0x1A,
    ACC_INT_STAT_1  = 0x1D,
    TEMP_MSB        = 0x22,
    TEMP_LSB        = 0x23,
    ACC_CONF        = 0x40,
    ACC_RANGE       = 0x41,
    INT1_IO_CTRL    = 0x53,
    INT2_IO_CTRL    = 0x54,
    INT_MAP_DATA    = 0x58,
    ACC_SELF_TEST   = 0x6D,
    ACC_PWR_CONF    = 0x7C,
    ACC_PWR_CTRL    = 0x7D,
    ACC_SOFTRESET   = 0x7E
}BMI088AccRegEnum;

typedef enum 
{
    GYRO_CHIP_ID        = 0x00,
    RATE_X_LSB          = 0x02,
    RATE_X_MSB          = 0x03,
    RATE_Y_LSB          = 0x04,
    RATE_Y_MSB          = 0x05,
    RATE_Z_LSB          = 0x06,
    RATE_Z_MSB          = 0x07,
    GYRO_INT_STAT_1     = 0x0A,
    GYRO_RANGE          = 0x0F,
    GYRO_BANDWIDTH      = 0x10,
    GYRO_LPM1           = 0x11,
    GYRO_SOFTRESET      = 0x14,
    GYRO_INT_CTRL       = 0x15,
    INT3_INT4_IO_CONF   = 0x16,
    INT3_INT4_IO_MAP    = 0x18,
    GYRO_SELF_TEST      = 0x3C
}BMI088GyroRegEnum;

/******************************************************************************
 * pubilc functions
 *****************************************************************************/

rt_err_t bmi088_acce_get_id(struct rt_sensor_device *sensor, void *args);
rt_err_t bmi088_gyro_get_id(struct rt_sensor_device *sensor, void *args);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_ */