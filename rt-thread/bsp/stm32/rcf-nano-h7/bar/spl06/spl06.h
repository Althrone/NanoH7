/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\bar\spl06\spl06.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_BAR_SPL06_SPL06_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_BAR_SPL06_SPL06_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <rtthread.h>
#include "sensor.h"
#include "drv_spi.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum 
{
    SPL06_PSR_B2_REG_ADDR       = 0x00,
    SPL06_PSR_B1_REG_ADDR       = 0x01,
    SPL06_PSR_B0_REG_ADDR       = 0x02,

    SPL06_TMP_B2_REG_ADDR       = 0x03,
    SPL06_TMP_B1_REG_ADDR       = 0x04,
    SPL06_TMP_B0_REG_ADDR       = 0x05,

    SPL06_PRS_CFG_REG_ADDR      = 0x06,
    SPL06_TMP_CFG_REG_ADDR      = 0x07,
    SPL06_MEAS_CFG_REG_ADDR     = 0x08,
    SPL06_CFG_REG_ADDR          = 0x09,
    SPL06_INT_STS_REG_ADDR      = 0x0A,
    SPL06_FIFO_STS_REG_ADDR     = 0x0B,
    SPL06_RESET_REG_ADDR        = 0x0C,
    SPL06_ID_REG_ADDR           = 0x0D,

    SPL06_COEF_C0_REG_ADDR      = 0x10,
    SPL06_COEF_C0_C1_REG_ADDR   = 0x11,
    SPL06_COEF_C1_REG_ADDR      = 0x12,
    SPL06_COEF_C00H_REG_ADDR    = 0x13,
    SPL06_COEF_C00L_REG_ADDR    = 0x14,
    SPL06_COEF_C00_C11_REG_ADDR = 0x15,
    SPL06_COEF_C10H_REG_ADDR    = 0x16,
    SPL06_COEF_C10L_REG_ADDR    = 0x17,
    SPL06_COEF_C01H_REG_ADDR    = 0x18,
    SPL06_COEF_C01L_REG_ADDR    = 0x19,
    SPL06_COEF_C11H_REG_ADDR    = 0x1A,
    SPL06_COEF_C11L_REG_ADDR    = 0x1B,
    SPL06_COEF_C20H_REG_ADDR    = 0x1C,
    SPL06_COEF_C20L_REG_ADDR    = 0x1D,
    SPL06_COEF_C21H_REG_ADDR    = 0x1E,
    SPL06_COEF_C21L_REG_ADDR    = 0x1F,
    SPL06_COEF_C30H_REG_ADDR    = 0x20,
    SPL06_COEF_C30L_REG_ADDR    = 0x21
}SPL06RegAddrEnum;

/******************************************************************************
 * pubilc functions
 *****************************************************************************/

void spl06_init(void);
void spl06_reset(void);

rt_err_t spl06_get_id(struct rt_sensor_device *sensor, void *args);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_BAR_SPL06_SPL06_H_ */