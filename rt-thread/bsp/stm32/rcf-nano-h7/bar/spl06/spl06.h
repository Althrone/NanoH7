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

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t PM_PRC  :4;
        volatile rt_uint8_t PM_RATE :3;
        volatile rt_uint8_t         :1;
    }B;
}Spl06PrsCfgRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t TMP_PRC     :3;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t TMP_RATE    :3;
        volatile rt_uint8_t TMP_EXT     :1;
    }B;
}Spl06TmpCfgRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t MEAS_CRTL   :3;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t PRS_RDY     :1;
        volatile rt_uint8_t TMP_RDY     :1;
        volatile rt_uint8_t SENSOR_RDY  :1;
        volatile rt_uint8_t COEF_RDY    :1;
    }B;
}Spl06MeasCfgRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t SPI_MODE        :1;
        volatile rt_uint8_t FIFO_EN         :1;
        volatile rt_uint8_t PRS_SHIFT_EN    :1;
        volatile rt_uint8_t TMP_SHIFT_EN    :1;
        volatile rt_uint8_t INT_TMP         :1;
        volatile rt_uint8_t INT_PRS         :1;
        volatile rt_uint8_t INT_FIFO        :1;
        volatile rt_uint8_t INT_HL          :1;
    }B;
}Spl06CfgRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t INT_PRS         :1;
        volatile rt_uint8_t INT_TMP         :1;
        volatile rt_uint8_t INT_FIFO_FULL   :1;
        volatile rt_uint8_t                 :5;
    }B;
}Spl06IntStsRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t FIFO_EMPTY  :1;
        volatile rt_uint8_t FIFO_FULL   :1;
        volatile rt_uint8_t             :6;
    }B;
}Spl06FifoStsRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t SOFT_RST    :4;
        volatile rt_uint8_t             :3;
        volatile rt_uint8_t FIFO_FLUSH  :1;
    }B;
}Spl06ResetRegUnion;

/******************************************************************************
 * pubilc functions
 *****************************************************************************/

void spl06_init(rt_sensor_t sensor);
void spl06_reset(rt_sensor_t sensor);

rt_err_t spl06_get_id(struct rt_sensor_device *sensor, void *args);
rt_size_t _spl06_baro_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_BAR_SPL06_SPL06_H_ */