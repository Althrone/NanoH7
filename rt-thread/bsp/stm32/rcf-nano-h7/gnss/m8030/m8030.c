/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\m8030\m8030.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "m8030.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/

rt_err_t m8030_init(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;

    rt_device_t serial=rt_device_find(sensor->config.intf.dev_name);

    //先通过多次复位的方式探测波特率
    // UBX-CFG-RST
    rt_uint8_t ubx_cfg_rst_tx_buf[8+4];
    rt_device_write(serial,0,ubx_cfg_rst_tx_buf,sizeof(ubx_cfg_rst_tx_buf));

    // UBX-MON-VER
}