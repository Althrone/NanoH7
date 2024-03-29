/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\bar\spl06\spl06.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "spl06.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void spl06_init(void)
{
    spl06_reset();

    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");

    rt_uint8_t send_buf[2]={SPL06_CFG_REG_ADDR,0b10100001};//压力数据就绪终端，三线spi模式
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    send_buf[0]=SPL06_TMP_CFG_REG_ADDR;
    send_buf[1]=0b11110000;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

    send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
    send_buf[1]=0b01110001;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

    send_buf[0]=SPL06_MEAS_CFG_REG_ADDR;
    send_buf[1]=0b00000111;//连续采样温度和压力
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

void spl06_reset(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");

    rt_uint8_t send_buf[2]={SPL06_RESET_REG_ADDR,0x09};//0b1001
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    rt_thread_mdelay(52);//12+40
}

/**
 * @brief   获取传感器ID
 * @param   sensor:
 * @param   args: id就放在这里
 **/
rt_err_t spl06_get_id(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");

    //先在这里配置成3线模式，后续移到其他函数里面

    rt_uint8_t send_buf[2]={0x80|SPL06_ID_REG_ADDR,0x00};
    rt_uint8_t recv_buf[2]={0};

    result=rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    *(rt_uint32_t *)args = (rt_uint32_t)recv_buf[1];

    return result;
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
