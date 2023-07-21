/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\bn880\sensor_intf_bn880.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "sensor_intf_bn880.h"
#include <rtdbg.h>
#include "drv_usart_v2.h"

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

/**
 * @brief   导出到自动初始化的函数
 * @note    自动初始化成默认的传感器状态
 **/
int rt_hw_bn880_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name="uart2";
    cfg.intf.type=RT_SENSOR_INTF_UART;//这个好像没用到
    // cfg.intf.user_data=;
    // cfg.irq_pin.pin = irq_pin;
    // cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    //查找是否存在 gnss_0,gnss_1...
    char new_dev_name[]="0";
    char exist_dev_name[]="gnss_0";
    while(rt_device_find(exist_dev_name))//如果是NULL的话就会跳出while
    {
        ++new_dev_name[0];
        ++exist_dev_name[4];
        if(new_dev_name[0]>'9')
        {
            return -RT_ERROR;//超过10个gnss了
        }
    }
    // rt_hw_bn880_init(new_dev_name,&cfg);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_bn880_port);

/******************************************************************************
 * private functions definition
 *****************************************************************************/
