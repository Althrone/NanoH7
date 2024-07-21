// /******************************************************************************
//  * NanoH7 - UAV firmware base on RT-Thread
//  * Copyright (C) 2023 - 2024 Althrone <mail>
//  * 
//  * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\m8030\sensor_intf_m8030.c
//  * 
//  * ref: Specification of <some UM RM or Datasheet>
//  *****************************************************************************/

// /******************************************************************************
//  * includes
//  *****************************************************************************/

// #include "sensor_intf_m8030.h"

// /******************************************************************************
//  * private macros
//  *****************************************************************************/

// /******************************************************************************
//  * pubilc variables
//  *****************************************************************************/

// /******************************************************************************
//  * private types
//  *****************************************************************************/

// /******************************************************************************
//  * private variables
//  *****************************************************************************/

// static struct rt_sensor_ops sensor_ops =
// {
//     // m8030_acce_fetch_data,
//     // m8030_acce_control
// };

// /******************************************************************************
//  * private functions declaration
//  *****************************************************************************/

// /******************************************************************************
//  * pubilc functions definition
//  *****************************************************************************/

// int rt_hw_m8030_init(const char *name, struct rt_sensor_config *cfg)
// {
//     rt_int8_t result;
//     rt_sensor_t sensor_t = RT_NULL;

//     sensor_t = rt_calloc(1, sizeof(struct rt_sensor_device));
//     if (sensor_t == RT_NULL)//空间开辟失败
//     {
//         return -RT_ENOMEM;
//         goto __exit;
//     }

//     sensor_t->info.type       = RT_SENSOR_CLASS_GNSS;
//     sensor_t->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
//     sensor_t->info.model      = name;//传入的就是传感器名字
//     sensor_t->info.unit       = RT_SENSOR_UNIT_DMS;
//     sensor_t->info.intf_type  = RT_SENSOR_INTF_UART;
//     sensor_t->info.range_max  = 180;
//     sensor_t->info.range_min  = 0;
//     sensor_t->info.period_min = 100;//ms
//     // sensor_t->info.fifo_max   = 1024;//bst-mis-an005.pdf

//     rt_memcpy(&sensor_t->config, sensor_cfg, sizeof(struct rt_sensor_config));
//     sensor_t->ops = &sensor_ops;

//     result = rt_hw_sensor_register(sensor_t, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
//     if (result != RT_EOK)
//     {
//         LOG_E("device register err code: %d", result);
//         rt_free(sensor_t);
//         return -RT_ERROR;
//     }
//     LOG_I("sensor register success");//注册成功

//     //查找uart端口是否存在
//     if(rt_device_find(sensor_t->config.intf.dev_name)==RT_NULL)
//     {
//         LOG_E("Can't find %s bus device",sensor_t->config.intf.dev_name);
//         rt_free(sensor_t);
//         return -RT_ERROR;
//     }

//     //找串口设备
//     rt_device_t uart_dev=RT_NULL;
//     uart_dev=rt_device_find(sensor_t->config.intf.dev_name);
//     if (uart_dev == RT_NULL)
//     {
//         LOG_E("uart device not find");
//         rt_free(sensor_t);
//         return -RT_ERROR;
//     }

//     struct serial_configure uart_cfg=RT_SERIAL_CONFIG_DEFAULT;

//     uart_cfg.baud_rate=BAUD_RATE_9600;

//     rt_device_control(uart_dev,RT_DEVICE_CTRL_CONFIG,&uart_cfg);

//     // m8030_init(sensor_t);
    
//     //读取设备id

// __exit:

// }


// /******************************************************************************
//  * private functions definition
//  *****************************************************************************/
