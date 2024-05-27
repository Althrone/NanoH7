/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\imu\bmi088\sensor_intf_bmi088.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "sensor_intf_bmi088.h"
#include <rtdbg.h>
#include "drv_spi.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

static rt_size_t bmi088_acce_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len);

static rt_err_t bmi088_acce_control(struct rt_sensor_device *sensor, int cmd, void *args);//args是32位(指针都是4个字节)

static rt_size_t bmi088_gyro_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len);

static rt_err_t bmi088_gyro_control(struct rt_sensor_device *sensor, int cmd, void *args);//args是32位(指针都是4个字节)

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

static struct rt_sensor_ops acce_sensor_ops =
{
    bmi088_acce_fetch_data,
    bmi088_acce_control
};

int rt_hw_bmi088_acce_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;
    rt_sensor_t sensor_temp = RT_NULL;
    struct rt_sensor_module *module = RT_NULL;

    module = rt_calloc(1, sizeof(struct rt_sensor_module));
    if (module == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto __exit;
    }

    /* 注册mag模块 */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)//空间开辟失败
        {
            return -RT_ENOMEM;
            goto __exit;
        }

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_acce->info.model      = name;//传入的就是传感器名字
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_acce->info.range_max  = 24000;
        sensor_acce->info.range_min  = 3000;
        sensor_acce->info.period_min = 1;//实际上最小是0.625ms
        // sensor_acce->info.fifo_max   = 1024;//bst-mis-an005.pdf

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &acce_sensor_ops;
        sensor_acce->module = module;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            result = -RT_ERROR;
            goto __exit;
        }
    }

    /* 注册temp模块 */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)//空间开辟失败
        {
            return -RT_ENOMEM;
            goto __exit;
        }

        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = name;//传入的就是传感器名字
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_temp->info.range_max  = 150;//85
        sensor_temp->info.range_min  = -104;//-40
        sensor_temp->info.period_min = 1280;//实际上最小是0.625ms
        // sensor_temp->info.fifo_max   = 1024;//bst-mis-an005.pdf

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &temp_sensor_ops;
        sensor_temp->module = module;

        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            result = -RT_ERROR;
            goto __exit;
        }
    }

    module->sen[0]=sensor_acce;
    module->sen[1]=sensor_temp;
    module->sen_num=2;

    char spi_bus_name[5]={0};
    rt_strncpy(spi_bus_name,sensor_acce->config.intf.dev_name,4);

    //查找通讯总线
    if(rt_device_find(spi_bus_name)==RT_NULL)
    {
        LOG_E("Can't find %s bus device",spi_bus_name);
        result = -RT_ERROR;
        goto __exit;
    }//建议放在_init中

    //挂载到SPI总线
    result = rt_hw_spi_device_attach(spi_bus_name,
                                     cfg->intf.dev_name,
                                     GPIOD, GPIO_PIN_6);
    if (result != RT_EOK)
    {
        LOG_E("attach fail");
        result = -RT_ERROR;
        goto __exit;
    }
    
    //配置SPI设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(cfg->intf.dev_name);
    if (spi_dev == RT_NULL)
    {
        LOG_E("spi device not find");
        result = -RT_ERROR;
        goto __exit;
    }

    struct rt_spi_configuration spi_cfg;

    spi_cfg.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.data_width=8;
    spi_cfg.max_hz=1*1000*1000;

    spi_dev->bus->owner=spi_dev;//将bus->owner变量赋值为自身

    result = rt_spi_configure(spi_dev, &spi_cfg);
    if (result != RT_EOK)
    {
        LOG_E("spi config fail");
        result = -RT_ERROR;
        goto __exit;
    }

    bmi088_acce_init(sensor_acce);

    //读取设备ID，这里开始用到自己写的东西了
    rt_uint8_t id = 0x00;
    if(rt_device_control(sensor_acce, RT_SENSOR_CTRL_GET_ID, &id)==-RT_ENOSYS)
    {
        LOG_E("get device id faile");
        result = -RT_ERROR;
        goto __exit;
    }
    LOG_I("device id: 0x%x!", id);

    return RT_EOK;
__exit:
    if(sensor_acce) 
    {
        if(sensor_acce->data_buf)
            rt_free(sensor_acce->data_buf);

        rt_free(sensor_acce);
    }
    if(sensor_temp) 
    {
        if(sensor_temp->data_buf)
            rt_free(sensor_temp->data_buf);

        rt_free(sensor_temp);
    }
    if (module)
        rt_free(module);

    return result;
}

/**
 * @brief   bmi088的acce作为复合模块，通过传感器模块互锁
 * @note    acce和gyro不作为复合模块，通过spi总线互锁
 **/
int rt_hw_bmi088_acce_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name="spi10";
    cfg.intf.type=RT_SENSOR_INTF_SPI;//这个好像没用到
    cfg.intf.user_data=(void*)GET_PIN(D,6);
    // cfg.irq_pin.pin = irq_pin;
    // cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    //由于是复合传感器，不用旧方法
    //查找是否存在 acce_0,acce_1...
    // char new_dev_name[]="0";
    // char exist_dev_name[]="acce_0";//rtt中加速度计缩写是acce
    // while(rt_device_find(exist_dev_name))//如果是NULL的话就会跳出while
    // {
    //     ++new_dev_name[0];
    //     ++exist_dev_name[4];
    //     if(new_dev_name[0]>'9')
    //     {
    //         return -RT_ERROR;//超过10个acce了
    //     }
    // }
    rt_hw_bmi088_acce_init("bmi088",&cfg);

    return RT_EOK;
}
// INIT_DEVICE_EXPORT(rt_hw_bmi088_acce_port);

static struct rt_sensor_ops gyro_sensor_ops =
{
    bmi088_gyro_fetch_data,
    bmi088_gyro_control
};

int rt_hw_bmi088_gyro_init(const char *name, struct rt_sensor_config *gyro_cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_gyro = RT_NULL;

    sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_gyro == RT_NULL)//空间开辟失败
        return -RT_ERROR;

    sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
    sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
    sensor_gyro->info.model      = "bmi088";
    sensor_gyro->info.unit       = RT_SENSOR_UNIT_MDPS;
    sensor_gyro->info.intf_type  = RT_SENSOR_INTF_SPI;
    sensor_gyro->info.range_max  = 2000000;
    sensor_gyro->info.range_min  = 125000;
    sensor_gyro->info.period_min = 1;//实际上最小是0.5ms
    // sensor_gyro->info.fifo_max   = 3200;//bst-mis-an005.pdf

    rt_memcpy(&sensor_gyro->config, gyro_cfg, sizeof(struct rt_sensor_config));
    sensor_gyro->ops = &gyro_sensor_ops;

    result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor_gyro);
        return -RT_ERROR;
    }
    LOG_I("sensor register success");//注册成功

    char spi_bus_name[5]={0};
    rt_strncpy(spi_bus_name,sensor_gyro->config.intf.dev_name,4);

    //查找通讯总线
    if(rt_device_find(spi_bus_name)==RT_NULL)
    {
        LOG_E("Can't find %s bus device",spi_bus_name);
        rt_free(sensor_gyro);
        return -RT_ERROR;
    }//建议放在_init中

    //挂载到SPI总线
    result = rt_hw_spi_device_attach(spi_bus_name,
                                     sensor_gyro->config.intf.dev_name,
                                     GPIOB, GPIO_PIN_6);
    if (result != RT_EOK)
    {
        LOG_E("attach fail");
        rt_free(sensor_gyro);
        return -RT_ERROR;
    }
    
    //配置SPI设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor_gyro->config.intf.dev_name);
    if (spi_dev == RT_NULL)
    {
        LOG_E("spi device not find");
        rt_free(sensor_gyro);
        return -RT_ERROR;
    }

    struct rt_spi_configuration spi_cfg;

    spi_cfg.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.data_width=8;
    spi_cfg.max_hz=1*1000*1000;

    spi_dev->bus->owner=spi_dev;//将bus->owner变量赋值为自身

    result = rt_spi_configure(spi_dev, &spi_cfg);
    if (result != RT_EOK)
    {
        LOG_E("spi config fail");
        rt_free(sensor_gyro);
        return -RT_ERROR;
    }

    bmi088_gyro_init(sensor_gyro);

    //读取设备ID，这里开始用到自己写的东西了
    rt_uint8_t id = 0x00;
    if(rt_device_control(sensor_gyro, RT_SENSOR_CTRL_GET_ID, &id)==-RT_ENOSYS)
    {
        LOG_E("get device id faile");
        return -RT_ERROR;
    }
    LOG_I("device id: 0x%x!", id);

    return RT_EOK;
}

/**
 * @brief   导出到自动初始化的函数
 * @note    自动初始化成默认的传感器状态
 **/
int rt_hw_bmi088_gyro_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name="spi11";
    cfg.intf.type=RT_SENSOR_INTF_SPI;//这个好像没用到
    cfg.intf.user_data=(void*)GET_PIN(B,6);
    // cfg.irq_pin.pin = irq_pin;
    // cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    //查找是否存在 gyro_0,gyro_1...
    char new_dev_name[]="0";
    char exist_dev_name[]="gyro_0";//rtt中加速度计缩写是acce
    while(rt_device_find(exist_dev_name))//如果是NULL的话就会跳出while
    {
        ++new_dev_name[0];
        ++exist_dev_name[4];
        if(new_dev_name[0]>'9')
        {
            return -RT_ERROR;//超过10个gyro了
        }
    }
    rt_hw_bmi088_gyro_init(new_dev_name,&cfg);

    return RT_EOK;
}
// INIT_DEVICE_EXPORT(rt_hw_bmi088_gyro_port);

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_size_t bmi088_acce_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if(sensor->info.type==RT_SENSOR_CLASS_ACCE)
    {
        if(sensor->parent.open_flag&RT_SENSOR_MODE_POLLING)
        {
            return _bmi088_acce_polling_get_data(sensor, buf, len);
        }
        // else if (sensor->parent.open_flag & RT_SENSOR_MODE_INT)
        // {
        //     // return _xxx_acc_int_get_data(sensor, buf, len);
        // }
        // else if (sensor->parent.open_flag & RT_SENSOR_MODE_FIFO)
        // {
        //     // return _xxx_acc_fifo_get_data(sensor, buf, len);
        // }
        else
            return 0;
    }
    else if(sensor->info.type==RT_SENSOR_CLASS_TEMP)
    {
        if(sensor->parent.open_flag&RT_SENSOR_MODE_POLLING)
        {
            return _bmi088_temp_polling_get_data(sensor, buf, len);
        }
        // else if (sensor->parent.open_flag & RT_SENSOR_MODE_INT)
        // {
        //     // return _xxx_acc_int_get_data(sensor, buf, len);
        // }
        // else if (sensor->parent.open_flag & RT_SENSOR_MODE_FIFO)
        // {
        //     // return _xxx_acc_fifo_get_data(sensor, buf, len);
        // }
        else
            return 0;
    }
    else
        return 0;
}

static rt_err_t bmi088_acce_control(struct rt_sensor_device *sensor, int cmd, void *args)//args是32位(指针都是4个字节)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
        //考虑添加一个复位命令
    case RT_SENSOR_CTRL_GET_ID:
        result=_bmi088_acce_get_id(sensor,args);
        break;
    // case RT_SENSOR_CTRL_GET_INFO://由框架实现，在驱动中不需要实现
    //     break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result=bmi088_acce_set_range(sensor,(rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result=bmi088_acce_set_odr(sensor,(rt_uint16_t)args);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        sensor->parent.open_flag=(rt_uint16_t)args;
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        result=-RT_ERROR;
        break;
    }
    return result;
}

static rt_size_t bmi088_gyro_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if(sensor->parent.open_flag&RT_DEVICE_FLAG_RDONLY)
    {
        // return _hmc5883l_mag_polling_get_data(sensor, buf, len);
    }
    else if (sensor->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        // return _xxx_acc_int_get_data(sensor, buf, len);
    }
    else if (sensor->parent.open_flag & RT_DEVICE_FLAG_FIFO_RX)
    {
        // return _xxx_acc_fifo_get_data(sensor, buf, len);
    }
    else
    {
        return 0;
    }
}

static rt_err_t bmi088_gyro_control(struct rt_sensor_device *sensor, int cmd, void *args)//args是32位(指针都是4个字节)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
        //考虑添加一个复位命令
    case RT_SENSOR_CTRL_GET_ID:
        result=bmi088_gyro_get_id(sensor,args);
        break;
    case RT_SENSOR_CTRL_GET_INFO:
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        result=-RT_ERROR;
        break;
    }
    return result;
}