/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\bar\spl06\sensor_intf_spl06.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "spl06.h"
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

static rt_size_t spl06_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len);
static rt_err_t spl06_control(struct rt_sensor_device *sensor, int cmd, void *args);//args是32位(指针都是4个字节)

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

static struct rt_sensor_ops sensor_ops =
{
    spl06_fetch_data, 
    spl06_control
};

int rt_hw_spl06_init(const char *name, struct rt_sensor_config *baro_cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_baro = RT_NULL;

    sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_baro == RT_NULL)//空间开辟失败
        return -RT_ERROR;

    sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
    sensor_baro->info.vendor     = RT_SENSOR_VENDOR_GOERTEK;
    sensor_baro->info.model      = "spl06-001";
    sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
    sensor_baro->info.intf_type  = RT_SENSOR_INTF_SPI;
    sensor_baro->info.range_max  = 110000;
    sensor_baro->info.range_min  = 30000;
    sensor_baro->info.period_min = 8;//实际上是7.8125ms
    sensor_baro->info.fifo_max   = 32;

    rt_memcpy(&sensor_baro->config, baro_cfg, sizeof(struct rt_sensor_config));
    sensor_baro->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor_baro);
        return -RT_ERROR;
    }
    LOG_I("sensor register success");//注册成功

    //查找通讯总线
    if(rt_device_find("spi1")==RT_NULL)
    {
        LOG_E("Can't find %s bus device",baro_cfg->intf.dev_name);
        rt_free(sensor_baro);
        return -RT_ERROR;
    }//建议放在_init中

    //挂载到SPI总线
    result = rt_hw_spi_device_attach("spi1","spi13",
                                     GPIOD, GPIO_PIN_7);
    if (result != RT_EOK)
    {
        LOG_E("attach fail");
        rt_free(sensor_baro);
        return -RT_ERROR;
    }

    //配置SPI设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");
    if (spi_dev == RT_NULL)
    {
        LOG_E("spi device not find");
        rt_free(sensor_baro);
        return -RT_ERROR;
    }

    struct rt_spi_configuration spi_cfg;
    spi_cfg.mode=RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.data_width=8;
    spi_cfg.max_hz=9*1000*1000;
    // spi_dev->user_data=

    spi_dev->bus->owner=spi_dev;//将bus->owner变量赋值为自身

    result = rt_spi_configure(spi_dev, &spi_cfg);
    if (result != RT_EOK)
    {
        LOG_E("spi config fail");
        rt_free(sensor_baro);
        return -RT_ERROR;
    }
    

    //将初始化命令放到这里
    //获取设备信息
    // struct rt_sensor_info info;
    // if(rt_device_control(sensor_mag,RT_SENSOR_CTRL_GET_INFO,&info))//sensor_mag的父类是rt_device_t 不知道行不行
    // {
    //     LOG_E("get device info faile");
    //     return -RT_ERROR;
    // }
    // LOG_I("vendor :%d", info.vendor);
    // LOG_I("model  :%s", info.model);
    // LOG_I("unit   :%d", info.unit);
    // LOG_I("intf_type :%d", info.intf_type);
    // LOG_I("period_min:%d", info.period_min);

    spl06_init(sensor_baro);

    //读取设备ID，这里开始用到自己写的东西了
    rt_uint8_t id = 0x00;
    if(rt_device_control(sensor_baro, RT_SENSOR_CTRL_GET_ID, &id)==-RT_ENOSYS)
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
int rt_hw_spl06_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name="spi13";
    cfg.intf.type=RT_SENSOR_INTF_SPI;//这个好像没用到
    cfg.intf.user_data=(void*)GET_PIN(D,7);
    // cfg.irq_pin.pin = irq_pin;
    // cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    //查找是否存在 baro_0,baro_1...
    char new_dev_name[]="0";
    char exist_dev_name[]="baro_0";//rtt中气压计缩写是baro
    while(rt_device_find(exist_dev_name))//如果是NULL的话就会跳出while
    {
        ++new_dev_name[0];
        ++exist_dev_name[4];
        if(new_dev_name[0]>'9')
        {
            return -RT_ERROR;//超过10个baro了
        }
    }
    rt_hw_spl06_init(new_dev_name,&cfg);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_spl06_port);

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_size_t spl06_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
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

static rt_err_t spl06_control(struct rt_sensor_device *sensor, int cmd, void *args)//args是32位(指针都是4个字节)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
        //考虑添加一个复位命令
    case RT_SENSOR_CTRL_GET_ID:
        result=spl06_get_id(sensor,args);
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
