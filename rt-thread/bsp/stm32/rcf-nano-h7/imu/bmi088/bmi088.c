/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\imu\bmi088\bmi088.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "bmi088.h"
#include <rtdbg.h>

/******************************************************************************
 * private macros
 *****************************************************************************/

#define DBG_TAG  "bmi088"

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

rt_err_t bmi088_acce_init(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    bmi088_acce_reset();

    rt_uint8_t send_buf[3]={0x80|ACC_CHIP_ID,0x00,0x00};
    rt_uint8_t recv_buf[3]={0};

    //切换回spi模式
    send_buf[0]=0x80|ACC_CHIP_ID;
    send_buf[1]=0x00;
    while (recv_buf[2]!=0x1E)
    {
        result=rt_spi_transfer(spi_dev,send_buf,recv_buf,3);
    }

    //使能acc
    send_buf[0]=ACC_PWR_CTRL;
    send_buf[1]=0x04;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//相当于电源开关

    //等待50ms
    rt_thread_mdelay(50);

    //进入运行模式
    send_buf[0]=ACC_PWR_CONF;
    send_buf[1]=0x00;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//Suspend mode是挂起模式，相当于省电一些

    //配置INT12引脚
    send_buf[0]=INT1_IO_CTRL;
    send_buf[1]=0x0A;//这次用了INT1 输出，推挽，高电平有效
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //配置引脚映射功能
    send_buf[0]=INT_MAP_DATA;
    send_buf[1]=0x04;//数据就绪中断映射到int1
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

rt_err_t bmi088_acce_reset(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    //通过读取id号切换至spi模式
    rt_uint8_t send_buf[3]={0x80|ACC_CHIP_ID,0x00,0x00};
    rt_uint8_t recv_buf[3]={0};
    while (recv_buf[2]!=0x1E)
    {
        result=rt_spi_transfer(spi_dev,send_buf,recv_buf,3);
    }

    //发送复位命令
    send_buf[0]=ACC_SOFTRESET;
    send_buf[1]=0xB6;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //等待1ms
    rt_thread_mdelay(1);
}

/**
 * @brief   获取传感器ID
 * @param   sensor:
 * @param   args: id就放在这里
 **/
rt_err_t bmi088_acce_get_id(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    rt_uint8_t send_buf[3]={0x80|ACC_CHIP_ID,0x00,0x00};
    rt_uint8_t recv_buf[3]={0};

    while (recv_buf[2]!=0x1E)
    {
        result=rt_spi_transfer(spi_dev,send_buf,recv_buf,3);

        // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);//acc
        // HAL_SPI_TransmitReceive(&hspi1,&BMI_Data[0],&BMI_Data[2],2,1000);
        // // rt_thread_mdelay(5);
        // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);


    }
    
    // result=rt_spi_transfer(spi_dev,send_buf,recv_buf,1);

    *(rt_uint8_t *)args = (rt_uint32_t)recv_buf[2];

    return result;
}

rt_err_t bmi088_acce_set_range(struct rt_sensor_device *sensor, void *args)
{
    //芯片默认6g

    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    rt_uint8_t send_buf[2]={ACC_RANGE,0x00};
    //判断args是多少
    switch (*(rt_int32_t*)args)
    {
    case 3000:
        send_buf[1]=0x00;
        break;
    case 6000:
        send_buf[1]=0x01;
        break;
    case 12000:
        send_buf[1]=0x02;
        break;
    case 24000:
        send_buf[1]=0x03;
        break;
    default:
        result=-RT_ERROR;
        LOG_E("输入值必须是3000，6000，12000或者24000");
        break;
    }

    if(result==RT_EOK)
    {
        result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    }

    return result;
}

rt_err_t bmi088_acce_set_odr(struct rt_sensor_device *sensor, void *args)
{
    //芯片默认100Hz
    rt_err_t result=RT_EOK;

    // struct rt_spi_device *spi_dev=RT_NULL;
    // spi_dev=(struct rt_spi_device *)&(sensor->parent);

    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    rt_uint8_t send_buf[]={0x80|ACC_CONF,0x00,0x00};
    rt_uint8_t recv_buf[3]={0};

    //读取ACC_CONF的原始值
    result=rt_spi_transfer(spi_dev,send_buf,recv_buf,3);

    switch ((rt_uint16_t)args)
    {
    // case 12.5//这里的输入参数不支持
    case 25:
        send_buf[1]=recv_buf[2]&0xF0|0x06;
        break;
    case 50:
        send_buf[1]=recv_buf[2]&0xF0|0x07;
        break;
    case 100:
        send_buf[1]=recv_buf[2]&0xF0|0x08;
        break;
    case 200:
        send_buf[1]=recv_buf[2]&0xF0|0x09;
        break;
    case 400:
        send_buf[1]=recv_buf[2]&0xF0|0x0A;
        break;
    case 800:
        send_buf[1]=recv_buf[2]&0xF0|0x0B;
        break;
    case 1600:
        send_buf[1]=recv_buf[2]&0xF0|0x0C;
        break;
    default:
        result=-RT_ERROR;
        LOG_E("输入值必须是25、50、100、200、400、800或者1600，12.5此命令不支持");
        break;
    }

    if(result!=-RT_ERROR)
    {
        send_buf[0]=ACC_CONF;
        result=rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    }
    return result;
}

rt_err_t bmi088_gyro_init(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi11");

    bmi088_gyro_reset();

    //新数据产生中断
    rt_uint8_t send_buf[]={GYRO_INT_CTRL,0x80};
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //int3推挽 高电平有效
    send_buf[0]=INT3_INT4_IO_CONF;
    send_buf[1]=0b1101;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //路由到int3
    send_buf[0]=INT3_INT4_IO_MAP;
    send_buf[1]=1;
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

rt_err_t bmi088_gyro_reset(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi11");

    rt_uint8_t send_buf[]={GYRO_SOFTRESET,0xB6};
    // rt_uint8_t recv_buf[3]={0};
    result=rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    rt_thread_mdelay(30);
}

/**
 * @brief   获取传感器ID
 * @param   sensor:
 * @param   args: id就放在这里
 **/
rt_err_t bmi088_gyro_get_id(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi11");

    rt_uint8_t send_buf[2]={0x80|GYRO_CHIP_ID,0x00};
    rt_uint8_t recv_buf[2]={0};

    result=rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    *(rt_uint8_t *)args = (rt_uint32_t)recv_buf[1];

    return result;
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
