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

rt_err_t bmi088_acce_init(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");

    bmi088_acce_reset();

    //等待1ms
    rt_thread_mdelay(1);

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
    result=rt_spi_transfer(spi_dev,send_buf,2);//相当于电源开关

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

    *(rt_uint32_t *)args = (rt_uint32_t)recv_buf[2];

    return result;
}

rt_err_t bmi088_acce_set_range(struct rt_sensor_device *sensor, void *args)
{
    //芯片默认6g

    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi10");


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

    *(rt_uint32_t *)args = (rt_uint32_t)recv_buf[1];

    return result;
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
