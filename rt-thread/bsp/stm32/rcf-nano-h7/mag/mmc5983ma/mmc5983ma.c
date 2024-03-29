#include "mmc5983ma.h"

void mmc5893ma_init(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    mmc5893ma_reset();

    rt_uint8_t send_buf[]={CTLR_0_REG,0x24};//开启测量完成中断，开启自动sr
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    send_buf[0]=CTLR_2_REG;
    send_buf[1]=0x88;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

void mmc5893ma_reset(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi12");

    rt_uint8_t send_buf[]={CTLR_1_REG,0x80};
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    rt_thread_mdelay(10);
}

/**
 * @brief   获取传感器ID
 * @param   sensor:
 * @param   args: id就放在这里
 **/
rt_err_t mmc5893ma_get_id(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi12");

    // rt_uint8_t send_buf[]={0x80|0x0D};
    rt_uint8_t send_buf[]={0x80|ID_REG};
    rt_uint8_t recv_buf[]={0xFF};

    // //在这里调用spi hal程序
    // HAL_GPIO_WritePin(cs->GPIOx, cs->GPIO_Pin, GPIO_PIN_RESET);

    // HAL_SPI_TransmitReceive()

    // HAL_GPIO_WritePin(cs->GPIOx, cs->GPIO_Pin, GPIO_PIN_SET);

    // result=rt_spi_transfer(spi_dev,send_buf,recv_buf,1);
    result=rt_spi_send_then_recv(spi_dev,send_buf,sizeof(send_buf),recv_buf,sizeof(send_buf));
    // result=rt_spi_send(spi_dev,send_buf,sizeof(send_buf));
    // result=rt_spi_recv(spi_dev,recv_buf,sizeof(recv_buf));
    *(rt_uint8_t *)args = recv_buf[0];
    return result;
}