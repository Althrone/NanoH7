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

typedef struct
{
    rt_int16_t c0;
    rt_int16_t c1;
    rt_int32_t c00;
    rt_int32_t c10;
    rt_int16_t c01;
    rt_int16_t c11;
    rt_int16_t c20;
    rt_int16_t c21;
    rt_int16_t c30;
}Spl06CoefStruct;

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

static Spl06CoefStruct s_spl06_coef={0};

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void spl06_init(rt_sensor_t sensor)
{
    spl06_reset(sensor);

    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    Spl06CfgRegUnion cfg={
        .B.SPI_MODE=1,//3wire
        .B.INT_PRS=1,
        .B.INT_HL=1,
    };
    rt_uint8_t send_buf[2]={SPL06_CFG_REG_ADDR,cfg.r};//压力数据就绪中断，三线spi模式
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //Enable P shift
    //Sports (addr 0x09)Start background measurements (addr 0x08)

    send_buf[0]=SPL06_TMP_CFG_REG_ADDR;//Sports 0xA0
    send_buf[1]=0b11110000;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

    send_buf[0]=SPL06_PRS_CFG_REG_ADDR;//Sports 0x26
    send_buf[1]=0b01110001;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

    send_buf[0]=SPL06_MEAS_CFG_REG_ADDR;
    send_buf[1]=0b00000111;//连续采样温度和压力
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

void spl06_reset(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    Spl06ResetRegUnion rst_cmd={
        .B.SOFT_RST=9,
    };
    rt_uint8_t send_buf[2]={SPL06_RESET_REG_ADDR,rst_cmd.r};//0b1001
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    rt_thread_mdelay(52);//12ms Time to sensor ready + 40ms Time to coefficients are available 
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

    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    *(rt_uint32_t *)args = (rt_uint32_t)recv_buf[1];

    return result;
}

rt_err_t spl06_polling_get_coef(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");

    rt_uint8_t send_buf[]={0x80|SPL06_COEF_C0_REG_ADDR};
    rt_uint8_t recv_buf[18]={0};

    rt_spi_send_then_recv(spi_dev,send_buf,1,recv_buf,18);

    s_spl06_coef.c0=((recv_buf[0]&0x80)>0)?0xF000|(((rt_uint16_t)recv_buf[0])<<4)|(recv_buf[1]>>4):
                                                  (((rt_uint16_t)recv_buf[0])<<4)|(recv_buf[1]>>4);

    s_spl06_coef.c1=(((recv_buf[1]<<4)&0x80)>0)?0xF000|(((rt_uint16_t)recv_buf[1])<<8)|recv_buf[2]:
                                                       (((rt_uint16_t)recv_buf[1])<<8)|recv_buf[2];

    // s_spl06_coef.c00=((recv_buf[3]&0x80)>0)?
}

// rt_err_t spl06_polling_get_mag(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
rt_err_t spl06_polling_get_baro(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi13");
    //读取stat？
    rt_uint8_t send_buf[]={0x80|SPL06_INT_STS_REG_ADDR,0x00};
    rt_uint8_t recv_buf[2]={0};
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
