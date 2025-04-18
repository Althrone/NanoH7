/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\mag\mmc5983ma\mmc5983ma.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "mmc5983ma.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

#define DBG_TAG  "5983"
#include <drv_log.h>

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

//由于5983无法读出寄存器值，因此只能维护一个寄存器表保存写入的cr0-cr3
static Mmc5893maCtrl0RegUnion gs_mmc5983ma_cr0={0};
static Mmc5893maCtrl1RegUnion gs_mmc5983ma_cr1={0};
static Mmc5893maCtrl2RegUnion gs_mmc5983ma_cr2={0};
static Mmc5893maCtrl3RegUnion gs_mmc5983ma_cr3={0};

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void mmc5893ma_init(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    mmc5893ma_reset(sensor);//复位命令感觉无效

    rt_uint8_t send_buf[2]={0};
    rt_uint8_t recv_buf[2];

    //读sr
    send_buf[0]=0x80|MMC5983MA_SR_ADDR;
    send_buf[1]=0;
    do
    {
        rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    }while(((Mmc5893maStatRegUnion*)(&recv_buf[1]))->B.OTP_Rd_Done!=1);

    //cr1 设置BW
    gs_mmc5983ma_cr1.B.BW=3;//bw 0.5ms 800Hz

    send_buf[0]=MMC5983MA_CR1_ADDR;
    send_buf[1]=gs_mmc5983ma_cr1.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //CR0
    gs_mmc5983ma_cr0.B.Auto_SR_en=1;//开启自动sr
    gs_mmc5983ma_cr0.B.INT_meas_done_en=1;//开启测量完成中断
    gs_mmc5983ma_cr0.B.TM_M=1,//开启连续采样需要设置这一位x轴才有反应，不知道是不是假芯片
    gs_mmc5983ma_cr0.B.TM_T=0,//开启连续采样需要设置这一位x轴才有反应，不知道是不是假芯片

    send_buf[0]=MMC5983MA_CR0_ADDR;
    send_buf[1]=gs_mmc5983ma_cr0.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    // cr2 CM_Freq
    gs_mmc5983ma_cr2.B.Cm_freq=7;
    gs_mmc5983ma_cr2.B.Cmm_en=1;
    gs_mmc5983ma_cr2.B.En_prd_set=1;
    gs_mmc5983ma_cr2.B.Prd_set=1;

    send_buf[0]=MMC5983MA_CR2_ADDR;
    send_buf[1]=gs_mmc5983ma_cr2.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
}

void mmc5893ma_reset(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    Mmc5893maCtrl1RegUnion cr1={
        .B.SW_RST=1,
    };
    rt_uint8_t send_buf[]={MMC5983MA_CR1_ADDR,0xff};
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    rt_thread_mdelay(10);//MEMSIC MMC5983MA Rev A Page 15 of 20 Formal release date: 4/3/2019
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
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    rt_uint8_t send_buf[]={0x80|MMC5983MA_PID1_ADDR,0x00};
    rt_uint8_t recv_buf[2]={0};

    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    if(recv_buf[1]!=0x30)
        result=RT_ERROR;
    else
        *(rt_uint8_t *)args = recv_buf[1];

    return result;
}

rt_err_t mmc5893ma_set_odr(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    rt_uint8_t send_buf[2]={0};
    // rt_uint8_t recv_buf[2]={0};

    //判断是否为连续测量模式
    if(gs_mmc5983ma_cr2.B.Cmm_en)//连续模式
    {
        switch (*(rt_uint16_t*)args)
        {
        case 1://hz
            if((gs_mmc5983ma_cr2.B.Cm_freq==1)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=1;
            gs_mmc5983ma_cr1.B.BW=0;
            break;
        case 10:
            if((gs_mmc5983ma_cr2.B.Cm_freq==2)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=2;
            gs_mmc5983ma_cr1.B.BW=0;
            break;
        case 20:
            if((gs_mmc5983ma_cr2.B.Cm_freq==3)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=3;
            gs_mmc5983ma_cr1.B.BW=0;
            break;
        case 50:
            if((gs_mmc5983ma_cr2.B.Cm_freq==4)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=4;
            gs_mmc5983ma_cr1.B.BW=0;
            break;
        case 100:
            if((gs_mmc5983ma_cr2.B.Cm_freq==5)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=5;
            gs_mmc5983ma_cr1.B.BW=0;
            break;
        case 200:
            if((gs_mmc5983ma_cr2.B.Cm_freq==6)&&(gs_mmc5983ma_cr1.B.BW==1))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=6;
            gs_mmc5983ma_cr1.B.BW=1;
            break;
        case 1000:
            if((gs_mmc5983ma_cr2.B.Cm_freq==7)&&(gs_mmc5983ma_cr1.B.BW==3))
                break;
            gs_mmc5983ma_cr2.B.Cm_freq=7;
            gs_mmc5983ma_cr1.B.BW=3;
            break;
        default:
            result=-RT_ERROR;
            LOG_E("连续模式必须是1、10、20、50、100、200或者1000");
            break;
        }
    }
    else//单次模式输入的参数是采样带宽
    {
        switch (*(rt_uint16_t*)args)
        {
        case 100:
            if((gs_mmc5983ma_cr2.B.Cm_freq==0)&&(gs_mmc5983ma_cr1.B.BW==0))
                break;
            gs_mmc5983ma_cr1.B.BW=0;
            gs_mmc5983ma_cr2.B.Cm_freq=0;
            break;
        case 200:
            if((gs_mmc5983ma_cr2.B.Cm_freq==0)&&(gs_mmc5983ma_cr1.B.BW==1))
                break;
            gs_mmc5983ma_cr1.B.BW=1;
            gs_mmc5983ma_cr2.B.Cm_freq=0;
            break;
        case 400:
            if((gs_mmc5983ma_cr2.B.Cm_freq==0)&&(gs_mmc5983ma_cr1.B.BW==2))
                break;
            gs_mmc5983ma_cr1.B.BW=2;
            gs_mmc5983ma_cr2.B.Cm_freq=0;
            break;
        case 800:
            if((gs_mmc5983ma_cr2.B.Cm_freq==0)&&(gs_mmc5983ma_cr1.B.BW==3))
                break;
            gs_mmc5983ma_cr1.B.BW=3;
            gs_mmc5983ma_cr2.B.Cm_freq=0;
            break;
        default:
            result=-RT_ERROR;
            LOG_E("单次模式必须是100、200、400或者800");
            break;
        }
    }

    if(result==RT_EOK)
    {
        send_buf[0]=MMC5983MA_CR1_ADDR;
        send_buf[1]=gs_mmc5983ma_cr1.r;
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,sizeof(send_buf));

        send_buf[0]=MMC5983MA_CR2_ADDR;
        send_buf[1]=gs_mmc5983ma_cr2.r;
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,sizeof(send_buf));
    }

    return result;
}

rt_size_t _mmc5893ma_mag_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
{
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
        return 0;//传感器数据返回长度只有0和1，0表示失败

    rt_uint8_t send_buf[8]={0};
    rt_uint8_t recv_buf[8]={0};

    //写入sr
    send_buf[0]=MMC5983MA_SR_ADDR;
    send_buf[1]=1;
    if(rt_spi_transfer(spi_dev,send_buf,recv_buf,2)!=2)
        return 0;//传感器数据返回长度只有0和1，0表示失败

    send_buf[0]=0x80|MMC5983MA_X_OUT_0_ADDR;
    if(rt_spi_transfer(spi_dev,send_buf,recv_buf,8)!=8)
        return 0;//传感器数据返回长度只有0和1，0表示失败

    //获取到的值不是补码，而是一个无符号数
    rt_int32_t x=(((rt_uint32_t)recv_buf[1])<<10)|(((rt_uint32_t)recv_buf[2])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[7]))->B.Xout;
    rt_int32_t y=(((rt_uint32_t)recv_buf[3])<<10)|(((rt_uint32_t)recv_buf[4])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[7]))->B.Yout;
    rt_int32_t z=(((rt_uint32_t)recv_buf[5])<<10)|(((rt_uint32_t)recv_buf[6])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[7]))->B.Zout;

    x-=131072;
    y-=131072;
    z-=131072;
    
    sensor_data->type = RT_SENSOR_CLASS_MAG;
    sensor_data->data.mag.x = x*0.0625;//系数是0.0625mG
    sensor_data->data.mag.y = y*0.0625;
    sensor_data->data.mag.z = z*0.0625;
    sensor_data->timestamp = rt_sensor_get_ts();

    // gs_mmc5983ma_cr0.B.TM_M=1,//开启连续采样需要设置这一位x轴才有反应，不知道是不是假芯片
    // gs_mmc5983ma_cr0.B.TM_T=0,//开启连续采样需要设置这一位x轴才有反应，不知道是不是假芯片
    // send_buf[0]=MMC5983MA_CR0_ADDR;
    // send_buf[1]=gs_mmc5983ma_cr0.r;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    return 1;
}

rt_size_t _mmc5893ma_temp_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
{
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
        return 0;//传感器数据返回长度只有0和1，0表示失败
    
    rt_uint8_t send_buf[2]={0};
    rt_uint8_t recv_buf[2]={0};

    //关闭连续采样
    // send_buf[0]=MMC5983MA_CR2_ADDR;
    // send_buf[1]=0;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    // rt_thread_mdelay(20);

    //直接复位拉倒
    send_buf[0]=MMC5983MA_CR1_ADDR;
    send_buf[1]=0xff;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    rt_thread_mdelay(10);//MEMSIC MMC5983MA Rev A Page 15 of 20 Formal release date: 4/3/2019

    //读sr
    send_buf[0]=0x80|MMC5983MA_SR_ADDR;
    send_buf[1]=0;
    do
    {
        rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    }while(((Mmc5893maStatRegUnion*)(&recv_buf[1]))->B.OTP_Rd_Done!=1);

    //触发温度采样
    // gs_mmc5983ma_cr0.B.TM_M=0;
    // gs_mmc5983ma_cr0.B.TM_T=1;
    send_buf[0]=MMC5983MA_CR0_ADDR;
    send_buf[1]=2;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //读取状态
    do
    {
        send_buf[0]=0x80|MMC5983MA_SR_ADDR;
        send_buf[1]=0;
        rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
        // rt_thread_mdelay(1);
    } while (((Mmc5893maStatRegUnion)recv_buf[1]).B.Meas_T_Done==0);

    //读取数据
    send_buf[0]=0x80|MMC5983MA_T_OUT_ADDR;
    send_buf[1]=0x00;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    float t=-75+recv_buf[1]*0.8f;

    sensor_data->type = RT_SENSOR_CLASS_TEMP;
    sensor_data->data.temp=t*10;
    sensor_data->timestamp = rt_sensor_get_ts();

    //清除sr
    send_buf[0]=MMC5983MA_SR_ADDR;
    send_buf[1]=2;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    //跑一次init
    send_buf[0]=MMC5983MA_CR1_ADDR;
    send_buf[1]=gs_mmc5983ma_cr1.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    send_buf[0]=MMC5983MA_CR0_ADDR;
    send_buf[1]=gs_mmc5983ma_cr0.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    send_buf[0]=MMC5983MA_CR2_ADDR;
    send_buf[1]=gs_mmc5983ma_cr2.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
//这个是可以的 但是采样率会降低
    // send_buf[0]=MMC5983MA_CR2_ADDR;
    // send_buf[1]=gs_mmc5983ma_cr2.r;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    // send_buf[0]=MMC5983MA_CR0_ADDR;
    // send_buf[1]=gs_mmc5983ma_cr0.r;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    return 1;
}

/**
 * @brief   主要用来设置open_flag
 * @param   args: RT_SENSOR_MODE_POLLING RT_SENSOR_MODE_INT RT_SENSOR_MODE_FIFO
 **/
rt_err_t _mmc5893ma_set_mode(struct rt_sensor_device *sensor, void *args)
{
    sensor->parent.open_flag=(rt_uint16_t)args;
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/