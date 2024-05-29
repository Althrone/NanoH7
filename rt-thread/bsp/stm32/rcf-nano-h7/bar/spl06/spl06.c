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
#include <rtdbg.h>

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

static Spl06CoefStruct s_spl06_coef={0};

//都是默认1s一次 超采也是1次

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

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

/**
 * @brief   仅获取气压
 **/
rt_size_t _spl06_baro_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
{
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
        return 0;//传感器数据返回长度只有0和1，0表示失败

    rt_uint8_t send_buf[1+6]={0};
    rt_uint8_t recv_buf[1+6]={0};

    // // 读取stat清除中断
    // rt_uint8_t send_buf[]={0x80|SPL06_INT_STS_REG_ADDR,0x00};
    // rt_uint8_t recv_buf[2]={0};
    // rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    send_buf[0]=0x80|SPL06_PSR_B2_REG_ADDR;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,sizeof(send_buf));

    rt_int32_t Traw=(recv_buf[4]&0x80)?0xFF000000|(((rt_uint32_t)recv_buf[4])<<16)|(((rt_uint32_t)recv_buf[5])<<8)|((rt_uint32_t)recv_buf[6]):
                                                  (((rt_uint32_t)recv_buf[4])<<16)|(((rt_uint32_t)recv_buf[5])<<8)|((rt_uint32_t)recv_buf[6]);
    rt_int32_t Praw=(recv_buf[1]&0x80)?0xFF000000|(((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]):
                                                  (((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]);
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

/**
 * @brief   自动按照最高精度设置
 * @note    两个传感器的采样值都可以从1-128，temp的超采貌似限死1次
 */
static rt_err_t spl06_auto_set_odr(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    rt_uint8_t send_buf[3]={0};
    rt_uint8_t recv_buf[3]={0};

    //读shift
    send_buf[0]=0x80|SPL06_CFG_REG_ADDR;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    if(!strcmp(args,"low"))//气象站模式
    {
        send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
        send_buf[1]=0x01;//1hz 2次超采
        send_buf[2]=0x80;//1hz 1次超采
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,3);
        //可能需要手动清空shift
        send_buf[0]=SPL06_CFG_REG_ADDR;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.TMP_SHIFT_EN=0;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.PRS_SHIFT_EN=0;
        send_buf[1]=recv_buf[1];
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    }
    else if(!strcmp(args,"std"))//室内导航模式
    {
        send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
        send_buf[1]=0x14;//2hz 16次超采
        send_buf[2]=0x80;//1hz 1次超采
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,3);
        //Enable Pshift
        send_buf[0]=SPL06_CFG_REG_ADDR;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.TMP_SHIFT_EN=0;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.PRS_SHIFT_EN=1;
        send_buf[1]=recv_buf[1];
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    }
    else if(!strcmp(args,"high"))//运动模式
    {
        send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
        send_buf[1]=0x26;//4Hz 64次超采
        send_buf[2]=0xA0;//4Hz 1次超采
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,3);
        //Enable Pshift
        send_buf[0]=SPL06_CFG_REG_ADDR;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.TMP_SHIFT_EN=0;
        (*(Spl06CfgRegUnion*)&recv_buf[1]).B.PRS_SHIFT_EN=1;
        send_buf[1]=recv_buf[1];
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    }
    
    send_buf[0]=0x80|SPL06_PRS_CFG_REG_ADDR;
    send_buf[1]=0;
    send_buf[2]=0;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,3);//读

    const rt_uint8_t rate_tbl[]={1,2,4,8,16,32,64,128};//hz
    const rt_uint16_t pressure_measurement_time_msx10_tbl[]={36,52,84,148,276,532,1044,2068};//ms*10

    rt_uint8_t pm_rate=rate_tbl[((Spl06PrsCfgRegUnion)recv_buf[1]).B.PM_RATE];//输出速度 单位hz
    rt_uint16_t pm_time=pressure_measurement_time_msx10_tbl[((Spl06PrsCfgRegUnion)recv_buf[1]).B.PM_PRC];
    rt_uint8_t tm_rate=rate_tbl[((Spl06TmpCfgRegUnion)recv_buf[2]).B.TMP_RATE];//输出速度 单位hz

    rt_uint32_t toltal_time=0;

    if(sensor->info.type==RT_SENSOR_CLASS_BARO)//设置的是气压计
    {
        switch (*(rt_uint16_t*)args)
        {
        case 1:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=0;
            break;
        case 2:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=1;
            break;
        case 4:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=2;
            break;
        case 8:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=3;
            break;
        case 16:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=4;
            break;
        case 32:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=5;
            break;
        case 64:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=6;
            break;
        case 128:
            (*(Spl06PrsCfgRegUnion*)&recv_buf[1]).B.PM_RATE=7;
            break;
        default:
            result = -RT_EINVAL;//args参数错误
            break;
        }
        toltal_time=(*(rt_uint16_t*)args)*pm_time+tm_rate*36;
    }
    else if(sensor->info.type==RT_SENSOR_CLASS_TEMP)
    {
        switch (*(rt_uint16_t*)args)
        {
        case 1:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=0;
            break;
        case 2:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=1;
            break;
        case 4:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=2;
            break;
        case 8:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=3;
            break;
        case 16:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=4;
            break;
        case 32:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=5;
            break;
        case 64:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=6;
            break;
        case 128:
            (*(Spl06TmpCfgRegUnion*)&recv_buf[2]).B.TMP_RATE=7;
            break;
        default:
            result = -RT_EINVAL;//args参数错误
            break;
        }
        toltal_time=(*(rt_uint16_t*)args)*36+pm_rate*pm_time;
    }
    else
        result=-RT_EINVAL;//sensor->info.type参数错误

    if(toltal_time>10000)//1s
    {
        LOG_E("setting over 1s");
        result=-RT_ERROR;//sensor->info.type参数错误
    }

    if(result==RT_EOK)//写入配置
    {
        if(sensor->info.type==RT_SENSOR_CLASS_BARO)
        {
            send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
            send_buf[1]=recv_buf[1];
        }
        else if(sensor->info.type==RT_SENSOR_CLASS_TEMP)
        {
            send_buf[0]=SPL06_TMP_CFG_REG_ADDR;
            send_buf[1]=recv_buf[2];
        }
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);
    }

    return result;
}