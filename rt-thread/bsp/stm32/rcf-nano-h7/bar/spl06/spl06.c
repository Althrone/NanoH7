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

static Spl06CoefStruct gs_spl06_coef={0};

//都是默认1s一次 超采也是1次
static rt_uint8_t gs_p_osr_indx=0;//0 1 2 3 4 ...
static rt_uint8_t gs_t_osr_indx=0;//0 1 2 3 4 ...
static const float kgs_scale_factor_tbl[]={524288.f,1572864.f,3670016.f,7864320.f,
                                           253952.f, 516096.f,1040384.f,2088960.f};//1 2 4 8 16 ...

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

static rt_err_t spl06_polling_get_coef(rt_sensor_t sensor);

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

    //读取MMEAS_CFG确定coef和sensor两个位置
    Spl06MeasCfgRegUnion meas_cfg;
    do
    {
        rt_uint8_t send_buf=0x80|SPL06_MEAS_CFG_REG_ADDR;
        rt_spi_send_then_recv(spi_dev,&send_buf,1,&meas_cfg,1);
        
    } while ((meas_cfg.B.COEF_RDY!=1)||(meas_cfg.B.SENSOR_RDY!=1));

    //读取coef
    spl06_polling_get_coef(sensor);
    
    //设置输出频率等？

    //Enable P shift
    //Sports (addr 0x09)Start background measurements (addr 0x08)

    // send_buf[0]=SPL06_TMP_CFG_REG_ADDR;//Sports 0xA0
    // send_buf[1]=0b11110000;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

    // send_buf[0]=SPL06_PRS_CFG_REG_ADDR;//Sports 0x26
    // send_buf[1]=0b01110001;
    // rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);//tmp使用外部

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
rt_err_t _spl06_get_id(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    rt_uint8_t send_buf[2]={0x80|SPL06_ID_REG_ADDR,0x00};
    rt_uint8_t recv_buf[2]={0};

    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    if(recv_buf[1]!=0x10)
        result=RT_ERROR;
    else
        *(rt_uint8_t *)args = recv_buf[1];

    return result;
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
    if(rt_spi_transfer(spi_dev,send_buf,recv_buf,sizeof(send_buf))!=sizeof(send_buf))
        return 0;

    rt_int32_t Traw=(recv_buf[4]&0x80)?0xFF000000|(((rt_uint32_t)recv_buf[4])<<16)|(((rt_uint32_t)recv_buf[5])<<8)|((rt_uint32_t)recv_buf[6]):
                                                  (((rt_uint32_t)recv_buf[4])<<16)|(((rt_uint32_t)recv_buf[5])<<8)|((rt_uint32_t)recv_buf[6]);
    rt_int32_t Praw=(recv_buf[1]&0x80)?0xFF000000|(((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]):
                                                  (((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]);

    float Traw_sc=Traw/kgs_scale_factor_tbl[gs_t_osr_indx];
    float Praw_sc=Praw/kgs_scale_factor_tbl[gs_p_osr_indx];

    //单位Pa
    float Pcomp = gs_spl06_coef.c00+
                  Praw_sc*(gs_spl06_coef.c10+Praw_sc*(gs_spl06_coef.c20+Praw_sc*gs_spl06_coef.c30))+
                  Traw_sc*gs_spl06_coef.c01+
                  Traw_sc*Praw_sc*(gs_spl06_coef.c11+Praw_sc*gs_spl06_coef.c21);
    
    sensor_data->type = RT_SENSOR_CLASS_BARO;
    sensor_data->data.baro = Pcomp;
    sensor_data->timestamp = rt_sensor_get_ts();

    return 1;
}

rt_size_t _spl06_temp_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
{
     //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
        return 0;//传感器数据返回长度只有0和1，0表示失败
    
    rt_uint8_t send_buf[1+3]={0};
    rt_uint8_t recv_buf[1+3]={0};

    send_buf[0]=0x80|SPL06_TMP_B2_REG_ADDR;
    if(rt_spi_transfer(spi_dev,send_buf,recv_buf,sizeof(send_buf))!=sizeof(send_buf))
        return 0;
    
    rt_int32_t Traw=(recv_buf[4]&0x80)?0xFF000000|(((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]):
                                                  (((rt_uint32_t)recv_buf[1])<<16)|(((rt_uint32_t)recv_buf[2])<<8)|((rt_uint32_t)recv_buf[3]);
    
    float Traw_sc=Traw/kgs_scale_factor_tbl[gs_t_osr_indx];

    float Tcomp = gs_spl06_coef.c0*0.5 + gs_spl06_coef.c1*Traw_sc;//摄氏度

    sensor_data->type = RT_SENSOR_CLASS_TEMP;
    sensor_data->data.temp=Tcomp*10;
    sensor_data->timestamp = rt_sensor_get_ts();

    return 1;
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_err_t spl06_polling_get_coef(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
        result=-RT_ERROR;

    rt_uint8_t send_buf[]={0x80|SPL06_COEF_C0_REG_ADDR};
    rt_uint8_t recv_buf[18]={0};

    result=rt_spi_send_then_recv(spi_dev,send_buf,1,recv_buf,18);

    gs_spl06_coef.c0=(recv_buf[0]&0x80)?0xF000|(((rt_uint16_t)recv_buf[0])<<4)|(recv_buf[1]>>4):
                                               (((rt_uint16_t)recv_buf[0])<<4)|(recv_buf[1]>>4);

    gs_spl06_coef.c1=(recv_buf[1]&0x80)?0xF000|(((rt_uint16_t)(recv_buf[1]&0x0F))<<8)|recv_buf[2]:
                                               (((rt_uint16_t)(recv_buf[1]&0x0F))<<8)|recv_buf[2];

    gs_spl06_coef.c00=(recv_buf[3]&0x80)?0xFFF00000|(((rt_uint32_t)recv_buf[3])<<12)|(((rt_uint32_t)recv_buf[4])<<4)|(recv_buf[5]>>4):
                                                    (((rt_uint32_t)recv_buf[3])<<12)|(((rt_uint32_t)recv_buf[4])<<4)|(recv_buf[5]>>4);
    
    gs_spl06_coef.c10=(recv_buf[5]&0x08)?0xFFF00000|(((rt_uint32_t)(recv_buf[5]&0x0F))<<16)|(((rt_uint32_t)recv_buf[6])<<8)|recv_buf[7]:
                                                    (((rt_uint32_t)(recv_buf[5]&0x0F))<<16)|(((rt_uint32_t)recv_buf[6])<<8)|recv_buf[7];

    gs_spl06_coef.c01=(((rt_uint16_t)recv_buf[8])<<8)|recv_buf[9];
    gs_spl06_coef.c11=(((rt_uint16_t)recv_buf[10])<<8)|recv_buf[11];
    gs_spl06_coef.c20=(((rt_uint16_t)recv_buf[12])<<8)|recv_buf[13];
    gs_spl06_coef.c21=(((rt_uint16_t)recv_buf[14])<<8)|recv_buf[15];
    gs_spl06_coef.c30=(((rt_uint16_t)recv_buf[16])<<8)|recv_buf[17];
    
    return result;
}

/**
 * @brief   按照手册的三个应用场景
 * @param   args: "low" "std" "high"
 **/
static rt_err_t spl06_set_with_use_case(struct rt_sensor_device *sensor, const char *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);
    if(spi_dev==RT_NULL)
    {
        result = -RT_ERROR;
        goto _exit;
    }

    rt_uint8_t send_buf[3]={0};
    rt_uint8_t recv_buf[3]={0};
    Spl06MeasCfgRegUnion meas_cfg;
    Spl06CfgRegUnion cfg_reg;

    //读取 SPL06_MEAS_CFG_REG_ADDR SPL06_CFG_REG_ADDR
    send_buf[0]=0x80|SPL06_MEAS_CFG_REG_ADDR;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,3);
    meas_cfg.r=recv_buf[1];
    cfg_reg.r=recv_buf[2];

    if(!strcmp(args,"low"))//气象站模式
    {
        send_buf[0]=SPL06_PRS_CFG_REG_ADDR;
        send_buf[1]=0x01;//1hz 2次超采
        send_buf[2]=0x80;//1hz 1次超采
        if(rt_spi_transfer(spi_dev,send_buf,RT_NULL,3)!=3)
        {
            result = -RT_ERROR;
            goto _exit;
        }
        //手动清空shift
        send_buf[0]=SPL06_MEAS_CFG_REG_ADDR;
        (*(Spl06MeasCfgRegUnion*)&recv_buf[1]).B.MEAS_CRTL=7;//连续采样

        (*(Spl06CfgRegUnion*)&recv_buf[2]).B.TMP_SHIFT_EN=0;
        (*(Spl06CfgRegUnion*)&recv_buf[2]).B.PRS_SHIFT_EN=0;
        send_buf[1]=recv_buf[2];
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
    else
        result=-RT_EINVAL;//args参数非法

    if(result==RT_EOK)
        rt_spi_transfer(spi_dev,send_buf,RT_NULL,3);
_exit:
    return result;
}
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