#define DBG_TAG  "5983"
#include <drv_log.h>

#include "mmc5983ma.h"

//需要测试连续模式下temp是不是也是连续的，还是说只有mag连续
void mmc5893ma_init(rt_sensor_t sensor)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    mmc5893ma_reset(sensor);

    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    //cr1 设置BW
    Mmc5893maCtrl1RegUnion cr1={
        .B.BW=1,//200Hz
    };
    send_buf[0]=MMC5983MA_CR1_ADDR;//
    send_buf[1]=cr1.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //cr2 CM_Freq
    Mmc5893maCtrl2RegUnion cr2={
        .B.Cm_freq=6,
        // .B.Cmm_en=1,
        // .B.En_prd_set=1,
        // Prd_set不知道干嘛用的
    };
    send_buf[0]=MMC5983MA_CR2_ADDR;
    send_buf[1]=cr2.r;
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //CR0 Auto_SR_en
    Mmc5893maCtrl0RegUnion cr0={
        .B.Auto_SR_en=1,
        // .B.INT_meas_done_en=1,
    };
    send_buf[0]=MMC5983MA_CR0_ADDR;//开启测量完成中断，开启自动sr，中断只是用来亮灯
    send_buf[1]=cr0.r;//开启测量完成中断，开启自动sr，中断只是用来亮灯
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //CR2 CMM_EN
    send_buf[0]=0x80|MMC5983MA_CR2_ADDR;
    send_buf[1]=0;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    ((Mmc5893maCtrl2RegUnion*)(&recv_buf[1]))->B.Cmm_en=1;
    send_buf[0]=MMC5983MA_CR2_ADDR;
    send_buf[1]=recv_buf[1];
    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    //CR0 INT_MEAS_DONE_EN
    send_buf[0]=0x80|MMC5983MA_CR0_ADDR;
    send_buf[1]=0;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);
    ((Mmc5893maCtrl0RegUnion*)(&recv_buf[1]))->B.INT_meas_done_en=1;
    send_buf[0]=MMC5983MA_CR0_ADDR;
    send_buf[1]=recv_buf[1];
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
    rt_uint8_t send_buf[]={MMC5983MA_CR1_ADDR,0x80};
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

    rt_uint8_t send_buf[]={0x80|MMC5983MA_PID1_ADDR};
    rt_uint8_t recv_buf[1]={0};

    result=rt_spi_send_then_recv(spi_dev,send_buf,sizeof(send_buf),recv_buf,sizeof(recv_buf));

    *(rt_uint8_t *)args = recv_buf[0];
    return result;
}

rt_err_t mmc5893ma_set_odr(struct rt_sensor_device *sensor, void *args)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    rt_uint8_t send_buf[]={0x80|MMC5983MA_CR1_ADDR};
    rt_uint8_t recv_buf[2]={0};

    //读取CR1 CR2原始值
    result=rt_spi_send_then_recv(spi_dev,send_buf,sizeof(send_buf),recv_buf,sizeof(recv_buf));

    //判断是否为连续测量模式
    if((*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cmm_en)//连续模式
    {
        (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=0;
        switch (*(rt_uint16_t*)args)
        {
        case 1://hz
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=1;
            break;
        case 10:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=2;
            break;
        case 20:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=3;
            break;
        case 50:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=4;
            break;
        case 100:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=5;
            break;
        case 200:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=6;
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=1;
            break;
        case 1000:
            (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=7;
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=3;
            break;
        default:
            result=-RT_ERROR;
            LOG_E("连续模式必须是1、10、20、50、100、200或者1000");
            break;
        }
    }
    else//单次模式
    {
        (*(Mmc5893maCtrl2RegUnion*)(&recv_buf[1])).B.Cm_freq=0;
        switch (*(rt_uint16_t*)args)
        {
        case 100:
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=0;
            break;
        case 200:
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=1;
            break;
        case 400:
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=2;
            break;
        case 800:
            (*(Mmc5893maCtrl1RegUnion*)(&recv_buf[0])).B.BW=3;
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
        rt_spi_send_then_send(spi_dev,send_buf,sizeof(send_buf),
                              recv_buf,sizeof(recv_buf));
    }

    return result;
}

// rt_err_t mmc5893ma_polling_get_mag(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
rt_err_t mmc5893ma_polling_get_mag(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi12");
    // spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    //通过向sr的MEAS_T_DONE和MEAS_M_DONE写1清除中断标志
    Mmc5893maStatRegUnion sr_clear={
        .B.Meas_M_Done=1,
        .B.Meas_T_Done=1,
    };
    rt_uint8_t send_buf[]={MMC5983MA_SR_ADDR,sr_clear.r};
    rt_uint8_t recv_buf[7]={0};

    rt_spi_transfer(spi_dev,send_buf,RT_NULL,2);

    send_buf[0]=0x80|MMC5983MA_CR3_ADDR;
    send_buf[1]=0;
    rt_spi_transfer(spi_dev,send_buf,recv_buf,1);

    // send_buf[0]=0x80|MMC5983MA_X_OUT_0_ADDR;
    // result=rt_spi_send_then_recv(spi_dev,send_buf,1,recv_buf,7);

    // rt_int32_t x=((recv_buf[0]&0x80)>0)?0xFFFC0000|(((rt_uint32_t)recv_buf[0])<<10)|(((rt_uint32_t)recv_buf[1])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Xout:
    //                                                (((rt_uint32_t)recv_buf[0])<<10)|(((rt_uint32_t)recv_buf[1])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Xout;

    // rt_int32_t y=((recv_buf[2]&0x80)>0)?0xFFFC0000|(((rt_uint32_t)recv_buf[2])<<10)|(((rt_uint32_t)recv_buf[3])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Yout:
    //                                                (((rt_uint32_t)recv_buf[2])<<10)|(((rt_uint32_t)recv_buf[3])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Yout;

    // rt_int32_t z=((recv_buf[4]&0x80)>0)?0xFFFC0000|(((rt_uint32_t)recv_buf[4])<<10)|(((rt_uint32_t)recv_buf[5])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Zout:
    //                                                (((rt_uint32_t)recv_buf[4])<<10)|(((rt_uint32_t)recv_buf[5])<<2)|((Mmc5893maXyzOut2RegUnion*)(&recv_buf[6]))->B.Zout;

    // sensor_data->type = RT_SENSOR_CLASS_ACCE
    // sensor_data->data.acce.x = acceleration.x;
    // sensor_data->data.acce.y = acceleration.y;
    // sensor_data->data.acce.z = acceleration.z;
    // sensor_data->timestamp = rt_sensor_get_ts();

    return result;
}

rt_err_t mmc5893ma_polling_get_temp(void)
{
    rt_err_t result=RT_EOK;
    //查找总线设备
    struct rt_spi_device *spi_dev=RT_NULL;
    spi_dev=(struct rt_spi_device *)rt_device_find("spi12");
    // spi_dev=(struct rt_spi_device *)rt_device_find(sensor->config.intf.dev_name);

    //温度不做int清除的操作

    rt_uint8_t send_buf[]={0x80|MMC5983MA_T_OUT_ADDR,0x00};
    rt_uint8_t recv_buf[2]={0};
    rt_spi_transfer(spi_dev,send_buf,recv_buf,2);

    float t=-75+recv_buf[1]*0.8f;

    return result;
}