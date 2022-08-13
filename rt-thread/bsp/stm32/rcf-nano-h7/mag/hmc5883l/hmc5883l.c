#include "hmc5883l.h"
#include "drv_soft_i2c.h"

//获取到总线设备后保存到这里
//同一种传感器支持十个
//通过宏定义修改
// static struct rt_i2c_bus_device bus[1];

/**
 * @brief   
 * @param   bus_name: 通讯总线名称
 * @note    初始化接口
 **/
static void hmc5883l_init(const char *bus_name)
{
    struct rt_i2c_bus_device *bus_dev = RT_NULL;
    bus_dev = (struct rt_i2c_bus_device *)rt_device_find(bus_name);
}

rt_size_t hmc5883l_get_raw_data(struct rt_sensor_device *sensor,struct hmc5883l_3axes *data)
{

    struct rt_i2c_bus_device *bus;
    // rt_i2c_master_send();
}


static rt_err_t _hmc5883l_get_mag_raw(struct hmc5883l_dev *dev, struct hmc5883l_3axes *mag)
{
    rt_uint8_t buffer[6];

    struct rt_i2c_msg msgs[2];

    rt_uint8_t reg=X_MSB_REG;

    msgs[0].addr=dev->mag_id;
    msgs[0].flags=RT_I2C_WR;
    msgs[0].buf=&reg;
    msgs[0].len=1;

    msgs[1].addr=dev->mag_id;
    msgs[1].flags=RT_I2C_RD;
    msgs[1].buf=buffer;
    msgs[1].len=sizeof(buffer);

    if(rt_i2c_transfer((struct rt_i2c_bus_device *)(dev->mag_bus),msgs,2)==2)
    {
        mag->x = (rt_int16_t)(buffer[1] | ((rt_int16_t)buffer[0] << 8));
        mag->y = (rt_int16_t)(buffer[5] | ((rt_int16_t)buffer[4] << 8));
        mag->z = (rt_int16_t)(buffer[3] | ((rt_int16_t)buffer[2] << 8));
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

rt_size_t hmc5883l_get_mag(struct hmc5883l_dev *dev, struct hmc5883l_data *buf)
{
    struct hmc5883l_3axes tmp;

    _hmc5883l_get_mag_raw(dev,&tmp);
    buf->x=((float)tmp.x)/230.0f;
    buf->y=((float)tmp.y)/230.0f;
    buf->z=((float)tmp.z)/230.0f;

    return 1;//???
}

/**
 * @brief   获取传感器ID
 * @param   sensor:
 * @param   args: id就放在这里
 **/
rt_err_t hmc5883l_mag_get_id(struct rt_sensor_device *sensor, void *args)
{
    //查找总线设备
    struct rt_i2c_bus_device *bus=RT_NULL;
    bus=(struct rt_i2c_bus_device *)rt_device_find(sensor->config.intf.dev_name);
    // rt_uint8_t send_buf[]={0x0D};
    rt_uint8_t send_buf[]={ID_A_REG};
    //发送ID寄存器地址
    if(rt_i2c_master_send(bus,sensor->config.intf.user_data,RT_I2C_WR,send_buf,sizeof(send_buf))!=sizeof(send_buf))
    {
        return -RT_ERROR;
    }
    //接收ID到args
    rt_uint8_t recv_buf[3]={0xFF,0xFF,0xFF};
    if(rt_i2c_master_recv(bus,sensor->config.intf.user_data,RT_I2C_RD,recv_buf,sizeof(recv_buf))!=sizeof(recv_buf))
    {
        return -RT_ERROR;
    }
    //转存到args中
    // *(rt_uint32_t *)args = ((rt_uint32_t)recv_buf[0]<<16)|
    //                        ((rt_uint32_t)recv_buf[1]<<8)|
    //                        ((rt_uint32_t)recv_buf[2]<<0);

    *(rt_uint32_t *)args = (rt_uint32_t)recv_buf[0];
    return RT_EOK;
}