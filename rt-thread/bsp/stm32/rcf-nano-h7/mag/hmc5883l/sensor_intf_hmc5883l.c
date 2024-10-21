#include "hmc5883l.h"
#include <rtdbg.h>

struct rt_sensor_data mag_data;

static rt_size_t _hmc5883l_mag_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len)
{
    //从hmc5883l.c获取数据
    //由于rtt中三轴型数据是整型存储，讨论浮点型无意义

    // struct hmc5883l_3axes mag_data;
    // hmc5883l_get_raw_data(sensor,&mag_data);//在其中调用rtt的iic函数

    // sensor_data->type = RT_SENSOR_CLASS_MAG;
    // sensor_data->data.mag.x = mag_data.x*1000;//rtt使用的是毫高斯
    // sensor_data->data.mag.y = mag_data.y*1000;
    // sensor_data->data.mag.z = mag_data.z*1000;
    // sensor_data->timestamp = rt_sensor_get_ts();

    return 0;
}

static rt_size_t hmc5883l_mag_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if(sensor->parent.open_flag&RT_DEVICE_FLAG_RDONLY)
    {
        return _hmc5883l_mag_polling_get_data(sensor, buf, len);
    }
    else if (sensor->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        // return _xxx_acc_int_get_data(sensor, buf, len);
    }
    else if (sensor->parent.open_flag & RT_DEVICE_FLAG_FIFO_RX)
    {
        // return _xxx_acc_fifo_get_data(sensor, buf, len);
    }

    return 0;
}

static rt_err_t hmc5883l_mag_control(struct rt_sensor_device *sensor, int cmd, void *args)//args是32位(指针都是4个字节)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
        //考虑添加一个复位命令
    case RT_SENSOR_CTRL_GET_ID:
        result=hmc5883l_mag_get_id(sensor,args);
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

static struct rt_sensor_ops sensor_ops =
{
    hmc5883l_mag_fetch_data, 
    hmc5883l_mag_control
};

int rt_hw_hmc5883l_init(const char *name, struct rt_sensor_config *mag_cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_mag = RT_NULL;

    sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_mag == RT_NULL)//空间开辟失败
        return -RT_ERROR;

    sensor_mag->info.type       = RT_SENSOR_CLASS_MAG;
    sensor_mag->info.vendor     = RT_SENSOR_VENDOR_HONEYWELL;
    sensor_mag->info.model      = "hmc5883l";
    sensor_mag->info.unit       = RT_SENSOR_UNIT_MGAUSS;
    sensor_mag->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor_mag->info.range_max  = 8100;
    sensor_mag->info.range_min  = 880;
    sensor_mag->info.period_min = 5;
    // sensor_mag->info.fifo_max   = 0;

    rt_memcpy(&sensor_mag->config, mag_cfg, sizeof(struct rt_sensor_config));
    sensor_mag->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, RT_NULL);

    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor_mag);
        return -RT_ERROR;
    }
    else
    {
        LOG_I("sensor register success");//注册成功

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

        // //读取设备ID，这里开始用到自己写的东西了
        // rt_uint8_t id = 0xFF;
        // if(rt_device_control(sensor_mag, RT_SENSOR_CTRL_GET_ID, &id))
        // {
        //     LOG_E("get device id faile");
        //     return -RT_ERROR;
        // }
        // LOG_I("device id: 0x%x!", id);

        return RT_EOK;
    }
}

//需要创建两个函数，一个是自动初始化用的 一个是进程入口函数
//命令好像是可以通过传感器通用的cfg命令控制

// /**
//  * @brief   入口函数
//  **/
// static void thread_entry(void *parameter)
// {
//     //查找设备
//     rt_device_t mag_dev = RT_NULL;
//     mag_dev = rt_device_find(parameter);//传入设备名字

//     //打开设备
//     rt_device_open(mag_dev,RT_DEVICE_FLAG_RDWR);//传感器注册的时候也用到 RT_DEVICE_FLAG_RDWR 这个参数
//     //初始化

//     //读取数据
//     while (1)
//     {
//         /* code */
//         if(rt_device_read(mag_dev,0,&mag_data,1))//数据读取到全局变量中
//             break;
//     }
//     //显示读取设备出错
//     LOG_E("faile to read mag data\n");
//     //关闭设备
//     rt_device_close(mag_dev);
//     LOG_I("mag close");
// }

void rt_hmc5883l_init(void)
{

}

/**
 * @brief   导出到自动初始化的函数
 * @note    自动初始化成默认的传感器状态
 **/
int rt_hw_hmc5883l_port(void)
{
    //查找通讯总线
    if(rt_device_find("i2c2")==RT_NULL)
    {
        LOG_E("Can't find i2c2 bus device");
        return -RT_ERROR;
    }

    struct rt_sensor_config cfg;

    cfg.intf.dev_name="i2c2";
    cfg.intf.type=RT_SENSOR_INTF_I2C;//这个好像没用到
    cfg.intf.user_data=(void*)HMC5883L_ADDR;//rtt使用七位地址
    // cfg.intf.user_data=(void*)0x0D;//rtt使用七位地址
    // cfg.irq_pin.pin = irq_pin;
    // cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    //查找是否存在 mag_0,mag_1...
    char new_dev_name[]="0";
    char exist_dev_name[]="mag_0";
    while(rt_device_find(exist_dev_name))//如果是NULL的话就会跳出while
    {
        ++new_dev_name[0];
        ++exist_dev_name[4];
    }
    rt_hw_hmc5883l_init(new_dev_name,&cfg);//用户不用管这个磁力计是什么型号的

    return RT_EOK;
}
// INIT_DEVICE_EXPORT(rt_hw_hmc5883l_port);