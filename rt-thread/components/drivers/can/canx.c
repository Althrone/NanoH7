/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\components\drivers\can\canx.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/


/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

static rt_err_t rt_canx_init(struct rt_device      *dev);
static rt_err_t rt_canx_open(struct rt_device      *dev, 
                             rt_uint16_t            oflag);
static rt_err_t rt_canx_close(struct rt_device     *dev);
static rt_size_t rt_canx_read(struct rt_device     *dev,
                              rt_off_t              pos,
                              void                 *buffer,
                              rt_size_t             size);
static rt_size_t rt_canx_write(struct rt_device    *dev,
                               rt_off_t             pos,
                               const void          *buffer,
                               rt_size_t            size);
static rt_err_t rt_canx_control(struct rt_device   *dev,
                                int                 cmd,
                                void               *args);
/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

rt_err_t rt_hw_canx_register(struct rt_canx_device * canx, 
                             const char *name, 
                             const struct rt_canx_ops *ops, 
                             const void *user_data)
{
    rt_err_t ret=RT_EOK;
    struct rt_device *device;
    RT_ASSERT(canx != RT_NULL);
    //获取canx的抽象设备类
    device=&canx->parent;
    //设备类里的具体设备类型
    device->type=RT_Device_Class_CAN;
    //回调函数
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    //设备类操作函数
    #ifdef RT_USING_DEVICE_OPS
      device->ops         = &canx_device_ops;
    #else
      device->init        = rt_canx_init;
      device->open        = rt_canx_open;
      device->close       = rt_canx_close;
      device->read        = rt_canx_read;
      device->write       = rt_canx_write;
      device->control     = rt_canx_control;
    #endif
    device->user_data   = user_data;

    /* register a character device */
    ret = rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);

    return ret;
}

/* ISR for can interrupt */
void rt_hw_canx_isr(struct rt_canx_device *canx, rt_uint64_t event)
{
    switch (event&0xff)
    {
    case RT_CANX_EVENT_RX_IND:
    {
        struct rt_canx_msg tmpmsg;
        if(canx->ops->recvmsg(canx,&tmpmsg,event>>8)==-RT_ERROR)
            break;
        //tmpmsg存入指定地址
        //通过canid判断放入软件buf(static buf)还是软件fifo(ringbuf）
        break;
    }
    case RT_CANX_EVENT_TX_DONE:
    {
        //释放一个锁以允许客户写入
    }
    case RT_CANX_EVENT_BUS_OFF:
    {
        //busoff处理
        break;
    }
    default:
        break;
    }
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_err_t rt_canx_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;

    struct rt_canx_device *canx;
    RT_ASSERT(dev != RT_NULL);
    canx = (struct rt_canx_device *)dev;

    //

    /* apply configuration */
    if (canx->ops->configure)
        result = canx->ops->configure(canx, &canx->config);
    else
        result = -RT_ENOSYS;

    return result;
}

static rt_size_t rt_canx_read(struct rt_device *dev,
                             rt_off_t          pos,
                             void             *buffer,
                             rt_size_t         size)
{
    struct rt_canx_device *canx;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    canx = (struct rt_canx_device *)dev;

    if ((dev->open_flag & RT_DEVICE_FLAG_INT_RX) && (dev->ref_count > 0))
    {
        return _canx_int_rx(canx, buffer, size);
    }

    return 0;
}

static rt_err_t rt_canx_open(struct rt_device      *dev, 
                             rt_uint16_t            oflag)
{
    //为static can buf 和fifo分配空间
}