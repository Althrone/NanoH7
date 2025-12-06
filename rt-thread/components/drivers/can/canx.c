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

#include <rthw.h>
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

static const rt_uint8_t DLCtoBytes[] = { 0,  1,  2,  3,  4,  5,  6,  7,  8, 12, 16, 20, 24, 32, 48, 64};
static const rt_uint8_t BytestoDLC[] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  9,  9,  9, 10, 10, 10,
                                        10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13,
                                        13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 
                                        14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 
                                        15};

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
    device=&(canx->parent);
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
    canx->ops            = ops;
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
        /* disable interrupt */
        rt_base_t level = rt_hw_interrupt_disable();
        struct rt_canx_rx_fifo *rx_fifo = (struct rt_canx_rx_fifo *)canx->canx_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_ringbuffer_put(rx_fifo->rxrb[event>>8],&tmpmsg,2+DLCtoBytes[tmpmsg.header.dlc]);

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        /* invoke callback */
        if(canx->parent.rx_indicate != RT_NULL)
        {
            canx->parent.rx_indicate(&canx->parent, event>>8);
        }
        break;
    }
    case RT_CANX_EVENT_TX_DONE:
    {
        int index=find_bit_by_bsr(event>>8);
        //释放一个锁以允许客户写入
        struct rt_canx_tx_fifo *tx_fifo = (struct rt_canx_tx_fifo *)canx->canx_tx;
        RT_ASSERT(tx_fifo != RT_NULL);
        if(index!=-1)
            rt_completion_done(&tx_fifo->txbuf[index].completion);
        else
            rt_completion_done(&tx_fifo->txbuf[tx_fifo->txrbnum-1].completion);
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

rt_inline int _canx_int_rx(struct rt_canx_device *canx, rt_uint8_t *data, rt_size_t len)
{
    struct rt_canx_rx_fifo *rx_fifo;
    RT_ASSERT(canx != RT_NULL);

    rx_fifo = (struct rt_canx_rx_fifo *)canx->canx_rx;
    RT_ASSERT(rx_fifo != RT_NULL);

    // while(len)
    // {
        rt_base_t level;
        level = rt_hw_interrupt_disable();
        //根据帧头找到对应的ringbuf
        size_t i = 0;
        struct rt_canx_msg peek_msg;
        for (; i < rx_fifo->rxrbnum; i++)
        {
            rt_ringbuffer_peek(&rx_fifo->rxrb[i], (rt_uint8_t *)&peek_msg, 5);
            if((peek_msg.header.id  == ((struct rt_canx_msg*)data)->header.id)&&
               (peek_msg.header.rtr == ((struct rt_canx_msg*)data)->header.rtr)&&
               (peek_msg.header.ide == ((struct rt_canx_msg*)data)->header.ide)&&
               (peek_msg.header.fdf == ((struct rt_canx_msg*)data)->header.fdf)&&
               (peek_msg.header.brs == ((struct rt_canx_msg*)data)->header.brs)&&
               (peek_msg.header.dlc == ((struct rt_canx_msg*)data)->header.dlc))
                break;
        }
        if(i==rx_fifo->rxrbnum)
            return 0;

        //从ringbuf中取出数据
        rt_ringbuffer_get(&rx_fifo->rxrb[i], data, DLCtoBytes[peek_msg.header.dlc]);
        
        rt_hw_interrupt_enable(level);

        return 1;
        
    // }
}

rt_inline int _canx_int_tx(struct rt_canx_device *canx, rt_uint8_t *data, rt_size_t len)
{
    struct rt_canx_tx_fifo *tx_fifo;
    RT_ASSERT(canx != RT_NULL);

    tx_fifo = (struct rt_canx_tx_fifo *)canx->canx_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    rt_base_t level = rt_hw_interrupt_disable();

    //找到对应这个帧的ringbuf
    size_t i = 0;
    for (; i < tx_fifo->txrbnum; i++)
    {
        if((tx_fifo->txbuf->header.id == ((struct rt_canx_msg*)data)->header.id)&&
           (tx_fifo->txbuf->header.rtr == ((struct rt_canx_msg*)data)->header.rtr)&&
           (tx_fifo->txbuf->header.ide == ((struct rt_canx_msg*)data)->header.ide)&&
           (tx_fifo->txbuf->header.fdf == ((struct rt_canx_msg*)data)->header.fdf)&&
           (tx_fifo->txbuf->header.brs == ((struct rt_canx_msg*)data)->header.brs)&&
           (tx_fifo->txbuf->header.dlc == ((struct rt_canx_msg*)data)->header.dlc))
            break;
    }
    if(i==tx_fifo->txrbnum)
            return 0;
    
    rt_ringbuffer_put(tx_fifo->txbuf[i].txrb, data, DLCtoBytes[tx_fifo->txbuf->header.dlc]);
    
    //如果ringbuf满了
        // rt_sem_take(&(tx_fifo->sem), RT_WAITING_FOREVER);

    rt_hw_interrupt_enable(level);

    return 0;
}

static rt_err_t rt_canx_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;

    struct rt_canx_device *canx;
    RT_ASSERT(dev != RT_NULL);
    canx = (struct rt_canx_device *)dev;

    /* initialize rx/tx */
    canx->canx_rx=RT_NULL;
    canx->canx_tx=RT_NULL;

    /* apply configuration */
    if (canx->ops->configure)
        result = canx->ops->configure(canx, &canx->config);
    else
        result = -RT_ENOSYS;

    return result;
}

static rt_err_t rt_canx_open(struct rt_device *dev, rt_uint16_t oflag)
{
    //为static can buf 和fifo分配空间

    struct rt_canx_device *canx;
    RT_ASSERT(dev != RT_NULL);
    canx = (struct rt_canx_device *)dev;

    /* get open flags */
    dev->open_flag = oflag & 0xff;
    if (canx->canx_rx == RT_NULL)
    {
        if(oflag & RT_DEVICE_FLAG_INT_RX)
        {
            struct rt_canx_rx_fifo *rx_fifo;

            rx_fifo = (struct rt_canx_rx_fifo *)rt_malloc(sizeof(struct rt_canx_rx_fifo)+
                      canx->config.num_of_rx_buf * sizeof(struct rt_ringbuffer *));

            rx_fifo->rxrbnum= canx->config.num_of_rx_buf;

            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanRecv)&&(cycle!=0))?frame_len*canx->config.tx_event_fifo_coef:0),
            rt_uint8_t rxbuflen_tbl[] ={CAN_MSG_MATRIX};
            #undef Y

            //周期接收帧创建空间
            size_t j=0;
            for (size_t i = 0; i < sizeof(rxbuflen_tbl); i++)
            {
                if(rxbuflen_tbl[i] == 0)
                    continue;
                rx_fifo->rxrb[j] = rt_ringbuffer_create(rxbuflen_tbl[i]);
                j++;
            }
            //事件接收帧创建空间
            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanRecv)&&(cycle==0))?frame_len*canx->config.rx_event_fifo_coef:0)+
              rt_uint8_t txbuflen =CAN_MSG_MATRIX 0;
              rx_fifo->rxrb[rx_fifo->rxrbnum-1] = rt_ringbuffer_create(txbuflen);
            #undef Y

            canx->canx_rx = rx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
            /* configure low level device */
            canx->ops->control(canx, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        }
        else
        {
            canx->canx_rx = RT_NULL;
        }
    }

    if (canx->canx_tx == RT_NULL)
    {
        if(oflag & RT_DEVICE_FLAG_INT_TX)
        {
            struct rt_canx_tx_fifo *tx_fifo;

            tx_fifo = (struct rt_canx_tx_fifo *)rt_malloc(sizeof(struct rt_canx_tx_fifo)+
                      canx->config.num_of_tx_buf * sizeof(struct rt_canx_tx_buf));

            tx_fifo->txrbnum = canx->config.num_of_tx_buf;

            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanSend)&&(cycle!=0))?frame_len*canx->config.tx_event_fifo_coef:0),
            rt_uint8_t txbuflen_tbl[] ={CAN_MSG_MATRIX};//只有发送的周期帧有长度
            #undef Y

            #define Y(h_fdf,h_brs,h_ide,h_id,dir,frame_len,cycle) \
              {.id = h_id, .rtr = 0, .ide = h_ide, .fdf = h_fdf, .brs = h_brs, .dlc = BytestoDLC[frame_len]},
              struct rt_canx_header tmpheader[] = {CAN_MSG_MATRIX};
            #undef Y

            //给周期发送的创建空间
            size_t j=0;
            for (size_t i = 0; i < sizeof(txbuflen_tbl); i++)
            {
                if(txbuflen_tbl[i] == 0)
                    continue;
                tx_fifo->txbuf[j].header=tmpheader[i];
                tx_fifo->txbuf[j].txrb = rt_ringbuffer_create(txbuflen_tbl[i]);
                j++;
            }
            //给非周期帧创建空间
            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanSend)&&(cycle==0))?frame_len*canx->config.tx_event_fifo_coef:0)+
              rt_uint8_t txbuflen =CAN_MSG_MATRIX 0;
              tx_fifo->txbuf[tx_fifo->txrbnum-1].txrb = rt_ringbuffer_create(txbuflen);
            #undef Y

            canx->canx_tx = tx_fifo;

            for (size_t i = 0; i < tx_fifo->txrbnum; i++)
            {
                rt_completion_init(&tx_fifo->txbuf[i].completion);
            }

            dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
            /* configure low level device */
            canx->ops->control(canx, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
        }
        else
        {
            canx->canx_tx = RT_NULL;
        }
    }

    return RT_EOK;
}

static rt_err_t rt_canx_close(struct rt_device *dev)
{
    struct rt_canx_device *canx;
    
    RT_ASSERT(dev != RT_NULL);
    canx = (struct rt_canx_device *)dev;

    /* this device has more reference count */
    if (dev->ref_count > 1) return RT_EOK;

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        struct rt_canx_rx_fifo* rx_fifo;
        
        /* configure low level device */
        canx->ops->control(canx, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_RX);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;

        rx_fifo = (struct rt_canx_rx_fifo*)canx->canx_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        //删除所有ringbuf
        for (size_t i = 0; i < rx_fifo->rxrbnum; i++)
        {
            rt_ringbuffer_destroy(rx_fifo->rxrb[i]);
        }

        rt_free(rx_fifo);
        canx->canx_rx = RT_NULL;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        struct rt_canx_tx_fifo* tx_fifo;

        /* configure low level device */
        canx->ops->control(canx, RT_DEVICE_CTRL_CLR_INT, (void*)RT_DEVICE_FLAG_INT_TX);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;

        tx_fifo = (struct rt_canx_tx_fifo*)canx->canx_tx;
        RT_ASSERT(tx_fifo != RT_NULL);

        for (size_t i = 0; i < tx_fifo->txrbnum; i++)
        {
            rt_ringbuffer_destroy(tx_fifo->txbuf[i].txrb);
        }

        rt_free(tx_fifo);
        canx->canx_tx = RT_NULL;
    }

    canx->ops->control(canx, RT_DEVICE_CTRL_CLOSE, RT_NULL);
    dev->open_flag &= ~RT_DEVICE_FLAG_ACTIVATED;

    return RT_EOK;
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

static rt_size_t rt_canx_write(struct rt_device *dev,
                               rt_off_t          pos,
                               const void       *buffer,
                               rt_size_t         size)
{
    struct rt_canx_device *canx;
    
    RT_ASSERT(dev != RT_NULL);
    if( size == 0) return 0;

    canx = (struct rt_canx_device *)dev;

    if( (dev->open_flag & RT_DEVICE_FLAG_INT_TX) && (dev->ref_count > 0))
    {
        return _canx_int_tx(canx, buffer, size);
    }

    return 0;
}

static void canx_tx_thread_entry(void *parameter)
{
    //循环执行
    //通过tx发送完成等中断判断要将哪个数据塞进去发送
    rt_device_t dev = (rt_device_t)parameter;
    RT_ASSERT(dev != RT_NULL);
    struct rt_canx_device *canx;
    canx = (struct rt_canx_device *)dev;

    struct rt_canx_tx_fifo *tx_fifo;
    RT_ASSERT(canx != RT_NULL);

    tx_fifo = (struct rt_canx_tx_fifo *)canx->canx_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    while(1)
    {
        //发送
        for (size_t i = 0; i < tx_fifo->txrbnum-1; i++)//前面的周期buf发一个等一次
        {
            canx->ops->sendmsg(canx,tx_fifo->txbuf[i].txrb,i);
            rt_completion_wait(&(tx_fifo->txbuf[i].completion), RT_WAITING_FOREVER);
        }
        //事件fifo放到没空间
        rt_completion_wait(&(tx_fifo->txbuf[tx_fifo->txrbnum-1].completion), RT_WAITING_FOREVER);
        
    }
}