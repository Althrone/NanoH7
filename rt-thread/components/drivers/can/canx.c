// SPDX-License-Identifier: Apache-2.0
/** ***************************************************************************
 * @file        canx.c
 * @brief       using rt_ringbuf instead of rt_list.
 * @author      RT-Thread Development Team
 * @author      Althrone
 * @date        2026-02-27
 * @version     1.0.0
 * 
 * @copyright   Copyright (c) 2006-2021 RT-Thread Development Team
 * @copyright   Copyright (c) 2023-2025 Althrone
 * 
 * @par         Change Logs
 * 
 * | Date       | Author            | Notes                                     |
 * |------------|-------------------|-------------------------------------------|
 * | 2015-05-14 | aubrcool@qq.com   | first version                             |
 * | 2015-07-06 | Bernard           | code cleanup and remove RT_CAN_USING_LED  |
 * | 2026-02-27 | Althrone          | using rt_ringbuf instead of rt_list       |
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

        //务必确保buf是帧大小的整数倍
        if((event>>8)>(rx_fifo->rxrbnum-1))
            event = (rx_fifo->rxrbnum-1)<<8;
        //针对事件帧，可能要过滤一下
        //对齐放置，空白填充A5
        rt_ringbuffer_put_force(rx_fifo->rxrb[event>>8],(rt_uint8_t*)&tmpmsg,sizeof(tmpmsg.header)+DLCtoBytes[tmpmsg.header.dlc]);
        rt_uint8_t remainder = rx_fifo->rx_event_max_len - sizeof(tmpmsg.header) - DLCtoBytes[tmpmsg.header.dlc];
        for (size_t i = 0; i < remainder; i++)
        {
            rt_ringbuffer_putchar_force(rx_fifo->rxrb[event>>8],0xA5);
        }

        // for (size_t i = 0; i < rx_fifo->rxrbnum; i++)
        // {
        //     rt_uint16_t len=rt_ringbuffer_data_len(rx_fifo->rxrb[i]);
        //     if(len>=8)
        //     {
        //         rt_kprintf("buf %d\tlen %d\t",i,len);
        //         struct rt_canx_msg tmpprint;
        //         rt_ringbuffer_get(rx_fifo->rxrb[i],(rt_uint8_t *)&tmpprint,sizeof(tmpprint.header));
        //         rt_ringbuffer_get(rx_fifo->rxrb[i],tmpprint.data,DLCtoBytes[tmpprint.header.dlc]);
        //         rt_kprintf("id 0x%x\t",tmpprint.header.id);
        //         for(size_t j=0;j<DLCtoBytes[tmpprint.header.dlc];j++)
        //         {
        //             rt_kprintf("%x ",tmpprint.data[j]);
        //         }

        //         rt_kprintf("time %d\t",tmpprint.header.rxts);
        //         rt_kprintf("\n");
        //     }
        // }

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
        //顺着id找到对应index
        struct rt_canx_tx_fifo *tx_fifo = (struct rt_canx_tx_fifo *)canx->canx_tx;
        RT_ASSERT(tx_fifo != RT_NULL);
        size_t i = 0;
        for (; i < tx_fifo->txrbnum-1; i++)
        {
            if(tx_fifo->txbuf[i].header.id==event>>8)
                break;
        }
        
        //释放一个锁以允许客户写入
        // if(i<tx_fifo->txrbnum-1)
        //     rt_completion_done(&tx_fifo->txbuf[i].completion);
        // else
        //     rt_completion_done(&tx_fifo->txbuf[tx_fifo->txrbnum-1].completion);
        break;
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
        struct rt_canx_header peek_header;
        for (; i < rx_fifo->rxrbnum; i++)
        {
            rt_ringbuffer_peek(&rx_fifo->rxrb[i], (rt_uint8_t *)&peek_header, sizeof(peek_header));
            if((peek_header.id  == ((struct rt_canx_msg*)data)->header.id)&&
               (peek_header.rtr == ((struct rt_canx_msg*)data)->header.rtr)&&
               (peek_header.ide == ((struct rt_canx_msg*)data)->header.ide)&&
               (peek_header.fdf == ((struct rt_canx_msg*)data)->header.fdf)&&
               (peek_header.brs == ((struct rt_canx_msg*)data)->header.brs)&&
               (peek_header.dlc == ((struct rt_canx_msg*)data)->header.dlc))
                break;
        }
        if(i==rx_fifo->rxrbnum)
            return 0;

        //从ringbuf中取出数据
        rt_ringbuffer_get(rx_fifo->rxrb[i], data, DLCtoBytes[peek_header.dlc]);
        
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
    for (; i < tx_fifo->txrbnum-1; i++)
    {
        if((tx_fifo->txbuf[i].header.id  == ((struct rt_canx_msg*)data)->header.id)&&
           (tx_fifo->txbuf[i].header.rtr == ((struct rt_canx_msg*)data)->header.rtr)&&
           (tx_fifo->txbuf[i].header.ide == ((struct rt_canx_msg*)data)->header.ide)&&
           (tx_fifo->txbuf[i].header.fdf == ((struct rt_canx_msg*)data)->header.fdf)&&
           (tx_fifo->txbuf[i].header.brs == ((struct rt_canx_msg*)data)->header.brs)&&
           (tx_fifo->txbuf[i].header.dlc == ((struct rt_canx_msg*)data)->header.dlc))
            break;
    }

    //需要区分周期发送和立刻发送的情况

    if(i==tx_fifo->txrbnum-1)//立即发送
    {
        rt_hw_interrupt_enable(level);
        canx->ops->sendmsg(canx,data,32);
        return 0;
    }
    
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
        result = canx->ops->configure(canx, &(canx->config));
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
              (((dir==kOpenCanRecv)&&(cycle!=0))?(sizeof(struct rt_canx_header)+frame_len)*canx->config.tx_event_fifo_coef:0),
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
            //应该创建最长帧的倍数空间，且存储的时候，对齐存储，本质实现类似can自己的fifo
            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanRecv)&&(cycle==0))?(sizeof(struct rt_canx_header)+frame_len):0),
              rt_uint8_t rx_event_tbl[] ={CAN_MSG_MATRIX};
            #undef Y
            rx_fifo->rx_event_max_len = 0;
            for (size_t i = 0; i < sizeof(rx_event_tbl); i++)
            {
                if(rx_event_tbl[i] > rx_fifo->rx_event_max_len)
                    rx_fifo->rx_event_max_len = rx_event_tbl[i];
            }
            rx_fifo->rxrb[rx_fifo->rxrbnum-1] = rt_ringbuffer_create(rx_fifo->rx_event_max_len*canx->config.rx_event_fifo_coef);

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
              {(((dir==kOpenCanSend)&&(cycle!=0))?(sizeof(struct rt_canx_header)+frame_len)*canx->config.tx_event_fifo_coef:0),cycle},
            rt_uint8_t txbuflen_tbl[][2] ={CAN_MSG_MATRIX};//只有发送的周期帧有长度
            #undef Y

            #define Y(h_fdf,h_brs,h_ide,h_id,dir,frame_len,cycle) \
              {.id = h_id, .rtr = 0, .ide = h_ide, .fdf = h_fdf, .brs = h_brs, .dlc = BytestoDLC[frame_len]},
              struct rt_canx_header tmpheader[] = {CAN_MSG_MATRIX};
            #undef Y

            //给周期发送的创建空间
            size_t j=0;
            for (size_t i = 0; i < sizeof(txbuflen_tbl)/sizeof(txbuflen_tbl[0]); i++)
            {
                if(txbuflen_tbl[i][0] == 0)
                    continue;
                tx_fifo->txbuf[j].header=tmpheader[i];
                tx_fifo->txbuf[j].txrb = rt_ringbuffer_create(txbuflen_tbl[i][0]);
                tx_fifo->txbuf[j].cycle_ms = txbuflen_tbl[i][1];
                tx_fifo->txbuf[j].next_send_time = 0;
                j++;
            }
            //给非周期帧创建空间
            #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
              (((dir==kOpenCanSend)&&(cycle==0))?(sizeof(struct rt_canx_header)+frame_len):0),
              rt_uint8_t tx_event_tbl[] ={CAN_MSG_MATRIX};
            #undef Y
            tx_fifo->tx_event_max_len = 0;
            for (size_t i = 0; i < sizeof(tx_event_tbl); i++)
            {
                if(tx_event_tbl[i] > tx_fifo->tx_event_max_len)
                    tx_fifo->tx_event_max_len = tx_event_tbl[i];
            }
            struct rt_canx_header empty_header = {0};
            tx_fifo->txbuf[tx_fifo->txrbnum-1].header = empty_header;
            tx_fifo->txbuf[tx_fifo->txrbnum-1].txrb = rt_ringbuffer_create(tx_fifo->tx_event_max_len*canx->config.tx_event_fifo_coef);

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

static rt_err_t rt_canx_control(struct rt_device   *dev,
                                int                 cmd,
                                void               *args)
{
    struct rt_canx_device *canx;
    rt_err_t res;

    res = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    canx = (struct rt_canx_device *)dev;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;
    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;
    case RT_DEVICE_CTRL_CONFIG:
        /* configure device */
        res = canx->ops->configure(canx, (struct canx_configure *)args);
    default:
        /* control device */
        if(canx->ops->control != RT_NULL)
            res = canx->ops->control(canx, cmd, args);
        else
            res = -RT_ENOSYS;
        break;
    }
}

/**
 * @brief 用于将txfifo的数据发送出去
 * 
 * @param parameter 
 */
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

    // 获取最小的非0周期
    #define Y(fdf,brs,ide,id,dir,frame_len,cycle) \
        (((dir==kOpenCanSend)&&(cycle!=0))?cycle:0),
        rt_uint8_t cyc_tbl[] ={CAN_MSG_MATRIX};//只有发送的周期帧有长度
    #undef Y
    rt_uint8_t min_cycle = 0;
    for (size_t i = 0; i < sizeof(cyc_tbl); i++)
    {
        if (cyc_tbl[i] == 0)
            continue;
        if(min_cycle == 0)
            min_cycle = cyc_tbl[i];
        if(min_cycle > cyc_tbl[i])
            min_cycle = cyc_tbl[i];
    }

    if(min_cycle/2>=10)//周期太短，就等10ms
        min_cycle/=2;

    rt_tick_t now;

    while(1)
    {
        // //发送
        // for (size_t i = 0; i < tx_fifo->txrbnum-1; i++)//前面的周期buf发一个等一次
        // {
        //     canx->ops->sendmsg(canx,tx_fifo->txbuf[i].txrb,i);
        //     rt_completion_wait(&(tx_fifo->txbuf[i].completion), RT_WAITING_FOREVER);
        // }
        // //事件fifo放到没空间
        // rt_completion_wait(&(tx_fifo->txbuf[tx_fifo->txrbnum-1].completion), RT_WAITING_FOREVER);

        // //周期帧周期到了就发
        // //事件帧有就发，发到fifo空为止

        //事件帧直接在对应的线程内发，这个线程只发周期的

        now = rt_tick_get();
        for (size_t i = 0; i < tx_fifo->txrbnum-1; i++)//前面的周期buf发一个等一次
        {
            if ((now - tx_fifo->txbuf[i].next_send_time) < (RT_TICK_MAX / 2))
            {
                //应该在txfifo有东西的前提下才发送
                //或者如果帧为空 发默认帧
                //如果fifo为空，不发
                if(rt_ringbuffer_data_len(tx_fifo->txbuf[i].txrb) !=0)
                    canx->ops->sendmsg(canx,tx_fifo->txbuf[i].txrb,i);
                tx_fifo->txbuf[i].next_send_time += rt_tick_from_millisecond(tx_fifo->txbuf[i].cycle_ms);
            }
        }

        rt_thread_mdelay(min_cycle);//这个值也建议动态做，减少cpu占用
    }
}