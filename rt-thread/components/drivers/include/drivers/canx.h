/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\components\drivers\include\drivers\canx.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

#ifndef CANX_H_
#define CANX_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <rtthread.h>

/******************************************************************************
 * macros
 *****************************************************************************/

#ifdef RT_CANX_USING_FD
  #ifdef RT_CANX_CALC_BITTIMING
    #define RT_CANX_CONFIG_DEFAULT         \
    {                                     \
        {1*1000*1000,  /* 1Mbps */         \
        8000},          /* 80.00% */        \
        {5*1000*1000,  /* 5Mbps */         \
        8000},          /* 80.00% */        \
        RT_TRUE,\
        1\
    }
  #else
    #define RT_CANX_CONFIG_DEFAULT         \
    {                                     \
        1,  /* 1Mbps */         \
        2          /* 80.00% */        \
        3,  /* 5Mbps */         \
        4          /* 80.00% */        \
        1,  /* 1Mbps */         \
        2          /* 80.00% */        \
        3,  /* 5Mbps */         \
        4          /* 80.00% */        \
    }
  #endif /* RT_CANX_CALC_BITTIMING */
#else /* Classic CAN */

#endif /* RT_CANX_USING_FD */



typedef enum
{
    RX_CANX_RES_DISABLE,//关闭终端电阻
    RX_CANX_RES_ENABLE, //使能终端电阻
    RX_CANX_RES_AUTO,   //自动判断是否使用终端电阻
    RX_CANX_RES_UNSPTR, //不支持终端电阻控制
}canx_res_config;

typedef struct
{
  #ifdef RT_CANX_CALC_BITTIMING
    rt_uint32_t baud_rate;//波特率误差±1%以内，优先满足
    rt_uint16_t sample_point;//万分数，比如85.00%输入8500
  #else
    //canx定义的四个参数为直接计算参数，对应寄存器是否需要-1要自行实现
    rt_uint16_t brp;
    rt_uint16_t sjw;
    rt_uint16_t seg1;
    rt_uint16_t seg2;
  #endif /* RT_CANX_CALC_BITTIMING */
}canx_baud;

struct canx_configure
{
    canx_baud nominal_baud;
  #ifdef RT_CANX_USING_FD
    canx_baud data_baud;
    rt_bool_t is_iso;
    // rt_bool_t 延迟补偿
  #endif /* RT_CANX_USING_FD */
    canx_res_config termination_resistor;
};

typedef enum
{
  kOpenCanClassic,
  kOpenCanFd,
}OpenCanFrameFmt;

typedef enum
{
  kOpenCanSend,
  kOpenCanRecv,
}OpenCanMsgDirEnum;

typedef enum
{
    kOpenCanStdId,
    kOpenCanExtId,
}OpenCanIdType;

struct rt_canx_device
{
    struct rt_device  parent;
    const struct rt_canx_ops *ops;
    struct canx_configure config;
};
typedef struct rt_canx_device* rt_canx_t;

struct rt_canx_msg
{
    rt_uint32_t id  : 29;
    rt_uint32_t rtr:1;//远程帧标志，canfd取消
    rt_uint32_t ide:1;//扩展id标志
    rt_uint32_t fdf:1;//FD帧格式标志

    rt_uint32_t brs:1;//bit rate switch
    rt_uint32_t esi:1;//fd使用，一个错误标志
    rt_uint32_t dlc:4;
    //user data
    rt_uint32_t upd:1;//指示数据已经更新，用户读后清除
    rt_uint32_t ovr:1;//用户没有读数据，中断直接重写buf了

    rt_uint32_t :24;//可以用来放user data
    rt_uint8_t data[64];
};

struct rt_canx_ops
{
    rt_err_t (*configure)(struct rt_canx_device *can, struct can_configure *cfg);
    rt_err_t (*control)(struct rt_canx_device *can, int cmd, void *arg);
    int (*sendmsg)(struct rt_canx_device *can, const void *buf, rt_uint32_t boxno);
    int (*recvmsg)(struct rt_canx_device *can, void *buf, rt_uint32_t boxno);
};

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    //收发相关事件
    RT_CANX_EVENT_RX_IND,
    RT_CANX_EVENT_TX_DONE,
    RT_CANX_EVENT_TX_FAIL,
    //CAN错误状态机相关事件
    RT_CANX_EVENT_ERROR_WARNING,
    RT_CANX_EVENT_ERROR_PASSIVE,
    RT_CANX_EVENT_BUS_OFF,
}RtCanxEvent;

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

rt_err_t rt_hw_canx_register(struct rt_canx_device * canx, 
                             const char *name, 
                             const struct rt_canx_ops *ops, 
                             const void *user_data);
void rt_hw_canx_isr(struct rt_canx_device *can, rt_uint64_t event);

#ifdef __cplusplus
}
#endif

#endif /* CANX_H_ */