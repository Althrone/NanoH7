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
    #define RT_CANX_CONFIGDEFAULT         \
    {                                     \
        1*1000*1000,  /* 1Mbps */         \
        8000          /* 80.00% */        \
        5*1000*1000,  /* 5Mbps */         \
        8000          /* 80.00% */        \
    }
  #else
    #define RT_CANX_CONFIGDEFAULT         \
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

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CANX_H_ */