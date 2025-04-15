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

#define RT_CANX_CALC_BITTIMING

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
    rt_uint16_t sample_point_permille;
  #else
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

/* Default config for canx_configure structure */
// #define RT_CANX_CONFIG_DEFAULT          \
// {                                       \
//     {                                   \
//         1000000,                        \
//         800,                            \
//     }                                   \
// #ifdef RT_CANX_USING_FD
//     {
//         5000000,
//         800,
//     }
// #endif /* RT_CANX_USING_FD */
// }

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

typedef struct
{
    char name[16];
    rt_uint32_t id;
    rt_uint8_t len;
    OpenCanMsgDirEnum dir;
    rt_uint16_t time;
}CanxMatrixStruct;

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