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

//自动挑选出接收带周期的帧，分配到rxbuf中，
// CanxMatrixStruct g_canx_matrix[]={
//     {"aaa",0x123,8,kCanxRxMsg,10}
// }

#define CAN_MSG_MATRIX \
    X(kOpenCanStdId,    0x083,  8,  10  )   \
    X(kOpenCanStdId,    0x0A2,  8,  20  )   \
    X(kOpenCanStdId,    0x0D3,  8,  10  )   \
    X(kOpenCanStdId,    0x0D4,  24, 10  )   \
    X(kOpenCanStdId,    0x0FE,  8,  20  )   \
    X(kOpenCanStdId,    0x101,  8,  10  )   \
    X(kOpenCanStdId,    0x102,  8,  20  )   \
    X(kOpenCanStdId,    0x119,  8,  20  )   \
    X(kOpenCanStdId,    0x11D,  8,  20  )   \
    X(kOpenCanStdId,    0x121,  8,  10  )   \
    X(kOpenCanStdId,    0x124,  8,  10  )   \
    X(kOpenCanStdId,    0x132,  8,  20  )   \
    X(kOpenCanStdId,    0x133,  8,  10  )   \
    X(kOpenCanStdId,    0x142,  8,  10  )   \
    X(kOpenCanStdId,    0x144,  8,  10  )   \
    X(kOpenCanStdId,    0x188,  8,  20  )   \
    X(kOpenCanStdId,    0x1B0,  16, 20  )   \
    X(kOpenCanStdId,    0x1FB,  8,  20  )   \
    X(kOpenCanStdId,    0x30C,  8,  100 )   \
    X(kOpenCanStdId,    0x320,  8,  100 )   \
    X(kOpenCanStdId,    0x33C,  8,  100 )   \
    X(kOpenCanStdId,    0x341,  8,  100 )   \
    X(kOpenCanStdId,    0x3DA,  8,  100 )   \
    X(kOpenCanStdId,    0x3F2,  8,  100 )   \
    X(kOpenCanStdId,    0x443,  8,  100 )   \
    X(kOpenCanStdId,    0x4DA,  8,  1000)   \
    X(kOpenCanStdId,    0x584,  8,  500 )   \
    X(kOpenCanStdId,    0x599,  8,  500 )   \
    X(kOpenCanStdId,    0x5B5,  8,  500 )   \
    X(kOpenCanStdId,    0x5B6,  8,  500 )   \
    X(kOpenCanStdId,    0x643,  16, 500 )   \
    X(kOpenCanStdId,    0x741,  8,  0   )   \
    X(kOpenCanStdId,    0x749,  8,  0   )   \
    X(kOpenCanStdId,    0x7DF,  8,  0   )

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/
