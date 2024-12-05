/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\crsf\crfs.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "rtthread.h"
#include <stm32h7xx.h>

/******************************************************************************
 * macros
 *****************************************************************************/

typedef enum
{
    // CRSF_FRAMETYPE_GPS
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED=0x16,
}crfstype;

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/
void crfs_decode(rt_uint8_t* pbuf,rt_uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_ */