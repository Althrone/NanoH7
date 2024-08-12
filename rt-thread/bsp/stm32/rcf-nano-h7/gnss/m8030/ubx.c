/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\m8030\ubx.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "ubx.h"

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

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

typedef enum
{
    UBX_USE_UTC_TIME,
    UBX_USE_GPS_TIME,
    UBX_USE_GLONASS_TIME,//>=18
    UBX_USE_BEIDOU_TIME,//>=18
    UBX_USE_GALILEO_TIME,//>=18
    UBX_USE_NACIV_TIME,//>=29
}UbxTimeSysEnum;

/**
 * @brief   设置测量和导航频率，以及系统使用的时间系统
 * @param   measRate: 单位ms,ver<24 >50ms ver>=24 >25ms
 * @param   navRate: 5的话意思是5次测量才做一次导航，最大127，<v18=1
 */
void ubx_cfg_rate(rt_uint16_t measRate,
                  rt_uint16_t navRate,
                  UbxTimeSysEnum timeRef)
{
    // Nav. update rate2 Single GNSS up to 18 Hz
    // 2 Concurrent GNSS up to 10 Hz

    //其实多导航系统下，最快就是measRate=100ms navRate=1cycle
}
/**
 * @brief   关闭/开启报文函数
 * @note    UBX-CFG-RATE控制总速率，这个控制单个报文速率
 * @note    port_rate输入长度为6的数组，对应每个端口
 */
void ubx_cfg_msg(UbxMsgCtrlEnum ctrl,rt_bool_t* port_rate,
                 UbxMsgClassEnum msg_class,rt_uint8_t msg_id)
{

}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

void ubx_encode(rt_uint8_t msg_class,rt_uint8_t msg_id,rt_size_t size,rt_uint8_t* p_payload)
{

}