/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\libraries\HAL_Drivers\config\h7\mcan_config.h
 * 
 * ref: Specification of <mcan_users_manual_v331>
 *****************************************************************************/

#ifndef NANOH7_rt_thread_bsp_stm32_libraries_HAL_Drivers_config_h7_mcan_config_h_
#define NANOH7_rt_thread_bsp_stm32_libraries_HAL_Drivers_config_h7_mcan_config_h_

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

 //自动挑选出接收带周期的帧，分配到rxbuf中
    //canindex指报文所属的can外设，从1开始，0表示这个报文无对应外设，弃用报文都设成这个
    //canindx idtype  id  send/recv   buf/fifo    dlc cyc 
#define CAN_MSG_MATRIX \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x083,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x0A2,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x0D3,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x0D4,  kOpenCanRecv, 24, 10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x0FE,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x101,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x102,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x119,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x11D,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x121,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x124,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x132,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x133,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x142,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x144,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x188,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x1B0,  kOpenCanRecv, 16, 20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x1FB,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x30C,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x320,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x33C,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x341,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x3DA,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x3F2,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x443,  kOpenCanSend, 8,  100 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x4DA,  kOpenCanRecv, 8,  1000)   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x584,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x599,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x5B5,  kOpenCanSend, 8,  500 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x5B6,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x643,  kOpenCanSend, 16, 500 )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x741,  kOpenCanRecv, 8,  0   )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x749,  kOpenCanSend, 8,  0   )   \
    Y(MCAN1_INDEX,    kOpenCanStdId,  0x7DF,  kOpenCanRecv, 8,  0   )



#if defined(BSP_USING_MCAN1)
#ifndef MCAN1_CONFIG
#define MCAN1_CONFIG                                                \
    {                                                               \
        .name = "mcan1",                                            \
        .Instance = FDCAN1,                                         \
        .irq0_type = FDCAN1_IT0_IRQn,                               \
        .irq1_type = FDCAN1_IT1_IRQn,                               \
        .tx_pin_name = BSP_MCAN1_TX_PIN,                            \
        .rx_pin_name = BSP_MCAN1_RX_PIN,                            \
    }
#endif /* MCAN1_CONFIG */
#endif /* BSP_USING_MCAN1 */

#if defined(BSP_USING_MCAN2)
#ifndef MCAN2_CONFIG
#define MCAN2_CONFIG                                                \
    {                                                               \
        .name = "mcan2",                                            \
        .Instance = FDCAN2,                                         \
        .irq0_type = FDCAN2_IT0_IRQn,                               \
        .irq1_type = FDCAN2_IT1_IRQn,                               \
        .tx_pin_name = BSP_MCAN2_TX_PIN,                            \
        .rx_pin_name = BSP_MCAN2_RX_PIN,                            \
    }
#endif /* MCAN2_CONFIG */
#endif /* BSP_USING_MCAN2 */

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/
void _mcan_msg_ram_auto_cfg(void);
uint8_t _mcan_test(void);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_libraries_HAL_Drivers_config_h7_mcan_config_h_ */