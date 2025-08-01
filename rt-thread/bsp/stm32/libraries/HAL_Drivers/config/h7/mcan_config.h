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

    //canindx      fdf brs ide    id      send/recv   dlc cyc 
#define CAN_MSG_MATRIX \
    Y(MCAN1_INDEX,  1,  1,  1,  0x083,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x0A2,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x0D3,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x0D4,  kOpenCanRecv, 24, 10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x0FE,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x101,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x102,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x119,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x11D,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x121,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x124,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x132,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x133,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x142,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x144,  kOpenCanRecv, 8,  10  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x188,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x1B0,  kOpenCanRecv, 16, 20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x1FB,  kOpenCanRecv, 8,  20  )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x30C,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x320,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x33C,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x341,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x3DA,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x3F2,  kOpenCanRecv, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x443,  kOpenCanSend, 8,  100 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x4DA,  kOpenCanRecv, 8,  1000)   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x584,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,  0,  0,  1,  0x599,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,  0,  0,  1,  0x5B5,  kOpenCanSend, 8,  500 )   \
    Y(MCAN1_INDEX,  0,  0,  1,  0x5B6,  kOpenCanRecv, 8,  500 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x643,  kOpenCanSend, 16, 500 )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x741,  kOpenCanRecv, 8,  0   )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x749,  kOpenCanSend, 8,  0   )   \
    Y(MCAN1_INDEX,  1,  1,  1,  0x7DF,  kOpenCanRecv, 8,  0   )

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

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_libraries_HAL_Drivers_config_h7_mcan_config_h_ */