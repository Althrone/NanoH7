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