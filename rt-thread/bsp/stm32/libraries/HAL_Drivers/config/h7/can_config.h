/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\libraries\HAL_Drivers\config\h7\can_config.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

#ifndef __CAN_CONFIG_H__
#define __CAN_CONFIG_H__

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

#if defined(BSP_USING_CAN1)
#ifndef CAN1_CONFIG
#define CAN1_CONFIG                                                 \
    {                                                               \
        .name = "can1",                                             \
        .Instance = FDCAN1,                                         \
        .irq0_type = FDCAN1_IT0_IRQn,                               \
        .irq1_type = FDCAN1_IT1_IRQn,                               \
        .tx_pin_name = BSP_CAN1_TX_PIN,                             \
        .rx_pin_name = BSP_CAN1_RX_PIN,                             \
    }
#endif /* CAN1_CONFIG */
#endif /* BSP_USING_CAN1 */

#if defined(BSP_USING_CAN2)
#ifndef CAN2_CONFIG
#define CAN2_CONFIG                                                 \
    {                                                               \
        .name = "can2",                                             \
        .Instance = FDCAN2,                                         \
        .irq0_type = FDCAN2_IT0_IRQn,                               \
        .irq1_type = FDCAN2_IT1_IRQn,                               \
        .tx_pin_name = BSP_CAN2_TX_PIN,                             \
        .rx_pin_name = BSP_CAN2_RX_PIN,                             \
    }
#endif /* CAN2_CONFIG */
#endif /* BSP_USING_CAN2 */

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

#endif /* __CAN_CONFIG_H__ */