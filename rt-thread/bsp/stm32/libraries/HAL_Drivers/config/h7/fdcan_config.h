/******************************************************************************
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\libraries\HAL_Drivers\config\h7\fdcan_config.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

#ifndef __FDCAN_CONFIG_H__
#define __FDCAN_CONFIG_H__

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

#define FDCAN1_MSG_RAM_STD_FILTER_NBR 0 //0-128
#define FDCAN1_MSG_RAM_OFFSET FDCAN1_MSG_RAM_STD_FILTER_NBR*

#ifdef BSP_USING_FDCAN1
#define FDCAN1_MSG_RAM_CFG                                                     \
    {                                                                         \
        .Init.MessageRAMOffset = 0,                                           \
        .Init.StdFiltersNbr = FDCAN1_MSG_RAM_STD_FILTER_NBR,             \
        .Init.ExtFiltersNbr = 0,                                           \
    }
#endif /* BSP_USING_FDCAN1 */

#ifdef BSP_USING_FDCAN1
#ifndef FDCAN1_BUS_CONFIG
#define FDCAN1_BUS_CONFIG                                                     \
    {                                                                         \
        .Instance = FDCAN1,                                                   \
    }
#endif /* FDCAN1_BUS_CONFIG */
#endif /* BSP_USING_FDCAN1 */

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

#endif /* __FDCAN_CONFIG_H__ */