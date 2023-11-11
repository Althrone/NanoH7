/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-11     zylx         first version
 */

#ifndef __TIM_CONFIG_H__
#define __TIM_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TIM_DEV_INFO_CONFIG
#define TIM_DEV_INFO_CONFIG                     \
    {                                           \
        .maxfreq = 1000000,                     \
        .minfreq = 3000,                        \
        .maxcnt  = 0xFFFF,                      \
        .cntmode = HWTIMER_CNTMODE_UP,          \
    }
#endif /* TIM_DEV_INFO_CONFIG */

#ifdef BSP_USING_TIM1
#ifndef TIM1_CONFIG
#define TIM1_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM1,                    \
       .tim_irqn                = TIM1_UP_IRQn,  \
       .name                    = "timer1",                \
    }
#endif /* TIM1_CONFIG */
#endif /* BSP_USING_TIM1 */

#ifdef BSP_USING_TIM4
#ifndef TIM4_CONFIG
#define TIM4_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM4,                    \
       .tim_irqn                = TIM4_IRQn,  \
       .name                    = "timer4",                \
    }
#endif /* TIM4_CONFIG */
#endif /* BSP_USING_TIM4 */

#ifdef BSP_USING_TIM11
#ifndef TIM11_CONFIG
#define TIM11_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM11,                    \
       .tim_irqn                = TIM1_TRG_COM_TIM11_IRQn,  \
       .name                    = "timer11",                \
    }
#endif /* TIM11_CONFIG */
#endif /* BSP_USING_TIM11 */

#ifdef BSP_USING_TIM12
#ifndef TIM12_CONFIG
#define TIM12_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM12,                    \
       .tim_irqn                = TIM8_BRK_TIM12_IRQn,       \
       .name                    = "timer12",                \
    }
#endif /* TIM12_CONFIG */
#endif /* BSP_USING_TIM12 */

#ifdef BSP_USING_TIM13
#ifndef TIM13_CONFIG
#define TIM13_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM13,                    \
       .tim_irqn                = TIM8_UP_TIM13_IRQn,       \
       .name                    = "timer13",                \
    }
#endif /* TIM13_CONFIG */
#endif /* BSP_USING_TIM13 */

#ifdef BSP_USING_TIM14
#ifndef TIM14_CONFIG
#define TIM14_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM14,                    \
       .tim_irqn                = TIM8_TRG_COM_TIM14_IRQn,  \
       .name                    = "timer14",                \
    }
#endif /* TIM14_CONFIG */
#endif /* BSP_USING_TIM14 */

#ifdef BSP_USING_TIM15
#ifndef TIM15_CONFIG
#define TIM15_CONFIG                                        \
    {                                                       \
       .tim_handle.Instance     = TIM15,                    \
       .tim_irqn                = TIM15_IRQn,  \
       .name                    = "timer15",                \
    }
#endif /* TIM15_CONFIG */
#endif /* BSP_USING_TIM15 */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_CONFIG_H__ */
