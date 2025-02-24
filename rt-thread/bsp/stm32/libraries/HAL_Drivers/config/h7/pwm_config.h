/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-13     zylx         first version
 * 2022-04-14     Miaowulue    add PWM1
 */

#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_PWM1
#ifndef PWM1_CONFIG
#define PWM1_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM1,         \
       .name                    = "pwm1",       \
       .channel                 = 0             \
    }
#endif /* PWM1_CONFIG */
#endif /* BSP_USING_PWM1 */

#if defined(BSP_PWM1_CH2_USING_DMA)
#ifndef PWM1_CH2_DMA_CONFIG
#define PWM1_CH2_DMA_CONFIG                                         \
    {                                                               \
        .Instance = PWM1_CH2_DMA_INSTANCE,                          \
        .request = PWM1_CH2_DMA_REQUEST,                            \
        .dma_rcc = PWM1_CH2_DMA_RCC,                                \
        .dma_irq = PWM1_CH2_DMA_IRQ,                                \
    }
#endif /* PWM1_CH2_DMA_CONFIG */
#endif /* BSP_PWM1_CH2_USING_DMA */

#ifdef BSP_USING_PWM2
#ifndef PWM2_CONFIG
#define PWM2_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM2,         \
       .name                    = "pwm2",       \
       .channel                 = 0             \
    }
#endif /* PWM2_CONFIG */
#endif /* BSP_USING_PWM2 */

#ifdef BSP_USING_PWM3
#ifndef PWM3_CONFIG
#define PWM3_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM3,         \
       .name                    = "pwm3",       \
       .channel                 = 0             \
    }
#endif /* PWM3_CONFIG */
#endif /* BSP_USING_PWM3 */

#ifdef BSP_USING_PWM4
#ifndef PWM4_CONFIG
#define PWM4_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM4,         \
       .name                    = "pwm4",       \
       .channel                 = 0             \
    }
#endif /* PWM4_CONFIG */
#endif /* BSP_USING_PWM4 */

#ifdef BSP_USING_PWM5
#ifndef PWM5_CONFIG
#define PWM5_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM5,         \
       .name                    = "pwm5",       \
       .channel                 = 0             \
    }
#endif /* PWM5_CONFIG */
#endif /* BSP_USING_PWM5 */

#ifdef BSP_USING_PWM12
#ifndef PWM12_CONFIG
#define PWM12_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM12,         \
       .name                    = "pwm12",       \
       .channel                 = 0             \
    }
#endif /* PWM12_CONFIG */
#endif /* BSP_USING_PWM12 */

#ifdef BSP_USING_PWM15
#ifndef PWM15_CONFIG
#define PWM15_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM15,         \
       .name                    = "pwm15",       \
       .channel                 = 0             \
    }
#endif /* PWM15_CONFIG */
#endif /* BSP_USING_PWM15 */

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
