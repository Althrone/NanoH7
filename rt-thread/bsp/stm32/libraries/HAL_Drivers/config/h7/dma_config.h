/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-02     zylx         first version
 * 2019-01-08     SummerGift   clean up the code
 * 2020-05-02     whj4674672   support stm32h7 dma1 and dma2
 */

#ifndef __DMA_CONFIG_H__
#define __DMA_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DMA1 stream0 */
#if defined(BSP_PWM1_CH1_USING_DMA) && !defined(PWM1_CH1_DMA_INSTANCE)
#define PWM1_CH1_DMA_IRQHandler          DMA1_Stream0_IRQHandler
#define PWM1_CH1_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define PWM1_CH1_DMA_INSTANCE            DMA1_Stream0
#define PWM1_CH1_DMA_REQUEST             DMA_REQUEST_TIM1_CH1
#define PWM1_CH1_DMA_IRQ                 DMA1_Stream0_IRQn
#endif

/* DMA1 stream1 */
#if defined(BSP_PWM1_CH2_USING_DMA) && !defined(PWM1_CH2_DMA_INSTANCE)
#define PWM1_CH2_DMA_IRQHandler          DMA1_Stream1_IRQHandler
#define PWM1_CH2_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define PWM1_CH2_DMA_INSTANCE            DMA1_Stream1
#define PWM1_CH2_DMA_REQUEST             DMA_REQUEST_TIM1_CH2
#define PWM1_CH2_DMA_IRQ                 DMA1_Stream1_IRQn
#endif

/* DMA1 stream2 */
#if defined(BSP_PWM1_CH3_USING_DMA) && !defined(PWM1_CH3_DMA_INSTANCE)
#define PWM1_CH3_DMA_IRQHandler          DMA1_Stream2_IRQHandler
#define PWM1_CH3_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define PWM1_CH3_DMA_INSTANCE            DMA1_Stream2
#define PWM1_CH3_DMA_REQUEST             DMA_REQUEST_TIM1_CH3
#define PWM1_CH3_DMA_IRQ                 DMA1_Stream2_IRQn
#endif

/* DMA1 stream3 */
#if defined(BSP_PWM1_CH4_USING_DMA) && !defined(PWM1_CH4_DMA_INSTANCE)
#define PWM1_CH4_DMA_IRQHandler          DMA1_Stream3_IRQHandler
#define PWM1_CH4_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define PWM1_CH4_DMA_INSTANCE            DMA1_Stream3
#define PWM1_CH4_DMA_REQUEST             DMA_REQUEST_TIM1_CH4
#define PWM1_CH4_DMA_IRQ                 DMA1_Stream3_IRQn
#endif

/* DMA1 stream4 */
#if defined(BSP_UART2_RX_USING_DMA) && !defined(UART2_RX_DMA_INSTANCE)
#define UART2_DMA_RX_IRQHandler          DMA1_Stream4_IRQHandler
#define UART2_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART2_RX_DMA_INSTANCE            DMA1_Stream4
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_USART2_RX
#define UART2_RX_DMA_IRQ                 DMA1_Stream4_IRQn
#endif

/* DMA1 stream5 */
#if defined(BSP_UART4_RX_USING_DMA) && !defined(UART4_RX_DMA_INSTANCE)
#define UART4_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler
#define UART4_RX_DMA_RCC                 RCC_AHB1ENR_DMA1EN
#define UART4_RX_DMA_INSTANCE            DMA1_Stream5
#define UART4_RX_DMA_REQUEST             DMA_REQUEST_UART4_RX
#define UART4_RX_DMA_IRQ                 DMA1_Stream5_IRQn
#endif

/* DMA1 stream6 */
#if defined(BSP_SPI5_TX_USING_DMA) && !defined(SPI5_TX_DMA_INSTANCE)
#define SPI5_DMA_TX_IRQHandler           DMA1_Stream6_IRQHandler
#define SPI5_TX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI5_TX_DMA_INSTANCE             DMA1_Stream6
#define SPI5_TX_DMA_IRQ                  DMA1_Stream6_IRQn
#endif

/* DMA1 stream7 */
#if defined(BSP_SPI3_TX_USING_DMA) && !defined(SPI3_TX_DMA_INSTANCE)
#define SPI3_DMA_TX_IRQHandler           DMA1_Stream7_IRQHandler
#define SPI3_TX_DMA_RCC                  RCC_AHB1ENR_DMA1EN
#define SPI3_TX_DMA_INSTANCE             DMA1_Stream7
#define SPI3_TX_DMA_IRQ                  DMA1_Stream7_IRQn
#endif

/* DMA2 stream0 */
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE)
#define SPI1_DMA_RX_IRQHandler           DMA2_Stream0_IRQHandler
#define SPI1_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI1_RX_DMA_INSTANCE             DMA2_Stream0
#define SPI1_RX_DMA_IRQ                  DMA2_Stream0_IRQn
#endif

/* DMA2 stream1 */
#if defined(BSP_SPI4_TX_USING_DMA) && !defined(SPI4_TX_DMA_INSTANCE)
#define SPI4_DMA_TX_IRQHandler           DMA2_Stream1_IRQHandler
#define SPI4_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI4_TX_DMA_INSTANCE             DMA2_Stream1
#define SPI4_TX_DMA_IRQ                  DMA2_Stream1_IRQn
#endif

/* DMA2 stream2 */
#if defined(BSP_SPI1_RX_USING_DMA) && !defined(SPI1_RX_DMA_INSTANCE)
#define SPI1_DMA_RX_IRQHandler           DMA2_Stream2_IRQHandler
#define SPI1_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI1_RX_DMA_INSTANCE             DMA2_Stream2
#define SPI1_RX_DMA_IRQ                  DMA2_Stream2_IRQn
#endif

/* DMA2 stream3 */
#if defined(BSP_SPI5_RX_USING_DMA) && !defined(SPI5_RX_DMA_INSTANCE)
#define SPI5_DMA_RX_IRQHandler           DMA2_Stream3_IRQHandler
#define SPI5_RX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI5_RX_DMA_INSTANCE             DMA2_Stream3
#define SPI5_RX_DMA_IRQ                  DMA2_Stream3_IRQn
#endif

/* DMA2 stream4 */
#if defined(BSP_SPI5_TX_USING_DMA) && !defined(SPI5_TX_DMA_INSTANCE)
#define SPI5_DMA_TX_IRQHandler           DMA2_Stream4_IRQHandler
#define SPI5_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI5_TX_DMA_INSTANCE             DMA2_Stream4
#define SPI5_TX_DMA_IRQ                  DMA2_Stream4_IRQn
#endif

/* DMA2 stream5 */
#if defined(BSP_SPI1_TX_USING_DMA) && !defined(SPI1_TX_DMA_INSTANCE)
#define SPI1_DMA_TX_IRQHandler           DMA2_Stream5_IRQHandler
#define SPI1_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI1_TX_DMA_INSTANCE             DMA2_Stream5
#define SPI1_TX_DMA_IRQ                  DMA2_Stream5_IRQn
#endif

/* DMA2 stream6 */
#if defined(BSP_SPI5_TX_USING_DMA) && !defined(SPI5_TX_DMA_INSTANCE)
#define SPI5_DMA_TX_IRQHandler           DMA2_Stream6_IRQHandler
#define SPI5_TX_DMA_RCC                  RCC_AHB1ENR_DMA2EN
#define SPI5_TX_DMA_INSTANCE             DMA2_Stream6
#define SPI5_TX_DMA_IRQ                  DMA2_Stream6_IRQn
#endif

/* DMA2 stream7 */
#if defined(BSP_QSPI_USING_DMA) && !defined(QSPI_DMA_INSTANCE)
#define QSPI_DMA_IRQHandler              DMA2_Stream7_IRQHandler
#define QSPI_DMA_RCC                     RCC_AHB1ENR_DMA2EN
#define QSPI_DMA_INSTANCE                DMA2_Stream7
#define QSPI_DMA_IRQ                     DMA2_Stream7_IRQn
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DMA_CONFIG_H__ */
