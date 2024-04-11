/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include <stm32h7xx.h>
#include "drv_common.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------- CHIP CONFIG BEGIN --------------------------*/

#define CHIP_FAMILY_STM32
#define CHIP_SERIES_STM32H7
#define CHIP_NAME_STM32H750XBHX

/*-------------------------- CHIP CONFIG END --------------------------*/

/*-------------------------- ROM/RAM CONFIG BEGIN --------------------------*/
 #define ROM_START              ((uint32_t)0x08000000)
 #define ROM_SIZE               (2*1024*1024)
 #define ROM_END                ((uint32_t)(ROM_START + ROM_SIZE))

// #define RAM_START              (0x24000000)
// #define RAM_SIZE               (512 * 1024)
// #define RAM_END                (RAM_START + RAM_SIZE)

/*-------------------------- ROM/RAM CONFIG END --------------------------*/

/*-------------------------- CLOCK CONFIG BEGIN --------------------------*/

#define BSP_CLOCK_SOURCE                  ("HSE")
#define BSP_CLOCK_SOURCE_FREQ_MHZ         ((int32_t)8)
#define BSP_CLOCK_SYSTEM_FREQ_MHZ         ((int32_t)480)

/*-------------------------- CLOCK CONFIG END --------------------------*/

/*-------------------------- UART CONFIG BEGIN --------------------------*/

/** After configuring corresponding UART or UART DMA, you can use it.
 *
 * STEP 1, define macro define related to the serial port opening based on the serial port number
 *                 such as     #define BSP_USING_UART1
 *
 * STEP 2, according to the corresponding pin of serial port, define the related serial port information macro
 *                 such as     #define BSP_UART1_TX_PIN       "PA9"
 *                             #define BSP_UART1_RX_PIN       "PA10"
 *
 * STEP 3, if you want using SERIAL DMA, you must open it in the RT-Thread Settings.
 *                 RT-Thread Setting -> Components -> Device Drivers -> Serial Device Drivers -> Enable Serial DMA Mode
 *
 * STEP 4, according to serial port number to define serial port tx/rx DMA function in the board.h file
 *                 such as     #define BSP_UART1_RX_USING_DMA
 *
 */

#define BSP_UART1_RX_BUFSIZE   256
#define BSP_UART1_TX_BUFSIZE   256
#define BSP_UART2_RX_BUFSIZE   256
#define BSP_UART2_TX_BUFSIZE   256
#define BSP_UART3_RX_BUFSIZE   256
#define BSP_UART3_TX_BUFSIZE   256
#define BSP_UART4_RX_BUFSIZE   64
#define BSP_UART4_TX_BUFSIZE   256
#define BSP_UART6_RX_BUFSIZE   256
#define BSP_UART6_TX_BUFSIZE   256
#define BSP_UART8_RX_BUFSIZE   256
#define BSP_UART8_TX_BUFSIZE   256

#define BSP_I2C2_SCL_PIN        GET_PIN(B,10)
#define BSP_I2C2_SDA_PIN        GET_PIN(B,11)

#define STM32_FLASH_START_ADRESS       ROM_START
#define STM32_FLASH_SIZE               ROM_SIZE
#define STM32_FLASH_END_ADDRESS        ROM_END

#define RAM_START              (0x24000000)
#define RAM_SIZE               (512)
#define RAM_END                (RAM_START + RAM_SIZE * 1024)

#define STM32_SRAM1_SIZE               RAM_SIZE
#define STM32_SRAM1_START              RAM_START
#define STM32_SRAM1_END                RAM_END

#if defined(__ARMCC_VERSION)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __rtt_ram_begin;//ld分配的heap和stack之后的区域
extern int _estack;//ram结束地址
#define HEAP_BEGIN      (&__rtt_ram_begin)
#define HEAP_END        (&_estack)
#endif

// #define HEAP_END        STM32_SRAM1_END

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif
