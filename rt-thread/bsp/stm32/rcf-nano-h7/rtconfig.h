// /* RT-Thread config file */

// #ifndef __RTTHREAD_CFG_H__
// #define __RTTHREAD_CFG_H__

// #include <rtthread.h>

// #if defined(__CC_ARM) || defined(__CLANG_ARM)
// #include "RTE_Components.h"

// #if defined(RTE_USING_FINSH)
// #define RT_USING_FINSH
// #endif //RTE_USING_FINSH

// #elif defined(__GNUC__)

// #define RT_USING_FINSH

// #endif //(__CC_ARM) || (__CLANG_ARM)

// // <<< Use Configuration Wizard in Context Menu >>>
// // <h>Basic Configuration
// // <o>Maximal level of thread priority <8-256>
// //  <i>Default: 32
// #define RT_THREAD_PRIORITY_MAX  8
// // <o>OS tick per second
// //  <i>Default: 1000   (1ms)
// #define RT_TICK_PER_SECOND  1000
// // <o>Alignment size for CPU architecture data access
// //  <i>Default: 4
// #define RT_ALIGN_SIZE   4
// // <o>the max length of object name<2-16>
// //  <i>Default: 8
// #define RT_NAME_MAX    8
// // <c1>Using RT-Thread components initialization
// //  <i>Using RT-Thread components initialization
// #define RT_USING_COMPONENTS_INIT
// // </c>

// #define RT_USING_USER_MAIN

// // <o>the stack size of main thread<1-4086>
// //  <i>Default: 512
// #define RT_MAIN_THREAD_STACK_SIZE     256

// // </h>

// // <h>Debug Configuration
// // <c1>enable kernel debug configuration
// //  <i>Default: enable kernel debug configuration
// //#define RT_DEBUG
// // </c>
// // <o>enable components initialization debug configuration<0-1>
// //  <i>Default: 0
// #define RT_DEBUG_INIT 0
// // <c1>thread stack over flow detect
// //  <i> Diable Thread stack over flow detect
// //#define RT_USING_OVERFLOW_CHECK
// // </c>
// // </h>

// // <h>Hook Configuration
// // <c1>using hook
// //  <i>using hook
// //#define RT_USING_HOOK
// // </c>
// // <c1>using idle hook
// //  <i>using idle hook
// //#define RT_USING_IDLE_HOOK
// // </c>
// // </h>

// // <e>Software timers Configuration
// // <i> Enables user timers
// #define RT_USING_TIMER_SOFT         0
// #if RT_USING_TIMER_SOFT == 0
//     #undef RT_USING_TIMER_SOFT
// #endif
// // <o>The priority level of timer thread <0-31>
// //  <i>Default: 4
// #define RT_TIMER_THREAD_PRIO        4
// // <o>The stack size of timer thread <0-8192>
// //  <i>Default: 512
// #define RT_TIMER_THREAD_STACK_SIZE  512
// // </e>

// // <h>IPC(Inter-process communication) Configuration
// // <c1>Using Semaphore
// //  <i>Using Semaphore
// #define RT_USING_SEMAPHORE
// // </c>
// // <c1>Using Mutex
// //  <i>Using Mutex
// //#define RT_USING_MUTEX
// // </c>
// // <c1>Using Event
// //  <i>Using Event
// //#define RT_USING_EVENT
// // </c>
// // <c1>Using MailBox
// //  <i>Using MailBox
// #define RT_USING_MAILBOX
// // </c>
// // <c1>Using Message Queue
// //  <i>Using Message Queue
// //#define RT_USING_MESSAGEQUEUE
// // </c>
// // </h>

// // <h>Memory Management Configuration
// // <c1>Dynamic Heap Management
// //  <i>Dynamic Heap Management
// #define RT_USING_HEAP
// #if defined(RT_USING_HEAP)
// // #define RT_HEAP_SIZE_MAN//数组形式（手动）分配堆大小
// #define RT_HEAP_SIZE_AUTO//自动分配堆大小（将所有剩余的ram都作为堆）
// #endif
// // </c>
// // <c1>using small memory
// //  <i>using small memory
// #define RT_USING_SMALL_MEM
// // </c>
// // <c1>using tiny size of memory
// //  <i>using tiny size of memory
// //#define RT_USING_TINY_SIZE
// // </c>
// // </h>

// // <h>Console Configuration
// // <c1>Using console
// //  <i>Using console
// #define RT_USING_CONSOLE
// // </c>
// // <o>the buffer size of console <1-1024>
// //  <i>the buffer size of console
// //  <i>Default: 128  (128Byte)
// #define RT_CONSOLEBUF_SIZE          128
// // </h>

// #if defined(RT_USING_FINSH)
//     #define FINSH_USING_MSH
//     #define FINSH_USING_MSH_ONLY
//     // <h>Finsh Configuration
//     // <o>the priority of finsh thread <1-7>
//     //  <i>the priority of finsh thread
//     //  <i>Default: 6
//     #define __FINSH_THREAD_PRIORITY     5
//     #define FINSH_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 8 * __FINSH_THREAD_PRIORITY + 1)
//     // <o>the stack of finsh thread <1-4096>
//     //  <i>the stack of finsh thread
//     //  <i>Default: 4096  (4096Byte)
//     #define FINSH_THREAD_STACK_SIZE     512
//     // <o>the history lines of finsh thread <1-32>
//     //  <i>the history lines of finsh thread
//     //  <i>Default: 5
//     #define FINSH_HISTORY_LINES         1

//     #define FINSH_USING_SYMTAB
//     // </h>
// #endif

// // <<< end of configuration section >>>

// #endif


#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 16
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256

/* kservice optimization */

#define RT_DEBUG

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME ""//使用segger rtt
// #define RT_CONSOLE_DEVICE_NAME "uart1"//使用串口
#define RT_VER_NUM 0x40100
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10

#define FINSH_ECHO_DISABLE_DEFAULT//不做回显

/* Device Drivers */
#define DFS_USING_POSIX/////////
#define RT_USING_NEWLIB////

#define RT_USING_DEVICE_IPC
#define RT_USING_SERIAL
// #define RT_USING_SERIAL_V2
#define RT_SERIAL_USING_DMA
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS//位操作的意思就是模拟IIC，spi也有类似的
#define RT_USING_PIN
#define RT_USING_CAN
#define RT_USING_CANX
#define RT_USING_ADC
#define RT_USING_SPI
#define RT_USING_USB_DEVICE
#define RT_USING_SDIO
#define RT_USING_HWTIMER
#define RT_USING_PWM

#define RT_USING_DFS
#define RT_USING_DFS_ELMFAT
#define DFS_USING_WORKDIR
#define RT_DFS_ELM_USE_LFN 1
#define RT_DFS_ELM_MAX_LFN 255

#define RT_AUDIO_REPLAY_MP_BLOCK_SIZE 4096
#define RT_AUDIO_REPLAY_MP_BLOCK_COUNT 2
#define RT_AUDIO_RECORD_PIPE_SIZE 2048

/* Using USB */

#define RT_USB_DEVICE_COMPOSITE
#define RT_USB_DEVICE_CDC
#define RT_USB_DEVICE_MSTORAGE
#define RT_USB_MSTORAGE_DISK_NAME "sd0"
#define RT_USBD_THREAD_STACK_SZ (1024*5)
#define RT_VCOM_TASK_STK_SIZE (1024*2)

/* C/C++ and POSIX layer */
#define RT_USING_LIBC
#define RT_USING_POSIX

#define RT_LIBC_DEFAULT_TIMEZONE 8

/* POSIX (Portable Operating System Interface) layer */


/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */


/* Network */


/* Utilities */


/* RT-Thread Utestcases */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */


/* XML: Extensible Markup Language */


/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */


/* u8g2: a monochrome graphic library */


/* PainterEngine: A cross-platform graphics application framework written in C language */


/* tools packages */


/* system packages */

/* enhanced kernel services */


/* POSIX extension functions */


/* acceleration: Assembly language or algorithmic acceleration packages */


/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */


/* AI packages */


/* miscellaneous packages */

/* project laboratory */

/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */

#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32H7

/* Hardware Drivers Config */

#define SOC_STM32H750XB
#define BOARD_STM32H750_ARTPI

/* Board extended module */


/* Onboard Peripheral Drivers */

// #define BSP_USING_USB_TO_USART

/* On-chip Peripheral Drivers */

#define BSP_USING_UART1
#define BSP_USING_UART2
#define BSP_UART2_RX_USING_DMA
// #define BSP_USING_UART3
#define BSP_USING_UART4
#define BSP_UART4_RX_USING_DMA
#define BSP_USING_UART6
#define BSP_USING_UART8
#define BSP_USING_I2C2
#define BSP_USING_SPI1
// #define BSP_SPI1_RX_USING_DMA
// #define BSP_SPI1_TX_USING_DMA
#define BSP_USING_FDCAN1
#define BSP_USING_SDIO
#define BSP_USING_SDIO1
#define BSP_USING_USBD
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1
#define BSP_PWM1_CH1_USING_DMA
#define BSP_USING_PWM1_CH2
#define BSP_PWM1_CH2_USING_DMA
#define BSP_USING_PWM1_CH3
#define BSP_PWM1_CH3_USING_DMA
#define BSP_USING_PWM1_CH4
#define BSP_PWM1_CH4_USING_DMA
#define BSP_USING_TIM
#define BSP_USING_TIM1
#define BSP_USING_TIM2//用于定时任务
#define BSP_USING_TIM4
#define BSP_USING_TIM12
#define BSP_USING_TIM15
#define BSP_USING_PWM4
#define BSP_USING_PWM4_CH1
#define BSP_USING_PWM4_CH2
#define BSP_USING_PWM4_CH3
#define BSP_USING_PWM4_CH4
#define BSP_USING_PWM12
#define BSP_USING_PWM12_CH1
#define BSP_USING_PWM12_CH2
#define BSP_USING_PWM15
#define BSP_USING_PWM15_CH1
#define BSP_USING_PWM15_CH2
#define BSP_USING_ADC1

/* External Libraries */


#endif
