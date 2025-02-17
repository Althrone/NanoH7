/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\dshot\dshot.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_PROTOCOL_DSHOT_DSHOT_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_PROTOCOL_DSHOT_DSHOT_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "rtthread.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/*
推导不同速度的dshot对应的定时器周期和占空比
以dshot150为例，代表150kbit/s，因此传输1bit耗时1/150,000≈6.67us
Dshot中1的高电平时长是0的两倍，dshot150的逻辑1高电平时间为5us，逻辑0高电平时间为2.5us
5us/6.67us=0.000005*150000=0.75=3/4=6/8，因此低电平为3/8
当前项目核心频率为600MHz（我超频了），对应的定时器时钟频率为300MHz
对于dshot1200，TIM设定的周期计数寄存器应该是300M/1200K=250
因此，逻辑1高电平计数时间为250*3/4=187.5，逻辑0 250*3/8=93.75
 */

#define DSHOT_LOGIC1    188
#define DSHOT_LOGIC0    94

#define DSHOT_BUF_LEN   16

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

void dshot_set_throttles(rt_uint16_t m1,rt_uint16_t m2,
                         rt_uint16_t m3,rt_uint16_t m4);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_dshot_dshot_h_ */