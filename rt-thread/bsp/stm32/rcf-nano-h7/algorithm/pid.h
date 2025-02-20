/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\algorithm\pid.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_algorithm_pid_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_algorithm_pid_h_

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

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef struct
{
    double p;
    double i;
    double d;

    double ek;
    double ek_1;
    double ek_2;

    double out;
    double outmax;
    double outmin;
}PidParamStruct;

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

void pid_inc(PidParamStruct* pid,double e);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_algorithm_pid_h_ */