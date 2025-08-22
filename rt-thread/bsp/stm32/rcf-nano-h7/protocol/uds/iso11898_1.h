/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso11898_1.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso11898_1_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso11898_1_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    kComplete,
    kNot_Complete,
    kAborted,
}Transfer_StatusEnum;

struct L_DataOps
{
    void (*request)(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data);
    void (*confirm)(uint32_t Identifier, Transfer_StatusEnum Transfer_Status);
    void (*indication)(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data);
};

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso11898_1_h_ */