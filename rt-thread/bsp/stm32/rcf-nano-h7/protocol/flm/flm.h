/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2026 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\flm\flm.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_flm_flm_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_flm_flm_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <stdint.h>
#include "FlashOS.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

struct flm_ops
{
    /* common device interface */
    /*
     *  Initialize Flash Programming Functions
     *    Parameter:      adr:  Device Base Address
     *                    clk:  Clock Frequency (Hz)
     *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
     *    Return Value:   0 - OK,  1 - Failed
     */
    int (*Init)         (unsigned long adr,
                         unsigned long clk,
                         unsigned long fnc);
    /*
     *  De-Initialize Flash Programming Functions
     *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
     *    Return Value:   0 - OK,  1 - Failed
     */
    int (*UnInit)       (unsigned long fnc);
    int (*BlankCheck)   (unsigned long adr,
                         unsigned long sz,
                         unsigned char pat);
    /*
     *  Erase complete Flash Memory
     *    Return Value:   0 - OK,  1 - Failed
     */
    int (*EraseChip)    (void);
    /*
     *  Erase Sector in Flash Memory
     *    Parameter:      adr:  Sector Address
     *    Return Value:   0 - OK,  1 - Failed
     */
    int (*EraseSector)  (unsigned long adr);
    /*
     *  Program Page in Flash Memory
     *    Parameter:      adr:  Page Start Address
     *                    sz:   Page Size
     *                    buf:  Page Data
     *    Return Value:   0 - OK,  1 - Failed
     */
    int (*ProgramPage)  (unsigned long adr,
                         unsigned long sz,
                         unsigned char *buf);
    long(*Verify)       (unsigned long adr,
                         unsigned long sz,
                         unsigned char *buf);
    int* pPrgData;

    struct FlashDevice* pDevDscr;
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

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_flm_flm_h_ */