/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-08     Althrone     first version
 */

#include "drv_common.h"

#include "drv_lin.h"

#ifdef RT_USING_LIN

/******************************************************************************
 * includes
 *****************************************************************************/

/******************************************************************************
 * private macros
 *****************************************************************************/

#define P1(n)  ((n>>7)&0x01)
#define P0(n)  ((n>>6)&0x01)
#define ID5(n) ((n>>5)&0x01)
#define ID4(n) ((n>>4)&0x01)
#define ID3(n) ((n>>3)&0x01)
#define ID2(n) ((n>>2)&0x01)
#define ID1(n) ((n>>1)&0x01)
#define ID0(n) ((n>>0)&0x01)

#define LIN_FRAME_TABLE \
    X(0x3C, 8, kOpenLinRecv)  \
    X(0x30, 8, kOpenLinRecv)  \
    X(0x31, 8, kOpenLinRecv)  \
    X(0x04, 8, kOpenLinRecv)  \
    X(0x33, 8, kOpenLinRecv)  \
    X(0x03, 8, kOpenLinRecv)  \
    X(0x12, 8, kOpenLinRecv)  \
    X(0x14, 8, kOpenLinRecv)  \
    X(0x15, 8, kOpenLinSend)  \
    X(0x36, 8, kOpenLinSend)  \
    X(0x1A, 8, kOpenLinSend)  \
    X(0x39, 8, kOpenLinRecv)

#define OPENLIN_ASSERT(expr) ((expr) ? (void)0U : assert_failed((rt_uint8_t *)__FILE__, __LINE__))

void assert_failed(rt_uint8_t *file, rt_uint32_t line)
{
    while(1);
}

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

const OpenLinMsgStruct g_lin_frame[] = {
    #define X(id, len, dir) {id, len, dir},
    LIN_FRAME_TABLE
    #undef X
};

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

static OpenLinIdFilterUnion aaa={
  #define X(id, len, dir) (1ULL << id) |
    .id_tbl=LIN_FRAME_TABLE 0,
  #undef X
};

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/

/**
 * @return 返回格式为0b_P1P0_00_0000
 */
static rt_uint8_t id_parity_calc(rt_uint8_t id)
{
    OPENLIN_ASSERT(id>0x3F);
    // P0 = ID0 ⊕ ID1 ⊕ ID2 ⊕ ID4 
    // P1 = ¬ (ID1 ⊕ ID3 ⊕ ID4 ⊕ ID5)

    return (((0xFF & (ID0(id) ^ ID1(id) ^ ID2(id) ^ ID4(id))) << 6)|
            ((0xFF ^ (ID1(id) ^ ID3(id) ^ ID4(id) ^ ID5(id))) << 7));

    // sizeof(OpenLinIdFilterStruct) 
}

static rt_bool_t pid_check(rt_uint8_t pid)
{
    rt_bool_t retval=RT_FALSE;
    rt_uint8_t parity=id_parity_calc(pid&0x3F);

    if((pid&0xC0)==parity)
        retval=RT_TRUE;
    
    return retval;
}

static rt_uint8_t id2pid(rt_uint8_t id)
{
    rt_uint8_t parity=id_parity_calc(id);

    return parity|id;
}

static rt_uint8_t calc_checksum(rt_uint8_t* pbuf,rt_size_t buf_len)
{
    rt_uint16_t checksum=0;

    for (rt_size_t i = 0; i < buf_len; i++)
    {
        checksum+=pbuf[i];

        if (checksum > 0xff)
        {
            checksum -= 0xff;
        }
    }

    return ~checksum;
}

#endif /* RT_USING_LIN */
