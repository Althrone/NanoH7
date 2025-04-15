/*
 * Copyright (c) 2006-2020
 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-08     Althrone     first version
 */

#ifndef __DRV_OPENLIN_H__
#define __DRV_OPENLIN_H__

#include <drv_common.h>

typedef union
{
    rt_uint64_t id_tbl;
    struct
    {
        rt_uint8_t ID_0x00:1;
        rt_uint8_t ID_0x01:1;
        rt_uint8_t ID_0x02:1;
        rt_uint8_t ID_0x03:1;
        rt_uint8_t ID_0x04:1;
        rt_uint8_t ID_0x05:1;
        rt_uint8_t ID_0x06:1;
        rt_uint8_t ID_0x07:1;
        rt_uint8_t ID_0x08:1;
        rt_uint8_t ID_0x09:1;
        rt_uint8_t ID_0x0A:1;
        rt_uint8_t ID_0x0B:1;
        rt_uint8_t ID_0x0C:1;
        rt_uint8_t ID_0x0D:1;
        rt_uint8_t ID_0x0E:1;
        rt_uint8_t ID_0x0F:1;
        rt_uint8_t ID_0x10:1;
        rt_uint8_t ID_0x11:1;
        rt_uint8_t ID_0x12:1;
        rt_uint8_t ID_0x13:1;
        rt_uint8_t ID_0x14:1;
        rt_uint8_t ID_0x15:1;
        rt_uint8_t ID_0x16:1;
        rt_uint8_t ID_0x17:1;
        rt_uint8_t ID_0x18:1;
        rt_uint8_t ID_0x19:1;
        rt_uint8_t ID_0x1A:1;
        rt_uint8_t ID_0x1B:1;
        rt_uint8_t ID_0x1C:1;
        rt_uint8_t ID_0x1D:1;
        rt_uint8_t ID_0x1E:1;
        rt_uint8_t ID_0x1F:1;
        rt_uint8_t ID_0x20:1;
        rt_uint8_t ID_0x21:1;
        rt_uint8_t ID_0x22:1;
        rt_uint8_t ID_0x23:1;
        rt_uint8_t ID_0x24:1;
        rt_uint8_t ID_0x25:1;
        rt_uint8_t ID_0x26:1;
        rt_uint8_t ID_0x27:1;
        rt_uint8_t ID_0x28:1;
        rt_uint8_t ID_0x29:1;
        rt_uint8_t ID_0x2A:1;
        rt_uint8_t ID_0x2B:1;
        rt_uint8_t ID_0x2C:1;
        rt_uint8_t ID_0x2D:1;
        rt_uint8_t ID_0x2E:1;
        rt_uint8_t ID_0x2F:1;
        rt_uint8_t ID_0x30:1;
        rt_uint8_t ID_0x31:1;
        rt_uint8_t ID_0x32:1;
        rt_uint8_t ID_0x33:1;
        rt_uint8_t ID_0x34:1;
        rt_uint8_t ID_0x35:1;
        rt_uint8_t ID_0x36:1;
        rt_uint8_t ID_0x37:1;
        rt_uint8_t ID_0x38:1;
        rt_uint8_t ID_0x39:1;
        rt_uint8_t ID_0x3A:1;
        rt_uint8_t ID_0x3B:1;
        rt_uint8_t ID_0x3C:1;
        rt_uint8_t ID_0x3D:1;
        rt_uint8_t ID_0x3E:1;
        rt_uint8_t ID_0x3F:1;
    }B;
}OpenLinIdFilterUnion;


typedef struct
{
    rt_uint8_t ID_0x00:1;
    rt_uint8_t ID_0x01:1;
    rt_uint8_t ID_0x02:1;
    rt_uint8_t ID_0x03:1;
    rt_uint8_t ID_0x04:1;
    rt_uint8_t ID_0x05:1;
    rt_uint8_t ID_0x06:1;
    rt_uint8_t ID_0x07:1;
    rt_uint8_t ID_0x08:1;
    rt_uint8_t ID_0x09:1;
    rt_uint8_t ID_0x0A:1;
    rt_uint8_t ID_0x0B:1;
    rt_uint8_t ID_0x0C:1;
    rt_uint8_t ID_0x0D:1;
    rt_uint8_t ID_0x0E:1;
    rt_uint8_t ID_0x0F:1;
    rt_uint8_t ID_0x10:1;
    rt_uint8_t ID_0x11:1;
    rt_uint8_t ID_0x12:1;
    rt_uint8_t ID_0x13:1;
    rt_uint8_t ID_0x14:1;
    rt_uint8_t ID_0x15:1;
    rt_uint8_t ID_0x16:1;
    rt_uint8_t ID_0x17:1;
    rt_uint8_t ID_0x18:1;
    rt_uint8_t ID_0x19:1;
    rt_uint8_t ID_0x1A:1;
    rt_uint8_t ID_0x1B:1;
    rt_uint8_t ID_0x1C:1;
    rt_uint8_t ID_0x1D:1;
    rt_uint8_t ID_0x1E:1;
    rt_uint8_t ID_0x1F:1;
    rt_uint8_t ID_0x20:1;
    rt_uint8_t ID_0x21:1;
    rt_uint8_t ID_0x22:1;
    rt_uint8_t ID_0x23:1;
    rt_uint8_t ID_0x24:1;
    rt_uint8_t ID_0x25:1;
    rt_uint8_t ID_0x26:1;
    rt_uint8_t ID_0x27:1;
    rt_uint8_t ID_0x28:1;
    rt_uint8_t ID_0x29:1;
    rt_uint8_t ID_0x2A:1;
    rt_uint8_t ID_0x2B:1;
    rt_uint8_t ID_0x2C:1;
    rt_uint8_t ID_0x2D:1;
    rt_uint8_t ID_0x2E:1;
    rt_uint8_t ID_0x2F:1;
    rt_uint8_t ID_0x30:1;
    rt_uint8_t ID_0x31:1;
    rt_uint8_t ID_0x32:1;
    rt_uint8_t ID_0x33:1;
    rt_uint8_t ID_0x34:1;
    rt_uint8_t ID_0x35:1;
    rt_uint8_t ID_0x36:1;
    rt_uint8_t ID_0x37:1;
    rt_uint8_t ID_0x38:1;
    rt_uint8_t ID_0x39:1;
    rt_uint8_t ID_0x3A:1;
    rt_uint8_t ID_0x3B:1;
    rt_uint8_t ID_0x3C:1;
    rt_uint8_t ID_0x3D:1;
    rt_uint8_t ID_0x3E:1;
    rt_uint8_t ID_0x3F:1;
}OpenLinIdFilterStruct;

typedef enum
{
    kOpenLinSend,
    kOpenLinRecv
}OpenLinMsgDirEnum;

typedef struct
{
    rt_uint8_t          id;
    rt_size_t           len;
    OpenLinMsgDirEnum   dir;
}OpenLinMsgStruct;

#endif /* __DRV_OPENLIN_H__ */


