/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\crsf\crfs.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "crfs.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

crsf_channels_s crsf_channels_value __attribute__((section(".test_data")));
uint16_t ch0;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint16_t ch4;
uint16_t ch5;
uint16_t ch6;
uint16_t ch7;
uint16_t ch8;
uint16_t ch9;
uint16_t ch10;
uint16_t ch11;
uint16_t ch12;
uint16_t ch13;
uint16_t ch14;
uint16_t ch15;

/******************************************************************************
 * private variables
 *****************************************************************************/

extern CRC_HandleTypeDef hcrc;

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void crfs_decode(rt_uint8_t* pbuf,rt_uint32_t size)
{
    //检查开头是不是0xC8
    if(pbuf[0]!=0xC8)
        return;

    //获取假定的长度值
    rt_uint8_t len=pbuf[1];
    rt_uint32_t crc;

    //计算CRC
    // rt_uint8_t buf[]={0x29,0xEA,0xEE,0x53,0x49,0x59,0x49,0x20,0x46,0x4D,0x33,0x30,0x00,0x45,0x4C,0x52,0x53,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x00};
    // crc=HAL_CRC_Calculate(&hcrc,buf,sizeof(buf));
    crc=HAL_CRC_Calculate(&hcrc,&pbuf[2],len-1);
    if(crc!=pbuf[len+1])
        return;

    switch ((CrfsPacketTypesEnum)pbuf[2])
    {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:

        rt_memcpy(&crsf_channels_value,&pbuf[3],sizeof(crsf_channels_value));
        ch0=crsf_channels_value.ch0;
        ch1=crsf_channels_value.ch1;
        ch2=crsf_channels_value.ch2;
        ch3=crsf_channels_value.ch3;
        ch4=crsf_channels_value.ch4;
        ch5=crsf_channels_value.ch5;
        ch6=crsf_channels_value.ch6;
        ch7=crsf_channels_value.ch7;
        ch8=crsf_channels_value.ch8;
        ch9=crsf_channels_value.ch9;
        ch10=crsf_channels_value.ch10;
        ch11=crsf_channels_value.ch11;
        ch12=crsf_channels_value.ch12;
        ch13=crsf_channels_value.ch13;
        ch14=crsf_channels_value.ch14;
        ch15=crsf_channels_value.ch15;
        // rt_kprintf("%03x %03x %03x %03x %03x %03x %03x %03x %03x %03x %03x %03x %03x %03x %03x\n\r",
        //            ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15);
        break;
    
    default:
        break;
    }
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
