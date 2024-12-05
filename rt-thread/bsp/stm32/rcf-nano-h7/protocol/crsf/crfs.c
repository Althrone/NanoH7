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

typedef struct
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    rt_uint8_t armStatus; // optional ExpressLRS 4.0
}crsf_channels_s;

typedef struct
{
    unsigned ch0;
    unsigned ch1;
    unsigned ch2;
    unsigned ch3;
    unsigned ch4;
    unsigned ch5;
    unsigned ch6;
    unsigned ch7;
    unsigned ch8;
    unsigned ch9;
    unsigned ch10;
    unsigned ch11;
    unsigned ch12;
    unsigned ch13;
    unsigned ch14;
    unsigned ch15;
    rt_uint16_t armStatus; // optional ExpressLRS 4.0
    rt_uint8_t numSV;
    rt_int32_t lon;
    rt_int32_t lat;
    rt_int32_t height;
    rt_int32_t hMSL;
    rt_uint32_t hAcc;
    rt_uint32_t vAcc;
    rt_int32_t velN;
}crsf_cc;

crsf_channels_s channel __attribute__((section(".test_data")));

crsf_cc chh __attribute__((section(".test_data")));

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
    {
        return;
    }
    //获取假定的长度值
    rt_uint8_t len=pbuf[1];
    rt_uint32_t crc;

    switch (pbuf[2])
    {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:

        crc=HAL_CRC_Calculate(&hcrc,&pbuf[2],len-1);

        chh.ch1 = ((pbuf[3]>>0) | (pbuf[4]<<8)) & 0x07FF;
        chh.ch2 = ((pbuf[4]>>3) | (pbuf[5]<<5)) & 0x07FF;
        chh.ch3 = ((pbuf[5]>>6) | (pbuf[6]<<2) | (pbuf[7]<<10)) & 0x07FF;
        chh.ch4 = ((pbuf[7]>>1) | (pbuf[8]<<7)) & 0x07FF;
        chh.ch5 = ((pbuf[8]>>4) | (pbuf[9]<<4)) & 0x07FF;
        chh.ch6 = ((pbuf[9]>>7) | (pbuf[10]<<1)|(pbuf[11]<<9)) & 0x07FF;		
        chh.ch7 = ((pbuf[11]>>2) | (pbuf[12]<<6)) & 0x07FF;		//剩3
        chh.ch8 = ((pbuf[12]>>5) | (pbuf[13]<<3)) & 0x07FF;

        // rt_memcpy((rt_uint8_t*)&channel,(rt_uint8_t*)&(pbuf[3]),len-2);
        // chh.ch0=channel.ch0;
        // chh.ch1=channel.ch1;
        // chh.ch2=channel.ch2;
        // chh.ch3=channel.ch3;
        // chh.ch4=channel.ch4;
        // chh.ch5=channel.ch5;
        // chh.ch6=channel.ch6;
        // chh.ch7=channel.ch7;
        // chh.ch8=channel.ch8;
        // chh.ch9=channel.ch9;
        // chh.ch10=channel.ch10;
        // chh.ch11=channel.ch11;
        // chh.ch12=channel.ch12;
        // chh.ch13=channel.ch13;
        // chh.ch14=channel.ch14;
        // chh.ch15=channel.ch15;

        break;
    
    default:
        break;
    }
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_err_t crc(const rt_uint8_t* pbuf,rt_size_t size,rt_uint8_t* crc)
{

}