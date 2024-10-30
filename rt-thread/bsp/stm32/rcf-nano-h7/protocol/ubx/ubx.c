/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\m8030\ubx.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "ubx.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

static rt_err_t ubx_encode(rt_uint8_t msg_class,rt_uint8_t msg_id,rt_uint16_t size,const rt_uint8_t* p_payload,rt_uint8_t* pbuf);
static rt_err_t Fletcher8(const rt_uint8_t* pbuf,rt_size_t size,rt_uint8_t* ck_a,rt_uint8_t* ck_b);

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

UbxNavPvtStruct navpvt;

void ubx_decode(rt_uint8_t* pbuf,rt_uint32_t size)
{
    //检查开头是不是μb
    if((pbuf[0]!=0xB5)||(pbuf[1]!=0x62))
    {
        return;
    }
    //获取假定的长度值
    rt_uint16_t len=pbuf[4]|(((rt_uint16_t)pbuf[5])<<8);
    rt_uint8_t ck_a=0;
    rt_uint8_t ck_b=0;
    Fletcher8(pbuf+2,len+4,&ck_a,&ck_b);
    if((pbuf[len+6]!=ck_a)||(pbuf[len+6+1]!=ck_b))
    {
        return;
    }
    //根据id解码
    switch (pbuf[2])
    {
    case UBX_NAV_MSG:
        switch (pbuf[3])
        {
        case UBX_NAV_PVT_ID:
            rt_memcpy((rt_uint8_t*)&navpvt,(rt_uint8_t*)&(pbuf[4]),len);
            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
}

typedef enum
{
    UBX_USE_UTC_TIME,
    UBX_USE_GPS_TIME,
    UBX_USE_GLONASS_TIME,//>=18
    UBX_USE_BEIDOU_TIME,//>=18
    UBX_USE_GALILEO_TIME,//>=18
    UBX_USE_NACIV_TIME,//>=29
}UbxTimeSysEnum;

/**
 * @brief   获取协议版本号
 * @note    单纯发送请求，接收版本号在另一个函数内
 */
void ubx_request_version(void)
{
    //0xB5 0x62 0x0A 0x04 0 see below CK_A CK_B

    rt_uint8_t send_buf[8];
    ubx_encode(UBX_MON_MSG,UBX_MON_VER_ID,0,RT_NULL,send_buf);


}

void ubx_decode_version(rt_uint8_t* pbuf,rt_size_t buf_len)
{
    // #include<stdio.h>
    // #include <strings.h>
    // strnicmp("aaa","aaa",3);
    // #include<stdlib.h>
    // rt_uint8_t a= strncasecmp("aaa","aaa",3);

    char protver_str[5]={0};//格式为xx.yy->xxyy+结束符
    rt_uint16_t protver;//xx.yy->xxyy

    //查找协议版本号
    rt_size_t list_num=(buf_len-40)/30;//算出extern的行数
    for (rt_size_t i = 0; i < list_num; i++)
    {
        if(strncasecmp(pbuf+40+i*30,"PROTVER",7)==0)//匹配
        {
            strncpy(protver_str,pbuf+40+i*30+7+1,2);//xx
            strncpy(protver_str+2,pbuf+40+i*30+7+1+2+1,2);//yy
            protver=atoi(protver_str);
        }
    }
}

/**
 * @brief   设置测量和导航频率，以及系统使用的时间系统
 * @param   measRate: 单位ms,ver<24 >50ms ver>=24 >25ms
 * @param   navRate: 5的话意思是5次测量才做一次导航，最大127，<v18=1
 */
void ubx_cfg_rate(rt_uint16_t measRate,
                  rt_uint16_t navRate,
                  UbxTimeSysEnum timeRef)
{
    // Nav. update rate2 Single GNSS up to 18 Hz
    // 2 Concurrent GNSS up to 10 Hz

    //其实多导航系统下，最快就是measRate=100ms navRate=1cycle
}
/**
 * @brief   关闭/开启报文函数
 * @note    UBX-CFG-RATE控制总速率，这个控制单个报文速率
 * @note    port_rate输入长度为6的数组，对应每个端口
 */
void ubx_cfg_msg(UbxMsgCtrlEnum ctrl,rt_bool_t* port_rate,
                 UbxMsgClassEnum msg_class,rt_uint8_t msg_id)
{

}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_err_t ubx_encode(rt_uint8_t msg_class,rt_uint8_t msg_id,rt_uint16_t size,const rt_uint8_t* p_payload,rt_uint8_t* pbuf)
{
    pbuf[0]=0xB5;
    pbuf[1]=0x62;
    pbuf[2]=msg_class;
    pbuf[3]=msg_id;
    memcpy(pbuf+4,&size,2);
    memcpy(pbuf+4+2,p_payload,size);
    rt_uint8_t ck_a=0;
    rt_uint8_t ck_b=0;
    Fletcher8(2+2+size,p_payload+2,&ck_a,&ck_b);
    pbuf[4+2+size]=ck_a;
    pbuf[4+2+size+1]=ck_b;
}


static rt_err_t Fletcher8(const rt_uint8_t* pbuf,rt_size_t size,rt_uint8_t* ck_a,rt_uint8_t* ck_b)
{
    rt_uint8_t CK_A = 0;
    rt_uint8_t CK_B = 0;

    for(rt_uint16_t i=0;i<size;++i)
    {
        CK_A = CK_A + pbuf[i];
        CK_B = CK_B + CK_A;
    }

    *ck_a=CK_A;
    *ck_b=CK_B;

    return RT_EOK;
    


    // //CK_A和CK_B等于串口数据的最后两项的情况下就正确
    // if((CK_A==p_payload[size-2])&&((CK_B==p_payload[size-1])))
    //     return RT_EOK;
    // else
    //     return RT_ERROR;
}