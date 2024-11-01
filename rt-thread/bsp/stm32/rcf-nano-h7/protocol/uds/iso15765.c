/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso15765.c
 * 
 * ref: Specification of <ISO 15765-2:2016>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "iso15765.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

typedef enum
{
    // ERROR_ADDR,
	kNormalAddrFormat,      //标准地址，地址信息不占用can数据帧域
	kNormalFixedAddrFormat, //标准混合，29位id用，地址信息不占用can数据帧域
	kExtendedAddrFormat,    //扩展地址，N_TA（目标地址）放在数据帧域第一字节
    kMixedAddrFormat,       //混合地址，N_AE(地址扩展)放在数据帧域第一字节
	// MIXED_29BIT,//29位混合地址，N_AE(地址扩展)放在数据帧域第一字节
	// MIXED_11BIT,//11位混合地址，N_AE(地址扩展)放在数据帧域第一字节
}AddrFormatEnum;//tp层地址格式枚举

typedef struct
{
    rt_uint32_t diag_phy_req_addr;  //诊断物理请求报文地址
    rt_uint32_t diag_func_req_addr; //诊断功能请求报文地址
    rt_uint32_t diag_resp_addr;     //诊断响应报文地址
    AddrFormatEnum addr_format;   //地址格式
}DiagAddrStruct;

typedef struct
{
    rt_size_t num_of_group;
    DiagAddrStruct group[]; 
}DiagAddrGroupTblStruct;

DiagAddrGroupTblStruct g_uds_addr_tbl={
    2,
    {
        //phy  fun   resp
        {0x700  ,0x7df  ,0x701  ,kNormalAddrFormat},//zcanpro默认配置
        {3      ,4      ,5      ,kNormalFixedAddrFormat},//测试用的
    }
};



/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void test(rt_uint32_t id,rt_bool_t ide,rt_uint8_t dlc)
{
    //ide==1，id按29位解析
    //dlc 0~F 分别表示 0 1 2 3 4 5 6 7 8 12 16 20 24 32 48 64
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_int32_t _is_diag_addr(rt_uint32_t id)
{
    rt_int32_t tbl_index=-1;
    
    for (rt_size_t i = 0;; i < g_uds_addr_tbl.num_of_group; i++)//从表中查找
    {
        if((g_uds_addr_tbl.group[i].diag_phy_req_addr==id)||
           (g_uds_addr_tbl.group[i].diag_func_req_addr==id))
        {//如果id等于表中的功能地址或者物理地址就返回对应的index
            tbl_index=i;
            break;
        }
    }

    return tbl_index;
}

void NormalAddrFrame2Buf(void)
{
    
}

/**
 * @brief   
 * @note    详见10.3.1 Addressing formats
 * @note    目前仅实现解析出Normal addressing
 **/
static void AddrForamtAnalysis(rt_uint32_t id,rt_bool_t ide)
{

    rt_int32_t index=_is_diag_addr(id);
    if(index==-1)
    {
        goto _exit;
    }

    switch (g_uds_addr_tbl.group[index].addr_format)
    {
    case kNormalAddrFormat:
        /* code */
        break;
    case kNormalFixedAddrFormat:
        /* code */
        break;
    case kExtendedAddrFormat:
        /* code */
        break;
    case kMixedAddrFormat:
        /* code */
        break;
    default://不可能进入
        break;
    }

  _exit:
    // return ret;
}