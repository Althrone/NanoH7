/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso15765_2_cfg.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "iso15765_2_cfg.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

uint8_t gs_phy_tp2session_buf[0xFFF]={0};//暂且设置为非转义ff_dl的最大值
uint8_t gs_func_tp2session_buf[64-2]={0};//暂且设置为最长fdcan帧减去单帧N_PCI bytes的长度
uint8_t gs_resp_session2tp_buf[0xFFF]={0};//暂且设置为非转义ff_dl的最大值

struct rt_ringbuffer* gs_phy_datalink2tp_rb=NULL;
struct rt_ringbuffer* gs_func_datalink2tp_rb=NULL;
struct rt_ringbuffer* gs_resp_tp2datalink_rb=NULL;

N_SDU g_n_sdu_tbl[]={
    {//物理地址 can接收的
        //地址相关配置参数
        .can_id=0x700,
        .is_extended=false,
        .Mtype = kDiagnostics,//无N_AE
        .N_AI = {
            .N_TAtype = kPhyCanBaseFmt,//限定为11位classical can 物理地址
        },
        //与上层交换数据的缓冲区的配置
        .Length=NULL,//根据SF_DL FF_DL变化
        .MessageData=gs_phy_tp2session_buf,
        //客户端发送流控帧的，两个参数作为配置，配置完不能改
        .STmin = 10,//10ms
        .BS = 8,//8帧

        .role=kSduClient,
        .dir=kSduRecv,
        .is_padding=true,
        .padding_val=0xCC,//发出的帧的填充值
        .RX_DL = 8,//默认为8，获取首帧的时候根据首帧长度改变
        .N_WFTmax=4,

        //一些状态参数
        .FlowStatus=kFsCTS,
        .SequenceNumber=0,
        .l_recv_ind=false,

        .sdu_index=0,
        .BS_cnt=0,
        .N_WFTcnt=0,

        .N_As_timing_enable=false,
        .N_Ar_timing_enable=false,
        .N_Bs_timing_enable=false,
        .N_Br_timing_enable=false,
        .N_Cs_timing_enable=false,
        .N_Cr_timing_enable=false,
        .N_As=0,
        .N_Ar=0,
        .N_Bs=0,
        .N_Br=0,
        .N_Cs=0,
        .N_Cr=0,

        .datalink_rb=gs_phy_datalink2tp_rb,
    },
    {//功能地址 can接收的
        //地址相关配置参数
        .can_id=0x7df,
        .is_extended=false,
        .Mtype = kDiagnostics,//无N_AE
        .N_AI = {
            .N_TAtype = kFuncCanBaseFmt,//限定为11位classical can 功能地址
        },
        //与上层交换数据的缓冲区的配置
        .Length=NULL,//根据SF_DL FF_DL变化
        .MessageData=gs_func_tp2session_buf,
        //客户端发送流控帧的，两个参数作为配置，配置完不能改
        .STmin = 10,//10ms
        .BS = 8,//8帧

        .role=kSduClient,
        .dir=kSduRecv,
        .is_padding=true,
        .padding_val=0xCC,//发出的帧的填充值
        .RX_DL = 8,//默认为8，获取首帧的时候根据首帧长度改变
        .N_WFTmax=NULL,//单帧没有这玩意

        //一些状态参数
        .FlowStatus=NULL,
        .SequenceNumber=NULL,
        .l_recv_ind=false,

        .sdu_index=0,
        .BS_cnt=NULL,
        .N_WFTcnt=NULL,

        .N_As_timing_enable=false,
        .N_Ar_timing_enable=false,
        .N_Bs_timing_enable=false,
        .N_Br_timing_enable=false,
        .N_Cs_timing_enable=false,
        .N_Cr_timing_enable=false,
        .N_As=0,
        .N_Ar=0,
        .N_Bs=0,
        .N_Br=0,
        .N_Cs=0,
        .N_Cr=0,
    },
    {//响应地址 我们发送的
        //地址相关配置参数
        .can_id=0x701,
        .is_extended=false,
        .Mtype = kDiagnostics,//无N_AE
        .N_AI = {
            .N_TAtype = kFuncCanBaseFmt,//限定为11位classical can 功能地址
        },
        //与上层交换数据的缓冲区的配置
        .Length=NULL,//根据上层发下来的长度变化
        .MessageData=gs_resp_session2tp_buf,
        //发送多帧情况下，接收到流控帧就更新这两个参数
        .STmin = 10,//10
        .BS = 8,//8帧

        .role=kSduClient,
        .dir=kSduSend,
        .is_padding=true,
        .padding_val=0xCC,//发出的帧的填充值
        .TX_DL=8,//发送一帧的长度，配置了不能改
        .N_WFTmax=NULL,//对于流控接收方不用管这个

        //一些状态参数
        .FlowStatus=kFsCTS,
        .SequenceNumber=0,
        .n_send_req=false,

        .sdu_index=0,
        .BS_cnt=0,
        .N_WFTcnt=NULL,//对于流控接收方不用管这个

        .N_As_timing_enable=false,
        .N_Ar_timing_enable=false,
        .N_Bs_timing_enable=false,
        .N_Br_timing_enable=false,
        .N_Cs_timing_enable=false,
        .N_Cr_timing_enable=false,
        .N_As=0,
        .N_Ar=0,
        .N_Bs=0,
        .N_Br=0,
        .N_Cs=0,
        .N_Cr=0,
    }

    //N_SA和N_TA只有特殊开头的29bit id才做识别
    //Normal fixed
    // 0b100 0 0 218 N_TA N_SA extid phy
    // 0b100 0 0 219 N_TA N_SA extid func
    //Mixed addressing
    // 0b100 0 0 206 N_TA N_SA N_AE remote diag extid phy
    // 0b100 0 0 205 N_TA N_SA N_AE remote diag extid func
};

const size_t kg_num_of_sdu=sizeof(g_n_sdu_tbl)/sizeof(N_SDU);

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/
