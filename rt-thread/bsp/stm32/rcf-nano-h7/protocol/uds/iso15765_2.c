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

#include "iso15765_2.h"

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
    rt_uint32_t diag_phy_req_addr;  //诊断物理请求报文地址
    rt_uint32_t diag_func_req_addr; //诊断功能请求报文地址
    rt_uint32_t diag_resp_addr;     //诊断响应报文地址
}DiagAddrStruct;

/******************************************************************************
 * private variables
 *****************************************************************************/

typedef enum
{
    kDoCanTpCTS,
    kDoCanTpWAIT,
    kDoCanTpOVFLW,
}DoCanTpFlowStatusEnum;

typedef struct
{
    DoCanTpFlowStatusEnum flow_status;
    rt_uint8_t block_size;
    rt_uint8_t st_min;
}DoCanTpStat;

DoCanTpStat gs_docan_tp_stat;

typedef N_Result (*N_Result_pfun_void)(rt_uint8_t* data_len,rt_uint8_t* data_buf); /* pointer to function */

typedef enum
{
    kDoCanTpIdle,
    kDoCanTpRxSf,
    kDoCanTpRxFf,
    kDoCanTpRxFc,
    kDoCanTpRxCf,
    kDoCanTpTxSf,
    kDoCanTpTxFf,
    kDoCanTpTxFc,
    kDoCanTpTxCf,
    kDoCanTpError,
}DoCanTpStateEnum;

typedef enum
{
    kDoCanTpWaitEvent,
    kDoCanTpRecvSfEvent,
    kDoCanTpRecvFfEvent,
    kDoCanTpRecvCfEvent,
    kDoCanTpRecvFcEvent,
    kDoCanTpSendSfEvent,
    kDoCanTpSendFfEvent,
    kDoCanTpSendCfEvent,
    kDoCanTpSendFcEvent,
    kDoCanTpFinishEvent,
}DoCanTpEventEnum;

typedef struct
{
    DoCanTpStateEnum state;
    DoCanTpEventEnum event;
}DoCanTpSM;

static N_Result _DelIdle(rt_uint8_t* data_len,rt_uint8_t* data_buf);
static N_Result _DelRxSf(rt_uint8_t* data_len,rt_uint8_t* rx_buf);
static N_Result _DelRxFf(rt_uint8_t* data_len,rt_uint8_t* rx_buf);
static N_Result _DelRxFc(rt_uint8_t* data_len,rt_uint8_t* rx_buf);
static N_Result _DelRxCf(rt_uint8_t* data_len,rt_uint8_t* rx_buf);
static N_Result _DelTxSf(rt_uint8_t* data_len,rt_uint8_t* tx_buf);
static N_Result _DelTxFf(rt_uint8_t* data_len,rt_uint8_t* tx_buf);
static N_Result _DelTxFc(rt_uint8_t* data_len,rt_uint8_t* tx_buf);
static N_Result _DelTxCf(rt_uint8_t* data_len,rt_uint8_t* tx_buf);
static N_Result _DelError(rt_uint8_t* data_len,rt_uint8_t* data_buf);

static DoCanTpSM gs_docan_tp_sm={
    .state=kDoCanTpIdle,
    .event=kDoCanTpWaitEvent,
};

// docan tp层有限状态机状态转移表
const static N_Result_pfun_void kgs_docan_tp_fsm_tbl[7][3]={
    /* Actual State ->  Idle        RxSf        TxSf        Error                      RxFf RxFc RxCf  TxFf TxFc TxCf */
    /*Wait*/            {_DelIdle,  _DelError,  _DelError},
    /*RecvSf*/          {_DelRxSf,  _DelError,  _DelError},
    /*RecvFf*/          {_DelRxFf,  _DelError,  _DelError},
    /*RecvCf*/          {_DelError, _DelError,  _DelError},
    /*RecvFc*/          {_DelError, _DelError,  _DelError},
    /*SendSf*/          {_DelTxSf,  _DelError,  _DelError},
    /*Finish*/          {_DelError, _DelIdle,   _DelIdle},
};

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

struct rt_ringbuffer * g_docan_tp_rx_rb=RT_NULL;
struct rt_ringbuffer * g_docan_tp_tx_rb=RT_NULL;

void DoCanTpInit(void)
{
    g_docan_tp_rx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    g_docan_tp_tx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    // #if(DOCAN_TP_FF_DL<=4095)

    // #else
    // #endif
}

static rt_uint8_t gs_data_len=0;
static rt_uint8_t gs_rx_buf[64]={0};
static rt_uint8_t gs_tx_buf[64]={0};
static rt_uint8_t* gs_pbuf=gs_rx_buf;

rt_bool_t N_As_timing_enable=0;
rt_bool_t N_Ar_timing_enable=0;
rt_bool_t N_Bs_timing_enable=0;
rt_bool_t N_Br_timing_enable=0;
rt_bool_t N_Cs_timing_enable=0;
rt_bool_t N_Cr_timing_enable=0;

rt_time_t N_As=0;
rt_time_t N_Ar=0;
rt_time_t N_Bs=0;
rt_time_t N_Br=0;
rt_time_t N_Cs=0;
rt_time_t N_Cr=0;

/**
 * @brief   网络层定时器任务
 */
void NetworkLayerTimingTask(rt_uint16_t ms)
{
    if(N_As_timing_enable&&(N_As<N_AS_TIMEOUT_MS))
    {
        N_As+=ms;
    }
}

void DoCanTpTask(void)
{
    //状态机放在这里
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
    kgs_docan_tp_fsm_tbl[gs_docan_tp_sm.event][gs_docan_tp_sm.state](&gs_data_len,gs_pbuf);
    #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    kgs_docan_tp_fsm_tbl[gs_docan_tp_sm.event][gs_docan_tp_sm.state](data_len,pbuf+1);//地址字节不传入，但是整个包的长度要传入，后面还要用
    #else
        #error
    #endif 
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

/**
 * @brief   TP层空闲时读取环形队列是否有数据，有的话将长度和数据读取后做帧类型判断，然后返回上层
 * @note    Idle固定从rb读取数据到rx_buf
 */
static N_Result _DelIdle(rt_uint8_t* CAN_DL,rt_uint8_t* rx_buf){
    //在此之前要获取rb长度
    if(rt_ringbuffer_data_len(g_docan_tp_rx_rb))
        return N_OK;//buf为空，不需要执行
    rt_ringbuffer_get(g_docan_tp_rx_rb,CAN_DL,1);
    // if(rt_ringbuffer_data_len(g_docan_tp_rx_rb)<data_len)
    // {
    //     //等待？？？一般来说不会出现这种情况
    // }
    //无论如何都将dlc+data[]的整个包读出来
    rt_ringbuffer_get(g_docan_tp_rx_rb,rx_buf,*CAN_DL);

    //根据地址格式判断读取的长度是否满足最小长度
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
    if(*CAN_DL<2)//Table 12 CAN_DL value==0或者1非法
    #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    if(*CAN_DL<3)//Table 12 CAN_DL value==0或者1或2非法
    #else
        #error
    #endif    
        return N_UNEXP_PDU;

    //判断是什么类型的帧，只处理 N_PCI Byte #1的Bits 7 – 4
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
    switch ((N_PCItype)(rx_buf[0]>>4))
    #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    switch ((N_PCItype)(rx_buf[1]>>4))
    #else
        #error
    #endif  
    {
    case kNPciSF:
        gs_docan_tp_sm.event=kDoCanTpRecvSfEvent;
        break;
    case kNPciFF:
        gs_docan_tp_sm.event=kDoCanTpRecvFfEvent;
        break;
    case kNPciCF:
        gs_docan_tp_sm.event=kDoCanTpRecvCfEvent;
        break;
    case kNPciFC:
        gs_docan_tp_sm.event=kDoCanTpRecvFcEvent;
        break;
    default:
        return N_UNEXP_PDU;
        break;
    }

    return N_OK;
}
static N_Result _DelRxSf(rt_uint8_t* CAN_DL,rt_uint8_t* rx_buf){
    //更新状态机状态
    gs_docan_tp_sm.state=kDoCanTpRxSf;

    #if (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CANFD)
    if(*CAN_DL>DOCAN_TP_RX_DL)
    #else//DOCAN_TP_CAN_TYPE_CAN
    if(*CAN_DL>8)
    #endif
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        return N_UNEXP_PDU;
    }

    rt_uint8_t sf_dl=0;

    if(*CAN_DL<=8){
        sf_dl=rx_buf[0]&0x0F;
        //对sfdl做判断
        /*If the network layer receives a SF where SF_DL is equal to 0, then 
          the network layer shall ignore the received SF N_PDU*/
        if(sf_dl==0)
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
        /*If the network layer receives an SF where SF_DL is greater than 
          (CAN_DL – 1) of the received frame when using normal addressing or 
          greater than (CAN_DL – 2) of the received frame for extended or mixed
          addressing, then the network layer shall ignore the received SF N_PDU*/
        #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
        if(sf_dl>(*CAN_DL-1))
        #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
        if(sf_dl>(*CAN_DL-2))
        #endif
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
        /*In the case of CAN frame data padding (see 10.4.2.1): If the network 
          layer receives an SF where the CAN_DL does not equal to 8, then the 
          network layer shall ignore the received SF N_PDU*/
        #if (DOCAN_TP_DATA_LEN_MODE==DOCAN_TP_DATA_LEN_FIX)
        if(*CAN_DL!=8)
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
        /*In the case of CAN frame data optimization (see 10.4.2.2): If the 
          network layer receives an SF where the value of SF_DL does not match 
          the valid values shown in Table 12, then the network layer shall 
          ignore the received SF N_PDU*/
        #elif (DOCAN_TP_DATA_LEN_MODE==DOCAN_TP_DATA_LEN_AUTO)
        #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
        if(sf_dl!=(*CAN_DL-1))
        #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
        if(sf_dl!=(*CAN_DL-2))
        #endif
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
        #else
            #error
        #endif
    }
    #if (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CANFD)
    else//CAN_DL的最大值受DOCAN_TP_RX_DL限制
    {
        /*If the network layer receives an SF where the low nibble of the first
          PCI byte is not 00002, then the network layer shall ignore the 
          received SF N_PDU*/
        if(rx_buf[0]!=0)
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }

        sf_dl=rx_buf[1];

        /*If the network layer receives an SF where the value of SF_DL does not
          fall into the valid range shown in Table 13, then the network layer 
          shall ignore the received SF N_PDU*/
        switch (*CAN_DL)
        {
        #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
        #if (DOCAN_TP_RX_DL>=12)
        case 12:
            if(sf_dl<8||sf_dl>10)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=16)
        case 16:
            if(sf_dl<11||sf_dl>14)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=20)
        case 20:
            if(sf_dl<15||sf_dl>18)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=24)
        case 24:
            if(sf_dl<19||sf_dl>22)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=32)
        case 32:
            if(sf_dl<23||sf_dl>30)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=48)
        case 48:
            if(sf_dl<31||sf_dl>46)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL==64)
        case 64:
            if(sf_dl<47||sf_dl>62)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
        #if (DOCAN_TP_RX_DL>=12)
        case 12:
            if(sf_dl<7||sf_dl>9)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=16)
        case 16:
            if(sf_dl<10||sf_dl>13)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=20)
        case 20:
            if(sf_dl<14||sf_dl>17)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=24)
        case 24:
            if(sf_dl<18||sf_dl>21)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=32)
        case 32:
            if(sf_dl<22||sf_dl>29)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL>=48)
        case 48:
            if(sf_dl<30||sf_dl>45)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #if (DOCAN_TP_RX_DL==64)
        case 64:
            if(sf_dl<46||sf_dl>61)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
            break;
        #endif
        #else
            #error
        #endif
        default:
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
            break;
        }
    }
    #endif

    //copy len=sf_dl data to uds app buf
    gs_docan_tp_sm.event=kDoCanTpFinishEvent;
    return N_OK;
}
static N_Result _DelRxFf(rt_uint8_t* CAN_DL,rt_uint8_t* rx_buf){
    //更新状态机状态
    gs_docan_tp_sm.state=kDoCanTpRxFf;

    /*If the network layer receives an N_PDU indicating a FirstFrame and 
      CAN_DL < 8, then the network layer shall ignore the FF N_PDU*/
    if(*CAN_DL<8)
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        return N_UNEXP_PDU;
    }

    //姑且当作值是正确的，获取为ff_dl
    rt_uint32_t ff_dl=0;
    #if (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CAN)
    ff_dl=((rx_buf[0]&0x0F)<<8)|rx_buf[1];
    #elif (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CANFD)
    if((rx_buf[0]==0x10)&&((rx_buf[1]==0x00)))
    {
        ff_dl=(rx_buf[2]<<24)|(rx_buf[3]<<16)|(rx_buf[4]<<8)|rx_buf[5];
    }
    else
    {
        ff_dl=((rx_buf[0]&0x0F)<<8)|rx_buf[1];
    }
    #endif

    /*If the network layer receives a FirstFrame with FF_DL that is greater 
      than the available receiver buffer size, then this shall be considered as
      an error condition. The network layer shall abort the message reception 
      and send an FC N_PDU with the parameter FlowStatus = Overflow.*/
    if(ff_dl>UDS_APP_RX_BUF_SIZE)
    {
        gs_docan_tp_sm.event=kDoCanTpSendFcEvent;
        gs_docan_tp_stat.flow_status=kDoCanTpOVFLW;
        return N_BUFFER_OVFLW;
    }

    /*If the network layer receives a FirstFrame with an FF_DL that is less 
      than FF_DLmin, the network layer shall ignore the received FF N_PDU and 
      not transmit a FC N_PDU.*/
    #if (DOCAN_TP_TX_DL==8)
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
    if(ff_dl<8)
    #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    if(ff_dl<7)
    #endif
    #else/*DOCAN_TP_TX_DL > 8*/
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR))
    if(ff_dl<DOCAN_TP_TX_DL-1)
    #elif ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    if(ff_dl<DOCAN_TP_TX_DL-2)
    #endif
    #endif
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        return N_UNEXP_PDU;
    }

    /**
     * NOTE ---------------------------------待处理---------------------
     * Legacy devices that only support the 12 bit version of FF_DL will not 
     * send a FlowControl frame if the escape sequence is used as these devices 
     * would interpret FF_DL to be less than FF_DLmin and as such, not send an 
     * FC N_PDU.
     */

    /*If a FirstFrame is received with the escape sequence (where all bits of 
      the lower nibble of PCI byte 1 and all bits of PCI byte 2 are set to 0’s) 
      and the FF_DL =< 4 095, then the network layer shall ignore the FF N_PDU 
      and not transmit an FC N_PDU.*/
    if(ff_dl<=4095)
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        return N_UNEXP_PDU;
    }

    //copy len=sf_dl data to uds app buf
    gs_docan_tp_sm.event=kDoCanTpSendFcEvent;
    gs_docan_tp_stat.flow_status=kDoCanTpCTS;

    // L_Data.indication (FF)
    return N_OK;
}
static N_Result _DelRxFc(rt_uint8_t* CAN_DL,rt_uint8_t* rx_buf){return N_OK;}
static N_Result _DelRxCf(rt_uint8_t* CAN_DL,rt_uint8_t* rx_buf){return N_OK;}
static N_Result _DelTxSf(rt_uint8_t* CAN_DL,rt_uint8_t* tx_buf){return N_OK;}
static N_Result _DelTxFf(rt_uint8_t* CAN_DL,rt_uint8_t* tx_buf){return N_OK;}
static N_Result _DelTxFc(rt_uint8_t* CAN_DL,rt_uint8_t* tx_buf){

    //tx_buf填充N_PCI和FS
    tx_buf[0]=(kNPciFC<<4)|gs_docan_tp_stat.flow_status;
    //BS
    if(gs_docan_tp_stat.flow_status==kDoCanTpCTS)
    {
        // tx_buf[1]=
    }
}
static N_Result _DelTxCf(rt_uint8_t* CAN_DL,rt_uint8_t* tx_buf){}
static N_Result _DelError(rt_uint8_t* CAN_DL,rt_uint8_t* data_buf){}