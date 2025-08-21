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

extern N_SDU g_n_sdu_tbl[];
extern const size_t kg_num_of_sdu;

static void N_USData_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt);
static N_Result N_USData_confirm(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, N_UserExtStruct N_UserExt);
static N_Result N_USData_indication(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt);
static void N_USData_FF_indication(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, size_t Length, N_UserExtStruct N_UserExt);
static void N_ChangeParameter_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, uint8_t Parameter_Value, N_UserExtStruct N_UserExt);
static Result_ChangeParameter N_ChangeParameter_confirm(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, N_UserExtStruct N_UserExt);

const struct N_USDataOps N_USData={
    .request=N_USData_request,
    .confirm=N_USData_confirm,
    .indication=N_USData_indication,
};

const struct N_USData_FFOps N_USData_FF={
    .indication=N_USData_FF_indication,
};

const struct N_ChangeParameterOps N_ChangeParameter={
    .request=N_ChangeParameter_request,
    .confirm=N_ChangeParameter_confirm,
};

//tp层给上层的buf
uint8_t tp2up_layer_buf[0xFFF];

/******************************************************************************
 * private types
 *****************************************************************************/

typedef struct
{
    uint32_t diag_phy_req_addr;  //诊断物理请求报文地址
    uint32_t diag_func_req_addr; //诊断功能请求报文地址
    uint32_t diag_resp_addr;     //诊断响应报文地址
}DiagAddrStruct;

typedef struct
{
    FlowStatusEnum FlowStatus;
    uint8_t block_size;
    uint8_t st_min;
}DoCanTpStat;

typedef N_Result (*N_Result_pfun_void)(struct rt_canx_msg* rx_can_msg); /* pointer to function */

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


/******************************************************************************
 * private variables
 *****************************************************************************/

DoCanTpStat gs_docan_tp_stat;

static N_Result _DelIdle(struct rt_canx_msg* rx_can_msg);
static N_Result _DelRxSf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelRxFf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelRxFc(struct rt_canx_msg* rx_can_msg);
static N_Result _DelRxCf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelTxSf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelTxFf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelTxFc(struct rt_canx_msg* rx_can_msg);
static N_Result _DelTxCf(struct rt_canx_msg* rx_can_msg);
static N_Result _DelError(struct rt_canx_msg* rx_can_msg);

static DoCanTpSM gs_docan_tp_sm={
    .state=kDoCanTpIdle,
    .event=kDoCanTpWaitEvent,
};

// docan tp层有限状态机状态转移表
const static N_Result_pfun_void kgs_docan_tp_fsm_tbl[10][5]={
    /* Current State    Idle    RxSf        RxFf        RxCf        RxFc    TxSf        Error                      RxFf RxFc RxCf  TxFf TxFc TxCf */
    /* Event  */
    /*  Wait  */    {_DelIdle,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvSf */    {_DelRxSf,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvFf */    {_DelRxFf,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvCf */    {_DelError, _DelError,  _DelError,  _DelRxCf,   _DelRxCf },
    /* RecvFc */    {_DelError, _DelError,  _DelError,  _DelError,  _DelRxFc },
    /* SendSf */    {_DelTxSf,  _DelError,  _DelError,  _DelError,  _DelError},
    /* SendFf */    {_DelTxFf,  _DelError,  _DelError,  _DelError,  _DelError},
    /* SendCf */    {_DelError, _DelError,  _DelError,  _DelError,  _DelError},
    /* SendFc */    {_DelError, _DelError,  _DelTxFc,   _DelTxFc,   _DelError},
    /* Finish */    {_DelError, _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle}
};

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

struct rt_ringbuffer * g_docan_tp_rx_rb=NULL;
struct rt_ringbuffer * g_docan_tp_tx_rb=NULL;

static uint8_t gs_data_len=0;
static uint8_t gs_rx_buf[64]={0};
static uint8_t gs_tx_buf[64]={0};
static uint8_t* gs_pbuf=gs_rx_buf;

bool N_As_timing_enable=0;
bool N_Ar_timing_enable=0;
bool N_Bs_timing_enable=0;
bool N_Br_timing_enable=0;
bool N_Cs_timing_enable=0;
bool N_Cr_timing_enable=0;

time_t N_As=0;
time_t N_Ar=0;
time_t N_Bs=0;
time_t N_Br=0;
time_t N_Cs=0;
time_t N_Cr=0;

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void DoCanTpInit(void)
{
    g_docan_tp_rx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    g_docan_tp_tx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    // #if(DOCAN_TP_FF_DL<=4095)

    // #else
    // #endif
}

/**
 * @brief   网络层定时器任务
 */
void NetworkLayerTimingTask(uint16_t ms)
{
    if(N_As_timing_enable&&(N_As<N_AS_TIMEOUT_MS))
    {
        N_As+=ms;
    }
}

void DoCanTpTask(void)
{
    //从ringbuf读取一个can帧出来
    kgs_docan_tp_fsm_tbl[gs_docan_tp_sm.event][gs_docan_tp_sm.state](&gs_data_len,gs_pbuf);
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

/**
 * @brief   TP层空闲时读取环形队列是否有数据，有的话将长度和数据读取后做帧类型判断，然后返回上层
 * @note    Idle固定从rb读取数据到rx_buf
 */
static N_Result _DelIdle(struct rt_canx_msg* rx_can_msg){

    //在此之前要获取rb长度
    if(rt_ringbuffer_data_len(g_docan_tp_rx_rb))
        return N_OK;//buf为空，不需要执行

    struct rt_canx_msg* tmp_ptr =NULL;
    tmp_ptr= (struct rt_canx_msg*)malloc(sizeof(struct rt_canx_msg));
    if(tmp_ptr==NULL)
        return N_ERROR;//分配失败

    rt_ringbuffer_get(g_docan_tp_rx_rb,(uint8_t *)tmp_ptr,8);//获取rt_canx_msg的头

    if(rt_ringbuffer_data_len(g_docan_tp_rx_rb)<DLCtoBytes[tmp_ptr->dlc])
        while(1);//////////////////////等待？？？一般来说不会出现这种情况

    struct rt_canx_msg* new_tmp_ptr = NULL;
    new_tmp_ptr=realloc(tmp_ptr,sizeof(struct rt_canx_msg)+DLCtoBytes[tmp_ptr->dlc]);
    if(new_tmp_ptr==NULL)
    {
        free(tmp_ptr);
        return N_ERROR;//分配失败
    }
    tmp_ptr=new_tmp_ptr;

    //将data[]的整个包读出来
    rt_ringbuffer_get(g_docan_tp_rx_rb,tmp_ptr->data,DLCtoBytes[tmp_ptr->dlc]);

    rx_can_msg=tmp_ptr;

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    for (; sdu_index < kg_num_of_sdu; sdu_index++)
    {
        //N_AI matches the local configuration of CAN (FD)
        if (rx_can_msg->id == g_n_sdu_tbl[sdu_index].can_id)// 找到匹配的N_SDU
        {
            if(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics)//mixed addr
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_AE)
                    break;
            else if(g_n_sdu_tbl[sdu_index].is_extended)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_TA)
                    break;
            else
                break;
        }
    }
    if(sdu_index>=kg_num_of_sdu)
    {
        //Ignore Frame
        free(rx_can_msg);
        //can帧塞回另一个buffer
        return N_UNEXP_PDU;//sdu tbl没有找到匹配的参数
    }

    //N_PCItype is the high nibble of N_PCI byte (Byte #1)
    uint8_t n_pci_byte1=rx_can_msg->data[0+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]>>4;

    switch (n_pci_byte1)
    {
    case kNPciSF:
        gs_docan_tp_sm.event=kDoCanTpRecvSfEvent;
        break;
    case kNPciFF:
        gs_docan_tp_sm.event=kDoCanTpRecvFfEvent;
        break;
    // case kNPciCF:
    //     gs_docan_tp_sm.event=kDoCanTpRecvCfEvent;
    //     break;
    // case kNPciFC:
    //     gs_docan_tp_sm.event=kDoCanTpRecvFcEvent;
    //     break;
    default:
        free(rx_can_msg);
        return N_UNEXP_PDU;//Ignore Frame
        break;
    }

    return N_OK;
}

static N_Result _DelRxSf(struct rt_canx_msg* rx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    for (; sdu_index < kg_num_of_sdu; sdu_index++)
    {
        //N_AI matches the local configuration of CAN (FD)
        if (rx_can_msg->id == g_n_sdu_tbl[sdu_index].can_id)// 找到匹配的N_SDU
        {
            if(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_AE)
                    break;
            else if(g_n_sdu_tbl[sdu_index].is_extended)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_TA)
                    break;
            else
                break;
        }
    }

    //本函数内的临时变量，降低代码长度
    MtypeEnum temp_Mtype=g_n_sdu_tbl[sdu_index].Mtype;
    uint8_t temp_N_SA=g_n_sdu_tbl[sdu_index].N_AI.N_SA;
    uint8_t temp_N_TA=g_n_sdu_tbl[sdu_index].N_AI.N_TA;
    N_TAtypeEnum temp_N_TAtype=g_n_sdu_tbl[sdu_index].N_AI.N_TAtype;
    uint8_t temp_N_AE=g_n_sdu_tbl[sdu_index].N_AI.N_AE;
    bool temp_is_extended=g_n_sdu_tbl[sdu_index].is_extended;
    bool temp_is_padding=g_n_sdu_tbl[sdu_index].is_padding;
    bool temp_is_remote_diagnostics=;

    uint8_t temp_sf_dl=1;//SF_DL最小的有效值
    uint8_t CAN_DL=DLCtoBytes[rx_can_msg->dlc];

    uint8_t N_PCI_bytes_offset=0;
    if((temp_Mtype==kRemoteDiagnostics)||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    {
        N_PCI_bytes_offset=1;
    }

    if(CAN_DL>8)
    {
        uint8_t n_pci_byte1=rx_can_msg->data[N_PCI_bytes_offset]&0x0F;

        //4 bit SF_DL value in Byte #1 low nibble equals zero
        if(n_pci_byte1!=0)
        {
            free(rx_can_msg);
            return N_UNEXP_PDU;//Ignore Frame
        }

        //Get 8 bit SF_DL value from Byte #2. Message data start offset +1
        temp_sf_dl=rx_can_msg->data[1+N_PCI_bytes_offset];
    }
    else//CAN_DL<=8
    {
        temp_sf_dl=rx_can_msg->data[N_PCI_bytes_offset]&0x0F;
    }

    //Check for valid SF_DL and process Single Frame (SF)

    // SF_DL error handling

    if(CAN_DL<=8)//Received CAN_DL is less or equal to 8:
    {
        /*If the network layer receives a SF where SF_DL is equal to 0, then 
          the network layer shall ignore the received SF N_PDU*/
        if(temp_sf_dl==0)
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            free(rx_can_msg);
            return N_UNEXP_PDU;
        }

        /*If the network layer receives an SF where SF_DL is greater than 
          (CAN_DL – 1) of the received frame when using normal addressing or 
          greater than (CAN_DL – 2) of the received frame for extended or mixed
          addressing, then the network layer shall ignore the received SF N_PDU*/
        if(temp_is_extended|(temp_Mtype==kRemoteDiagnostics))//extended or mixed addressing
        {
            if(temp_sf_dl>(CAN_DL-2))
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                free(rx_can_msg);
                return N_UNEXP_PDU;
            }
        }
        else//normal addressing
        {
            if(temp_sf_dl>(CAN_DL-1))
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                free(rx_can_msg);
                return N_UNEXP_PDU;
            }
        }

        /*In the case of CAN frame data padding (see 10.4.2.1): If the network 
          layer receives an SF where the CAN_DL does not equal to 8, then the 
          network layer shall ignore the received SF N_PDU*/
        if(temp_is_padding)// CAN frame data padding
        {
            if(CAN_DL!=8)
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                free(rx_can_msg);
                return N_UNEXP_PDU;
            }
        }
        /*In the case of CAN frame data optimization (see 10.4.2.2): If the 
          network layer receives an SF where the value of SF_DL does not match 
          the valid values shown in Table 12, then the network layer shall 
          ignore the received SF N_PDU*/
        else//CAN frame data optimization
        {
            //Table 12 — Allowed SF_DL values for a given addressing scheme with optimized CAN_DL
            const uint8_t table12[2][9]={
                {NULL,NULL,1,   2,3,4,5,6,7},//Normal
                {NULL,NULL,NULL,1,2,3,4,5,6} //Mixed or extended
            };

            if(temp_sf_dl!=table12[(temp_Mtype==kRemoteDiagnostics)|temp_is_extended][rx_can_msg->dlc])
            {
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                free(rx_can_msg);
                return N_UNEXP_PDU;
            }
        }
    }
    else//Received CAN_DL is greater than 8:
    {
        /*If the network layer receives an SF where the low nibble of the first
          PCI byte is not 00002, then the network layer shall ignore the 
          received SF N_PDU*/
        /*在Figure 9 — State flow — Verifying received CAN frames的
          4 bit SF_DL value in Byte #1 low nibble equals zero已经处理*/

        /*If the network layer receives an SF where the value of SF_DL does not
          fall into the valid range shown in Table 13, then the network layer 
          shall ignore the received SF N_PDU*/

        //Table 13 — Allowed SF_DL values for a given CAN_DL greater than 8 and addressing scheme
        typedef struct
        {
            uint8_t SF_DL_min;
            uint8_t SF_DL_max;
        }SF_DLRange;

        const SF_DLRange table13[2][7]={
            {{8,10},{11,14},{15,18},{19,22},{23,30},{31,46},{47,62}},//Normal
            {{7,9}, {10,13},{14,17},{18,21},{22,29},{30,45},{46,61}} //Mixed or extended
        };

        //根据 Mtype is_extended dlc 找到对应的元素，要求SF_DL在元素结构体的范围内
        if(table13[(temp_Mtype==kRemoteDiagnostics)|temp_is_extended][rx_can_msg->dlc-9].SF_DL_min>temp_sf_dl ||
        table13[(temp_Mtype==kRemoteDiagnostics)|temp_is_extended][rx_can_msg->dlc-9].SF_DL_max<temp_sf_dl )
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            free(rx_can_msg);
            return N_UNEXP_PDU;
        }
    }

    //千辛万苦，终于找到了一个合法的帧，接下来就是解析帧了
    uint8_t payload_offset=1;//接收到的can帧起码有一个字节被N_PCI bytes占用
    if(CAN_DL>8)
    {
        payload_offset++;
    }
    if(g_n_sdu_tbl[sdu_index].is_extended|(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics))
    {
        payload_offset++;
    }


    //copy len=sf_dl data to tp2up_layer_buf
    memcpy(tp2up_layer_buf,rx_can_msg->data+payload_offset,SF_DL);
    N_USData.indication(g_n_sdu_tbl[sdu_index].Mtype,g_n_sdu_tbl[sdu_index].N_AI.N_SA,g_n_sdu_tbl[sdu_index].N_AI.N_TA,)
    gs_docan_tp_sm.event=kDoCanTpFinishEvent;
    free(rx_can_msg);
    return N_OK;
}

static N_Result _DelRxFf(struct rt_canx_msg* rx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    for (; sdu_index < kg_num_of_sdu; sdu_index++)
    {
        //N_AI matches the local configuration of CAN (FD)
        if (rx_can_msg->id == g_n_sdu_tbl[sdu_index].can_id)// 找到匹配的N_SDU
        {
            if(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_AE)
                    break;
            else if(g_n_sdu_tbl[sdu_index].is_extended)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_TA)
                    break;
            else
                break;
        }
    }

    // 9.6.3.2 FF_DL error handling

    /*If the network layer receives an N_PDU indicating a FirstFrame and 
      CAN_DL < 8, then the network layer shall ignore the FF N_PDU*/
    uint8_t CAN_DL=DLCtoBytes[rx_can_msg->dlc];
    if(CAN_DL<8)
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        free(rx_can_msg);
        return N_UNEXP_PDU;
    }

    /* the retrieved value of RX_DL from the CAN_DL of the FF N_PDU (see 9.5.4 
       for definition of how the receiver determines RX_DL). */
    g_n_sdu_tbl[sdu_index].RX_DL=CAN_DL;

    /* The receiver determines the minimum value of FF_DL (FF_DLmin) from Table 
       14 based on the configured addressing scheme and the retrieved value of 
       RX_DL */
    //Table 14 — Minimum value of FF_DL based on the addressing scheme
    uint8_t FF_DLmin=7;
    if(g_n_sdu_tbl[sdu_index].RX_DL==8)
    {
        //mixed or extended addressing is used
        if(g_n_sdu_tbl[sdu_index].is_extended|(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics))
            FF_DLmin=7;
        else// normal addressing is used
            FF_DLmin=8;
    }
    else//大于8
    {
        //mixed or extended addressing is used
        if(g_n_sdu_tbl[sdu_index].is_extended|(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics))
            FF_DLmin=g_n_sdu_tbl[sdu_index].RX_DL-2;
        else// normal addressing is used
            FF_DLmin=g_n_sdu_tbl[sdu_index].RX_DL-1;
    }

    //escape sequence or not
    uint32_t tmp_ff_dl=(((uint16_t)(rx_can_msg->data[0+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]))<<8)|
                                   (rx_can_msg->data[1+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]);
    if(tmp_ff_dl==0)
    {
        // escape sequence
        tmp_ff_dl=(((uint32_t)(rx_can_msg->data[0+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]))<<24)|
                  (((uint32_t)(rx_can_msg->data[1+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]))<<16)|
                  (((uint32_t)(rx_can_msg->data[2+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]))<<8)|
                  (((uint32_t)(rx_can_msg->data[3+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]))<<0);

        /*If a FirstFrame is received with the escape sequence (where all bits of 
        the lower nibble of PCI byte 1 and all bits of PCI byte 2 are set to 0’s) 
        and the FF_DL =< 4 095, then the network layer shall ignore the FF N_PDU 
        and not transmit an FC N_PDU.*/
        if(tmp_ff_dl<=4095)
        {
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            free(rx_can_msg);
            return N_UNEXP_PDU;
        }
    }

    /*If the network layer receives a FirstFrame with FF_DL that is greater 
      than the available receiver buffer size, then this shall be considered as
      an error condition. The network layer shall abort the message reception 
      and send an FC N_PDU with the parameter FlowStatus = Overflow.*/
    if(tmp_ff_dl>g_n_sdu_tbl[sdu_index].sdu_buf_size)
    {
        gs_docan_tp_sm.event=kDoCanTpSendFcEvent;
        g_n_sdu_tbl[sdu_index].FlowStatus=kFsOVFLW;
        free(rx_can_msg);
        return N_BUFFER_OVFLW;
    }

    /*If the network layer receives a FirstFrame with an FF_DL that is less 
      than FF_DLmin, the network layer shall ignore the received FF N_PDU and 
      not transmit a FC N_PDU.*/
    if(tmp_ff_dl<FF_DLmin)
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        free(rx_can_msg);
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
    //在构造tmp_ff_dl那里做了

    uint32_t FF_DL=tmp_ff_dl;

    //复制首帧中的上层数据到 uds app buf
    //todo
    gs_docan_tp_sm.event=kDoCanTpSendFcEvent;
    g_n_sdu_tbl[sdu_index].FlowStatus=kFsCTS;
    g_n_sdu_tbl[sdu_index].sdu_len=FF_DL;
    g_n_sdu_tbl[sdu_index].SequenceNumber++;
    free(rx_can_msg);

    // L_Data.indication (FF)
    return N_OK;
}

static N_Result _DelRxFc(struct rt_canx_msg* rx_can_msg){

    size_t sdu_index = 0;
    for (; sdu_index < kg_num_of_sdu; sdu_index++)
    {
        if (rx_can_msg->id != g_n_sdu_tbl[sdu_index].can_id) {
            continue;  // ID不匹配，继续下一个
        }

        // ID匹配，根据类型检查数据
        bool data_matches = false;
        if (g_n_sdu_tbl[sdu_index].Mtype == kRemoteDiagnostics) {
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[sdu_index].N_AI.N_AE);
        } else if (g_n_sdu_tbl[sdu_index].is_extended) {
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[sdu_index].N_AI.N_TA);
        } else {
            data_matches = true;
        }

        if (data_matches) {
            break;  // 找到匹配的SDU
        }
    }
    if (sdu_index >= kg_num_of_sdu) {
        // 没有找到匹配的SDU
        return N_OK;
    }


    N_USData.confirm();
    return N_OK;
}

static N_Result _DelRxCf(struct rt_canx_msg* rx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    for (; sdu_index < kg_num_of_sdu; sdu_index++)
    {
        //N_AI matches the local configuration of CAN (FD)
        if (rx_can_msg->id == g_n_sdu_tbl[sdu_index].can_id)// 找到匹配的N_SDU
        {
            if(g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_AE)
                    break;
            else if(g_n_sdu_tbl[sdu_index].is_extended)
                if(rx_can_msg->data[0]==g_n_sdu_tbl[sdu_index].N_AI.N_TA)
                    break;
            else
                break;
        }
    }

    /* The payload data length CAN_DL of the received CAN frame has to match 
       the RX_DL value which was determined in the reception process of the 
       FirstFrame (FF). Only the last CF in the multi-frame transmission may 
       contain less than RX_DL bytes. */

    //CAN_DL correct for the determined RX_DL
    if(g_n_sdu_tbl[sdu_index].RX_DL==8)//首帧接收到的长度是8字节
    {
        if(g_n_sdu_tbl[sdu_index].is_padding)//填充模式
        {
            if(DLCtoBytes[rx_can_msg->dlc]!=8)
            {
                //ignore frame
                gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                free(rx_can_msg);
                return N_UNEXP_PDU;
            }
        }
        else//变长模式
        {
            if((g_n_sdu_tbl[sdu_index].Mtype==kRemoteDiagnostics)||g_n_sdu_tbl[sdu_index].is_extended)
            {
                size_t remaining_length=g_n_sdu_tbl[sdu_index].sdu_len-(g_n_sdu_tbl[sdu_index].sdu_index+1);
                if(remaining_length<=(8-2))//剩余长度<=6
                {
                    if(DLCtoBytes[rx_can_msg->dlc]!=remaining_length+2)
                    {
                        //ignore frame
                        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                        free(rx_can_msg);
                        return N_UNEXP_PDU;
                    }
                }
            }
            else//normal addressing
            {
                size_t remaining_length=g_n_sdu_tbl[sdu_index].sdu_len-(g_n_sdu_tbl[sdu_index].sdu_index+1);
                if(remaining_length<=(8-1))//剩余长度<=7
                {
                    if(DLCtoBytes[rx_can_msg->dlc]!=remaining_length+1)
                    {
                        //ignore frame
                        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
                        free(rx_can_msg);
                        return N_UNEXP_PDU;
                    }
                }
            }
        }        
    }
    else//can fd only
    {
        if(DLCtoBytes[rx_can_msg->dlc]!=g_n_sdu_tbl[sdu_index].RX_DL)
        {
            //ignore frame
            gs_docan_tp_sm.event=kDoCanTpFinishEvent;
            free(rx_can_msg);
            return N_UNEXP_PDU;
        }
    }

    //SequenceNumber (SN) error handling
    /* If a CF N_PDU message is received with an unexpected SequenceNumber not
       in accordance with the definition in 9.6.4.3, the message reception 
       shall be aborted and the network layer shall make an N_USData.indication
       service call with the parameter <N_Result> = N_WRONG_SN to the adjacent 
       upper layer. */
    uint8_t SequenceNumber = rx_can_msg->data[0+(g_n_sdu_tbl[sdu_index].Mtype|g_n_sdu_tbl[sdu_index].is_extended)]&0x0F;
    if(SequenceNumber!=g_n_sdu_tbl[sdu_index].SequenceNumber)
    {
        gs_docan_tp_sm.event=kDoCanTpFinishEvent;
        free(rx_can_msg);
        return N_WRONG_SN;
    }

    g_n_sdu_tbl[sdu_index].SequenceNumber++;
    //复制续帧的数据到sdu
    //更新sdu的index
    //判断还有没有剩余字节，或者需不需要发送流控帧

    return N_OK;
}

static N_Result _DelTxSf(struct rt_canx_msg* rx_can_msg){return N_OK;}
static N_Result _DelTxFf(struct rt_canx_msg* rx_can_msg){return N_OK;}
static N_Result _DelTxFc(struct rt_canx_msg* rx_can_msg){

    //tx_buf填充N_PCI和FS
    rx_can_msg->data[0]=(kNPciFC<<4)|gs_docan_tp_stat.FlowStatus;
    //BS
    if(gs_docan_tp_stat.FlowStatus==kFsCTS)
    {
        // rx_can_msg->data[1]=
    }
}
static N_Result _DelTxCf(struct rt_canx_msg* rx_can_msg){}
static N_Result _DelError(struct rt_canx_msg* rx_can_msg){}

static void N_USData_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt){}
static N_Result N_USData_confirm(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, N_UserExtStruct N_UserExt){}
static N_Result N_USData_indication(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt){}
static void N_USData_FF_indication(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, size_t Length, N_UserExtStruct N_UserExt){}
static void N_ChangeParameter_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, uint8_t Parameter_Value, N_UserExtStruct N_UserExt){}
static Result_ChangeParameter N_ChangeParameter_confirm(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, N_UserExtStruct N_UserExt){}