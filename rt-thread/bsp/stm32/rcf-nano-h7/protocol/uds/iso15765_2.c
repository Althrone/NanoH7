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

extern const struct L_DataOps L_Data;


static void N_USData_request            (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt);
static void N_USData_confirm            (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, N_ResultEnum N_Result, N_UserExtStruct N_UserExt);
static void N_USData_indication         (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_ResultEnum N_Result, N_UserExtStruct N_UserExt);
static void N_USData_FF_indication      (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, size_t Length, N_UserExtStruct N_UserExt);
static void N_ChangeParameter_request   (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, uint8_t Parameter_Value, N_UserExtStruct N_UserExt);
static void N_ChangeParameter_confirm   (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, Result_ChangeParameterEnum Result_ChangeParameter, N_UserExtStruct N_UserExt);

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

static void L_Data_request(struct rt_canx_msg* tx_can_msg);
static void L_Data_confirm(struct rt_canx_msg* tx_can_msg, Transfer_StatusEnum Transfer_Status);
static void L_Data_indication(struct rt_canx_msg* rx_can_msg);

const struct L_DataOps L_Data=
{
    .request=L_Data_request,
    .confirm=L_Data_confirm,
    .indication=L_Data_indication,
};

/******************************************************************************
 * private types
 *****************************************************************************/

typedef struct
{
    FlowStatusEnum FlowStatus;
    uint8_t block_size;
    uint8_t st_min;
}DoCanTpStat;

typedef N_ResultEnum (*N_Result_pfun_p_n_sdu)(N_SDU* p_n_sdu); /* pointer to function */

/******************************************************************************
 * private variables
 *****************************************************************************/

DoCanTpStat gs_docan_tp_stat;

static N_ResultEnum _DelIdle(N_SDU* p_n_sdu);
static N_ResultEnum _DelRxSf(N_SDU* p_n_sdu);
static N_ResultEnum _DelRxFf(N_SDU* p_n_sdu);
static N_ResultEnum _DelRxFc(N_SDU* p_n_sdu);
static N_ResultEnum _DelRxCf(N_SDU* p_n_sdu);
static N_ResultEnum _DelTxSf(N_SDU* p_n_sdu);
static N_ResultEnum _DelTxFf(N_SDU* p_n_sdu);
static N_ResultEnum _DelTxFc(N_SDU* p_n_sdu);
static N_ResultEnum _DelTxCf(N_SDU* p_n_sdu);
static N_ResultEnum _DelError(N_SDU* p_n_sdu);

// static DoCanTpSM gs_docan_tp_sm={
//     .state=kDoCanTpIdle,
//     .event=kDoCanTpWaitEvent,
// };

// docan tp层有限状态机状态转移表
const static N_Result_pfun_p_n_sdu kgs_docan_tp_fsm_tbl[10][9]={
    /* Current State    Idle    RxSf        RxFf        RxCf        RxFc        TxSf        TxFf        TxCf        TxFc        WaitTxOk    Error */
    /* Event  */
    /*  Wait  */    {_DelIdle,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvSf */    {_DelRxSf,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvFf */    {_DelRxFf,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError},
    /* RecvCf */    {_DelError, _DelError,  _DelError,  _DelRxCf,   _DelError,  _DelError,  _DelRxCf,   _DelError,  _DelRxCf },
    /* RecvFc */    {_DelError, _DelError,  _DelError,  _DelError,  _DelRxFc,   _DelError,  _DelError,  _DelRxFc,   _DelError},
    /* SendSf */    {_DelTxSf,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError},
    /* SendFf */    {_DelTxFf,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError,  _DelError},
    /* SendCf */    {_DelError, _DelError,  _DelError,  _DelError,  _DelTxCf,   _DelError,  _DelError,  _DelTxCf,   _DelError},
    /* SendFc */    {_DelError, _DelError,  _DelTxFc,   _DelTxFc,   _DelError,  _DelError,  _DelError,  _DelError,  _DelTxFc },
    /* Finish */    {_DelError, _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle,   _DelIdle }
};

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

uint8_t Bytes2ShortestDLC(uint8_t bytes)
{
    if(bytes<=8)
        return bytes;
    else if(bytes<=12)
        return 9;
    else if(bytes<=16)
        return 10;
    else if(bytes<=20)
        return 11;
    else if(bytes<=24)
        return 12;
    else if(bytes<=32)
        return 13;
    else if(bytes<=48)
        return 14;
    else if(bytes<=64)
        return 15;
    else
        return 0;
}

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/**
 * Peek data from ring buffer without moving the read pointer.
 *
 * @param rb            Pointer to the ring buffer.
 * @param ptr           Pointer to the data buffer where the data will be stored.
 * @param length        The desired length of data to peek. Must be less than or equal to the data length in the buffer.
 * @return              The actual number of bytes peeked.
 */
rt_size_t rt_ringbuffer_peek(struct rt_ringbuffer *rb, rt_uint8_t *ptr, rt_size_t length)
{
    rt_base_t level;
    rt_size_t len = 0;
    rt_uint16_t read_index, write_index;
    rt_uint16_t read_mirror, write_mirror;
    rt_int16_t buffer_size;

    /* 为了线程安全，禁止中断或加锁 */
    level = rt_hw_interrupt_disable();

    /* 保存当前读写下标和镜像位 */
    read_index = rb->read_index;
    write_index = rb->write_index;
    read_mirror = rb->read_mirror;
    write_mirror = rb->write_mirror;
    buffer_size = rb->buffer_size;

    /* 检查是否有足够的数据可读 */
    rt_size_t data_len = rt_ringbuffer_data_len(rb);
    if (length > data_len)
    {
        length = data_len; // 不能读取超过已有数据量的数据
    }

    if (length == 0)
    {
        rt_hw_interrupt_enable(level);
        return 0;
    }

    /* 计算从当前 read_index 开始，到缓冲区末尾的数据长度 */
    rt_size_t first_part_len = buffer_size - read_index;
    if (first_part_len > length)
    {
        first_part_len = length;
    }

    /* 复制第一部分数据 (从 read_index 到缓冲区结束) */
    if (ptr != RT_NULL && first_part_len > 0)
    {
        memcpy(ptr, &rb->buffer_ptr[read_index], first_part_len);
    }

    /* 如果需要，复制第二部分数据 (从缓冲区开始处环绕) */
    if (length > first_part_len)
    {
        rt_size_t second_part_len = length - first_part_len;
        if (ptr != RT_NULL && second_part_len > 0)
        {
            memcpy(ptr + first_part_len, &rb->buffer_ptr[0], second_part_len);
        }
    }

    len = length;

    /* 注意：这里我们没有修改 rb->read_index 和 rb->read_mirror */

    /* 恢复中断 */
    rt_hw_interrupt_enable(level);

    return len;
}
/**
 * Peek a character from ring buffer without moving the read pointer.
 *
 * @param rb            Pointer to the ring buffer.
 * @param ch            Pointer to the character where the data will be stored.
 * @return              1 if successful, 0 otherwise.
 */
rt_size_t rt_ringbuffer_peekchar(struct rt_ringbuffer *rb, rt_uint8_t *ch)
{
    rt_base_t level;
    rt_size_t ret = 0;

    if (rt_ringbuffer_data_len(rb) < 1)
    {
        return 0;
    }

    level = rt_hw_interrupt_disable();

    *ch = rb->buffer_ptr[rb->read_index]; // 直接读取当前读指针处的字符
    ret = 1;

    rt_hw_interrupt_enable(level);
    return ret;
}

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void DoCanTpInit(void)
{
    // g_docan_tp_rx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    // g_docan_tp_tx_rb=rt_ringbuffer_create(DOCAN_TP_FF_DL);
    // #if(DOCAN_TP_FF_DL<=4095)

    // #else
    // #endif
}

/**
 * @brief   网络层定时器任务
 */
void NetworkLayerTimingTask(uint16_t ms)
{
    // if(N_As_timing_enable&&(N_As<N_AS_TIMEOUT_MS))
    // {
    //     N_As+=ms;
    // }
}

void DoCanTpTask(void)
{
    for (size_t i = 0; i < kg_num_of_sdu; i++)
    {
        g_n_sdu_tbl[i].N_Result=kgs_docan_tp_fsm_tbl[g_n_sdu_tbl[i].sm.event][g_n_sdu_tbl[i].sm.state](&g_n_sdu_tbl[i]);
    }
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
static bool _SearchSduIndex(struct rt_canx_msg* rx_can_msg,size_t* sdu_index){
    size_t i = 0;
    bool data_matches = false;
    for (; i < kg_num_of_sdu; i++)
    {
        if (rx_can_msg->id != g_n_sdu_tbl[i].can_id) {
            continue;  // ID不匹配，继续下一个
        }

        // ID匹配，根据类型检查数据
        if (g_n_sdu_tbl[i].Mtype == kRemoteDiagnostics) {
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[i].N_AI.N_AE);
        } else if (g_n_sdu_tbl[i].is_extended) {
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[i].N_AI.N_TA);
        } else {
            data_matches = true;
        }

        if (data_matches) {
            break;  // 找到匹配的SDU
        }
    }

    if(i>=kg_num_of_sdu)
    {
        data_matches=false;
    }
    
    *sdu_index=i;
    return data_matches;
}

static N_ResultEnum _DelIdle(N_SDU* p_n_sdu){

    //如果有执行接收序列
    if(p_n_sdu->l_recv_ind)
    {
        //在此之前要获取rb长度
        if(rt_ringbuffer_data_len(p_n_sdu->datalink_rb)==0)//buf为空，数据还没进rb，不需要执行
            return N_OK;//几乎不可能出现

        struct rt_canx_msg peek_rx_msg;
        rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&peek_rx_msg,8);//获取rt_canx_msg的头

        if(rt_ringbuffer_data_len(p_n_sdu->datalink_rb)<DLCtoBytes[peek_rx_msg.dlc])
            while(1);//////////////////////等待？？？一般来说不会出现这种情况

        //将头+data[dlc]的整个包读出来
        rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&peek_rx_msg,8+DLCtoBytes[peek_rx_msg.dlc]);

        //N_PCItype is the high nibble of N_PCI byte (Byte #1)
        uint8_t n_pci_type=peek_rx_msg.data[0+(p_n_sdu->Mtype|p_n_sdu->is_extended)]>>4;

        switch (n_pci_type)
        {
        case kNPciSF:
            p_n_sdu->sm.event=kDoCanTpRecvSfEvent;
            p_n_sdu->l_recv_ind=false;
            break;
        case kNPciFF:
            p_n_sdu->sm.event=kDoCanTpRecvFfEvent;
            p_n_sdu->l_recv_ind=false;
            break;
        default:
            //丢弃这一段数据
            rt_ringbuffer_get(p_n_sdu->datalink_rb,(uint8_t*)&peek_rx_msg,8+DLCtoBytes[peek_rx_msg.dlc]);
            p_n_sdu->l_recv_ind=false;
            return N_UNEXP_PDU;//Ignore Frame
            break;
        }
        return N_OK;
    }

    //如果有执行发送序列
    if(p_n_sdu->n_send_req)
    {
        MtypeEnum temp_Mtype=p_n_sdu->Mtype;
        bool temp_is_extended=p_n_sdu->is_extended;
        bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);
        bool temp_is_padding=p_n_sdu->is_padding;
        uint8_t temp_TX_DL=p_n_sdu->TX_DL;
        //判断MessageData的长度是通过单帧发送还是多帧发送
        //上层msg长度+一字节n_pci_type长度+可能存在的N_TA或者N_AE长度
        size_t temp_sf_dl=p_n_sdu->Length;
        
        if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
        {
            if(temp_sf_dl<=6)
            {
                // if(temp_is_padding)
                // {
                //     //发送8字节的单帧
                // }
                // else{//发送temp_sf_dl+2的单帧
                //     }
                //都是发送单帧
                p_n_sdu->sm.event=kDoCanTpSendSfEvent;
            }
            else//temp_sf_dl>6
            {
                // if(temp_is_padding)
                // {
                //     //填充到tx_DL的长度发送单帧
                // }
                // else
                // {
                //     //强制填充到tx_dl以内支持的最小的可容纳此帧的dlc的长度发送单帧
                // }
                //都是发送单帧
                //如果超过tx_DL-3的长度，则发送多帧
                if(temp_sf_dl>temp_TX_DL-3)
                {
                    p_n_sdu->sm.event=kDoCanTpSendFfEvent;
                }
            }
        }
        else//normal addr
        {
            if(temp_sf_dl<=7)
            {
                // if(temp_is_padding)
                // {
                //     //发送8字节的单帧
                // }
                // else{//发送temp_sf_dl+1的单帧
                //     }
                //都是发送单帧
                p_n_sdu->sm.event=kDoCanTpSendSfEvent;
            }
            else//temp_sf_dl>7
            {
                // if(temp_is_padding)
                // {
                //     //填充到tx_DL的长度发送单帧
                // }
                // else
                // {
                //     //强制填充到最小的dlc的长度发送单帧
                // }
                //如果超过tx_DL-2的长度，则发送多帧
                if(temp_sf_dl>temp_TX_DL-2)
                {
                    p_n_sdu->sm.event=kDoCanTpSendFfEvent;
                }
            }
        }
        p_n_sdu->n_send_req=false;
        return N_OK;
    }

    return N_OK;//既没有接收到新的帧，也没有发送请求
}

static N_ResultEnum _DelRxSf(N_SDU* p_n_sdu){

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->is_extended;
    bool            temp_is_padding             =p_n_sdu->is_padding;
    bool            temp_is_remote_diagnostics  =(temp_Mtype==kRemoteDiagnostics);

    //从rb读取can帧
    struct rt_canx_msg rx_msg;
    rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8);//获取rt_canx_msg的头
    //将头+data[dlc]的整个包读出来
    rt_ringbuffer_read(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8+DLCtoBytes[rx_msg.dlc]);

    uint8_t temp_sf_dl=1;//SF_DL最小的有效值
    uint8_t CAN_DL=DLCtoBytes[rx_msg.dlc];

    uint8_t N_PCI_bytes_offset=0;
    if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    {
        N_PCI_bytes_offset=1;
    }

    if(CAN_DL>8)
    {
        uint8_t n_pci_byte1=rx_msg.data[N_PCI_bytes_offset]&0x0F;

        //4 bit SF_DL value in Byte #1 low nibble equals zero
        if(n_pci_byte1!=0)
        {
            return N_UNEXP_PDU;//Ignore Frame
        }

        //Get 8 bit SF_DL value from Byte #2. Message data start offset +1
        temp_sf_dl=rx_msg.data[1+N_PCI_bytes_offset];
    }
    else//CAN_DL<=8
    {
        temp_sf_dl=rx_msg.data[N_PCI_bytes_offset]&0x0F;
    }

    //Check for valid SF_DL and process Single Frame (SF)

    // SF_DL error handling

    if(CAN_DL<=8)//Received CAN_DL is less or equal to 8:
    {
        /*If the network layer receives a SF where SF_DL is equal to 0, then 
          the network layer shall ignore the received SF N_PDU*/
        if(temp_sf_dl==0)
        {
            p_n_sdu->sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }

        /*If the network layer receives an SF where SF_DL is greater than 
          (CAN_DL – 1) of the received frame when using normal addressing or 
          greater than (CAN_DL – 2) of the received frame for extended or mixed
          addressing, then the network layer shall ignore the received SF N_PDU*/
        if(temp_is_extended||temp_is_remote_diagnostics)//extended or mixed addressing
        {
            if(temp_sf_dl>(CAN_DL-2))
            {
                p_n_sdu->sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
        }
        else//normal addressing
        {
            if(temp_sf_dl>(CAN_DL-1))
            {
                p_n_sdu->sm.event=kDoCanTpFinishEvent;
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
                p_n_sdu->sm.event=kDoCanTpFinishEvent;
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

            if(temp_sf_dl!=table12[temp_is_remote_diagnostics|temp_is_extended][rx_msg.dlc])
            {
                p_n_sdu->sm.event=kDoCanTpFinishEvent;
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
        if(table13[temp_is_remote_diagnostics|temp_is_extended][rx_msg.dlc-9].SF_DL_min>temp_sf_dl ||
        table13[temp_is_remote_diagnostics|temp_is_extended][rx_msg.dlc-9].SF_DL_max<temp_sf_dl )
        {
            p_n_sdu->sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
    }

    //千辛万苦，终于找到了一个合法的帧，接下来就是解析帧了
    uint8_t SF_DL=temp_sf_dl;//SF_DL最小的有效值

    uint8_t payload_offset=1;//接收到的can帧起码有一个字节被N_PCI bytes占用
    if(CAN_DL>8)
    {
        payload_offset++;
    }
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        payload_offset++;
    }

    //copy len=sf_dl data to Transport2SessionBuf
    memcpy(p_n_sdu->MessageData,&rx_msg.data[payload_offset],SF_DL);
    N_UserExtStruct N_UserExt={
        .can_id=rx_msg.id,
        .ide=rx_msg.ide,
        .is_extended=temp_is_extended,
    };
    // can rx中断应调用L_Data.indication()
    N_USData.indication(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                        p_n_sdu->MessageData,SF_DL,
                        N_OK,
                        N_UserExt);
    p_n_sdu->sm.event=kDoCanTpFinishEvent;
    return N_OK;
}

static N_ResultEnum _DelRxFf(N_SDU* p_n_sdu){

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->is_extended;
    bool            temp_is_padding             =p_n_sdu->is_padding;
    bool            temp_is_remote_diagnostics  =(temp_Mtype==kRemoteDiagnostics);

    //从rb读取can帧
    struct rt_canx_msg rx_msg;
    rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8);//获取rt_canx_msg的头
    //将头+data[dlc]的整个包读出来
    rt_ringbuffer_read(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8+DLCtoBytes[rx_msg.dlc]);

    // 9.6.3.2 FF_DL error handling

    /*If the network layer receives an N_PDU indicating a FirstFrame and 
      CAN_DL < 8, then the network layer shall ignore the FF N_PDU*/
    uint8_t CAN_DL=DLCtoBytes[rx_msg.dlc];
    if(CAN_DL<8)
    {
        p_n_sdu->sm.event=kDoCanTpFinishEvent;
        return N_UNEXP_PDU;
    }

    /* the retrieved value of RX_DL from the CAN_DL of the FF N_PDU (see 9.5.4 
       for definition of how the receiver determines RX_DL). */
    p_n_sdu->RX_DL=CAN_DL;

    /* The receiver determines the minimum value of FF_DL (FF_DLmin) from Table 
       14 based on the configured addressing scheme and the retrieved value of 
       RX_DL */
    //Table 14 — Minimum value of FF_DL based on the addressing scheme
    uint8_t FF_DLmin=7;
    if(p_n_sdu->RX_DL==8)
    {
        //mixed or extended addressing is used
        if(temp_is_extended||temp_is_remote_diagnostics)
            FF_DLmin=7;
        else// normal addressing is used
            FF_DLmin=8;
    }
    else//大于8
    {
        //mixed or extended addressing is used
        if(temp_is_extended||temp_is_remote_diagnostics)
            FF_DLmin=p_n_sdu->RX_DL-2;
        else// normal addressing is used
            FF_DLmin=p_n_sdu->RX_DL-1;
    }

    uint8_t ff_dl_offset=0;
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        ff_dl_offset=1;
    }

    //escape sequence or not
    uint32_t tmp_ff_dl=(((uint16_t)((rx_msg.data[ff_dl_offset+0])&0x0F))<<8)|
                                    (rx_msg.data[ff_dl_offset+1]);
    if(tmp_ff_dl==0)
    {
        // escape sequence
        tmp_ff_dl=(((uint32_t)(rx_msg.data[ff_dl_offset+0]))<<24)|
                  (((uint32_t)(rx_msg.data[ff_dl_offset+1]))<<16)|
                  (((uint32_t)(rx_msg.data[ff_dl_offset+2]))<<8)|
                  (((uint32_t)(rx_msg.data[ff_dl_offset+3]))<<0);

        /*If a FirstFrame is received with the escape sequence (where all bits of 
        the lower nibble of PCI byte 1 and all bits of PCI byte 2 are set to 0’s) 
        and the FF_DL =< 4 095, then the network layer shall ignore the FF N_PDU 
        and not transmit an FC N_PDU.*/
        if(tmp_ff_dl<=4095)
        {
            p_n_sdu->sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
    }

    /*If the network layer receives a FirstFrame with FF_DL that is greater 
      than the available receiver buffer size, then this shall be considered as
      an error condition. The network layer shall abort the message reception 
      and send an FC N_PDU with the parameter FlowStatus = Overflow.*/
    if(tmp_ff_dl>p_n_sdu->msg_buf_max_size)
    {
        p_n_sdu->sm.event=kDoCanTpSendFcEvent;
        p_n_sdu->FlowStatus=kFsOVFLW;
        return N_BUFFER_OVFLW;
    }

    /*If the network layer receives a FirstFrame with an FF_DL that is less 
      than FF_DLmin, the network layer shall ignore the received FF N_PDU and 
      not transmit a FC N_PDU.*/
    if(tmp_ff_dl<FF_DLmin)
    {
        p_n_sdu->sm.event=kDoCanTpFinishEvent;
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

    uint8_t payload_offset=2;//接收到的can帧起码有两个字节被N_PCI bytes占用
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        payload_offset++;
    }
    if(FF_DL>4095)
    {
        payload_offset+=4;
    }
    uint8_t copy_len=CAN_DL-payload_offset;
    //复制首帧中的上层数据到 uds app buf
    memcpy(p_n_sdu->MessageData,&rx_msg.data[payload_offset],copy_len);
    p_n_sdu->msg_index+=copy_len;
    p_n_sdu->sm.event=kDoCanTpSendFcEvent;
    p_n_sdu->FlowStatus=kFsCTS;
    p_n_sdu->Length=FF_DL;
    p_n_sdu->SequenceNumber++;
    N_UserExtStruct N_UserExt={
        .can_id=rx_msg.id,
        .ide=rx_msg.ide,
        .is_extended=temp_is_extended,
    };

    /* Receiver N_USDataFF.ind: the transport/network layer issues to the 
       session layer the reception of a FirstFrame of a segmented message */
    N_USData_FF.indication(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                           FF_DL,
                           N_UserExt);
    p_n_sdu->sm.event=kDoCanTpSendFcEvent;
    return N_OK;
}

static N_ResultEnum _DelRxFc(N_SDU* p_n_sdu){

    //发完首帧或者bs最后一个续帧才会有发送流控的步骤，要判断是不是有东西来了
    if(p_n_sdu->l_recv_ind==false)//当前对话确实状态机在等待流控，但是由于流控帧还没发过来，进来也没用
        return N_OK;//出现这种情况应该是状态机设计有问题

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->is_extended;
    bool            temp_is_padding             =p_n_sdu->is_padding;
    bool            temp_is_remote_diagnostics  =(temp_Mtype==kRemoteDiagnostics);

    //从rb读取can帧
    struct rt_canx_msg rx_msg;
    rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8);//获取rt_canx_msg的头
    //将头+data[dlc]的整个包读出来
    rt_ringbuffer_read(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8+DLCtoBytes[rx_msg.dlc]);

    //9.6.5.2 FlowStatus (FS) error handling

    /* If an FC N_PDU message is received with an invalid (reserved) FS 
       parameter value, the message transmission shall be aborted and the 
       network layer shall make an N_USData.confirm service call with the 
       parameter <N_Result> = N_INVALID_FS to the adjacent upper layer */
    uint8_t fs_offset=0;
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        fs_offset=1;
    }
    FlowStatusEnum temp_fs=rx_msg.data[fs_offset]&0x0F;

    if(temp_fs>kFsOVFLW)
    {
        p_n_sdu->sm.event=kDoCanTpFinishEvent;
        N_UserExtStruct N_UserExt={
            .can_id=rx_msg.id,
            .ide=rx_msg.ide,
            .is_extended=temp_is_extended,
        };
        N_USData.confirm(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,N_INVALID_FS,N_UserExt);
        return N_INVALID_FS;
    }

    // 9.6.5.3 BlockSize (BS) parameter definition
    uint8_t bs_offset=1;
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        bs_offset=2;
    }
    uint8_t temp_BS=rx_msg.data[bs_offset];

    //9.6.5.4 SeparationTime minimum (STmin) parameter definition
    uint8_t stmin_offset=2;
    if(temp_is_extended||temp_is_remote_diagnostics)
    {
        stmin_offset=3;
    }
    uint8_t temp_STmin=rx_msg.data[stmin_offset];

    //9.6.5.5 SeparationTime minimum (STmin) error handling

    /* If an FC N_PDU message is received with a reserved STmin parameter 
       value, then the sending network entity shall use the longest STmin value
       specified by this part of ISO 15765 (7F16 = 127 ms) instead of the value
       received from the receiving network entity for the duration of the 
       on-going segmented message transmission. */
    if(((temp_STmin>=0x80)&&(temp_STmin<=0xF0))||
       ((temp_STmin>=0xFA)&&(temp_STmin<=0xFF)))//0x80 – 0xF0 ro 0xFA – 0xFF
    {
        temp_STmin=0x7F;
    }

    /* If the time between two subsequent CFs of a segmented data transmission 
       (N_As + N_Cs) is smaller than the value commanded by the receiver via 
       STmin, there is no guarantee that the receiver of the segmented data 
       transmission will correctly receive and process all frames. In any case,
       the receiver of the segmented data transmission is not required to 
       monitor adherence to the STmin value. */
    //意思就是不用管

    // 9.6.5.6 Dynamic BS/STmin values in subsequent FlowControl frames
    /* If the server is the receiver of a segmented message transfer (i.e. the 
       sender of the FlowControl frame), it may choose either to use the same 
       values for BS and STmin in subsequent FC (CTS) frames of the same 
       segmented message or to vary these values from FC to FC frame. */
    //我们是client，不用管

    /* If the client, connected to an ISO 15765-compliant in-vehicle diagnostic
       network, is the receiver of a segmented message transfer (i.e. the 
       sender of the FlowControl frame), it shall use the same values for BS 
       and STmin in subsequent FC (CTS) frames of the same segmented message. */
    //这一段丢到Tx_FC

    p_n_sdu->FlowStatus=temp_fs;

    switch (temp_fs)
    {
    case kFsCTS:
        /* If the client is the sender of a segmented data transmission (i.e. 
           the receiver of the FlowControl frame), it shall adjust to the 
           values of BS and STmin from each FC (CTS) received during the same 
           segmented data transmission. */
        //只有CTS的时候才需要更新BS和STmin
        p_n_sdu->STmin=temp_STmin;
        p_n_sdu->BS=temp_BS;
        p_n_sdu->sm.event=kDoCanTpSendCfEvent;//发送续帧
        break;
    case kFsWAIT:
        p_n_sdu->sm.event=kDoCanTpRecvFcEvent;//继续接收流控帧
        break;
    case kFsOVFLW:
        //返回会话层
        ///////////////////////////////////////////
        break;
    default:
        while(1);//不可能出现
        break;
    }

    return N_OK;
}

static N_ResultEnum _DelRxCf(N_SDU* p_n_sdu){

    //从rb读取can帧
    struct rt_canx_msg rx_msg;
    rt_ringbuffer_peek(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8);//获取rt_canx_msg的头
    //将头+data[dlc]的整个包读出来
    rt_ringbuffer_read(p_n_sdu->datalink_rb,(uint8_t*)&rx_msg,8+DLCtoBytes[rx_msg.dlc]);

    /* The payload data length CAN_DL of the received CAN frame has to match 
       the RX_DL value which was determined in the reception process of the 
       FirstFrame (FF). Only the last CF in the multi-frame transmission may 
       contain less than RX_DL bytes. */

    //CAN_DL correct for the determined RX_DL
    if(p_n_sdu->RX_DL==8)//首帧接收到的长度是8字节
    {
        if(p_n_sdu->is_padding)//填充模式
        {
            if(DLCtoBytes[rx_msg.dlc]!=8)
            {
                //ignore frame
                p_n_sdu->sm.event=kDoCanTpFinishEvent;
                return N_UNEXP_PDU;
            }
        }
        else//变长模式
        {
            if((p_n_sdu->Mtype==kRemoteDiagnostics)||p_n_sdu->is_extended)
            {
                size_t remaining_length=p_n_sdu->Length-p_n_sdu->msg_index;
                if(remaining_length<=(8-2))//剩余长度<=6
                {
                    if(DLCtoBytes[rx_msg.dlc]!=remaining_length+2)
                    {
                        //ignore frame
                        p_n_sdu->sm.event=kDoCanTpFinishEvent;
                        return N_UNEXP_PDU;
                    }
                }
            }
            else//normal addressing
            {
                size_t remaining_length=p_n_sdu->Length-p_n_sdu->msg_index;
                if(remaining_length<=(8-1))//剩余长度<=7
                {
                    if(DLCtoBytes[rx_msg.dlc]!=remaining_length+1)
                    {
                        //ignore frame
                        p_n_sdu->sm.event=kDoCanTpFinishEvent;
                        return N_UNEXP_PDU;
                    }
                }
            }
        }        
    }
    else//can fd only
    {
        if(DLCtoBytes[rx_msg.dlc]!=p_n_sdu->RX_DL)
        {
            //ignore frame
            p_n_sdu->sm.event=kDoCanTpFinishEvent;
            return N_UNEXP_PDU;
        }
    }

    //SequenceNumber (SN) error handling
    /* If a CF N_PDU message is received with an unexpected SequenceNumber not
       in accordance with the definition in 9.6.4.3, the message reception 
       shall be aborted and the network layer shall make an N_USData.indication
       service call with the parameter <N_Result> = N_WRONG_SN to the adjacent 
       upper layer. */
    uint8_t SequenceNumber = rx_msg.data[0+(p_n_sdu->Mtype|p_n_sdu->is_extended)]&0x0F;
    if(SequenceNumber!=p_n_sdu->SequenceNumber)
    {
        p_n_sdu->sm.event=kDoCanTpFinishEvent;
        return N_WRONG_SN;
    }

    p_n_sdu->SequenceNumber++;
    //复制续帧的数据到sdu
    //更新sdu的index
    //判断还有没有剩余字节，或者需不需要发送流控帧

    /* Receiver N_USData.ind: the transport/network layer issues to the session
       layer the completion of the segmented message*/
    //接受完了所有帧，通知上层

    return N_OK;
}

static N_ResultEnum _DelTxSf(N_SDU* p_n_sdu){

    MtypeEnum temp_Mtype=p_n_sdu->Mtype;
    bool temp_is_extended=p_n_sdu->is_extended;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);
    bool temp_is_padding=p_n_sdu->is_padding;
    size_t temp_sf_dl=p_n_sdu->Length;
    N_TAtypeEnum temp_n_tatype=p_n_sdu->N_AI.N_TAtype;

    struct rt_canx_msg tx_msg;
    tx_msg.id=p_n_sdu->can_id;
    tx_msg.rtr=0;//不使用
    switch (temp_n_tatype)
    {
    //普通11位can
    case kPhyCanBaseFmt:
    case kFuncCanBaseFmt:
        tx_msg.ide=0;
        tx_msg.fdf=0;
        break;
    //11位canfd
    case kPhyCanFdBaseFmt:
    case kFuncCanFdBaseFmt:
        tx_msg.ide=0;
        tx_msg.fdf=1;
        break;
    //29位can
    case kPhyCanExtFmt:
    case kFuncCanExtFmt:
        tx_msg.ide=1;
        tx_msg.fdf=0;
        break;
    //29位canfd
    case kPhyCanFdExtFmt:
    case kFuncCanFdExtFmt:
        tx_msg.ide=1;
        tx_msg.fdf=1;
        break;
    default:
        while(1);
        break;
    }
    
    tx_msg.brs=1;//从帧配置来决定，一般来说canfd都开了这个东西，can配了也没用
    // tx_msg.esi=不配置
    // tx_msg.dlc=下面决定

    if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
    {
        if(temp_is_extended)
            tx_msg.data[0]=p_n_sdu->N_AI.N_TA;
        if(temp_is_remote_diagnostics)
            tx_msg.data[0]=p_n_sdu->N_AI.N_AE;
        if(temp_sf_dl<=6)
        {
            tx_msg.data[1]=temp_sf_dl;
            memcpy(&tx_msg.data[2],p_n_sdu->MessageData,temp_sf_dl);
            if(temp_is_padding)
            {
                //发送8字节的单帧
                tx_msg.dlc=8;
                memset(&tx_msg.data[2+temp_sf_dl],p_n_sdu->padding_val,8-2-temp_sf_dl);
            }
            else
            {
                //发送temp_sf_dl+2的单帧
                tx_msg.dlc=temp_sf_dl+2;//一字节N_TA或者N_AE 一字节pci
            }
        }
        else//temp_sf_dl>6
        {
            tx_msg.data[1]=0;
            tx_msg.data[2]=temp_sf_dl;
            memcpy(&tx_msg.data[3],p_n_sdu->MessageData,temp_sf_dl);
            if(temp_is_padding)
            {
                //填充到tx_DL的长度发送单帧
                tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);
                memset(&tx_msg.data[3+temp_sf_dl],p_n_sdu->padding_val,p_n_sdu->TX_DL-3-temp_sf_dl);
            }
            else
            {
                //强制填充到tx_dl以内支持的最小的可容纳此帧的dlc的长度发送单帧
                tx_msg.dlc=Bytes2ShortestDLC(temp_sf_dl+3);//一字节N_TA或者N_AE 半字节帧类型 一个半字节SF_DL转义
                memset(&tx_msg.data[3+temp_sf_dl],p_n_sdu->padding_val,DLCtoBytes[tx_msg.dlc]-3-temp_sf_dl);
            }
        }
    }
    else//normal addr
    {
        if(temp_sf_dl<=7)
        {
            if(temp_is_padding)
            {
                tx_msg.dlc=8;
                //发送8字节的单帧
            }
            else
            {
                tx_msg.dlc=temp_sf_dl+1;
                //发送temp_sf_dl+1的单帧
            }
        }
        else//temp_sf_dl>7
        {
            if(temp_is_padding)
            {
                tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);
                //填充到tx_DL的长度发送单帧
            }
            else
            {
                tx_msg.dlc=Bytes2ShortestDLC(temp_sf_dl+2);//半字节帧类型 一个半字节SF_DL转义
                //强制填充到最小的dlc的长度发送单帧
            }
        }
    }

    // L_Data.request()
    //starts the N_As timer
    return N_OK;
}

static N_ResultEnum _DelTxFf(N_SDU* p_n_sdu){

    /* Sender L_Data.req: the transport/network layer transmits the FirstFrame 
       to the data link layer and starts the N_As timer */
    // L_Data.request()
    //starts the N_As timer
    return N_OK;
}

static N_ResultEnum _DelTxFc(N_SDU* p_n_sdu){

    // 9.8.4 Wait frame error handling
    /* If the receiver has transmitted N_WFTmax FlowControl wait network 
       protocol data units (FC N_PDU WAIT) in a row and, following this, it 
       cannot meet the performance requirement for the transmission of a 
       FlowControl ContinueToSend network protocol data unit (FC N_PDU CTS), 
       then the receiver side shall abort the message reception and issue an 
       N_USData.indication with <N_Result> set to N_WFT_OVRN to the higher 
       layer. */
    //N_USData.indication()
    // N_WFTmax
    //N_WFT_OVRN

    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (ContinueToSend and BlockSize value = 2d) to the data link 
       layer and starts the N_Ar timer */
    //我们是client，bs不能变
    //start N_Ar timer
    //stop N_Br timer

    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (Wait) to the data link layer and starts the N_Ar timer */
    //某些条件不满足，发送wait
    //start N_Ar timer
    //stop N_Br timer

    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (ContinueToSend) to the data link layer and starts the N_Ar 
       timer */
    //发送cts
    //start N_Ar timer
    //stop N_Br timer
    //跟上上条很类似

    //tx_buf填充N_PCI和FS
    // rx_can_msg->data[0]=(kNPciFC<<4)|gs_docan_tp_stat.FlowStatus;
    //BS
    if(gs_docan_tp_stat.FlowStatus==kFsCTS)
    {
        // rx_can_msg->data[1]=
    }
}

static N_ResultEnum _DelTxCf(N_SDU* p_n_sdu){
    /* Sender L_Data.req: the transport/network layer transmits the first 
       ConsecutiveFrame to the data link layer and starts the N_As timer */
    //start N_As timer
    //stop N_Cs timer

    /* Sender L_Data.req: when the N_Cs timer is elapsed (STmin), the 
       transport/network layer transmits the next ConsecutiveFrame to the data 
       link layer and starts the N_As timer */
    //N_Cs到达设定的STmin就继续发续帧（如果有的话）
    //start N_As timer
    //stop N_Cs timer

    /* Sender L_Data.req: the transport/network layer transmits the 
       ConsecutiveFrame to the data link layer and starts the N_As timer */
    //start N_As timer
    //stop N_Cs timer
    //本质上跟上上条一样

    /* Sender L_Data.req: when the N_Cs timer is elapsed (STmin), the 
       transport/network layer transmits the last ConsecutiveFrame to the data 
       link layer and starts the N_As timer */
    //最后一个续帧
    //N_Cs到达设定的STmin就继续发续帧（如果有的话）
    //start N_As timer
    //stop N_Cs timer
}

/* Sender N_USData.con: the transport/network layer issues to session layer the
   completion of the segmented message */
//应该在某个函数内调用N_USData_con

static N_ResultEnum _DelError(N_SDU* p_n_sdu){}

static bool _NetworkLayerParamCheck(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE,
                                    N_UserExtStruct N_UserExt,size_t* index)
{
    //找到传入的地址信息与配置表匹配的那一个
    size_t i = 0;
    bool data_matches = false;
    for (; i < kg_num_of_sdu; i++)
    {
        if (N_UserExt.can_id != g_n_sdu_tbl[i].can_id)//id不一致直接跳过
            continue;
        // if(N_UserExt.ide!= g_n_sdu_tbl[i].ide)//有可能id一样，但是一个是11位一个是27位
        //     continue;包含在N_AI.N_TAtype中
        if(N_TAtype!=g_n_sdu_tbl[i].N_AI.N_TAtype)
            continue;
        if(Mtype!=g_n_sdu_tbl[i].Mtype)
            continue;
        if(N_AE!=g_n_sdu_tbl[i].N_AI.N_AE)
            continue;
        if(N_UserExt.is_extended!=g_n_sdu_tbl[i].is_extended)
            continue;
        if(N_TA!=g_n_sdu_tbl[i].N_AI.N_TA)
            continue;
        if(N_SA!=g_n_sdu_tbl[i].N_AI.N_SA)
            continue;

        break;
    }
    if(i>=kg_num_of_sdu)
    {
        data_matches=false;
    }

    *index=i;
    return data_matches;
}

static void N_USData_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                             uint8_t* MessageData, size_t Length, 
                             N_UserExtStruct N_UserExt)
{
    size_t sdu_index=0;
    if(false==_NetworkLayerParamCheck(Mtype, N_SA, N_TA, N_TAtype, N_AE, N_UserExt,&sdu_index))
    {
        //理论上这里要上报一个错误
        while(1);
        return;
    }
    g_n_sdu_tbl[sdu_index].Length=Length;
    g_n_sdu_tbl[sdu_index].MessageData=MessageData;
    g_n_sdu_tbl[sdu_index].n_send_req=true;
}

static void N_USData_confirm        (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, N_ResultEnum N_Result, N_UserExtStruct N_UserExt){}
static void N_USData_indication     (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_ResultEnum N_Result, N_UserExtStruct N_UserExt){}
static void N_USData_FF_indication  (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, size_t Length, N_UserExtStruct N_UserExt){}
static void N_ChangeParameter_request(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, uint8_t Parameter_Value, N_UserExtStruct N_UserExt){}
static void N_ChangeParameter_confirm(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, Result_ChangeParameterEnum Result_ChangeParameter, N_UserExtStruct N_UserExt){}

static void L_Data_request(struct rt_canx_msg* tx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    if(_SearchSduIndex(tx_can_msg,&sdu_index)==false)
        return ;

    //发一个单帧 首帧 续帧
    g_n_sdu_tbl[sdu_index].N_As_timing_enable=true;
    // 只有发续帧才关N_Cs
    g_n_sdu_tbl[sdu_index].N_Cs_timing_enable=false;
    g_n_sdu_tbl[sdu_index].N_Cs=0;

    // 发流控帧
    g_n_sdu_tbl[sdu_index].N_Br_timing_enable=false;
    g_n_sdu_tbl[sdu_index].N_Br=0;
    g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=true;
}

static void L_Data_confirm(struct rt_canx_msg* tx_can_msg, Transfer_StatusEnum Transfer_Status){

    //其实不太可能传输不成功，这个确认函数放在tx_complete_callback里
    if(Transfer_Status!=kComplete){
        while(1);
        return;
    }

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    if(_SearchSduIndex(tx_can_msg,&sdu_index)==false)
        return ;

    bool temp_is_extended=g_n_sdu_tbl[sdu_index].is_extended;
    MtypeEnum temp_Mtype=g_n_sdu_tbl[sdu_index].Mtype;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);

    uint8_t N_PCI_bytes_offset=0;
    if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    {
        N_PCI_bytes_offset=1;
    }

    //N_PCItype is the high nibble of N_PCI byte (Byte #1)
    N_PCItype n_pci_type=tx_can_msg->data[N_PCI_bytes_offset]>>4;

    switch(n_pci_type){
        
        case kNPciSF://发送单帧
            /* Sender L_Data.con: the data link layer confirms to the 
               transport/network layer that the CAN frame has been 
               acknowledged; the sender stops the N_As timer */
            g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
            g_n_sdu_tbl[sdu_index].N_As=0;
            /* Sender N_USData.con: the transport/network layer issues to the 
               session layer the completion of the unsegmented message */
            break;
        case kNPciFF://发送首帧
            /* Sender L_Data.con: the data link layer confirms to the 
               transport/network layer that the CAN frame has been 
               acknowledged; the sender stops the N_As timer and starts the 
               N_Bs timer */
            //can发送完成中断中调用L_Data.con以停止N_As timer 开启N_Bs timer
            g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
            g_n_sdu_tbl[sdu_index].N_As=0;
            g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=true;
            break;
        case kNPciCF://发送连续帧
        {
            //判断是否为最后一个续帧或者本次块传输的最后一个block
            size_t remaining_length=g_n_sdu_tbl[sdu_index].Length-(g_n_sdu_tbl[sdu_index].msg_index);
            uint8_t tmp_rx_payload_len=g_n_sdu_tbl[sdu_index].TX_DL-1;//工程配置的续帧和首帧长度
            if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
                tmp_rx_payload_len-=1;
            
            /* Sender L_Data.con: the data link layer confirms to the 
               transport/network layer that the CAN frame has been 
               acknowledged; the sender stops the N_As timer */
            if(remaining_length<=tmp_rx_payload_len)
            {
                g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_As=0;
                break;
            }

            /* Sender L_Data.con: the data link layer confirms to the 
               transport/network layer that the CAN frame has been 
               acknowledged; the sender stops the N_As timer and starts the 
               N_Bs timer; the sender is waiting for the next FlowControl. */
            //bs耗尽，等待发流控
            //stop N_As timer
            //start N_Bs timer
            if(g_n_sdu_tbl[sdu_index].BS_cnt==g_n_sdu_tbl[sdu_index].BS-1)
            {
                g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_As=0;
                g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=true;
            }
            //6 14
            /* Sender L_Data.con: the data link layer confirms to the 
               transport/network layer that the CAN frame has been 
               acknowledged; the sender stops the N_As timer and starts the 
               N_Cs timer according to the separation time value (STmin) of the
               previous FlowControl */
            //can tx ok中断要调用L_Data.con
            //stop N_As timer
            //start N_Cs timer
            else
            {
                g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_As=0;
                g_n_sdu_tbl[sdu_index].N_Cs_timing_enable=true;
            }
            break;
        }
        case kNPciFC://发送流控帧
        {
            //流控增加判断FS
            FlowStatusEnum flow_status=tx_can_msg->data[N_PCI_bytes_offset]&0x0F;
            switch(flow_status)
            {
                //4 12
                /* Receiver L_Data.con: the data link layer confirms to the 
                   transport/network layer that the CAN frame has been 
                   acknowledged; the receiver stops the N_Ar timer and starts 
                   the N_Cr timer */
                //stop N_Ar timer
                //start N_Cr timer
                case kFsCTS:
                    g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=false;
                    g_n_sdu_tbl[sdu_index].N_Ar=0;
                    g_n_sdu_tbl[sdu_index].N_Cr_timing_enable=true;
                    break;
                /* Receiver L_Data.con: the data link layer confirms to the 
                   transport/network layer that the CAN frame has been 
                   acknowledged; the receiver stops the N_Ar timer and starts 
                   the N_Br timer */
                //发完等待流控的中断
                //stop N_Ar timer
                //start N_Br timer
                case kFsWAIT:
                    g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=false;
                    g_n_sdu_tbl[sdu_index].N_Ar=0;
                    g_n_sdu_tbl[sdu_index].N_Br_timing_enable=true;
                    break;
                case kFsOVFLW:
                    //发出的帧发送完成了
                    while(1);//还不知道怎么处理
                    break;
                default:
                    //进这里相当于这个程序发出去的东西有问题
                    while(1);
                    break;
            }
            break;
        }
        default:
            while(1);
            break;
    }

}

static void L_Data_indication(struct rt_canx_msg* rx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    if(_SearchSduIndex(rx_can_msg,&sdu_index)==false)
        return ;

    //kimi建议实现的时候将地址格式判断丢到can接收中断来判断N_PCI的偏移地址
    bool temp_is_extended=g_n_sdu_tbl[sdu_index].is_extended;
    MtypeEnum temp_Mtype=g_n_sdu_tbl[sdu_index].Mtype;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);

    uint8_t N_PCI_bytes_offset=0;
    if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    {
        N_PCI_bytes_offset=1;
    }

    //N_PCItype is the high nibble of N_PCI byte (Byte #1)
    N_PCItype n_pci_type=rx_can_msg->data[N_PCI_bytes_offset]>>4;

    //Table 21 — Network layer timing parameter values
    //找到表格内L_Data.indication的条件，根据表格启停各个timer
    //不用在乎后续的帧格式对不对，头对了就启停各个timer
    switch(n_pci_type)
    {
        case kNPciSF://接收到单帧
            /* Receiver L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame */
            while(g_n_sdu_tbl[sdu_index].l_recv_ind);//单帧来的太快了
            break;
        case kNPciFF://接收到首帧
            /* Receiver L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               receiver starts the N_Br timer */
            while(g_n_sdu_tbl[sdu_index].l_recv_ind);//理论上不会出现多次接收到首帧的情况
            g_n_sdu_tbl[sdu_index].N_Br_timing_enable=true;
            break;
        case kNPciCF://接收到续帧
        {
            //续帧增加判断是否最后一个续帧
            //最后一个续帧必停N_Cr，而且最后一个续帧有可能是本次块传输的最后一块，所以先校验
            size_t remaining_length=g_n_sdu_tbl[sdu_index].Length-(g_n_sdu_tbl[sdu_index].msg_index);
            //不判断真实的dlc长度，这个放在tp_task里判断
            //接收到一个帧，就认为长度和首帧长度是一致的
            uint8_t tmp_rx_payload_len=g_n_sdu_tbl[sdu_index].RX_DL-1;//减去续帧的N_PCI bytes长度
            if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
                tmp_rx_payload_len-=1;
            /* Receiver L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               receiver stops the N_Cr timer */
            //接收最后一个续帧
            //stop N_Cr timer
            if(remaining_length<=tmp_rx_payload_len)
            {
                g_n_sdu_tbl[sdu_index].N_Cr_timing_enable=false;//最后一个续帧，必停N_Cr
                g_n_sdu_tbl[sdu_index].N_Cr=0;
                break;
            }
            /* Receiver L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               receiver stops the N_Cr timer and starts the N_Br timer */
            //接收方判断bs到达才执行这个
            if(g_n_sdu_tbl[sdu_index].BS_cnt==g_n_sdu_tbl[sdu_index].BS-1)
            {
                g_n_sdu_tbl[sdu_index].N_Cr_timing_enable=false;//bs耗尽，必停N_Cr
                g_n_sdu_tbl[sdu_index].N_Cr=0;
                g_n_sdu_tbl[sdu_index].N_Br_timing_enable=true;
            }
            //6 14
            /* Receiver L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               receiver restarts the N_Cr timer */
            //接收到续帧，重新开始N_Cr 计数
            else
                g_n_sdu_tbl[sdu_index].N_Cr=0;//bs未耗尽，重置N_Cr
            break;
        }
        case kNPciFC://接收到流控帧
        {
            //流控增加判断FS
            FlowStatusEnum flow_status=rx_can_msg->data[N_PCI_bytes_offset]&0x0F;
            switch (flow_status)
            {
            //4 12
            /* Sender L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               sender stops the N_Bs timer and starts the N_Cs timer */
            //stop N_Bs timer
            //start N_Cs timer
            case kFsCTS:
                g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_Bs=0;
                g_n_sdu_tbl[sdu_index].N_Cs_timing_enable=true;
                break;
            /* Sender L_Data.ind: the data link layer issues to the 
               transport/network layer the reception of the CAN frame; the 
               sender restarts the N_Bs timer */
            //N_Bs清零继续等待
            case kFsWAIT:
                g_n_sdu_tbl[sdu_index].N_Bs=0;
                break;
            case kFsOVFLW:
                //准备终止传输
                g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_Bs=0;
                break;
            default:
                //9.6.5.2 FlowStatus (FS) error handling
                /* If an FC N_PDU message is received with an invalid 
                   (reserved) FS parameter value, the message transmission 
                   shall be aborted and the network layer shall make an 
                   N_USData.confirm service call with the parameter <N_Result> 
                   = N_INVALID_FS to the adjacent upper layer */
                //准备终止传输
                g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=false;
                g_n_sdu_tbl[sdu_index].N_Bs=0;
                break;
            }
            break;
        }
        default:
            break;
    }
    g_n_sdu_tbl[sdu_index].l_recv_ind=true;
}