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

extern const CanIdLutStruct g_n_canid_lut_tbl[];
extern const size_t kg_num_of_canid_lut;


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
static void L_Data_confirm(uint32_t Identifier, Transfer_StatusEnum Transfer_Status);
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

bool is_wait=false;

static N_ResultEnum _Idle(N_SDU* p_n_sdu);
static N_ResultEnum _RxSf(N_SDU* p_n_sdu);
static N_ResultEnum _RxFf(N_SDU* p_n_sdu);
static N_ResultEnum _RxFc(N_SDU* p_n_sdu);
static N_ResultEnum _RxCf(N_SDU* p_n_sdu);
static N_ResultEnum _TxSf(N_SDU* p_n_sdu);
static N_ResultEnum _WaitSfOk(N_SDU* p_n_sdu);
static N_ResultEnum _TxFf(N_SDU* p_n_sdu);
static N_ResultEnum _WaitFfOk(N_SDU* p_n_sdu);
static N_ResultEnum _TxCf(N_SDU* p_n_sdu);
static N_ResultEnum _WaitCfOk(N_SDU* p_n_sdu);
static N_ResultEnum _TxFc(N_SDU* p_n_sdu);
static N_ResultEnum _WaitFcOk(N_SDU* p_n_sdu);
static N_ResultEnum _Error(N_SDU* p_n_sdu);

// docan tp层有限状态机状态转移表
const static TpStateEnum kgs_docan_tp_state_tbl[kTpEventNum][kTpStateNum]={
    /* Current State    Idle    RxSf    TxSf        WaitSfOk        RxFf    TxFc        WaitFcOk    RxCf        TxFf        WaitFfOk    RxFc    TxCf        WaitCfOk    Error */
    /* Event  */
    /*  Wait  */    {   kIdle,  kError, kWaitSfOk,  kWaitSfOk, /**/ kError, kWaitFcOk,  kWaitFcOk,  kError,/**/ kWaitFfOk,  kWaitFfOk,  kError, kWaitCfOk,  kWaitCfOk,  kError},
    /*  RxSf  */    {   kRxSf,  kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kError,     kError, kError,     kError,     kError},
    /*  RxFf  */    {   kRxFf,  kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kError,     kError, kError,     kError,     kError},
    /*  RxCf  */    {   kError, kError, kError,     kError,    /**/ kError, kError,     kRxCf,      kRxCf, /**/ kError,     kError,     kError, kError,     kError,     kError},
    /*  RxFc  */    {   kError, kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kRxFc,      kRxFc,  kError,     kRxFc,      kError},
    /*  TxSf  */    {   kTxSf,  kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kError,     kError, kError,     kError,     kError},
    /*  TxFf  */    {   kTxFf,  kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kError,     kError, kError,     kError,     kError},
    /*  TxCf  */    {   kError, kError, kError,     kError,    /**/ kError, kError,     kError,     kError,/**/ kError,     kError,     kTxCf,  kError,     kTxCf,      kError},
    /*  TxFc  */    {   kError, kError, kError,     kError,    /**/ kTxFc,  kError,     kTxFc,      kTxFc, /**/ kError,     kError,     kError, kError,     kError,     kError},
    /* Finish */    {   kError, kIdle,  kError,     kIdle,     /**/ kIdle,  kError,     kError,     kIdle, /**/ kError,     kError,     kIdle,  kError,     kIdle,      kIdle}
};
const static N_Result_pfun_p_n_sdu kgs_docan_tp_fsm_tbl[10][14]={
    /* Current State    Idle    RxSf    TxSf        WaitSfOk        RxFf    TxFc        WaitFcOk    RxCf        TxFf        WaitFfOk    RxFc    TxCf        WaitCfOk    Error */
    /* Event  */
    /*  Wait  */    {   _Idle,  _Error, _WaitSfOk,  _WaitSfOk, /**/ _Error, _WaitFcOk,  _WaitFcOk,  _Error,/**/ _WaitFfOk,  _WaitFfOk,  _Error, _WaitCfOk,  _WaitCfOk,  kError},
    /*  RxSf  */    {   _RxSf,  _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /*  RxFf  */    {   _RxFf,  _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /*  RxCf  */    {   _Error, _Error, _Error,     _Error,    /**/ _Error, _Error,     _RxCf,      _RxCf, /**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /*  RxFc  */    {   _Error, _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _RxFc,      _RxFc,  _Error,     _RxFc,      kError},
    /*  TxSf  */    {   _TxSf,  _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /*  TxFf  */    {   _TxFf,  _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /*  TxCf  */    {   _Error, _Error, _Error,     _Error,    /**/ _Error, _Error,     _Error,     _Error,/**/ _Error,     _Error,     _TxCf,  _Error,     _TxCf,      kError},
    /*  TxFc  */    {   _Error, _Error, _Error,     _Error,    /**/ _TxFc,  _Error,     _TxFc,      _TxFc, /**/ _Error,     _Error,     _Error, _Error,     _Error,     kError},
    /* Finish */    {   _Error, _Idle,  _Error,     _Idle,     /**/ _Idle,  _Error,     _Error,     _Idle, /**/ _Error,     _Error,     _Idle,  _Error,     _Idle,      kIdle}
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

static void NetworkLayer_Timing_Start(N_SDU* p_n_sdu, NetworkLayerTimingParamEnum n_xy)
{
    switch(n_xy)
    {
        case N_As:
            p_n_sdu->N_As_timing_enable=true;
            break;
        case N_Ar:
            p_n_sdu->N_Ar_timing_enable=true;
            break;
        case N_Bs:
            p_n_sdu->N_Bs_timing_enable=true;
            break;
        case N_Br:
            p_n_sdu->N_Br_timing_enable=true;
            break;
        case N_Cs:
            p_n_sdu->N_Cs_timing_enable=true;
            break;
        case N_Cr:
            p_n_sdu->N_Cr_timing_enable=true;
            break;
        default:
            break;
    }
}

static void NetworkLayer_Timing_Stop(N_SDU* p_n_sdu, NetworkLayerTimingParamEnum n_xy)
{
    switch(n_xy)
    {
        case N_As:
            p_n_sdu->N_As_timing_enable=false;
            p_n_sdu->N_As=0;
            break;
        case N_Ar:
            p_n_sdu->N_Ar_timing_enable=false;
            p_n_sdu->N_Ar=0;
            break;
        case N_Bs:
            p_n_sdu->N_Bs_timing_enable=false;
            p_n_sdu->N_Bs=0;
            break;
        case N_Br:
            p_n_sdu->N_Br_timing_enable=false;
            p_n_sdu->N_Br=0;
            break;
        case N_Cs:
            p_n_sdu->N_Cs_timing_enable=false;
            p_n_sdu->N_Cs=0;
            break;
        case N_Cr:
            p_n_sdu->N_Cr_timing_enable=false;
            p_n_sdu->N_Cr=0;
            break;
        default:
            break;
    }
}

static void NetworkLayer_Timing_Param_Init(N_SDU* p_n_sdu)
{
    NetworkLayer_Timing_Stop(p_n_sdu, N_As);
    NetworkLayer_Timing_Stop(p_n_sdu, N_Ar);
    NetworkLayer_Timing_Stop(p_n_sdu, N_Bs);
    NetworkLayer_Timing_Stop(p_n_sdu, N_Br);
    NetworkLayer_Timing_Stop(p_n_sdu, N_Cs);
    NetworkLayer_Timing_Stop(p_n_sdu, N_Cr);
}

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/**
 * @brief   网络层定时器任务
 */
void NetworkLayerTimingTask(uint16_t ms)
{
    for (size_t i = 0; i < kg_num_of_sdu; i++)
    {
        MtypeEnum       temp_Mtype      =g_n_sdu_tbl[i].Mtype;
        uint8_t         temp_N_SA       =g_n_sdu_tbl[i].N_AI.N_SA;
        uint8_t         temp_N_TA       =g_n_sdu_tbl[i].N_AI.N_TA;
        N_TAtypeEnum    temp_N_TAtype   =g_n_sdu_tbl[i].N_AI.N_TAtype;
        uint8_t         temp_N_AE       =g_n_sdu_tbl[i].N_AI.N_AE;
        N_UserExtStruct temp_N_UserExt  ={
            .is_extended=g_n_sdu_tbl[i].N_UserExt.is_extended,
            .is_fixed=g_n_sdu_tbl[i].N_UserExt.is_fixed,
        };
        
        if(g_n_sdu_tbl[i].N_As_timing_enable)
        {
            g_n_sdu_tbl[i].N_As+=ms;
        }
        if( g_n_sdu_tbl[i].N_As>g_n_sdu_tbl[i].N_As_threshold)
        {
            /* Abort message transmission and issue N_USData.confirm with
               <N_Result> = N_TIMEOUT_A */
            N_USData.confirm(temp_Mtype, temp_N_SA, temp_N_TA, temp_N_TAtype, temp_N_AE, N_TIMEOUT_A, temp_N_UserExt);
        }

        if(g_n_sdu_tbl[i].N_Bs_timing_enable)
        {
            g_n_sdu_tbl[i].N_Bs+=ms;
        }
        if( g_n_sdu_tbl[i].N_Bs>g_n_sdu_tbl[i].N_Bs_threshold)
        {
            /* Abort message transmission and issue N_USData.confirm with
               <N_Result> = N_TIMEOUT_Bs */
            N_USData.confirm(temp_Mtype, temp_N_SA, temp_N_TA, temp_N_TAtype, temp_N_AE, N_TIMEOUT_Bs, temp_N_UserExt);
        }

        if(g_n_sdu_tbl[i].N_Ar_timing_enable)
        {
            g_n_sdu_tbl[i].N_Ar+=ms;
        }
        if( g_n_sdu_tbl[i].N_Ar>g_n_sdu_tbl[i].N_Ar_threshold)
        {
            /* Abort message transmission and issue N_USData.indication with
               <N_Result> = N_TIMEOUT_A */
            N_USData.indication(temp_Mtype, temp_N_SA, temp_N_TA, temp_N_TAtype, temp_N_AE,
                                NULL, 0,
                                N_TIMEOUT_A, temp_N_UserExt);
        }

        if(g_n_sdu_tbl[i].N_Cr_timing_enable)
        {
            g_n_sdu_tbl[i].N_Cr+=ms;
        }
        if( g_n_sdu_tbl[i].N_Cr>g_n_sdu_tbl[i].N_Cr_threshold)
        {
            /* Abort message transmission and issue N_USData.confirm with
               <N_Result> = N_TIMEOUT_Cr */
            N_USData.indication(temp_Mtype, temp_N_SA, temp_N_TA, temp_N_TAtype, temp_N_AE,
                                NULL, 0,
                                N_TIMEOUT_Cr, temp_N_UserExt);
        }
    }
}

void DoCanTpTask(void)
{
    for (size_t i = 0; i < kg_num_of_sdu; i++)
    {
        //先根据事件更新状态
        g_n_sdu_tbl[i].sm.state=kgs_docan_tp_state_tbl[g_n_sdu_tbl[i].sm.event][g_n_sdu_tbl[i].sm.state];
        //执行对应的动作
        //N_Result不应该在这里更新
        g_n_sdu_tbl[i].N_Result=kgs_docan_tp_fsm_tbl[g_n_sdu_tbl[i].sm.event][g_n_sdu_tbl[i].sm.state](&g_n_sdu_tbl[i]);
    }
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
static bool _SearchSduIndex(struct rt_canx_msg* rx_can_msg,size_t* sdu_index){

    //先从查找表找到caiid对应的N_SA和N_TA
    size_t j = 0;
    for (; j < kg_num_of_canid_lut; j++)
    {
        if(rx_can_msg->id == g_n_canid_lut_tbl[j].can_id)
        {
            N_TAtypeEnum temp_ta_type=(rx_can_msg->ide<<2)|(rx_can_msg->fdf<<1)+1;
            if((temp_ta_type==g_n_canid_lut_tbl[j].N_AI.N_TAtype)||
               (temp_ta_type+1==g_n_canid_lut_tbl[j].N_AI.N_TAtype))
            {
                break;
            }
        }
    }
    if(j == kg_num_of_canid_lut)
    {
        return false;
    }

    //通过查找表的N_AI找到对应的SDU
    size_t i = 0;
    bool data_matches = false;
    for (; i < kg_num_of_sdu; i++)
    {   
        //看一下N_AI的成员是否完全相同
        if ((g_n_canid_lut_tbl[j].Mtype                 != g_n_sdu_tbl[i].Mtype)|| 
            (g_n_canid_lut_tbl[j].N_AI.N_SA             != g_n_sdu_tbl[i].N_AI.N_SA)||
            (g_n_canid_lut_tbl[j].N_AI.N_TA             != g_n_sdu_tbl[i].N_AI.N_TA)||
            (g_n_canid_lut_tbl[j].N_AI.N_TAtype         != g_n_sdu_tbl[i].N_AI.N_TAtype)||
            (g_n_canid_lut_tbl[j].N_AI.N_AE             != g_n_sdu_tbl[i].N_AI.N_AE)||
            (g_n_canid_lut_tbl[j].N_UserExt.is_extended != g_n_sdu_tbl[i].N_UserExt.is_extended)||
            (g_n_canid_lut_tbl[j].N_UserExt.is_fixed    != g_n_sdu_tbl[i].N_UserExt.is_fixed))
        {
            continue;  // ID不匹配，继续下一个
        }

        // ID匹配，根据类型检查数据
        if (g_n_sdu_tbl[i].Mtype == kRemoteDiagnostics) {//is_mixed Mixed addressing
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[i].N_AI.N_AE);
            //理论上还要对29位canid进行匹配，这里暂时不做
        } else if (g_n_sdu_tbl[i].N_UserExt.is_extended) {//10.3.4 Extended addressing
            data_matches = (rx_can_msg->data[0] == g_n_sdu_tbl[i].N_AI.N_TA);
        //10.3.3	 Normal fixed addressing
        //理论上还要
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

static bool _SearchCanIdFromSdu(N_SDU* p_n_sdu,uint32_t* canid)
{
    size_t i = 0;
    bool id_matches = false;
    for (i = 0; i < kg_num_of_canid_lut; i++)
    {
        if ((g_n_canid_lut_tbl[i].Mtype                 != p_n_sdu->Mtype)|| 
            (g_n_canid_lut_tbl[i].N_AI.N_SA             != p_n_sdu->N_AI.N_SA)||
            (g_n_canid_lut_tbl[i].N_AI.N_TA             != p_n_sdu->N_AI.N_TA)||
            (g_n_canid_lut_tbl[i].N_AI.N_TAtype         != p_n_sdu->N_AI.N_TAtype)||
            (g_n_canid_lut_tbl[i].N_AI.N_AE             != p_n_sdu->N_AI.N_AE)||
            (g_n_canid_lut_tbl[i].N_UserExt.is_extended != p_n_sdu->N_UserExt.is_extended)||
            (g_n_canid_lut_tbl[i].N_UserExt.is_fixed    != p_n_sdu->N_UserExt.is_fixed))
        {
            continue;  // ID不匹配，继续下一个
        }

        id_matches = true;
    }

    if(id_matches)
    {
        *canid=g_n_canid_lut_tbl[i].can_id;
    }

    return id_matches;
}

/**
 * @brief   只负责给can帧添加id，ide，fdf，其他交给发送函数自己添加
 */
static void _CreatTxHead(N_SDU* p_n_sdu,struct rt_canx_msg* tx_head)
{
    N_TAtypeEnum temp_n_tatype=p_n_sdu->N_AI.N_TAtype;

    uint32_t temp_can_id=0;
    if(!_SearchCanIdFromSdu(p_n_sdu,&temp_can_id))
        return;
    tx_head->id=temp_can_id;
    tx_head->rtr=0;//不使用
    switch (temp_n_tatype)
    {
    //普通11位can
    case kPhyCanBaseFmt:
    case kFuncCanBaseFmt:
        tx_head->ide=0;
        tx_head->fdf=0;
        break;
    //11位canfd
    case kPhyCanFdBaseFmt:
    case kFuncCanFdBaseFmt:
        tx_head->ide=0;
        tx_head->fdf=1;
        break;
    //29位can
    case kPhyCanExtFmt:
    case kFuncCanExtFmt:
        tx_head->ide=1;
        tx_head->fdf=0;
        break;
    //29位canfd
    case kPhyCanFdExtFmt:
    case kFuncCanFdExtFmt:
        tx_head->ide=1;
        tx_head->fdf=1;
        break;
    default:
        while(1);
        break;
    }
}

static N_ResultEnum _Idle(N_SDU* p_n_sdu){

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
        uint8_t n_pci_type=peek_rx_msg.data[0+(p_n_sdu->Mtype|p_n_sdu->N_UserExt.is_extended)]>>4;

        switch (n_pci_type)
        {
        case kNPciSF:
            p_n_sdu->sm.event=kRxSfEvent;
            p_n_sdu->l_recv_ind=false;
            break;
        case kNPciFF:
            p_n_sdu->sm.event=kRxFfEvent;
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
        bool temp_is_extended=p_n_sdu->N_UserExt.is_extended;
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
                p_n_sdu->sm.event=kTxSfEvent;
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
                    p_n_sdu->sm.event=kTxFfEvent;
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
                p_n_sdu->sm.event=kTxSfEvent;
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
                    p_n_sdu->sm.event=kTxFfEvent;
                }
            }
        }
        p_n_sdu->n_send_req=false;
        return N_OK;
    }

    return N_OK;//既没有接收到新的帧，也没有发送请求
}

///////////////////////////////////////////////////////////////////////////////

static N_ResultEnum _RxSf(N_SDU* p_n_sdu){

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->N_UserExt.is_extended;
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
            p_n_sdu->sm.event=kFinishEvent;
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
                p_n_sdu->sm.event=kFinishEvent;
                return N_UNEXP_PDU;
            }
        }
        else//normal addressing
        {
            if(temp_sf_dl>(CAN_DL-1))
            {
                p_n_sdu->sm.event=kFinishEvent;
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
                p_n_sdu->sm.event=kFinishEvent;
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
                p_n_sdu->sm.event=kFinishEvent;
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
            p_n_sdu->sm.event=kFinishEvent;
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
        // .can_id=rx_msg.id,
        .is_extended=temp_is_extended,
    };
    // can rx中断应调用L_Data.indication()
    /* Receiver N_USData.ind: the transport/network layer issues to the session
       layer the completion of the unsegmented message */
    N_USData.indication(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                        p_n_sdu->MessageData,SF_DL,
                        N_OK,
                        N_UserExt);
    p_n_sdu->sm.event=kFinishEvent;
    //p_n_sdu的参数该清零清零
    return N_OK;
}

static N_ResultEnum _TxSf(N_SDU* p_n_sdu){

    MtypeEnum temp_Mtype=p_n_sdu->Mtype;
    bool temp_is_extended=p_n_sdu->N_UserExt.is_extended;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);
    bool temp_is_padding=p_n_sdu->is_padding;
    size_t temp_sf_dl=p_n_sdu->Length;

    struct rt_canx_msg tx_msg;
    _CreatTxHead(p_n_sdu,&tx_msg);
    
    tx_msg.brs=1;//从帧配置来决定，一般来说canfd都开了这个东西，can配了也没用
    // tx_msg.esi=不配置
    // tx_msg.dlc=下面决定

    if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
    {
        if(temp_is_extended)
            tx_msg.data[0]=p_n_sdu->N_AI.N_TA;
        else if(temp_is_remote_diagnostics)
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
            tx_msg.data[0]=temp_sf_dl;
            memcpy(&tx_msg.data[1],p_n_sdu->MessageData,temp_sf_dl);
            if(temp_is_padding)
            {
                //发送8字节的单帧
                tx_msg.dlc=8;
                memset(&tx_msg.data[1+temp_sf_dl],p_n_sdu->padding_val,8-1-temp_sf_dl);
            }
            else
            {
                //发送temp_sf_dl+1的单帧
                tx_msg.dlc=temp_sf_dl+1;
            }
        }
        else//temp_sf_dl>7
        {
            tx_msg.data[0]=0;
            tx_msg.data[1]=temp_sf_dl;
            memcpy(&tx_msg.data[2],p_n_sdu->MessageData,temp_sf_dl);
            if(temp_is_padding)
            {
                //填充到tx_DL的长度发送单帧
                tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);
                memset(&tx_msg.data[2+temp_sf_dl],p_n_sdu->padding_val,p_n_sdu->TX_DL-2-temp_sf_dl);
            }
            else
            {
                //强制填充到最小的dlc的长度发送单帧
                tx_msg.dlc=Bytes2ShortestDLC(temp_sf_dl+2);//半字节帧类型 一个半字节SF_DL转义
                memset(&tx_msg.data[2+temp_sf_dl],p_n_sdu->padding_val,DLCtoBytes[tx_msg.dlc]-2-temp_sf_dl);
            }
        }
    }

    /* Sender L_Data.req: the transport/network layer transmits the SingleFrame
       to the data link layer and starts the N_As timer */
    L_Data.request(&tx_msg);
    NetworkLayer_Timing_Start(p_n_sdu,N_As);//启动N_As定时器

    p_n_sdu->sm.event=kWaitEvent;

    return N_OK;
}

static N_ResultEnum _WaitSfOk(N_SDU* p_n_sdu){

    if(p_n_sdu->l_send_con)
    {
        p_n_sdu->sm.event=kFinishEvent;//如果有send.con就是完成
        p_n_sdu->l_send_con=false;

        MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
        uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
        uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
        N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
        uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
        bool            temp_is_extended            =p_n_sdu->N_UserExt.is_extended;
        N_UserExtStruct N_UserExt={
            // .can_id=p_n_sdu->can_id,
            .is_extended=temp_is_extended,
        };
    
        /* Sender N_USData.con: the transport/network layer issues to the 
           session layer the completion of the unsegmented message */
        N_USData.confirm(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                         N_OK,
                         N_UserExt);
        //p_n_sdu的参数该清零清零
        NetworkLayer_Timing_Param_Init(p_n_sdu);
    }
    else
        p_n_sdu->sm.event=kWaitEvent;
    return N_OK;
}

///////////////////////////////////////////////////////////////////////////////

static N_ResultEnum _RxFf(N_SDU* p_n_sdu){

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->N_UserExt.is_extended;
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
        p_n_sdu->sm.event=kFinishEvent;
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
            p_n_sdu->sm.event=kFinishEvent;
            return N_UNEXP_PDU;
        }
    }

    /*If the network layer receives a FirstFrame with FF_DL that is greater 
      than the available receiver buffer size, then this shall be considered as
      an error condition. The network layer shall abort the message reception 
      and send an FC N_PDU with the parameter FlowStatus = Overflow.*/
    if(tmp_ff_dl>p_n_sdu->msg_buf_max_size)
    {
        p_n_sdu->sm.event=kTxFcEvent;
        p_n_sdu->FlowStatus=kFsOVFLW;
        return N_BUFFER_OVFLW;
    }

    /*If the network layer receives a FirstFrame with an FF_DL that is less 
      than FF_DLmin, the network layer shall ignore the received FF N_PDU and 
      not transmit a FC N_PDU.*/
    if(tmp_ff_dl<FF_DLmin)
    {
        p_n_sdu->sm.event=kFinishEvent;
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
    p_n_sdu->sm.event=kTxFcEvent;
    p_n_sdu->FlowStatus=kFsCTS;
    p_n_sdu->Length=FF_DL;
    p_n_sdu->SequenceNumber++;
    N_UserExtStruct N_UserExt={
        // .can_id=rx_msg.id,
        .is_extended=temp_is_extended,
    };

    /* Receiver N_USDataFF.ind: the transport/network layer issues to the 
       session layer the reception of a FirstFrame of a segmented message */
    N_USData_FF.indication(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                           FF_DL,
                           N_UserExt);
    NetworkLayer_Timing_Start(p_n_sdu,N_Br);
    p_n_sdu->sm.event=kTxFcEvent;
    return N_OK;
}

static N_ResultEnum _TxFc(N_SDU* p_n_sdu){

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->N_UserExt.is_extended;
    bool            temp_is_padding             =p_n_sdu->is_padding;
    bool            temp_is_remote_diagnostics  =(temp_Mtype==kRemoteDiagnostics);

    struct rt_canx_msg tx_msg;
    _CreatTxHead(p_n_sdu,&tx_msg);

    //这个应该丢去 _WaitFcOk执行
    // 9.8.4 Wait frame error handling
    /* If the receiver has transmitted N_WFTmax FlowControl wait network 
       protocol data units (FC N_PDU WAIT) in a row and, following this, it 
       cannot meet the performance requirement for the transmission of a 
       FlowControl ContinueToSend network protocol data unit (FC N_PDU CTS), 
       then the receiver side shall abort the message reception and issue an 
       N_USData.indication with <N_Result> set to N_WFT_OVRN to the higher 
       layer. */
    if(p_n_sdu->N_WFTcnt>=p_n_sdu->N_WFTmax)
    {
        N_UserExtStruct N_UserExt={
            // .can_id=tx_msg.id,
            .is_extended=temp_is_extended,
        };
        N_USData.indication(temp_Mtype,temp_N_SA,temp_N_TA,temp_N_TAtype,temp_N_AE,
                            NULL,0,
                            N_WTF_OVRN,
                            N_UserExt);
        p_n_sdu->N_WFTcnt=0;//清零
    }

    uint8_t data_offset=0;
    if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
    {
        if(temp_is_extended)
            tx_msg.data[0]=temp_N_TA;
        else if(temp_is_remote_diagnostics)
            tx_msg.data[0]=temp_N_AE;

        data_offset++;
    }

    tx_msg.data[data_offset+0]=kNPciFC<<4;

    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (Wait) to the data link layer and starts the N_Ar timer */
    //某些条件不满足，发送wait
    if(is_wait)
    {
        tx_msg.data[data_offset+0]|=kFsWAIT;
        /*  If FlowStatus is set to Wait, the values of BS (BlockSize) and 
            STmin (SeparationTime minimum) in the FlowControl message are not 
            relevant and shall be ignored. */
        //写不写也没什么屌用
        tx_msg.data[data_offset+1]=p_n_sdu->BS;
        tx_msg.data[data_offset+2]=p_n_sdu->STmin;
        //可能要填充
        //dlc要动
        //直接发出去，然后在con那边wait++
        L_Data.request(&tx_msg);
        return ;//下面的不跑了
    }
    
    //3 11
    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (ContinueToSend and BlockSize value = 2d) to the data link 
       layer and starts the N_Ar timer */
    //我们是client，bs不能变

    /* Receiver L_Data.req: the transport/network layer transmits the 
       FlowControl (ContinueToSend) to the data link layer and starts the N_Ar 
       timer */
    //跟上一条很类似
    tx_msg.data[data_offset+0]|=kFsCTS;
    tx_msg.data[data_offset+1]|=p_n_sdu->BS;
    tx_msg.data[data_offset+2]|=p_n_sdu->STmin;
    //可能要填充
    //dlc要动
    L_Data.request(&tx_msg);

    p_n_sdu->sm.event=kWaitEvent;
    
}

static N_ResultEnum _WaitFcOk(N_SDU* p_n_sdu){
    return N_OK;
}

static N_ResultEnum _RxCf(N_SDU* p_n_sdu){

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
                p_n_sdu->sm.event=kFinishEvent;
                return N_UNEXP_PDU;
            }
        }
        else//变长模式
        {
            if((p_n_sdu->Mtype==kRemoteDiagnostics)||p_n_sdu->N_UserExt.is_extended)
            {
                size_t remaining_length=p_n_sdu->Length-p_n_sdu->msg_index;
                if(remaining_length<=(8-2))//剩余长度<=6
                {
                    if(DLCtoBytes[rx_msg.dlc]!=remaining_length+2)
                    {
                        //ignore frame
                        p_n_sdu->sm.event=kFinishEvent;
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
                        p_n_sdu->sm.event=kFinishEvent;
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
            p_n_sdu->sm.event=kFinishEvent;
            return N_UNEXP_PDU;
        }
    }

    //SequenceNumber (SN) error handling
    /* If a CF N_PDU message is received with an unexpected SequenceNumber not
       in accordance with the definition in 9.6.4.3, the message reception 
       shall be aborted and the network layer shall make an N_USData.indication
       service call with the parameter <N_Result> = N_WRONG_SN to the adjacent 
       upper layer. */
    uint8_t SequenceNumber = rx_msg.data[0+(p_n_sdu->Mtype|p_n_sdu->N_UserExt.is_extended)]&0x0F;
    if(SequenceNumber!=p_n_sdu->SequenceNumber)
    {
        p_n_sdu->sm.event=kFinishEvent;
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

///////////////////////////////////////////////////////////////////////////////

static N_ResultEnum _TxFf(N_SDU* p_n_sdu){

    MtypeEnum temp_Mtype=p_n_sdu->Mtype;
    bool temp_is_extended=p_n_sdu->N_UserExt.is_extended;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);
    bool temp_is_padding=p_n_sdu->is_padding;
    size_t temp_ff_dl=p_n_sdu->Length;

    struct rt_canx_msg tx_msg;
    _CreatTxHead(p_n_sdu,&tx_msg);
    
    tx_msg.brs=1;//从帧配置来决定，一般来说canfd都开了这个东西，can配了也没用
    // tx_msg.esi=不配置
    tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);//首帧的dlc长度永远是tp层定义的最大发送长度

    if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
    {
        if(temp_is_extended)
            tx_msg.data[0]=p_n_sdu->N_AI.N_TA;
        else if(temp_is_remote_diagnostics)
            tx_msg.data[0]=p_n_sdu->N_AI.N_AE;
        if(temp_ff_dl<=0xFFF)
        {
            tx_msg.data[1]=0x10|(temp_ff_dl>>8);
            tx_msg.data[2]=temp_ff_dl&0xFF;
            //填充到tx_DL的长度
            memcpy(&tx_msg.data[3],p_n_sdu->MessageData,p_n_sdu->TX_DL-3);
            p_n_sdu->msg_index=p_n_sdu->TX_DL-3;
        }
        else//超过0xFFF
        {
            tx_msg.data[1]=0x10;
            tx_msg.data[2]=0x00;
            tx_msg.data[3]=temp_ff_dl>>24;
            tx_msg.data[4]=temp_ff_dl>>16;
            tx_msg.data[5]=temp_ff_dl>>8;
            tx_msg.data[6]=temp_ff_dl&0xFF;
            //填充到tx_DL的长度
            memcpy(&tx_msg.data[7],p_n_sdu->MessageData,p_n_sdu->TX_DL-7);
            p_n_sdu->msg_index=p_n_sdu->TX_DL-7;
        }
    }
    else//normal address
    {
        if(temp_ff_dl<=0xFFF)
        {
            tx_msg.data[0]=0x10|(temp_ff_dl>>8);
            tx_msg.data[1]=temp_ff_dl&0xFF;
            //填充到tx_DL的长度
            memcpy(&tx_msg.data[2],p_n_sdu->MessageData,p_n_sdu->TX_DL-2);
            p_n_sdu->msg_index=p_n_sdu->TX_DL-2;
        }
        else//超过0xFFF
        {
            tx_msg.data[0]=0x10;
            tx_msg.data[1]=0x00;
            tx_msg.data[2]=temp_ff_dl>>24;
            tx_msg.data[3]=temp_ff_dl>>16;
            tx_msg.data[4]=temp_ff_dl>>8;
            tx_msg.data[5]=temp_ff_dl&0xFF;
            //填充到tx_DL的长度
            memcpy(&tx_msg.data[6],p_n_sdu->MessageData,p_n_sdu->TX_DL-6);
            p_n_sdu->msg_index=p_n_sdu->TX_DL-6;
        }
    }

    /* Sender L_Data.req: the transport/network layer transmits the FirstFrame 
       to the data link layer and starts the N_As timer */
    L_Data.request(&tx_msg);
    p_n_sdu->N_As_timing_enable=true;

    p_n_sdu->sm.event=kWaitEvent;

    return N_OK;
}

static N_ResultEnum _WaitFfOk(N_SDU* p_n_sdu){
    
    if(p_n_sdu->l_send_con)
    {
        p_n_sdu->sm.event=kFinishEvent;//如果有send.con就是完成
        p_n_sdu->l_send_con=false;
    }
    else
        p_n_sdu->sm.event=kRxFcEvent;
    return N_OK;
}

static N_ResultEnum _RxFc(N_SDU* p_n_sdu){

    //发完首帧或者bs最后一个续帧才会有发送流控的步骤，要判断是不是有东西来了
    if(p_n_sdu->l_recv_ind==false)//当前对话确实状态机在等待流控，但是由于流控帧还没发过来，进来也没用
        return N_OK;//出现这种情况应该是状态机设计有问题

    //本函数内的临时变量，降低代码长度
    MtypeEnum       temp_Mtype                  =p_n_sdu->Mtype;
    uint8_t         temp_N_SA                   =p_n_sdu->N_AI.N_SA;
    uint8_t         temp_N_TA                   =p_n_sdu->N_AI.N_TA;
    N_TAtypeEnum    temp_N_TAtype               =p_n_sdu->N_AI.N_TAtype;
    uint8_t         temp_N_AE                   =p_n_sdu->N_AI.N_AE;
    bool            temp_is_extended            =p_n_sdu->N_UserExt.is_extended;
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
        p_n_sdu->sm.event=kFinishEvent;
        N_UserExtStruct N_UserExt={
            // .can_id=rx_msg.id,
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
        p_n_sdu->sm.event=kTxCfEvent;//发送续帧
        break;
    case kFsWAIT:
        p_n_sdu->sm.event=kRxFcEvent;//继续接收流控帧
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

static N_ResultEnum _TxCf(N_SDU* p_n_sdu){

    MtypeEnum temp_Mtype=p_n_sdu->Mtype;
    bool temp_is_extended=p_n_sdu->N_UserExt.is_extended;
    bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);
    bool temp_is_padding=p_n_sdu->is_padding;
    size_t temp_ff_dl=p_n_sdu->Length;

    struct rt_canx_msg tx_msg;
    _CreatTxHead(p_n_sdu,&tx_msg);
    
    tx_msg.brs=1;//从帧配置来决定，一般来说canfd都开了这个东西，can配了也没用
    // tx_msg.esi=不配置

    // 从需要剩余的msg长度决定dlc长度
    size_t remaining_length=p_n_sdu->Length-p_n_sdu->msg_index;

    if(temp_is_extended||temp_is_remote_diagnostics)//带N_TA或者N_AE
    {
        if(temp_is_extended)
            tx_msg.data[0]=p_n_sdu->N_AI.N_TA;
        else if(temp_is_remote_diagnostics)
            tx_msg.data[0]=p_n_sdu->N_AI.N_AE;
        tx_msg.data[1]=(kNPciCF<<4)|p_n_sdu->SequenceNumber;

        if(remaining_length>p_n_sdu->TX_DL-2)//还有后续的续帧
        {
            tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);
            memcpy(&tx_msg.data[2],p_n_sdu->MessageData+p_n_sdu->msg_index,p_n_sdu->TX_DL-2);
            p_n_sdu->msg_index+=p_n_sdu->TX_DL-2;
            if(p_n_sdu->BS_cnt==p_n_sdu->BS)
            {
                p_n_sdu->BS_cnt=1;//从1开始算

            }
            else if(p_n_sdu->BS_cnt<p_n_sdu->BS)
            {
                p_n_sdu->BS_cnt++;

            }
            else//倒反天罡，计数值大于设计值
                while(1);
        }
        else//发完这帧就没有续帧了
        {
            if(temp_is_padding)
            {
                tx_msg.dlc=Bytes2ShortestDLC(p_n_sdu->TX_DL);
                memcpy(&tx_msg.data[2],p_n_sdu->MessageData+p_n_sdu->msg_index,remaining_length);
                memset(&tx_msg.data[2+remaining_length],p_n_sdu->padding_val,p_n_sdu->TX_DL-2-remaining_length);
            }
            else
            {
                tx_msg.dlc=Bytes2ShortestDLC(remaining_length);
                memcpy(&tx_msg.data[2],p_n_sdu->MessageData+p_n_sdu->msg_index,remaining_length);
                //强制填充到支持的最短的帧的长度
                memset(&tx_msg.data[2+remaining_length],p_n_sdu->padding_val,DLCtoBytes[tx_msg.dlc]-2-remaining_length);
            }
            
        }
    }
    else//normal address
    {
        if(remaining_length>p_n_sdu->TX_DL-1)//还有后续的续帧
        {
            //要判断bs
        }
        else//发完这帧就没有续帧了
        {

        }
    }

    p_n_sdu->N_Cs_timing_enable=false;
    p_n_sdu->N_Cs=0;
    p_n_sdu->N_As_timing_enable=true;

    /* Sender L_Data.req: the transport/network layer transmits the first 
       ConsecutiveFrame to the data link layer and starts the N_As timer */
    //start N_As timer
    //stop N_Cs timer
    //接受完流控的第一个续帧
    //每次发之前都要检查剩余的长度够不够再发下一个续帧，这决定开不开cs或者bs

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

static N_ResultEnum _WaitCfOk(N_SDU* p_n_sdu){
    return N_OK;
}

static N_ResultEnum _Error(N_SDU* p_n_sdu){}

///////////////////////////////////////////////////////////////////////////////

static bool _NetworkLayerParamCheck(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE,
                                    N_UserExtStruct N_UserExt,size_t* index)
{
    //找到传入的地址信息与配置表匹配的那一个
    size_t i = 0;
    bool data_matches = false;
    for (; i < kg_num_of_sdu; i++)
    {
        // if (N_UserExt.can_id != g_n_sdu_tbl[i].can_id)//id不一致直接跳过
        //     continue;
        // if(N_UserExt.ide!= g_n_sdu_tbl[i].ide)//有可能id一样，但是一个是11位一个是27位
        //     continue;包含在N_AI.N_TAtype中
        if(N_TAtype!=g_n_sdu_tbl[i].N_AI.N_TAtype)
            continue;
        if(Mtype!=g_n_sdu_tbl[i].Mtype)
            continue;
        if(N_AE!=g_n_sdu_tbl[i].N_AI.N_AE)
            continue;
        if(N_UserExt.is_extended!=g_n_sdu_tbl[i].N_UserExt.is_extended)
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

static void N_USData_request            (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
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

static void N_USData_confirm            (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                                         N_ResultEnum N_Result, 
                                         N_UserExtStruct N_UserExt){}
static void N_USData_indication         (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                                         uint8_t* MessageData, size_t Length, N_ResultEnum N_Result, 
                                         N_UserExtStruct N_UserExt){}
static void N_USData_FF_indication      (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                                         size_t Length, 
                                         N_UserExtStruct N_UserExt){}
static void N_ChangeParameter_request   (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                                         uint8_t Parameter, uint8_t Parameter_Value, 
                                         N_UserExtStruct N_UserExt){}
static void N_ChangeParameter_confirm   (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, 
                                         uint8_t Parameter, Result_ChangeParameterEnum Result_ChangeParameter, 
                                         N_UserExtStruct N_UserExt){}

///////////////////////////////////////////////////////////////////////////////

static void L_Data_request(struct rt_canx_msg* tx_can_msg){

    // 调用一个can发送函数
    // HAL_FDCAN_AddMessageToTxBuffer

    size_t sdu_index=0;
    if(_SearchSduIndex(tx_can_msg,&sdu_index)==false)
        return ;

    //时间参数放这里不太好
    // switch (UserExtPciType)
    // {
    // case kNPciSF://发一个单帧
    //     g_n_sdu_tbl[sdu_index].N_As_timing_enable=true;
    //     break;
    // case kNPciFF://发一个首帧
    //     g_n_sdu_tbl[sdu_index].N_As_timing_enable=true;
    //     break;
    // case kNPciCF://发一个续帧
    //     g_n_sdu_tbl[sdu_index].N_Cs_timing_enable=false;
    //     g_n_sdu_tbl[sdu_index].N_Cs=0;
    //     g_n_sdu_tbl[sdu_index].N_As_timing_enable=true;
    //     break;
    // case kNPciFC://发一个流控帧
    //     g_n_sdu_tbl[sdu_index].N_Br_timing_enable=false;
    //     g_n_sdu_tbl[sdu_index].N_Br=0;
    //     g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=true;
    //     break;
    // default:
    //     while(1);
    //     break;
    // }
}

static void L_Data_confirm(uint32_t Identifier, Transfer_StatusEnum Transfer_Status)
{
    //其实不太可能传输不成功，这个确认函数放在tx_complete_callback里
    if(Transfer_Status!=kComplete){
        while(1);
        return;
    }

    //根据PDU的ID查SDU tbl
    // size_t sdu_index=0;
    // if(_SearchSduIndex(tx_can_msg,&sdu_index)==false)
    //     return ;

    // bool temp_is_extended=g_n_sdu_tbl[sdu_index].N_UserExt.is_extended;
    // MtypeEnum temp_Mtype=g_n_sdu_tbl[sdu_index].Mtype;
    // bool temp_is_remote_diagnostics=(temp_Mtype==kRemoteDiagnostics);

    // uint8_t N_PCI_bytes_offset=0;
    // if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    // {
    //     N_PCI_bytes_offset=1;
    // }

    // //N_PCItype is the high nibble of N_PCI byte (Byte #1)
    // N_PCItype n_pci_type=tx_can_msg->data[N_PCI_bytes_offset]>>4;

    // switch(n_pci_type){
        
    //     case kNPciSF://发送单帧
    //         /* Sender L_Data.con: the data link layer confirms to the 
    //            transport/network layer that the CAN frame has been 
    //            acknowledged; the sender stops the N_As timer */
    //         g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
    //         g_n_sdu_tbl[sdu_index].N_As=0;
    //         break;
    //     case kNPciFF://发送首帧
    //         /* Sender L_Data.con: the data link layer confirms to the 
    //            transport/network layer that the CAN frame has been 
    //            acknowledged; the sender stops the N_As timer and starts the 
    //            N_Bs timer */
    //         //can发送完成中断中调用L_Data.con以停止N_As timer 开启N_Bs timer
    //         g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
    //         g_n_sdu_tbl[sdu_index].N_As=0;
    //         g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=true;
    //         break;
    //     case kNPciCF://发送连续帧
    //     {
    //         //判断是否为最后一个续帧或者本次块传输的最后一个block
    //         size_t remaining_length=g_n_sdu_tbl[sdu_index].Length-(g_n_sdu_tbl[sdu_index].msg_index);
    //         uint8_t tmp_rx_payload_len=g_n_sdu_tbl[sdu_index].TX_DL-1;//工程配置的续帧和首帧长度
    //         if(temp_is_remote_diagnostics||temp_is_extended)//带N_AE或者N_TA在第一字节的can帧
    //             tmp_rx_payload_len-=1;
            
    //         /* Sender L_Data.con: the data link layer confirms to the 
    //            transport/network layer that the CAN frame has been 
    //            acknowledged; the sender stops the N_As timer */
    //         if(remaining_length<=tmp_rx_payload_len)
    //         {
    //             g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
    //             g_n_sdu_tbl[sdu_index].N_As=0;
    //             break;
    //         }

    //         /* Sender L_Data.con: the data link layer confirms to the 
    //            transport/network layer that the CAN frame has been 
    //            acknowledged; the sender stops the N_As timer and starts the 
    //            N_Bs timer; the sender is waiting for the next FlowControl. */
    //         //bs耗尽，等待发流控
    //         //stop N_As timer
    //         //start N_Bs timer
    //         if(g_n_sdu_tbl[sdu_index].BS_cnt==g_n_sdu_tbl[sdu_index].BS-1)
    //         {
    //             g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
    //             g_n_sdu_tbl[sdu_index].N_As=0;
    //             g_n_sdu_tbl[sdu_index].N_Bs_timing_enable=true;
    //         }
    //         //6 14
    //         /* Sender L_Data.con: the data link layer confirms to the 
    //            transport/network layer that the CAN frame has been 
    //            acknowledged; the sender stops the N_As timer and starts the 
    //            N_Cs timer according to the separation time value (STmin) of the
    //            previous FlowControl */
    //         //can tx ok中断要调用L_Data.con
    //         //stop N_As timer
    //         //start N_Cs timer
    //         else
    //         {
    //             g_n_sdu_tbl[sdu_index].N_As_timing_enable=false;
    //             g_n_sdu_tbl[sdu_index].N_As=0;
    //             g_n_sdu_tbl[sdu_index].N_Cs_timing_enable=true;
    //         }
    //         break;
    //     }
    //     case kNPciFC://发送流控帧
    //     {
    //         //流控增加判断FS
    //         FlowStatusEnum flow_status=tx_can_msg->data[N_PCI_bytes_offset]&0x0F;
    //         switch(flow_status)
    //         {
    //             //4 12
    //             /* Receiver L_Data.con: the data link layer confirms to the 
    //                transport/network layer that the CAN frame has been 
    //                acknowledged; the receiver stops the N_Ar timer and starts 
    //                the N_Cr timer */
    //             //stop N_Ar timer
    //             //start N_Cr timer
    //             case kFsCTS:
    //                 g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=false;
    //                 g_n_sdu_tbl[sdu_index].N_Ar=0;
    //                 g_n_sdu_tbl[sdu_index].N_Cr_timing_enable=true;
    //                 break;
    //             /* Receiver L_Data.con: the data link layer confirms to the 
    //                transport/network layer that the CAN frame has been 
    //                acknowledged; the receiver stops the N_Ar timer and starts 
    //                the N_Br timer */
    //             //发完等待流控的中断
    //             //stop N_Ar timer
    //             //start N_Br timer
    //             case kFsWAIT:
    //                 g_n_sdu_tbl[sdu_index].N_Ar_timing_enable=false;
    //                 g_n_sdu_tbl[sdu_index].N_Ar=0;
    //                 g_n_sdu_tbl[sdu_index].N_Br_timing_enable=true;
    //                 break;
    //             case kFsOVFLW:
    //                 //发出的帧发送完成了
    //                 while(1);//还不知道怎么处理
    //                 break;
    //             default:
    //                 //进这里相当于这个程序发出去的东西有问题
    //                 while(1);
    //                 break;
    //         }
    //         break;
    //     }
    //     default:
    //         while(1);
    //         break;
    // }
    // g_n_sdu_tbl[sdu_index].l_send_con=true;

}

static void L_Data_indication(struct rt_canx_msg* rx_can_msg){

    //根据PDU的ID查SDU tbl
    size_t sdu_index=0;
    if(_SearchSduIndex(rx_can_msg,&sdu_index)==false)
        return ;

    //kimi建议实现的时候将地址格式判断丢到can接收中断来判断N_PCI的偏移地址
    bool temp_is_extended=g_n_sdu_tbl[sdu_index].N_UserExt.is_extended;
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