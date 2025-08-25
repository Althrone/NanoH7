/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso15765.h
 * 
 * ref: Specification of <ISO 15765-2:2016>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_PROTOCOL_UDS_ISO15765_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_PROTOCOL_UDS_ISO15765_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <stdint.h>
#include <time.h>
#include <stdbool.h>

#include "rtdevice.h"//引入rt_canx_msg类型

#include "iso15765_2_cfg.h"
// #include "iso14229_1_cfg.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    N_OK = 0,       /*This value means that the service execution has completed
                      successfully; it can be issued to a service user on both 
                      the sender and receiver side*/
    N_TIMEOUT_A,    /*This value is issued to the protocol user when the timer 
                      N_Ar/N_As has passed its time-out value N_Asmax/N_Armax; 
                      it can be issued to service user on both the sender and 
                      receiver side.*/
    N_TIMEOUT_Bs,   /*This value is issued to the service user when the timer 
                      N_Bs has passed its time-out value N_Bsmax; it can be 
                      issued to the service user on the sender side only.*/
    N_TIMEOUT_Cr,   /*This value is issued to the service user when the timer 
                      N_Cr has passed its time-out value N_Crmax; it can be 
                      issued to the service user on the receiver side only.*/
    N_WRONG_SN,     /*This value is issued to the service user upon reception 
                      of an unexpected sequence number (PCI.SN) value; it can 
                      be issued to the service user on the receiver side only.*/
    N_INVALID_FS,   /*This value is issued to the service user when an invalid
                      or unknown FlowStatus value has been received in a flow 
                      control (FC) N_PDU; it can be issued to the service user 
                      on the sender side only.*/
    N_UNEXP_PDU,    /*This value is issued to the service user upon reception 
                      of an unexpected protocol data unit; it can be issued to 
                      the service user on the receiver side only.*/
    N_WTF_OVRN,     /*This value is issued to the service user upon reception 
                      of flow control WAIT frame that exceeds the maximum 
                      counter N_WFTmax.*/
    N_BUFFER_OVFLW, /*This value is issued to the service user upon reception 
                      of a flow control (FC) N_PDU with FlowStatus = OVFLW. It 
                      indicates that the buffer on the receiver side of a 
                      segmented message transmission cannot store the number of
                      bytes specified by the FirstFrame DataLength (FF_DL) 
                      parameter in the FirstFrame and therefore the 
                      transmission of the segmented message was aborted. It can
                      be issued to the service user on the sender side only.*/
    N_ERROR         /*This is the general error value. It shall be issued to 
                      the service user when an error has been detected by the 
                      network layer and no other parameter value can be used to
                      better describe the error. It can be issued to the 
                      service user on both the sender and receiver side.*/
}N_Result;

typedef enum
{
    // N_OK,               /*This value means that the service execution has been 
                        //   completed successfully; it can be issued to a service
                        //   user on both the sender and receiver sides.*/
    N_RX_ON=1,            /*This value is issued to the service user to indicate 
                          that the service did not execute since reception of 
                          the message identified by <N_AI> was taking place; it
                          can be issued to the service user on the receiver 
                          side only.*/
    N_WRONG_PARAMETER,  /*This value is issued to the service user to indicate 
                          that the service did not execute due to an undefined 
                          <Parameter>; it can be issued to a service user on 
                          both the receiver and sender sides.*/
    N_WRONG_VALUE,      /*This value is issued to the service user to indicate 
                          that the service did not execute due to an 
                          out-of-range <Parameter_Value>; it can be issued to a
                          service user on both the receiver and sender sides.*/
}Result_ChangeParameter;

typedef enum
{
    //both ISO 17987-2 and ISO 15765-2
    kNPciSF,//单帧
    kNPciFF,//首帧
    kNPciCF,//续帧
    //ISO 15765-2 only
    kNPciFC,//流控帧
}N_PCItype;//Table 8 — Definition of N_PCItype bit values

typedef enum
{
    kDiagnostics=0,
    kRemoteDiagnostics
}MtypeEnum;

typedef enum
{
    kPhyCanBaseFmt=1,
    kFuncCanBaseFmt,
    kPhyCanFdBaseFmt,
    kFuncCanFdBaseFmt,
    kPhyCanExtFmt,
    kFuncCanExtFmt,
    kPhyCanFdExtFmt,
    kFuncCanFdExtFmt,
}N_TAtypeEnum;

typedef struct
{
    uint8_t N_TA;
    uint8_t N_SA;
    N_TAtypeEnum N_TAtype;
    uint8_t N_AE;
}N_AIStruct;

typedef enum
{
    kFsCTS,
    kFsWAIT,
    kFsOVFLW,
}FlowStatusEnum;

typedef struct
{
    MtypeEnum Mtype;
    N_AIStruct N_AI;
    uint8_t STmin;  //可修改
    uint8_t BS;     //可修改
    size_t sdu_buf_size; //对应多帧接收缓冲区大小
    uint8_t* sdu_buf;//传输给上层的buf,与ff_dl相关
    //非iso参数
    uint32_t can_id;
    bool is_extended;//设置此参数时N_TA有效,与Mtype互斥
    bool is_padding;//仅dlc<=8时有效
    FlowStatusEnum FlowStatus;
    uint8_t SequenceNumber;
    size_t sdu_index;//多帧传输时当前已经接收/发送的字节数
    size_t sdu_len;//=ff_dl
    uint8_t BS_cnt;
    size_t N_WFTmax;//固定值，发送流控的时候才用，指示我们（当前是client）在接收多帧时可以发送的多少个等待流控
    size_t N_WFTcnt;
    //从FF帧获取的参数，与sf no padding无关
    union
    {
        uint8_t RX_DL;//默认为8
        uint8_t TX_DL;//发送用的，这个配置了就不能被程序修改
    };
}N_SDU;

typedef struct
{
    uint32_t can_id;
    bool ide;
    bool is_extended;//设置此参数时N_TA有效,与Mtype互斥
}N_UserExtStruct;

struct N_USDataOps
{
    void (*request)     (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_UserExtStruct N_UserExt);
    void (*confirm)     (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, N_Result Result, N_UserExtStruct N_UserExt);
    void (*indication)  (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t* MessageData, size_t Length, N_Result Result, N_UserExtStruct N_UserExt);
};
struct N_USData_FFOps
{
    void (*indication)  (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, size_t Length, N_UserExtStruct N_UserExt);
};
struct N_ChangeParameterOps
{
    void (*request)     (MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, uint8_t Parameter_Value, N_UserExtStruct N_UserExt);
    Result_ChangeParameter (*confirm)(MtypeEnum Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtypeEnum N_TAtype, uint8_t N_AE, uint8_t Parameter, N_UserExtStruct N_UserExt);
};

typedef enum
{
    kComplete,
    kNot_Complete,
    kAborted,
}Transfer_StatusEnum;

struct L_DataOps
{
    void (*request)(struct rt_canx_msg* tx_can_msg);
    void (*confirm)(struct rt_canx_msg* tx_can_msg, Transfer_StatusEnum Transfer_Status);
    void (*indication)(struct rt_canx_msg* rx_can_msg);
};

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_PROTOCOL_UDS_ISO15765_H_ */