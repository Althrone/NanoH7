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

#include "rtthread.h"
#include "rtdevice.h"
#include "iso15765_2_cfg.h"
#include "iso14229_1_cfg.h"

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
    kNPciSF,//单帧
    kNPciFF,//首帧
    kNPciCF,//续帧
    kNPciFC,//流控帧
}N_PCItype;//Table 8 — Definition of N_PCItype bit values

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