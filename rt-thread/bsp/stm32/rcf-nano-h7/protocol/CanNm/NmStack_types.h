/**
 * OpenAUTOSAR - Open Source Automotive Open System Architect Labrary
 * Copyright (C) 2023 Althrone <mail>
 *
 * ref: Specification of NetworkManagement Interface AUTOSAR CP Release 4.4.0
 **/

#ifndef OPENAUTOSAR_NM_STACK_TYPES_H_
#define OPENAUTOSAR_NM_STACK_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    NM_MODE_BUS_SLEEP,          //Bus-Sleep Mode
    NM_MODE_PREPARE_BUS_SLEEP,  //Prepare-Bus Sleep Mode
    NM_MODE_SYNCHRONIZE,        //Synchronize Mode
    NM_MODE_NETWORK,            //Network Mode
}Nm_ModeType;                   //SWS_Nm_00274

typedef enum
{
    NM_STATE_UNINIT,            //
    NM_STATE_BUS_SLEEP,         //
    NM_STATE_PREPARE_BUS_SLEEP, //
    NM_STATE_READY_SLEEP,       //
    NM_STATE_NORMAL_OPERATION,  //
    NM_STATE_REPEAT_MESSAGE,    //
    NM_STATE_SYNCHRONIZE,       //
    NM_STATE_OFFLINE,           //
}Nm_StateType;                  //SWS_Nm_00275 */

typedef enum {
    NM_BUSNM_CANNM,             //CAN NM type
    NM_BUSNM_FRNM,              //FR NM type
    NM_BUSNM_UDPNM,             //UDP NM type
    NM_BUSNM_GENERICNM,         //Generic NM type
    NM_BUSNM_UNDEF,             //NM type undefined; it shall be defined as FFh
    NM_BUSNM_J1939NM,           //SAE J1939 NM type (address claiming)
    NM_BUSNM_LOCALNM,           //Local NM Type
}Nm_BusNmType;                  //SWS_Nm_00276

/******************************************************************************
 * pubilc functions
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OPENAUTOSAR_NM_STACK_TYPES_H_ */