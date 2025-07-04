// /******************************************************************************
//  * NanoH7 - UAV firmware base on RT-Thread
//  * Copyright (C) 2023 - 2025 Althrone <mail>
//  * 
//  * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\CanNm\CanNm.c
//  * 
//  * ref: Specification of <some UM RM or Datasheet>
//  *****************************************************************************/

// /******************************************************************************
//  * includes
//  *****************************************************************************/

// #include "CanNm.h"

// #include "stm32h7xx_hal_def.h"

// /******************************************************************************
//  * private macros
//  *****************************************************************************/

// // CanNmChannelConfig ggg={
// //     .CanNmTimeoutTime=2,
// //     .CanNmMsgCycleTime=0.5,
// //     .CanNmRepeatMessageTime=1.5,
// //     .CanNmWaitBusSleepTime=2,
// //     // .CanNmMsgCycleOffset=na,
// //     .CanNmPduNidPosition=CANNM_PDU_BYTE_0,
// //     .CanNmPduCbvPosition=CANNM_PDU_BYTE_1,
// //     .CanNmImmediateNmCycleTime=0.02,
// //     .CanNmImmediateNmTransmissions=10,
// //     .CanNmNodeId=0xB5,
// //     // .CanNmPnEnabled=naaaaaaaaaa,
// // };

// // CanNmPnInfo info={
// //     // .CanNmPnInfoLength=na,
// //     // .CanNmPnInfoOffset=na,
// // };

// // // NmUserDataLength=6;

// // CanNmGlobalConfig gggg={
// //     // .CanNmPnResetTime=naaaaaaaaa,
// // };

// /******************************************************************************
//  * pubilc variables
//  *****************************************************************************/

// /******************************************************************************
//  * private types
//  *****************************************************************************/

// typedef struct
// {
//     Nm_StateType    CanNmChannelState[CANNM_NUM_OF_CHANNEL];
//     Nm_ModeType     CanNmModuleState;
//     HAL_LockTypeDef Lock;
// }CanNm_UserType;

// CanNm_UserType      CanNmUser;


// /******************************************************************************
//  * private variables
//  *****************************************************************************/

// /******************************************************************************
//  * private functions declaration
//  *****************************************************************************/

// /******************************************************************************
//  * pubilc functions definition
//  *****************************************************************************/

// void CanNm_Init(const CanNm_ConfigType* cannmConfigPtr)
// {
//     for (size_t i = 0; i < CANNM_NUM_OF_CHANNEL; i++)
//     {
//         //状态机进入总线休眠
//         CanNmUser.CanNmChannelState[i]=NM_STATE_BUS_SLEEP;//SWS_CanNm_00141
//         //网络设为已释放状态
//         //SWS_CanNm_00143
        
//     }
//     //CanNm模块进入总线休眠状态
//     CanNmUser.CanNmModuleState=NM_MODE_BUS_SLEEP;//SWS_CanNm_00144
    
// }

// /******************************************************************************
//  * private functions definition
//  *****************************************************************************/
