// /******************************************************************************
//  * NanoH7 - UAV firmware base on RT-Thread
//  * Copyright (C) 2023 - 2025 Althrone <mail>
//  * 
//  * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\CanNm\CanNm.h
//  * 
//  * ref: Specification of <some UM RM or Datasheet>
//  *****************************************************************************/

// //不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
// #ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_CanNm_CanNm_h_
// #define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_CanNm_CanNm_h_

// #ifdef __cplusplus
// extern "C" {
// #endif

// /******************************************************************************
//  * includes
//  *****************************************************************************/

// #include <stdint.h>
// #include "arm_math.h"//引入float32_t
// #include "rtdef.h"//引入rt_bool_t

// //SWS_CanNm_00245
// // #include "ComStackTypes.h"
// #include "NmStack_types.h"//SWS_CanNm_00309
// // #include "StandardTypes.h"

// /******************************************************************************
//  * macros
//  *****************************************************************************/

// #define CANNM_NUM_OF_CHANNEL    1//一个通道代表一个can总线？

// /******************************************************************************
//  * pubilc types
//  *****************************************************************************/

// typedef enum
// {
//     CANNM_PDU_BYTE_0,
//     CANNM_PDU_BYTE_1,
//     CANNM_PDU_OFF
// }CanNmPduPositionEnum;

// typedef struct
// {
//     //Type      Name                            SWS Item            Min     Max     unit
//     uint16_t    Id;                             //ECUC_CanNm_00054  0       65535
//     PduInfoType Ref;                            //ECUC_CanNm_00039
// }CanNm_PduType;

// typedef struct
// {
//     //Type      Name                            SWS Item            Min     Max     unit
//     rt_bool_t   CanNmActiveWakeupBitEnabled;    //ECUC_CanNm_00084
//     rt_bool_t   CanNmAllNmMessagesKeepAwake;    //ECUC_CanNm_00068
//     rt_bool_t   CanNmBusLoadReductionActive;    //ECUC_CanNm_00042
//     uint8_t     CanNmCarWakeUpBitPosition;      //ECUC_CanNm_00075  0       7
//     uint8_t     CanNmCarWakeUpBytePosition;     //ECUC_CanNm_00076  0       7
//     rt_bool_t   CanNmCarWakeUpFilterEnabled;    //ECUC_CanNm_00077
//     rt_uint8_t  CanNmCarWakeUpFilterNodeId;     //ECUC_CanNm_00078  0       255
//     rt_bool_t   CanNmCarWakeUpRxEnabled;        //ECUC_CanNm_00074
//     float32_t   CanNmImmediateNmCycleTime;      //ECUC_CanNm_00057  0.001   65.535  s/秒
//     uint8_t     CanNmImmediateNmTransmissions;  //ECUC_CanNm_00056  0       255     times/次数
//     float32_t   CanNmMsgCycleOffset;            //ECUC_CanNm_00029  0       65.535  s/秒
//     float32_t   CanNmMsgCycleTime;              //ECUC_CanNm_00028  0.001   65.535  s/秒
//     float32_t   CanNmMsgReducedTime;            //ECUC_CanNm_00043  0.001   65.535  s/秒
//     uint8_t     CanNmMsgTimeoutTime;            //ECUC_CanNm_00030  0.001   65.535  s/秒
//     rt_bool_t   CanNmNodeDetectionEnabled;      //ECUC_CanNm_00088
//     uint8_t     CanNmNodeId;                    //ECUC_CanNm_00031  0       255     canid&0xFF
//     //          CanNmNodeIdEnabled;             //ECUC_CanNm_00090
//     CanNmPduPositionEnum    CanNmPduCbvPosition;//ECUC_CanNm_00026
//     CanNmPduPositionEnum    CanNmPduNidPosition;//ECUC_CanNm_00025
//     rt_bool_t   CanNmPnEnabled;                 //ECUC_CanNm_00066
//     rt_bool_t   CanNmPnEraCalcEnabled;          //ECUC_CanNm_00067
//     rt_bool_t   CanNmPnHandleMultipleNetworkRequests;   //ECUC_CanNm_00073
//     float32_t   CanNmRemoteSleepIndTime;        //ECUC_CanNm_00023  0.001   65.535  s/秒
//     float32_t   CanNmRepeatMessageTime;         //ECUC_CanNm_00022  0       65.535  s/秒
//     rt_bool_t   CanNmRepeatMsgIndEnabled;       //ECUC_CanNm_00089
//     float32_t   CanNmTimeoutTime;               //ECUC_CanNm_00020  0.002   65.535  s/秒
//     float32_t   CanNmWaitBusSleepTime;          //ECUC_CanNm_00021  0.001   65.535  s/秒
//     //todo      CanNmComMNetworkHandleRef;      //ECUC_CanNm_00018 ComMChannel
//     PduInfoType CanNmPnEraRxNSduRef;            //ECUC_CanNm_00079
//     CanNm_PduType CanNmRxPdu;                   //ECUC_CanNm_00038  1       *
//     CanNm_PduType CanNmTxPdu;                   //ECUC_CanNm_00036  0       1
//     CanNm_PduType CanNmUserDataTxPdu;           //ECUC_CanNm_00045  0       1
// }CanNm_ChannelConfigType;

// typedef struct
// {
//     uint8_t     CanNmPnFilterMaskByteIndex;     //ECUC_CanNm_00063  0       6
//     uint8_t     CanNmPnFilterMaskByteValue;     //ECUC_CanNm_00064  0       255
// }CanNm_PnFilterMaskByteType;

// typedef struct
// {
//     //Type      Name                            SWS Item            Min     Max     unit
//     uint8_t     CanNmPnInfoLength;              //ECUC_CanNm_00061  1       7
//     uint8_t     CanNmPnInfoOffset;              //ECUC_CanNm_00060  1       7
//     CanNm_PnFilterMaskByteType CanNmPnFilterMaskByte[1];//ECUC_CanNm_00069
// }CanNm_PnInfoType;

// typedef struct
// {
//     //Type      Name                            SWS Item            Min     Max     unit
//     //          CanNmBusLoadReductionEnabled;   //ECUC_CanNm_00040
//     //          CanNmBusSynchronizationEnabled; //ECUC_CanNm_00006
//     //          CanNmComControlEnabled;         //ECUC_CanNm_00013
//     //          CanNmComUserDataSupport;        //ECUC_CanNm_00044
//     rt_bool_t   CanNmCoordinatorSyncSupport;    //ECUC_CanNm_00080
//     rt_bool_t   CanNmDevErrorDetect;            //ECUC_CanNm_00002
//     //          CanNmGlobalPnSupport;           //ECUC_CanNm_00086
//     //          CanNmImmediateRestartEnabled;   //ECUC_CanNm_00009
//     rt_bool_t   CanNmImmediateTxconfEnabled;    //ECUC_CanNm_00041
//     float32_t   CanNmMainFunctionPeriod;        //ECUC_CanNm_00032  0       INF(-1) s/秒
//     //          CanNmPassiveModeEnabled;        //ECUC_CanNm_00010
//     //          CanNmPduRxIndicationEnabled;    //ECUC_CanNm_00011
//     rt_bool_t   CanNmPnEiraCalcEnabled;         //ECUC_CanNm_00070
//     float32_t   CanNmPnResetTime;               //ECUC_CanNm_00059  0.001   65.535  s/秒
//     //          CanNmRemoteSleepIndEnabled;     //ECUC_CanNm_00055
//     //          CanNmStateChangeIndEnabled;     //ECUC_CanNm_00012
//     //          CanNmUserDataEnabled;           //ECUC_CanNm_00004
//     //          CanNmVersionInfoApi;            //ECUC_CanNm_00003
//     PduInfoType CanNmPnEiraRxNSduRef;           //ECUC_CanNm_00072
//     CanNm_ChannelConfigType CanNmChannelConfig[CANNM_NUM_OF_CHANNEL]; //ECUC_CanNm_00017
//     CanNm_PnInfoType        CanNmPnInfo;        //ECUC_CanNm_00071

// }CanNm_GlobalConfigType;

// typedef CanNm_GlobalConfigType CanNm_ConfigType;

// typedef struct
// {
//     uint8_t     SrcNodeId;
//     struct
//     {
//         uint8_t RepeatMessageRequest:1;
//         uint8_t NmCoordinatorId:2;
//         uint8_t NmCoordinatorSleepReady:1;
//         uint8_t ActiveWakeup:1;
//         uint8_t :1;
//         uint8_t PartialNetworkInformation:1;
//         uint8_t :1;
//     }CBV;
    
// };

// #define CANNM_USER_DATA_ENABLED RT_TRUE//ECUC_CanNm_00004   Pre-processor switch    CanNmUserDataEnabled

// // NmUserDataLength???????????


// /******************************************************************************
//  * pubilc variables declaration
//  *****************************************************************************/

// /******************************************************************************
//  * pubilc functions declaration
//  *****************************************************************************/

// #ifdef __cplusplus
// }
// #endif

// #endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_CanNm_CanNm_h_ */