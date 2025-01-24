/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso14229_1.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "iso14229_1.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

//限定每个服务允许的NRC
typedef struct
{
    UdsAppSiEnum    service_id;
    union
    {
        rt_uint8_t R[32];
        struct
        {
            rt_uint8_t PR      :1;
            rt_uint8_t GR      :1;
            rt_uint8_t SNS     :1;
            rt_uint8_t SFNS    :1;
            rt_uint8_t IMLOIF  :1;
            rt_uint8_t RTL     :1;
            rt_uint8_t BRR     :1;
            rt_uint8_t CNC     :1;
            rt_uint8_t RSE     :1;
            rt_uint8_t NRFSC   :1;
            rt_uint8_t FPEORA  :1;
            rt_uint8_t ROOR    :1;
            rt_uint8_t SAD     :1;
            rt_uint8_t AR      :1;
            rt_uint8_t IK      :1;
            rt_uint8_t ENOA    :1;
            rt_uint8_t RTDNE   :1;
            rt_uint8_t SDTR    :1;
            rt_uint8_t SDTNA   :1;
            rt_uint8_t SDVF    :1;
            rt_uint8_t CVFITP  :1;
            rt_uint8_t CVFIS   :1;
            rt_uint8_t CVFICOT :1;
            rt_uint8_t CVFIT   :1;
            rt_uint8_t CVFIF   :1;
            rt_uint8_t CVFIC   :1;
            rt_uint8_t CVFISC   :1;
            rt_uint8_t CVFICE   :1;
            rt_uint8_t OVF     :1;
            rt_uint8_t CCF     :1;
            rt_uint8_t SARF    :1;
            rt_uint8_t SKCDF   :1;
            rt_uint8_t CDUF    :1;
            rt_uint8_t DAF     :1;
            rt_uint8_t UDNA    :1;
            rt_uint8_t TDS     :1;
            rt_uint8_t GPF     :1;
            rt_uint8_t WBSC    :1;
            rt_uint8_t RCRRP   :1;
            rt_uint8_t SFNSIAS :1;
            rt_uint8_t SNSIAS  :1;
            rt_uint8_t RPMTH   :1;
            rt_uint8_t RPMTL   :1;
            rt_uint8_t EIR     :1;
            rt_uint8_t EINR    :1;
            rt_uint8_t ERTTL   :1;
            rt_uint8_t TEMPTH  :1;
            rt_uint8_t TEMPTL  :1;
            rt_uint8_t VSTH    :1;
            rt_uint8_t VSTL    :1;
            rt_uint8_t TPTH    :1;
            rt_uint8_t TPTL    :1;
            rt_uint8_t TRNIN   :1;
            rt_uint8_t TRNIG   :1;
            rt_uint8_t BSNC    :1;
            rt_uint8_t SLNIP   :1;
            rt_uint8_t TCCL    :1;
            rt_uint8_t VTH     :1;
            rt_uint8_t VTL     :1;
            rt_uint8_t RTNA    :1;
        }B;
    }allow_nrc;
    rt_uint8_t      allow_section;
}UdsAppSrvCfgStruct;

const UdsAppSrvCfgStruct g_uds_app_srv_cfg[]={
    {//诊断会话
        .service_id=kUdsAppDscSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
    },
    {//ECU复位
        .service_id=kUdsAppErSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.SAD=1,
    },
    {//安全访问
        .service_id=kUdsAppSaSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.IK=1,
        .allow_nrc.B.ENOA=1,
        .allow_nrc.B.RTDNE=1,
    },
    {//通讯控制
        .service_id=kUdsAppCcSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
    },
    {//认证服务
        .service_id=kUdsAppArsSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.RSE=1,
    },
    {//测试器存在
        .service_id=kUdsAppTpSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
    },
    {//控制DTC设置
        .service_id=kUdsAppCdtcsSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
    },
    {//ResponseOnEvent
        .service_id=kUdsAppRoeSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
    },
    {//链路控制
        .service_id=kUdsAppLcSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
    },
    {//按DID读取数据
        .service_id=kUdsAppRdbiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.RTL=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
    },
    {//按地址读取memory
        .service_id=kUdsAppRmbaSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.RTL=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
    },
    {//按标识符读取缩放数据
        .service_id=kUdsAppRsdbiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
    },
    {//通过周期标识符读取数据
        .service_id=kUdsAppRdbpiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
    },
    {//动态定义数据标识符
        .service_id=kUdsAppDddiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.GPF=1,
    },
    {//WriteDataByIdentifier
        .service_id=kUdsAppWdbiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.GPF=1,
    },
    {//WriteMemoryByAddress
        .service_id=kUdsAppWmbaSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
        .allow_nrc.B.GPF=1,
    },
    {//ClearDiagnosticInformation
        .service_id=kUdsAppCdtciSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.GPF=1,
    },
    {//ReadDTCInformation
        .service_id=kUdsAppRdtciSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.ROOR=1,
    },
    {//InputOutputControlByIdentifier
        .service_id=kUdsAppIocbiSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
    },
    {//RoutineControl
        .service_id=kUdsAppRcSi,
        .allow_nrc.B.SFNS=1,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.GPF=1,
    },
    {//RequestDownload
        .service_id=kUdsAppRdSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
        .allow_nrc.B.UDNA=1,
    },
    {//RequestUpload
        .service_id=kUdsAppRuSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
        .allow_nrc.B.UDNA=1,
    },
    {//TransferData
        .service_id=kUdsAppTdSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.TDS=1,
        .allow_nrc.B.GPF=1,
        .allow_nrc.B.WBSC=1,
        .allow_nrc.B.VTH=1,
        .allow_nrc.B.VTL=1,
    },
    {//RequestTransferExit
        .service_id=kUdsAppRteSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.GPF=1,
    },
    {//RequestFileTransfer
        .service_id=kUdsAppRftSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.CNC=1,
        .allow_nrc.B.RSE=1,
        .allow_nrc.B.ROOR=1,
        .allow_nrc.B.SAD=1,
        .allow_nrc.B.AR=1,
        .allow_nrc.B.UDNA=1,
    },
    {//SecuredDataTransmission
        .service_id=kUdsAppSdtSi,
        .allow_nrc.B.IMLOIF=1,
        .allow_nrc.B.SDVF=1,
    },
};

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void UdsInit(void)
{
    //根据UDS_FF_DL
}

void UdsAppTask(void)
{

}

void UdsTask(void)
{
    // UdsTpTask();
    UdsAppTask();
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
