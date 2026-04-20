/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2026 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso15765_3.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_3_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_3_h_

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

/**
 * @brief Table A.1 — dataIdentifier definitions for network configuration data
 */
typedef enum
{
    kNetCfgDataDidAF        = 0xF010,   /**< addressFormat */
    kNetCfgDataDidRSPCID    = 0xF011,   /**< responseCANId */
    kNetCfgDataDidPRQCID    = 0xF012,   /**< physicalRequestCANId */
    kNetCfgDataDidFRQCID    = 0xF013,   /**< functionalRequestCANId */
    kNetCfgDataDidAFRQCID   = 0xF014,   /**< allFunctionalRequestCANIds */
    kNetCfgDataDidRSA       = 0xF015,   /**< remoteServerAddress */
    kNetCfgDataDidSDA       = 0xF016,   /**< serverDiagnosticAddress */
}NetCfgDataDidEnum;

/**
 * @brief Table A.6 — serverAddressFormat data parameter value definition
 */
typedef enum
{
    kSrvAddrFmtNAF  = 0x01, /**< normalAddressingFormat */
    kSrvAddrFmtFNAF = 0x02, /**< fixedNormalAddressingFormat */
    kSrvAddrFmtEAF  = 0x03, /**< extendedAddressingFormat */
    kSrvAddrFmtMAF  = 0x04, /**< mixedAddressingFormat */
}SrvAddrFmtEnum;

// CID0007
// CID0815
// CID1623
// 11BCID2428
// 29BCID2428

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_3_h_ */