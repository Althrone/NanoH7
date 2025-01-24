/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso15765_2_cfg.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_2_cfg_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_2_cfg_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

/******************************************************************************
 * macros
 *****************************************************************************/

#define DOCAN_TP_FF_DL      4095

#define DOCAN_TP_CAN_TYPE_CAN   0
#define DOCAN_TP_CAN_TYPE_CANFD 1
#define DOCAN_TP_CAN_TYPE   DOCAN_TP_CAN_TYPE_CANFD

#define DOCAN_TP_CANID_LEN_11   0
#define DOCAN_TP_CANID_LEN_29   1
#define DOCAN_TP_CANID_LEN  DOCAN_TP_CANID_LEN_11

#define DOCAN_TP_NORMAL_ADDR        0
#define DOCAN_TP_NORMAL_FIX_ADDR    1
#define DOCAN_TP_EXT_ADDR           2
#define DOCAN_TP_MIX29_ADDR         3
#define DOCAN_TP_MIX11_ADDR         4
#define DOCAN_TP_ADDR_FORMAT        DOCAN_TP_NORMAL_ADDR

#define DOCAN_TP_PHY_ADDR           0x18ce3214//0x700 0x18da3214
#define DOCAN_TP_FUNC_ADDR          0x18cd3214//0x7DF 0x18d23214
#define DOCAN_TP_RES_ADDR           0x701

#if (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)
    // #define DOCAN_TP_PHY_N_TA       0x000
    #ifndef DOCAN_TP_PHY_N_TA
        #error ext addr must define N_TA
    #endif
#endif

#if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR))
    #define DOCAN_TP_PHY_N_AE       0x000
    #ifndef DOCAN_TP_PHY_N_AE
        #error ext addr must define N_AE
    #endif
#endif

#define DOCAN_TP_DEFAULT_PADDIND_VALUE  0xCC
// #define DOCAN_TP_PADDIND_VALUE
#ifndef DOCAN_TP_PADDIND_VALUE
    #define DOCAN_TP_PADDIND_VALUE DOCAN_TP_DEFAULT_PADDIND_VALUE
#endif

#define DOCAN_TP_DATA_LEN_FIX 0
#define DOCAN_TP_DATA_LEN_AUTO 1
#define DOCAN_TP_DATA_LEN_MODE DOCAN_TP_DATA_LEN_AUTO

#if (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CAN)
#define DOCAN_TP_TX_DL 8    //DO NOT CONFIG!!!
#define DOCAN_TP_RX_DL 8    //DO NOT CONFIG!!!

#if ((DOCAN_TP_TX_DL!=8)&&(DOCAN_TP_RX_DL!=8))
    #error
#endif
#elif (DOCAN_TP_CAN_TYPE==DOCAN_TP_CAN_TYPE_CANFD)
//Table 6 — Definition of TX_DL configuration values
//不能小于8
//等于8时，如果是auto模式，实际上的CAN_DL可以是2~8，如果是fix，CAN_DL严格等于8，没数据的用PADDING填充
//大于8时，如果是auto模式，实际上的CAN_DL可以是2~8以及小于等于TX_DL的值，没数据的用PADDING填充，如果是fix，CAN_DL严格等于TX_DL，没数据的用PADDING填充
#define DOCAN_TP_TX_DL 12
#define DOCAN_TP_RX_DL 64

#if ((DOCAN_TP_TX_DL!=8)&&(DOCAN_TP_TX_DL!=12)&&(DOCAN_TP_TX_DL!=16)&&(DOCAN_TP_TX_DL!=20)&&\
     (DOCAN_TP_TX_DL!=24)&&(DOCAN_TP_TX_DL!=32)&&(DOCAN_TP_TX_DL!=48)&&(DOCAN_TP_TX_DL!=64))
    #error
#endif
#endif

//定时参数
#define N_AS_TIMEOUT_MS 1000
#define N_AR_TIMEOUT_MS 1000
#define N_BS_TIMEOUT_MS 1000
#define N_BR_TIMEOUT_MS 1000
#define N_CS_TIMEOUT_MS 1000
#define N_CR_TIMEOUT_MS 1000

//以下为自动处理部分

#if ((DOCAN_TP_MAX_DATA_LEN!=0)&&(DOCAN_TP_MAX_DATA_LEN!=1)&&(DOCAN_TP_MAX_DATA_LEN!=2)&&(DOCAN_TP_MAX_DATA_LEN!=3)&&\
     (DOCAN_TP_MAX_DATA_LEN!=4)&&(DOCAN_TP_MAX_DATA_LEN!=5)&&(DOCAN_TP_MAX_DATA_LEN!=6)&&(DOCAN_TP_MAX_DATA_LEN!=7)&&\
     (DOCAN_TP_MAX_DATA_LEN!=8)&&(DOCAN_TP_MAX_DATA_LEN!=12)&&(DOCAN_TP_MAX_DATA_LEN!=16)&&(DOCAN_TP_MAX_DATA_LEN!=20)&&\
     (DOCAN_TP_MAX_DATA_LEN!=24)&&(DOCAN_TP_MAX_DATA_LEN!=32)&&(DOCAN_TP_MAX_DATA_LEN!=48)&&(DOCAN_TP_MAX_DATA_LEN!=64))
    #error
#endif

#if (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_ADDR)
    #define DOCAN_TP_PHY_N_AI       DOCAN_TP_PHY_ADDR
    #define DOCAN_TP_FUNC_N_AI      DOCAN_TP_FUNC_ADDR
#elif (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR)
    #if((DOCAN_TP_PHY_ADDR>>16)!=0x18DA)
        #error phy addr format error
    #endif
    #if((DOCAN_TP_FUNC_ADDR>>16)!=0x18DB)
        #error func  addr format error
    #endif
    #define DOCAN_TP_PHY_N_TA       (DOCAN_TP_PHY_ADDR&0xFF00)>>8
    #define DOCAN_TP_PHY_N_SA       DOCAN_TP_PHY_ADDR&0xFF
    #define DOCAN_TP_FUNC_N_TA      (DOCAN_TP_FUNC_ADDR&0xFF00)>>8
    #define DOCAN_TP_FUNC_N_SA      DOCAN_TP_FUNC_ADDR&0xFF
#elif (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_EXT_ADDR)
    #define DOCAN_TP_PHY_N_AI       DOCAN_TP_PHY_ADDR
    #define DOCAN_TP_FUNC_N_AI      DOCAN_TP_FUNC_ADDR
    //N_TA在第一字节
#elif (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR)
    #if((DOCAN_TP_PHY_ADDR>>16)!=0x18CE)
        #error phy addr format error
    #endif
    #if((DOCAN_TP_FUNC_ADDR>>16)!=0x18CD)
        #error func  addr format error
    #endif
    #define DOCAN_TP_PHY_N_TA       (DOCAN_TP_PHY_ADDR&0xFF00)>>8
    #define DOCAN_TP_PHY_N_SA       DOCAN_TP_PHY_ADDR&0xFF
    #define DOCAN_TP_FUNC_N_TA      (DOCAN_TP_FUNC_ADDR&0xFF00)>>8
    #define DOCAN_TP_FUNC_N_SA      DOCAN_TP_FUNC_ADDR&0xFF
    //N_AE在第一字节
#elif (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR)
    #define DOCAN_TP_PHY_N_AI       DOCAN_TP_PHY_ADDR
    #define DOCAN_TP_FUNC_N_AI      DOCAN_TP_FUNC_ADDR
    //N_AE在第一字节
#else
    #error
#endif

#if (DOCAN_TP_CANID_LEN == DOCAN_TP_CANID_LEN_11)
    #if ((DOCAN_TP_ADDR_FORMAT==DOCAN_TP_NORMAL_FIX_ADDR)||(DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX29_ADDR))
        #error 
    #endif
#elif (DOCAN_TP_CANID_LEN == DOCAN_TP_CANID_LEN_29)
    #if (DOCAN_TP_ADDR_FORMAT==DOCAN_TP_MIX11_ADDR)
        #error 
    #endif
#else
    #error
#endif

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso15765_2_cfg_h_ */