/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\gnss\m8030\ubx.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_GNSS_M8030_UBX_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_GNSS_M8030_UBX_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "rtthread.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    M8030_I2C_DDC_PORT_ID=0,
    M8030_UART1_PORT_ID=1,
    M8030_USB_PORT_ID=3,
    M8030_SPI_PORT_ID=4,
}M8030PortIdEnum;

typedef enum
{
    UBX_GET_MSG,
    UBX_SET_MSG,
}UbxMsgCtrlEnum;

typedef enum
{
    UBX_NAV_MSG = 0x01,
    UBX_RXM_MSG = 0x02,
    UBX_INF_MSG = 0x04,
    UBX_ACK_MSG = 0x05,
    UBX_CFG_MSG = 0x06,
    UBX_UPD_MSG = 0x09,
    UBX_MON_MSG = 0x0A,
    UBX_AID_MSG = 0x0B,
    UBX_TIM_MSG = 0x0D,
    UBX_ESF_MSG = 0x10,
    UBX_MGA_MSG = 0x13,
    UBX_LOG_MSG = 0x21,
    UBX_SEC_MSG = 0x27,
    UBX_HNR_MSG = 0x28,
    UBX_NMEA_STD_MSG = 0xF0,
    UBX_NMEA_PUBX_MSG = 0xF1,
}UbxMsgClassEnum;

typedef enum
{
    UBX_NAV_AOPSTATUS_ID = 0x60,
    UBX_NAV_ATT_ID = 0x05,
    UBX_NAV_CLOCK_ID = 0x22,
    UBX_NAV_COV_ID = 0x36,
    UBX_NAV_DGPS_ID = 0x31,
    UBX_NAV_DOP_ID = 0x04,
    UBX_NAV_EELL_ID = 0x3d,
    UBX_NAV_EOE_ID = 0x61,
    UBX_NAV_GEOFENCE_ID = 0x39,
    UBX_NAV_HPPOSECEF_ID = 0x13,
    UBX_NAV_HPPOSLLH_ID = 0x14,
    UBX_NAV_NMI_ID = 0x28,
    UBX_NAV_ODO_ID = 0x09,
    UBX_NAV_ORB_ID = 0x34,
    UBX_NAV_POSECEF_ID = 0x01,
    UBX_NAV_POSLLH_ID = 0x02,
    UBX_NAV_PVT_ID = 0x07,
    UBX_NAV_RELPOSNED_ID = 0x3C,
    UBX_NAV_RESETODO_ID = 0x10,
    UBX_NAV_SAT_ID = 0x35,
    UBX_NAV_SBAS_ID = 0x32,
    UBX_NAV_SLAS_ID = 0x42,
    UBX_NAV_SOL_ID = 0x06,
    UBX_NAV_STATUS_ID = 0x03,
    UBX_NAV_SVINFO_ID = 0x30,
    UBX_NAV_SVIN_ID = 0x3B,
    UBX_NAV_TIMEBDS_ID = 0x24,
    UBX_NAV_TIMEGAL_ID = 0x25,
    UBX_NAV_TIMEGLO_ID = 0x23,
    UBX_NAV_TIMEGPS_ID = 0x20,
    UBX_NAV_TIMELS_ID = 0x26,
    UBX_NAV_TIMEUTC_ID = 0x21,
    UBX_NAV_VELECEF_ID = 0x11,
    UBX_NAV_VELNED_ID = 0x12,
}UbxNavMsgIdEnum;

typedef enum
{
    UBX_RXM_IMES_ID = 0x61,
    UBX_RXM_MEASX_ID = 0x14,
    UBX_RXM_PMREQ_ID = 0x41,
    // UBX_RXM_PMREQ_ID = 0x41,
    UBX_RXM_RAWX_ID = 0x15,
    // UBX_RXM_RAWX_ID = 0x15,
    UBX_RXM_RLM_ID = 0x59,
    // UBX_RXM_RLM_ID = 0x59,
    UBX_RXM_RTCM_ID = 0x32,
    UBX_RXM_SFRBX_ID = 0x13,
    // UBX_RXM_SFRBX_ID = 0x13,
    UBX_RXM_SVSI_ID = 0x20,
}UbxRxmMsgIdEnum;

typedef enum
{
    UBX_INF_DEBUG_ID = 0x04,
    UBX_INF_ERROR_ID = 0x00,
    UBX_INF_NOTICE_ID = 0x02,
    UBX_INF_TEST_ID = 0x03,
    UBX_INF_WARNING_ID = 0x01,
}UbxInfMsgIdEnum;

typedef enum
{
    UBX_ACK_ACK_ID = 0x01,
    UBX_ACK_NAK_ID = 0x00,
}UbxAckMsgIdEnum;

typedef enum
{
    UBX_CFG_ANT_ID = 0x13,
    UBX_CFG_BATCH_ID = 0x93,
    UBX_CFG_CFG_ID = 0x09,
    UBX_CFG_DAT_ID = 0x06,
    // UBX_CFG_DAT_ID = 0x06,
    UBX_CFG_DGNSS_ID = 0x70,
    UBX_CFG_DOSC_ID = 0x61,
    UBX_CFG_ESFALG_ID = 0x56,
    UBX_CFG_ESFA_ID = 0x4C,
    UBX_CFG_ESFG_ID = 0x4D,
    UBX_CFG_ESFWT_ID = 0x82,
    UBX_CFG_ESRC_ID = 0x60,
    UBX_CFG_GEOFENCE_ID = 0x69,
    UBX_CFG_GNSS_ID = 0x3E,
    UBX_CFG_HNR_ID = 0x5C,
    UBX_CFG_INF_ID = 0x02,
    // UBX_CFG_INF_ID = 0x02,
    UBX_CFG_ITFM_ID = 0x39,
    UBX_CFG_LOGFILTER_ID = 0x47,
    UBX_CFG_MSG_ID = 0x01,
    // UBX_CFG_MSG_ID = 0x01,
    // UBX_CFG_MSG_ID = 0x01,
    UBX_CFG_NAV5_ID = 0x24,
    UBX_CFG_NAVX5_ID = 0x23,
    // UBX_CFG_NAVX5_ID = 0x23,
    // UBX_CFG_NAVX5_ID = 0x23,
    UBX_CFG_NMEA_ID = 0x17,
    // UBX_CFG_NMEA_ID = 0x17,
    // UBX_CFG_NMEA_ID = 0x17,
    UBX_CFG_ODO_ID = 0x1E,
    UBX_CFG_PM2_ID = 0x3B,
    // UBX_CFG_PM2_ID = 0x3B,
    // UBX_CFG_PM2_ID = 0x3B,
    UBX_CFG_PMS_ID = 0x86,
    UBX_CFG_PRT_ID = 0x00,
    // UBX_CFG_PRT_ID = 0x00,
    // UBX_CFG_PRT_ID = 0x00,
    // UBX_CFG_PRT_ID = 0x00,
    // UBX_CFG_PRT_ID = 0x00,
    UBX_CFG_PWR_ID = 0x57,
    UBX_CFG_RATE_ID = 0x08,
    UBX_CFG_RINV_ID = 0x34,
    UBX_CFG_RST_ID = 0x04,
    UBX_CFG_RXM_ID = 0x11,
    // UBX_CFG_RXM_ID = 0x11,
    UBX_CFG_SBAS_ID = 0x16,
    UBX_CFG_SENIF_ID = 0x88,
    UBX_CFG_SLAS_ID = 0x8D,
    UBX_CFG_SMGR_ID = 0x62,
    UBX_CFG_SPT_ID = 0x64,
    UBX_CFG_TMODE2_ID = 0x3D,
    UBX_CFG_TMODE3_ID = 0x71,
    UBX_CFG_TP5_ID = 0x31,
    // UBX_CFG_TP5_ID = 0x31,
    // UBX_CFG_TP5_ID = 0x31,
    // UBX_CFG_TP5_ID = 0x31,
    UBX_CFG_TXSLOT_ID = 0x53,
    UBX_CFG_USB_ID = 0x1B,
}UbxCfgMsgIdEnum;

typedef enum
{
    UBX_UPD_SOS_ID = 0x14,
}UbxUpdMsgIdEnum;

typedef enum
{
    UBX_MON_BATCH_ID = 0x32,
    UBX_MON_GNSS_ID = 0x28,
    UBX_MON_HW2_ID = 0x0B,
    UBX_MON_HW_ID = 0x09,
    UBX_MON_IO_ID = 0x02,
    UBX_MON_MSGPP_ID = 0x06,
    UBX_MON_PATCH_ID = 0x27,
    // UBX_MON_PATCH_ID = 0x27,
    UBX_MON_RXBUF_ID = 0x07,
    UBX_MON_RXR_ID = 0x21,
    UBX_MON_SMGR_ID = 0x2E,
    UBX_MON_SPT_ID = 0x2F,
    UBX_MON_TXBUF_ID = 0x08,
    UBX_MON_VER_ID = 0x04,
    // UBX_MON_VER_ID = 0x04,
}UbxMonMsgIdEnum;

typedef enum
{
    UBX_AID_ALM_ID = 0x30,
    // UBX_AID_ALM_ID = 0x30,
    // UBX_AID_ALM_ID = 0x30,
    UBX_AID_AOP_ID = 0x33,
    // UBX_AID_AOP_ID = 0x33,
    // UBX_AID_AOP_ID = 0x33,
    UBX_AID_EPH_ID = 0x31,
    // UBX_AID_EPH_ID = 0x31,
    // UBX_AID_EPH_ID = 0x31,
    UBX_AID_HUI_ID = 0x02,
    // UBX_AID_HUI_ID = 0x02,
    UBX_AID_INI_ID = 0x01,
    // UBX_AID_INI_ID = 0x01,
}UbxAidMsgIdEnum;

typedef enum
{
    UBX_TIM_DOSC_ID = 0x11,
    UBX_TIM_FCHG_ID = 0x16,
    UBX_TIM_HOC_ID = 0x17,
    UBX_TIM_SMEAS_ID = 0x13,
    UBX_TIM_SVIN_ID = 0x04,
    UBX_TIM_TM2_ID = 0x03,
    UBX_TIM_TOS_ID = 0x12,
    UBX_TIM_TP_ID = 0x01,
    UBX_TIM_VCOCAL_ID = 0x15,
    // UBX_TIM_VCOCAL_ID = 0x15,
    // UBX_TIM_VCOCAL_ID = 0x15,
    UBX_TIM_VRFY_ID = 0x06,
}UbxTimMsgIdEnum;

typedef enum
{
    UBX_ESF_ALG_ID = 0x14,
    UBX_ESF_INS_ID = 0x15,
    UBX_ESF_MEAS_ID = 0x02,
    UBX_ESF_RAW_ID = 0x03,
    UBX_ESF_STATUS_ID = 0x10,
}UbxEsfMsgIdEnum;

typedef enum
{
    UBX_MGA_ACK_DATA0_ID = 0x60,
    UBX_MGA_ANO_ID = 0x20,
    UBX_MGA_BDS_EPH_ID = 0x03,
    // UBX_MGA_BDS_ALM_ID = 0x03,
    // UBX_MGA_BDS_HEALTH_ID = 0x03,
    // UBX_MGA_BDS_UTC_ID = 0x03,
    // UBX_MGA_BDS_IONO_ID = 0x03,
    UBX_MGA_DBD_ID = 0x80,
    // UBX_MGA_DBD_ID = 0x80,
    UBX_MGA_FLASH_DATA_ID = 0x21,
    // UBX_MGA_FLASH_STOP_ID = 0x21,
    // UBX_MGA_FLASH_ACK_ID = 0x21,
    UBX_MGA_GAL_EPH_ID = 0x02,
    // UBX_MGA_GAL_ALM_ID = 0x02,
    // UBX_MGA_GAL_TIMEOFFSET_ID = 0x02,
    // UBX_MGA_GAL_UTC_ID = 0x02,
    UBX_MGA_GLO_EPH_ID = 0x06,
    // UBX_MGA_GLO_ALM_ID = 0x06,
    // UBX_MGA_GLO_TIMEOFFSET_ID = 0x06,
    UBX_MGA_GPS_EPH_ID = 0x00,
    // UBX_MGA_GPS_ALM_ID = 0x00,
    // UBX_MGA_GPS_HEALTH_ID = 0x00,
    // UBX_MGA_GPS_UTC_ID = 0x00,
    // UBX_MGA_GPS_IONO_ID = 0x00,
    UBX_MGA_INI_POS_XYZ_ID = 0x40,
    // UBX_MGA_INI_POS_LLH_ID = 0x40,
    // UBX_MGA_INI_TIME_UTC_ID = 0x40,
    // UBX_MGA_INI_TIME_GNSS_ID = 0x40,
    // UBX_MGA_INI_CLKD_ID = 0x40,
    // UBX_MGA_INI_FREQ_ID = 0x40,
    // UBX_MGA_INI_EOP_ID = 0x40,
    UBX_MGA_QZSS_EPH_ID = 0x05,
    // UBX_MGA_QZSS_ALM_ID = 0x05,
    // UBX_MGA_QZSS_HEALTH_ID = 0x05,
}UbxMagMsgIdEnum;

typedef enum
{
    UBX_LOG_BATCH_ID = 0x11,
    UBX_LOG_CREATE_ID = 0x07,
    UBX_LOG_ERASE_ID = 0x03,
    UBX_LOG_FINDTIME_ID = 0x0E,
    // UBX_LOG_FINDTIME_ID = 0x0E,
    UBX_LOG_INFO_ID = 0x08,
    // UBX_LOG_INFO_ID = 0x08,
    UBX_LOG_RETRIEVEBATCH_ID = 0x10,
    UBX_LOG_RETRIEVEPOSEXTRA_ID = 0x0f,
    UBX_LOG_RETRIEVEPOS_ID = 0x0b,
    UBX_LOG_RETRIEVESTRING_ID = 0x0d,
    UBX_LOG_RETRIEVE_ID = 0x09,
    UBX_LOG_STRING_ID = 0x04,
}UbxLogMsgIdEnum;

typedef enum
{
    UBX_SEC_UNIQID_ID = 0x03,   //Unique chip ID
}UbxSecMsgIdEnum;

typedef enum
{
    UBX_HNR_ATT_ID = 0x01,
    UBX_HNR_INS_ID = 0x02,
    UBX_HNR_PVT_ID = 0x00,
}UbxHnrMsgIdEnum;

typedef enum
{
    UBX_NMEA_STD_DTM_ID = 0x0A,
    UBX_NMEA_STD_GBQ_ID = 0x44,
    UBX_NMEA_STD_GBS_ID = 0x09,
    UBX_NMEA_STD_GGA_ID = 0x00,
    UBX_NMEA_STD_GLL_ID = 0x01,
    UBX_NMEA_STD_GLQ_ID = 0x43,
    UBX_NMEA_STD_GNQ_ID = 0x42,
    UBX_NMEA_STD_GNS_ID = 0x0D,
    UBX_NMEA_STD_GPQ_ID = 0x40,
    UBX_NMEA_STD_GRS_ID = 0x06,
    UBX_NMEA_STD_GSA_ID = 0x02,
    UBX_NMEA_STD_GST_ID = 0x07,
    UBX_NMEA_STD_GSV_ID = 0x03,
    UBX_NMEA_STD_RMC_ID = 0x04,
    UBX_NMEA_STD_THS_ID = 0x0E,
    UBX_NMEA_STD_TXT_ID = 0x41,
    UBX_NMEA_STD_VLW_ID = 0x0F,
    UBX_NMEA_STD_VTG_ID = 0x05,
    UBX_NMEA_STD_ZDA_ID = 0x08,
}UbxNmeaStdMsgIdEnum;

typedef enum
{
    UBX_NMEA_PUBX_CONFIG_ID = 0x41,
    UBX_NMEA_PUBX_POSITION_ID = 0x00,
    UBX_NMEA_PUBX_RATE_ID = 0x40,
    UBX_NMEA_PUBX_SVSTATUS_ID = 0x03,
    UBX_NMEA_PUBX_TIME_ID = 0x04,
}UbxNmeaPubxMsgIdEnum;

typedef struct
{
    rt_uint8_t version;
    rt_uint8_t reserved1[3];
    rt_uint8_t uniqueId[5];
}UbxSecUniqIdStruct;

typedef struct
{
    char swVersion[30];
    char hwVersion[10];
    char extension[][30];
}UbxMonVerStruct;

typedef struct
{
    M8030PortIdEnum portID;
    rt_uint8_t reserved1;
    union
    {
        rt_uint16_t x2;
        struct
        {
            volatile rt_uint16_t en:1;
            volatile rt_uint16_t pol:1;
            volatile rt_uint16_t pin:5;
            volatile rt_uint16_t thres:9;
        }B;
    }txReady;
    union
    {
        rt_uint32_t x4;
        struct
        {
            volatile rt_uint32_t            :6;
            volatile rt_uint32_t charLen    :2;
            volatile rt_uint32_t            :1;
            volatile rt_uint32_t parity     :3;
            volatile rt_uint32_t nStopBits  :2;
            volatile rt_uint32_t            :18;
        }B;
    }mode;
    rt_uint32_t baudRate;
    union
    {
        rt_uint16_t x2;
        struct
        {
            volatile rt_uint16_t inUbx      :1;
            volatile rt_uint16_t inNmea     :1;
            volatile rt_uint16_t inRtcm     :1;
            volatile rt_uint16_t            :2;
            volatile rt_uint16_t inRtcm3    :1;
            volatile rt_uint16_t            :10;
        }B;
    }inProtoMask;
    union
    {
        rt_uint16_t x2;
        struct
        {
            volatile rt_uint16_t inUbx      :1;
            volatile rt_uint16_t inNmea     :1;
            volatile rt_uint16_t            :3;
            volatile rt_uint16_t inRtcm3    :1;
            volatile rt_uint16_t            :10;
        }B;
    }outProtoMask;
    union
    {
        rt_uint16_t x2;
        struct
        {
            volatile rt_uint16_t                    :1;
            volatile rt_uint16_t extendedTxTimeout  :1;
            volatile rt_uint16_t                    :14;
        }B;
    }flags;
    rt_uint8_t reserved2[2];
}UbxCfgUartPrtStruct;

typedef struct
{
    rt_uint32_t iTOW;
    rt_uint16_t year;
    rt_uint8_t month;
    rt_uint8_t day;
    rt_uint8_t hour;
    rt_uint8_t min;
    rt_uint8_t sec;
    rt_uint8_t valid;
    rt_uint32_t tAcc;
    rt_int32_t nano;
    rt_uint8_t fixType;
    rt_uint8_t flags;
    rt_uint8_t reserved1;
    rt_uint8_t numSV;
    rt_int32_t lon;
    rt_int32_t lat;
    rt_int32_t height;
    rt_int32_t hMSL;
    rt_uint32_t hAcc;
    rt_uint32_t vAcc;
    rt_int32_t velN;
    rt_int32_t velE;
    rt_int32_t velD;
    rt_int32_t gSpeed;
    rt_int32_t headMot;
    rt_uint32_t sAcc;
    rt_uint32_t headAcc;
    rt_uint16_t pDOP;
    rt_uint8_t reserved2[6];
    rt_int32_t headVeh;
    rt_uint8_t reserved3[4];
}UbxNavPvtStruct;


/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

void ubx_decode(rt_uint8_t* pbuf,rt_uint32_t size);


#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_gnss_m8030_ubx_h_ */