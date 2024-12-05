/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\crsf\crfs.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "rtthread.h"
#include <stm32h7xx.h>

/******************************************************************************
 * macros
 *****************************************************************************/

typedef enum
{
    CRSF_FRAMETYPE_GPS                          = 0x02, //GPS position, ground speed, heading, altitude, satellite count
    CRSF_FRAMETYPE_VARIO                        = 0x07, //Vertical speed
    CRSF_FRAMETYPE_BATTERY_SENSOR               = 0x08, //Battery voltage, current, mAh, remaining percent
    CRSF_FRAMETYPE_BARO_ALTITUDE                = 0x09, //Barometric altitude, vertical speed (optional)
    CRSF_FRAMETYPE_HEARTBEAT                    = 0x0B, //(CRSFv3) Heartbeat
    CRSF_FRAMETYPE_LINK_STATISTICS              = 0x14, //Signal information. Uplink/Downlink RSSI, SNR, Link Quality (LQ), RF mode, transmit power
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED           = 0x16, //Channels data (both handset to TX and RX to flight controller)
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED    = 0x17, //(CRSFv3) Channels subset data
    CRSF_FRAMETYPE_LINK_RX_ID                   = 0x1C, //Receiver RSSI percent, power?
    CRSF_FRAMETYPE_LINK_TX_ID                   = 0x1D, //Transmitter RSSI percent, power, fps?
    CRSF_FRAMETYPE_ATTITUDE                     = 0x1E, //Attitude: pitch, roll, yaw
    CRSF_FRAMETYPE_FLIGHT_MODE                  = 0x21, //Flight controller flight mode string
    CRSF_FRAMETYPE_DEVICE_PING                  = 0x28, //Sender requesting DEVICE_INFO from all destination devices
    CRSF_FRAMETYPE_DEVICE_INFO                  = 0x29, //Device name, firmware version, hardware version, serial number (PING response)
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY     = 0x2B, //Configuration item data chunk
    CRSF_FRAMETYPE_PARAMETER_READ               = 0x2C, //Configuration item read request
    CRSF_FRAMETYPE_PARAMETER_WRITE              = 0x2D, //Configuration item write request
    CRSF_FRAMETYPE_ELRS_STATUS                  = 0x2E, //!!Non Standard!! ExpressLRS good/bad packet count, status flags
    CRSF_FRAMETYPE_COMMAND                      = 0x32, //CRSF command execute
    CRSF_FRAMETYPE_RADIO_ID                     = 0x3A, //Extended type used for OPENTX_SYNC
    CRSF_FRAMETYPE_KISS_REQ                     = 0x78, //KISS request
    CRSF_FRAMETYPE_KISS_RESP                    = 0x79, //KISS response
    CRSF_FRAMETYPE_MSP_REQ                      = 0x7A, //MSP parameter request / command
    CRSF_FRAMETYPE_MSP_RESP                     = 0x7B, //MSP parameter response chunk
    CRSF_FRAMETYPE_MSP_WRITE                    = 0x7C, //MSP parameter write
    CRSF_FRAMETYPE_DISPLAYPORT_CMD              = 0x7D, //(CRSFv3) MSP DisplayPort control command
    CRSF_FRAMETYPE_ARDUPILOT_RESP               = 0x80, //Ardupilot output?
}CrfsPacketTypesEnum;

typedef enum
{
    CRSF_ADDRESS_BROADCAST          = 0x00, //Broadcast (all devices process packet)
    CRSF_ADDRESS_USB                = 0x10, //?
    CRSF_ADDRESS_BLUETOOTH          = 0x12, //Bluetooth module
    CRSF_ADDRESS_TBS_CORE_PNP_PRO   = 0x80, //?
    CRSF_ADDRESS_RESERVED1          = 0x8A, //Reserved, for one
    CRSF_ADDRESS_CURRENT_SENSOR     = 0xC0, //External current sensor
    CRSF_ADDRESS_GPS                = 0xC2, //External GPS
    CRSF_ADDRESS_TBS_BLACKBOX       = 0xC4, //External Blackbox logging device
    CRSF_ADDRESS_FLIGHT_CONTROLLER  = 0xC8, //Flight Controller (Betaflight / iNav)
    CRSF_ADDRESS_RESERVED2          = 0xCA, //Reserved, for two
    CRSF_ADDRESS_RACE_TAG           = 0xCC, //Race tag?
    CRSF_ADDRESS_RADIO_TRANSMITTER  = 0xEA, //Handset (EdgeTX), not transmitter
    CRSF_ADDRESS_CRSF_RECEIVER      = 0xEC, //Receiver hardware (TBS Nano RX / RadioMaster RP1)
    CRSF_ADDRESS_CRSF_TRANSMITTER   = 0xEE, //Transmitter module, not handset
    CRSF_ADDRESS_ELRS_LUA           = 0xEF, //!!Non-Standard!! Source address used by ExpressLRS Lua
}CrfsAddrEnum;

typedef struct __attribute__((packed))
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    rt_uint8_t armStatus; // optional ExpressLRS 4.0
}crsf_channels_s;

typedef struct
{
    rt_int32_t latitude;        //degrees * 1e7,    e.g. 28.0805804N sent as 0x10BCC1AC / 280805804
    int32_t longitude;          //同上
    int16_t ground_speed;       //km/h * 10,        e.g. 88 km/h sent as 0x0370 / 880
    int16_t ground_course;      //degrees * 100,    e.g. 90 degrees sent as 0x2328 / 9000
    uint16_t altitude;          //meters + 1000m,   e.g. 10m sent as 0x03F2 / 1010
    uint8_t satellite_count;    //星数
}CrfsGpsStruct;


/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/
void crfs_decode(rt_uint8_t* pbuf,rt_uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crfs_h_ */