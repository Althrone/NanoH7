/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\crsf\crsf.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crsf_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crsf_h_

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
}CrsfPacketTypesEnum;

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
}CrsfAddrEnum;

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
}CrsfGpsStruct;

typedef struct
{
    int16_t vertical_speed;     // cm/s             (e.g. 1.5m/s sent as 150)
}CrsfVarioStruct;

typedef struct
{
    int16_t voltage;    //in dV (Big Endian)
    //e.g. 25.2V sent as 0x00FC / 252
    //e.g. 3.7V sent as 0x0025 / 37
    //e.g. 3276.7V sent as 0x7FFF / 32767
    int16_t current;    //in dA (Big Endian)
    //e.g. 18.9A sent as 0x00BD / 189
    //e.g. 109.4A sent as 0x0446 / 1094
    int32_t used_capacity;  // in mAh
    // int24_t used_capacity;  // in mAh
    //e.g. 2199mAh used sent as 0x0897 / 2199
    int8_t estimated_battery_remaining; //in percent (%)
    //e.g. 100% full battery sent as 0x64 / 100
    //e.g. 20% battery remaining sent as 0x14 / 20
}CrsfBatteryStruct;

typedef struct
{
    uint16_t altitude;
    //If the high bit is not set, altitude value is in decimeters + 10000, allowing altitudes of -1000.0m to 2276.7m
    //10m sent as (100 + 10000) = 0x2774 / 10100
    //-10m sent as (-100 + 10000) = 0x26AC / 9900
    //If the high bit is set, altitude value is in meters, allowing altitudes of 0m - 32767m
    //10m sent as (10 | 0x8000) = 0x800A / 32768
    int16_t vertical_speed; // (optional) in cm/s (e.g. 1.5m/s sent as 150)

    //Vertical speed is listed as optional. This means if the payload is only 2 bytes, it just contains the altitude. If the payload is 4 bytes, it also includes the vertical speed.
}CrsfBaroStruct;

typedef struct
{
    uint16_t origin_device_address;     //(Big Endian)
    //e.g. Flight Controller is online sends 0xC8 / 200CRSF_ADDRESS_FLIGHT_CONTROLLER
}CrsfHeartBeatStruct;//CRSFv3

typedef struct
{
    uint8_t uplink_rssi_ant1;           // ( dBm * -1 )
    uint8_t uplink_rssi_ant2;           // ( dBm * -1 )
    uint8_t uplink_link_quality;        //Package success rate /  ( % )
    //Uplink LQ of 0 may used to indicate a disconnected status to the handset
    int8_t uplink_snr;                  // ( dB, or dB*4 for TBS I believe )
    uint8_t diversity_active_antenna;   // ( enum ant. 1 = 0, ant. 2 = 1 )
    uint8_t rf_mode;                    // ( 500Hz, 250Hz etc, varies based on ELRS Band or TBS )
    uint8_t uplink_tx_power;            // ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 50mW )
    uint8_t downlink_rssi;              //( dBm * -1 )
    uint8_t downlink_link_quality;      //package success rate /  ( % )
    int8_t downlink_snr;                //( dB )
}CrsfLinkStatisticsStruct;

typedef struct
{
    uint8_t config_byte;
    //bits 0-4 (5 bits) First channel number appearing in this packet (0 = ch1)
    //bits 5-6 (2 bits) Channel resolution
    //0 (b00) - 10 bits/channel
    //1 (b01) - 11 bits/channel
    //2 (b10) - 12 bits/channel
    //3 (b11) - 13 bits/channel
    //bit 7 (1 bit) reserved configuration bit
    uint8_t data[];// - channel data packed
}CrsfSubsetChannelsStruct;//CRSFv3

typedef struct
{
    int16_t pitch;// angle in radians/10000
    // e.g. 180 degrees sent as 0x7AB7 / 31415
    // e.g. 45 degrees sent as 0x1EAD / 7853
    // e.g. -45 degrees sent as 0xE153 / -7853
    int16_t roll;
    int16_t yaw;
    // All values must be in the +/-180 degree +/-PI radian range.
}CrsfUasAttitudeStruct;

typedef struct
{
    char flight_mode[14];// - Null-terminated string. Max length 14 characters including the null / 13 characters of string data.
    //e.g. ACRO sent as 41 43 52 4F 00
}CrsfFlightModeStruct;

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/
void crsf_decode(rt_uint8_t* pbuf,rt_uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_crsf_crsf_h_ */