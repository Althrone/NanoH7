/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\applications\can_matrix.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_applications_can_matrix_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_applications_can_matrix_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "stdint.h"
#include <rtdevice.h>

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef struct
{
    uint8_t EMS_EngSpd_H:8;
    uint8_t EMS_EngSpd_L:8;
    uint8_t             :8;
    uint8_t             :8;
    uint8_t             :8;
    uint8_t             :8;
    uint8_t             :8;
    uint8_t             :8;

}CanMsg_EMS1;

typedef struct
{
    uint8_t IBC_VehicleSpeed_H      :8;

    uint8_t IBC_ABSActiveSt         :1;
    uint8_t IBC_ABSFaultStSt        :1;
    uint8_t IBC_VehicleSpeedValid   :1;
    uint8_t IBC_VehicleSpeed_L      :5;

    uint8_t IBC_VDCActiveSt         :1;
    uint8_t                         :7;

    uint8_t                         :4;
    uint8_t IBC_BrkPedlModeSt       :2;
    uint8_t                         :2;

    uint8_t                         :8;

    uint8_t                         :3;
    uint8_t IBC_HBAActiveSt         :1;
    uint8_t                         :1;
    uint8_t IBC_ESCLampDisp         :2;
    uint8_t                         :1;

    uint8_t RollingCounter0A2       :4;
    uint8_t IBC_Warningon           :2;
    uint8_t                         :2;

    uint8_t Checksum0A2             :8;

}CanMsg_IBC_Base1_A2;

typedef struct
{
    uint8_t                 :8;
    uint8_t                 :8;
    uint8_t                 :8;
    uint8_t                 :8;
    uint8_t                 :8;

    uint8_t HAD_SteerTqEna  :2;
    uint8_t                 :6;

    uint8_t                 :8;
    uint8_t                 :8;
}CanMsg_HAD_LateralCtrlSts;

typedef struct
{
    uint8_t EPS_SteerPinionAg_H     :8;
    uint8_t EPS_SteerPinionAg_L     :8;
    uint8_t EPS_SteerPinionAgRate_H :8;
    uint8_t EPS_SteerPinionAgRate_L :8;

    uint8_t EPS_SteerPinionAgVld    :1;
    uint8_t EPS_SteerPinionAgRateVld:1;
    uint8_t                         :6;

    uint8_t                         :8;

    uint8_t RollingCounter0D3       :4;
    uint8_t                         :4;

    uint8_t Checksum0D3             :8;
}CanMsg_EPS_Pinionangle;

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_applications_can_matrix_h_ */