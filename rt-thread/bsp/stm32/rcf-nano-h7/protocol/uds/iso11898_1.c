/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso11898_1.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "iso11898_1.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

static void L_Data_request(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data);
static void L_Data_confirm(uint32_t Identifier, Transfer_StatusEnum Transfer_Status);
static void L_Data_indication(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data);

const struct L_DataOps L_Data=
{
    .request=L_Data_request,
    .confirm=L_Data_confirm,
    .indication=L_Data_indication,
};

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/

static void L_Data_request(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data){}
static void L_Data_confirm(uint32_t Identifier, Transfer_StatusEnum Transfer_Status){}
static void L_Data_indication(uint32_t Identifier, uint8_t Format, uint8_t DLC, uint8_t *Data){}