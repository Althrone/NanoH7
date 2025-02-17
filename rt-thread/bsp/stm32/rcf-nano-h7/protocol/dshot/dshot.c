/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\dshot\dshot.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "dshot.h"
#include <board.h>

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

extern TIM_HandleTypeDef* extern_tim_handle;

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

static rt_uint32_t gs_motor1_cmd[DSHOT_BUF_LEN]={0};
static rt_uint32_t gs_motor2_cmd[DSHOT_BUF_LEN]={0};
static rt_uint32_t gs_motor3_cmd[DSHOT_BUF_LEN]={0};
static rt_uint32_t gs_motor4_cmd[DSHOT_BUF_LEN]={0};

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

rt_uint16_t dshot_encode(rt_uint16_t packet, rt_bool_t telem);
static void pwmWriteDigital(rt_uint32_t *esc_cmd, rt_uint16_t value);

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void dshot_set_throttles(rt_uint16_t m1,rt_uint16_t m2,
                         rt_uint16_t m3,rt_uint16_t m4)
{
    if(m1>1999){m1=1999;}
    if(m2>1999){m2=1999;}
    if(m3>1999){m3=1999;}
    if(m4>1999){m4=1999;}

    m1=dshot_encode(m1,RT_FALSE);
    m2=dshot_encode(m2,RT_FALSE);
    m3=dshot_encode(m3,RT_FALSE);
    m4=dshot_encode(m4,RT_FALSE);

    pwmWriteDigital(gs_motor1_cmd,m1);
    pwmWriteDigital(gs_motor2_cmd,m2);
    pwmWriteDigital(gs_motor3_cmd,m3);
    pwmWriteDigital(gs_motor4_cmd,m4);

    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_1,gs_motor1_cmd,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_2,gs_motor2_cmd,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_3,gs_motor3_cmd,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_4,gs_motor4_cmd,16);
}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

rt_uint16_t dshot_encode(rt_uint16_t packet, rt_bool_t telem) {
    rt_uint16_t packet_telemetry = ((packet+48) << 1) | (telem & 1);
    rt_uint16_t csum = 0;
    rt_uint16_t csum_data = packet_telemetry;

    for (rt_uint8_t i = 0; i < 3; i++) {
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    return (packet_telemetry << 4) | csum; //append checksum
}

static void pwmWriteDigital(rt_uint32_t *esc_cmd, rt_uint16_t value)
{
    rt_uint16_t mask=0x8000;
    for (rt_uint8_t i = 0; i < 16; i++)
    {
        esc_cmd[i]=(value & mask) ? DSHOT_LOGIC1 : DSHOT_LOGIC0;
        mask>>1;
    }
}