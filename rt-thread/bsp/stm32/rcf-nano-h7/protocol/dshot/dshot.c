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

/******************************************************************************
 * private macros
 *****************************************************************************/

#define ESC_BIT_0     11
#define ESC_BIT_1     22
#define ESC_CMD_BUF_LEN 18
// uint16_t ESC_CMD[ESC_CMD_BUF_LEN]={0};

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

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

// u16 add_checksum_and_telemetry(u16 packet, u8 telem) {
//     u16 packet_telemetry = (packet << 1) | (telem & 1);
//     u8 i;
//     u16 csum = 0;
//     u16 csum_data = packet_telemetry;

//     for (i = 0; i < 3; i++) {
//         csum ^=  csum_data; // xor data by nibbles
//         csum_data >>= 4;
//     }
//     csum &= 0xf;
//     return (packet_telemetry << 4) | csum; //append checksum
// }

// static void pwmWriteDigital(uint16_t *esc_cmd, uint16_t value)
// {
// 	value = ( (value > 2047) ? 2047 : value );
// 	value = prepareDshotPacket(value, 0);
//     esc_cmd[0]  = (value & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[1]  = (value & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[2]  = (value & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[3]  = (value & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[4]  = (value & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[5]  = (value & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[6]  = (value & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[7]  = (value & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[8]  = (value & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[9]  = (value & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[10] = (value & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[11] = (value & 0x0010) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[12] = (value & 0x8) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[13] = (value & 0x4) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[14] = (value & 0x2) ? ESC_BIT_1 : ESC_BIT_0;
//     esc_cmd[15] = (value & 0x1) ? ESC_BIT_1 : ESC_BIT_0;

//     HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)esc_cmd,ESC_CMD_BUF_LEN);
// }