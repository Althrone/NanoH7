/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\libraries\HAL_Drivers\drv_mcan.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

/******************************************************************************
 * private macros
 *****************************************************************************/

#ifdef RT_USING_CAN

#if !defined(BSP_USING_MCAN1) && !defined(BSP_USING_MCAN2)
    #error "Please define at least one BSP_USING_MCANx"
#else

#include "drv_config.h"

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

 /* stm32 config class */
struct stm32_mcan_config
{
    const char *name;
    FDCAN_GlobalTypeDef *Instance;
    IRQn_Type irq0_type;
    IRQn_Type irq1_type;
    const char *tx_pin_name;
    const char *rx_pin_name;
};

enum
{
#ifdef BSP_USING_MCAN1
    MCAN1_INDEX,
#endif
#ifdef BSP_USING_MCAN2
    MCAN2_INDEX,
#endif
};

/******************************************************************************
 * private variables
 *****************************************************************************/

static struct stm32_mcan_config s_mcan_config[]=
{
#ifdef BSP_USING_MCAN1
    MCAN1_CONFIG,
#endif
#ifdef BSP_USING_MCAN2
    MCAN2_CONFIG,
#endif
};

//发送周期报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(cycle!=0))?1:0)+
    static const rt_uint8_t sk_mcan1_send_cycle_msg_num=CAN_MSG_MATRIX 0;
#undef Y

//发送事件报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(cycle==0))?1:0)+
    static const rt_uint8_t sk_mcan1_send_event_msg_num=CAN_MSG_MATRIX 0;
#undef Y

//标准id周期报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
      (dir==kOpenCanRecv)&&(cycle!=0))?1:0)+
    static const rt_uint8_t sk_mcan1_recv_cycle_stdmsg_num=CAN_MSG_MATRIX 0;
#undef Y

//扩展id周期报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
      (dir==kOpenCanRecv)&&(cycle!=0))?1:0)+
    static const rt_uint8_t sk_mcan1_recv_cycle_extmsg_num=CAN_MSG_MATRIX 0;
#undef Y

//标准id事件报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
      (dir==kOpenCanRecv)&&(cycle==0))?1:0)+
    static const rt_uint8_t sk_mcan1_recv_event_stdmsg_num=CAN_MSG_MATRIX 0;
#undef Y

//扩展id事件报文数量
#define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
      (dir==kOpenCanRecv)&&(cycle==0))?1:0)+
    static const rt_uint8_t sk_mcan1_recv_event_extmsg_num=CAN_MSG_MATRIX 0;
#undef Y

rt_uint8_t s_mcan1_rxbuf_ele_size=0;

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/

/**
 * @brief   根据报文表自动返回最小的适合报文的ram大小
 */
rt_uint8_t _mcan1_tx_size(void)
{
    //计算发送报文的dlc最大值，直接送到TxElmtSize
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>8))?1:0)|
        rt_bool_t dlc_more_than_8=CAN_MSG_MATRIX 0;
        //为0意味着没有比8大的帧，直接设置为8就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>12))?1:0)|
        rt_bool_t dlc_more_than_12=CAN_MSG_MATRIX 0;
        //为0意味着没有比12大的帧，直接设置为12就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>16))?1:0)|
        rt_bool_t dlc_more_than_16=CAN_MSG_MATRIX 0;
        //为0意味着没有比16大的帧，直接设置为16就行
    #undef Y
    
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>20))?1:0)|
        rt_bool_t dlc_more_than_20=CAN_MSG_MATRIX 0;
        //为0意味着没有比20大的帧，直接设置为20就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>24))?1:0)|
        rt_bool_t dlc_more_than_24=CAN_MSG_MATRIX 0;
        //为0意味着没有比24大的帧，直接设置为24就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>32))?1:0)|
        rt_bool_t dlc_more_than_32=CAN_MSG_MATRIX 0;
        //为0意味着没有比32大的帧，直接设置为32就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(dlc>48))?1:0)|
        rt_bool_t dlc_more_than_48=CAN_MSG_MATRIX 0;
        //为0意味着没有比48大的帧，直接设置为48就行
    #undef Y

    if(dlc_more_than_48)
        return FDCAN_DATA_BYTES_64;
    if(dlc_more_than_32)
        return FDCAN_DATA_BYTES_48;
    if(dlc_more_than_24)
        return FDCAN_DATA_BYTES_32;
    if(dlc_more_than_20)
        return FDCAN_DATA_BYTES_24;
    if(dlc_more_than_16)
        return FDCAN_DATA_BYTES_20;
    if(dlc_more_than_12)
        return FDCAN_DATA_BYTES_16;
    if(dlc_more_than_8)
        return FDCAN_DATA_BYTES_12;
    return FDCAN_DATA_BYTES_8;
}

/**
 * @brief   统计发送报文中周期报文数量和事件报文数量决定txbuf大小
 */
void _mcan1_txbuf_cfg(FDCAN_HandleTypeDef* pmcan)
{
    //周期报文数量
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(cycle!=0))?1:0)+
        rt_uint8_t num_of_mcan1_send_cyc_frame=CAN_MSG_MATRIX 0;
    #undef Y

    //事件报文数量
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(dir==kOpenCanSend)&&(cycle==0))?1:0)+
        rt_uint8_t num_of_mcan1_send_event_frame=CAN_MSG_MATRIX 0;
    #undef Y

    if((2*num_of_mcan1_send_event_frame+num_of_mcan1_send_cyc_frame)<=32)
    {
        pmcan->Init.TxBuffersNbr=num_of_mcan1_send_cyc_frame;
        pmcan->Init.TxFifoQueueElmtsNbr=2*num_of_mcan1_send_event_frame;
    }
    else if((num_of_mcan1_send_event_frame+num_of_mcan1_send_cyc_frame)<=32)
    {
        pmcan->Init.TxBuffersNbr=num_of_mcan1_send_cyc_frame;
        pmcan->Init.TxFifoQueueElmtsNbr=num_of_mcan1_send_event_frame;
    }
    else//如果周期+事件大于32个，全部配置成fifo类型
    {
        pmcan->Init.TxBuffersNbr=0;
        pmcan->Init.TxFifoQueueElmtsNbr=32;
    }

	pmcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;	
}

/**
 * @brief   由于扩展周期报文和标准周期报文都存放在buf，因此计算这两种报文的最大长度决定 RxBufferSize
 * @retval  Word,1Word=4Byte
 */
rt_uint8_t _mcan1_rxbuf_size(void)
{
    //计算发送报文的dlc最大值，直接送到TxElmtSize
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>8))?1:0)|
        rt_bool_t dlc_more_than_8=CAN_MSG_MATRIX 0;
        //为0意味着没有比8大的帧，直接设置为8就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>12))?1:0)|
        rt_bool_t dlc_more_than_12=CAN_MSG_MATRIX 0;
        //为0意味着没有比12大的帧，直接设置为12就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>16))?1:0)|
        rt_bool_t dlc_more_than_16=CAN_MSG_MATRIX 0;
        //为0意味着没有比16大的帧，直接设置为16就行
    #undef Y
    
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>20))?1:0)|
        rt_bool_t dlc_more_than_20=CAN_MSG_MATRIX 0;
        //为0意味着没有比20大的帧，直接设置为20就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>24))?1:0)|
        rt_bool_t dlc_more_than_24=CAN_MSG_MATRIX 0;
        //为0意味着没有比24大的帧，直接设置为24就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>32))?1:0)|
        rt_bool_t dlc_more_than_32=CAN_MSG_MATRIX 0;
        //为0意味着没有比32大的帧，直接设置为32就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>48))?1:0)|
        rt_bool_t dlc_more_than_48=CAN_MSG_MATRIX 0;
        //为0意味着没有比48大的帧，直接设置为48就行
    #undef Y

    if(dlc_more_than_48)
        return FDCAN_DATA_BYTES_64;
    if(dlc_more_than_32)
        return FDCAN_DATA_BYTES_48;
    if(dlc_more_than_24)
        return FDCAN_DATA_BYTES_32;
    if(dlc_more_than_20)
        return FDCAN_DATA_BYTES_24;
    if(dlc_more_than_16)
        return FDCAN_DATA_BYTES_20;
    if(dlc_more_than_12)
        return FDCAN_DATA_BYTES_16;
    if(dlc_more_than_8)
        return FDCAN_DATA_BYTES_12;
    return FDCAN_DATA_BYTES_8;
}

rt_uint8_t _mcan1_rx_std_event_msg_size(void)
{
    //计算发送报文的dlc最大值，直接送到TxElmtSize
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>8))?1:0)|
        rt_bool_t dlc_more_than_8=CAN_MSG_MATRIX 0;
        //为0意味着没有比8大的帧，直接设置为8就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>12))?1:0)|
        rt_bool_t dlc_more_than_12=CAN_MSG_MATRIX 0;
        //为0意味着没有比12大的帧，直接设置为12就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>16))?1:0)|
        rt_bool_t dlc_more_than_16=CAN_MSG_MATRIX 0;
        //为0意味着没有比16大的帧，直接设置为16就行
    #undef Y
    
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>20))?1:0)|
        rt_bool_t dlc_more_than_20=CAN_MSG_MATRIX 0;
        //为0意味着没有比20大的帧，直接设置为20就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>24))?1:0)|
        rt_bool_t dlc_more_than_24=CAN_MSG_MATRIX 0;
        //为0意味着没有比24大的帧，直接设置为24就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>32))?1:0)|
        rt_bool_t dlc_more_than_32=CAN_MSG_MATRIX 0;
        //为0意味着没有比32大的帧，直接设置为32就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>48))?1:0)|
        rt_bool_t dlc_more_than_48=CAN_MSG_MATRIX 0;
        //为0意味着没有比48大的帧，直接设置为48就行
    #undef Y

    if(dlc_more_than_48)
        return FDCAN_DATA_BYTES_64;
    if(dlc_more_than_32)
        return FDCAN_DATA_BYTES_48;
    if(dlc_more_than_24)
        return FDCAN_DATA_BYTES_32;
    if(dlc_more_than_20)
        return FDCAN_DATA_BYTES_24;
    if(dlc_more_than_16)
        return FDCAN_DATA_BYTES_20;
    if(dlc_more_than_12)
        return FDCAN_DATA_BYTES_16;
    if(dlc_more_than_8)
        return FDCAN_DATA_BYTES_12;
    return FDCAN_DATA_BYTES_8;
}

rt_uint8_t _mcan1_rx_ext_event_msg_size(void)
{
    //计算发送报文的dlc最大值，直接送到TxElmtSize
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>8))?1:0)|
        rt_bool_t dlc_more_than_8=CAN_MSG_MATRIX 0;
        //为0意味着没有比8大的帧，直接设置为8就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>12))?1:0)|
        rt_bool_t dlc_more_than_12=CAN_MSG_MATRIX 0;
        //为0意味着没有比12大的帧，直接设置为12就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>16))?1:0)|
        rt_bool_t dlc_more_than_16=CAN_MSG_MATRIX 0;
        //为0意味着没有比16大的帧，直接设置为16就行
    #undef Y
    
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>20))?1:0)|
        rt_bool_t dlc_more_than_20=CAN_MSG_MATRIX 0;
        //为0意味着没有比20大的帧，直接设置为20就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>24))?1:0)|
        rt_bool_t dlc_more_than_24=CAN_MSG_MATRIX 0;
        //为0意味着没有比24大的帧，直接设置为24就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>32))?1:0)|
        rt_bool_t dlc_more_than_32=CAN_MSG_MATRIX 0;
        //为0意味着没有比32大的帧，直接设置为32就行
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(dir==kOpenCanRecv)&&(cycle==0)&&(dlc>48))?1:0)|
        rt_bool_t dlc_more_than_48=CAN_MSG_MATRIX 0;
        //为0意味着没有比48大的帧，直接设置为48就行
    #undef Y

    if(dlc_more_than_48)
        return FDCAN_DATA_BYTES_64;
    if(dlc_more_than_32)
        return FDCAN_DATA_BYTES_48;
    if(dlc_more_than_24)
        return FDCAN_DATA_BYTES_32;
    if(dlc_more_than_20)
        return FDCAN_DATA_BYTES_24;
    if(dlc_more_than_16)
        return FDCAN_DATA_BYTES_20;
    if(dlc_more_than_12)
        return FDCAN_DATA_BYTES_16;
    if(dlc_more_than_8)
        return FDCAN_DATA_BYTES_12;
    return FDCAN_DATA_BYTES_8;
}



// void _mcan_msg_ram_auto_cfg(void)
// {
//     //带周期的stdid帧和extid帧少于64个，直接安排
//     if((num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>0)&&
//        (num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf<=64))
//     {


//         //处理事件报文

//         //如果stdid只有两个周期为0的帧
//         if(num_of_mcan1_recv_stdid_fifo==2)
//         {
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t id1=CAN_MSG_MATRIX 0;
//             #undef Y
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(id!=id1)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t id2=CAN_MSG_MATRIX 0;
//             #undef Y
//             FDCAN_FilterTypeDef sStdFifoFilterConfig;
//             sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
//             sStdFifoFilterConfig.FilterIndex=std_filt_index;
//             sStdFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
//             sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
//             sStdFifoFilterConfig.FilterID1=id1;
//             sStdFifoFilterConfig.FilterID2=id2;
//             sStdFifoFilterConfig.RxBufferIndex=0;//ignored
//             sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
//             std_filt_index++;
//         }
//         else if(num_of_mcan1_recv_stdid_fifo>2)
//         {
//             //如果是连续的，用range 如果不是 用classic filter mask
//             //range不好实现

//             //先找到符合的id中都是1的位
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//                 (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//                   (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
//                 rt_uint16_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
//             #undef Y

//             //查找符合的id中都是0的位
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//                 (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//                   (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
//                 rt_uint16_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
//             #undef Y

//             rt_uint16_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t filter_bit=CAN_MSG_MATRIX 0;
//             #undef Y

//             FDCAN_FilterTypeDef sStdFifoFilterConfig;
//             sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
//             sStdFifoFilterConfig.FilterIndex=std_filt_index;
//             sStdFifoFilterConfig.FilterType=FDCAN_FILTER_MASK;
//             sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
//             sStdFifoFilterConfig.FilterID1=filter_bit;
//             sStdFifoFilterConfig.FilterID2=mask_bit;
//             sStdFifoFilterConfig.RxBufferIndex=0;//ignored
//             sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
//             std_filt_index++;
//         }

//         //如果extid只有两个周期为0的帧
//         if(num_of_mcan1_recv_extid_fifo==2)
//         {
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t id1=CAN_MSG_MATRIX 0;
//             #undef Y
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(id!=id1)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t id2=CAN_MSG_MATRIX 0;
//             #undef Y
//             FDCAN_FilterTypeDef sExtFifoFilterConfig;
//             sExtFifoFilterConfig.IdType=FDCAN_EXTENDED_ID;
//             sExtFifoFilterConfig.FilterIndex=std_filt_index;
//             sExtFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
//             sExtFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;
//             sExtFifoFilterConfig.FilterID1=id1;
//             sExtFifoFilterConfig.FilterID2=id2;
//             sExtFifoFilterConfig.RxBufferIndex=0;//ignored
//             sExtFifoFilterConfig.IsCalibrationMsg=0;//ignored
//             std_filt_index++;
//         }
//         else if(num_of_mcan1_recv_stdid_fifo>2)
//         {
//             //如果是连续的，用range 如果不是 用classic filter mask
//             //range不好实现

//             //先找到符合的id中都是1的位
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//                 (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//                   (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
//                 rt_uint16_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
//             #undef Y

//             //查找符合的id中都是0的位
//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//                 (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//                   (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
//                 rt_uint16_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
//             #undef Y

//             rt_uint16_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

//             #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
//             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
//              (dir==kOpenCanRecv)&&(cycle==0))?(id):
//             uint32_t filter_bit=CAN_MSG_MATRIX 0;
//             #undef Y

//             FDCAN_FilterTypeDef sStdFifoFilterConfig;
//             sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
//             sStdFifoFilterConfig.FilterIndex=std_filt_index;
//             sStdFifoFilterConfig.FilterType=FDCAN_FILTER_MASK;
//             sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
//             sStdFifoFilterConfig.FilterID1=filter_bit;
//             sStdFifoFilterConfig.FilterID2=mask_bit;
//             sStdFifoFilterConfig.RxBufferIndex=0;//ignored
//             sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
//             std_filt_index++;
//         }


//     }
//     else if(num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>64)//有周期的帧多于64个
//     {
//         //周期短的先安排，
//     }
// }

void _mcan1_cfg_cyc_msg_ram(void)
{
    FDCAN_FilterTypeDef sFilterConfig[sk_mcan1_recv_cycle_stdmsg_num+sk_mcan1_recv_cycle_extmsg_num];
    rt_uint8_t i=0;//一次配置完后需要归零
    rt_uint8_t std_filt_index=0;
    rt_uint8_t ext_filt_index=0;
    rt_uint8_t rx_buf_index=0;

    //处理周期报文

    //配置rx stdid buf
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    if((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
        (dir==kOpenCanRecv)&&(cycle!=0)){\
        sFilterConfig[i].IdType=FDCAN_STANDARD_ID;\
        sFilterConfig[i].FilterIndex=std_filt_index;\
        sFilterConfig[i].FilterType=0;\
        sFilterConfig[i].FilterConfig=FDCAN_FILTER_TO_RXBUFFER;\
        sFilterConfig[i].FilterID1=id;\
        sFilterConfig[i].FilterID2=0;\
        sFilterConfig[i].RxBufferIndex=rx_buf_index;\
        sFilterConfig[i].IsCalibrationMsg=0;\
        i++;\
        std_filt_index++;\
        rx_buf_index++;\
    }
    CAN_MSG_MATRIX
    #undef Y

    //配置rx extid buf
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    if((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
        (dir==kOpenCanRecv)&&(cycle!=0)){\
        sFilterConfig[i].IdType=FDCAN_STANDARD_ID;\
        sFilterConfig[i].FilterIndex=ext_filt_index;\
        sFilterConfig[i].FilterType=0;\
        sFilterConfig[i].FilterConfig=FDCAN_FILTER_TO_RXBUFFER;\
        sFilterConfig[i].FilterID1=id;\
        sFilterConfig[i].FilterID2=0;\
        sFilterConfig[i].RxBufferIndex=rx_buf_index;\
        sFilterConfig[i].IsCalibrationMsg=0;\
        i++;\
        ext_filt_index++;\
        rx_buf_index++;\
    }
    CAN_MSG_MATRIX
    #undef Y
}

rt_err_t _stm32_mcan1_clk_cfg(FDCAN_HandleTypeDef* pmcan)
{
    //读取时钟
    #if defined(SOC_SERIES_STM32H7)
    uint32_t CAN_APB_CLOCK = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    #endif
}

rt_err_t _stm32_mcan1_init(FDCAN_HandleTypeDef* pmcan)
{
    pmcan->Instance=FDCAN1;

    //基本配置
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(frame_fmt==kOpenCanFd))?1:0)|
        rt_bool_t is_fd=CAN_MSG_MATRIX 0;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(brs==1))?1:0)|
        rt_bool_t is_brs=CAN_MSG_MATRIX 0;
    #undef Y
    if(is_fd)
    {
        if(is_brs)
            pmcan->Init.FrameFormat=FDCAN_FRAME_FD_BRS;
        else
            pmcan->Init.FrameFormat=FDCAN_FRAME_FD_NO_BRS;
    }
    else//class
        pmcan->Init.FrameFormat=FDCAN_FRAME_CLASSIC;
    
    pmcan->Init.Mode=FDCAN_MODE_NORMAL;

    //波特率配置

    //msg ram部分
    //统计报文数量(编译期已经统计好了)

    //没有周期报文
    if((sk_mcan1_recv_cycle_stdmsg_num+sk_mcan1_recv_cycle_extmsg_num)==0)
    {
        //直接不配置rxbuf，给fifo01和tx释放空间
        //如果std ext 任一类的event报文不存在，而存在的报文数量超过32个，则报文平均分配到两个fifo
    }
    //周期报文数量rxbuf放得下
    else if((sk_mcan1_recv_cycle_stdmsg_num+sk_mcan1_recv_cycle_extmsg_num)<=64)
    {
        //计算stdcyc+extcyc的最大报文长度
        s_mcan1_rxbuf_ele_size=_mcan1_rxbuf_size();
        //11 bit filter size
        rt_uint8_t std_filter_buf_size=sk_mcan1_recv_cycle_stdmsg_num;
        rt_uint8_t ext_filter_buf_size=sk_mcan1_recv_cycle_extmsg_num*2;
        rt_uint8_t rxbuf_size=s_mcan1_rxbuf_ele_size*(sk_mcan1_recv_cycle_stdmsg_num+sk_mcan1_recv_cycle_extmsg_num);
    }
    //周期报文数量大于rxbuf数量
    else
    {
        //长周期的丢去对应fifo，如果std ext 任一类的event报文不存在，则长周期的丢去空fifo
    }

    //如果recv event的任意一个或者都为0，fifo0 fifo1作他用
    //tx部分
    //配置 TxBuffersNbr TxFifoQueueElmtsNbr TxFifoQueueMode
    // _mcan1_txbuf_cfg(pmcan);
    pmcan->Init.TxElmtSize=_mcan1_tx_size();
}

void _mcan_msgram_conflit(void)
{

}

int rt_hw_mcan_init(void)
{

}
// INIT_BOARD_EXPORT(rt_hw_mcan_init);

#endif
#endif /* RT_USING_CAN */
