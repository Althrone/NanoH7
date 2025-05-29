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

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/******************************************************************************
 * private functions definition
 *****************************************************************************/



uint8_t _mcan_test(void)
{
    // 重定义Y宏以提取dlc参数
#define Y(inst, id_type, id, dir, dlc, cycle) dlc,
CAN_MSG_MATRIX

// 递归展开宏定义
#define ARG_MAX(...) ARG_MAX_IMPL(__VA_ARGS__)
#define ARG_MAX_IMPL(...) ARG_MAX_IMPL_(__VA_ARGS__)
#define ARG_MAX_IMPL_(x, ...) MAX(x, ARG_MAX_IMPL(__VA_ARGS__))
#define ARG_MAX_IMPL_(x) (x)  // 终止条件：单参数时返回自身

// 最终的最大dlc值
#define MAX_VALUE ARG_MAX( \
    8, 8\
)

return MAX_VALUE;
// 最终的最大dlc值
}
// #undef Y
//计算发送报文个数
//计算发送报文的dlc最大值，直接送到TxElmtSize
//计算周期发送报文数量
//计算事件发送报文数量

//如果周期数量小于32个 直接安排

//计算接收报文的个数
//

//通过识别信号矩阵决定信号如何分配
// static void _mcan_msg_ram_auto_cfg(void)
void _mcan_msg_ram_auto_cfg(void)
{

//周期非0的放到buf，0的是event信号放fifo
#define Y(inst,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
      (dir==kOpenCanRecv)&&(cycle!=0))?1:0)+
    rt_uint8_t num_of_mcan1_recv_stdid_buf=CAN_MSG_MATRIX 0;
#undef Y

#define Y(inst,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
      (dir==kOpenCanRecv)&&(cycle!=0))?1:0)+
    rt_uint8_t num_of_mcan1_recv_extid_buf=CAN_MSG_MATRIX 0;
#undef Y
//周期为0的数据
#define Y(inst,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
      (dir==kOpenCanRecv)&&(cycle==0))?1:0)+
    rt_uint8_t num_of_mcan1_recv_stdid_fifo=CAN_MSG_MATRIX 0;
#undef Y

#define Y(inst,id_type,id,dir,dlc,cycle) \
    (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
      (dir==kOpenCanRecv)&&(cycle==0))?1:0)+
    rt_uint8_t num_of_mcan1_recv_extid_fifo=CAN_MSG_MATRIX 0;
#undef Y

    //带周期的stdid帧和extid帧少于64个，直接安排
    if((num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>0)&&
       (num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf<=64))
    {
        FDCAN_FilterTypeDef sFilterConfig[num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf];
        rt_uint8_t i=0;//一次配置完后需要归零
        rt_uint8_t std_filt_index=0;
        rt_uint8_t ext_filt_index=0;
        rt_uint8_t rx_buf_index=0;
        //配置rx stdid buf
        #define Y(inst,id_type,id,dir,dlc,cycle) \
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
        #define Y(inst,id_type,id,dir,dlc,cycle) \
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

        //如果stdid只有两个周期为0的帧
        if(num_of_mcan1_recv_stdid_fifo==2)
        {
            #define Y(inst,id_type,id,dir,dlc,cycle) \
            ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
             (dir==kOpenCanRecv)&&(cycle==0))?(id):
            uint32_t id1=CAN_MSG_MATRIX 0;
            #undef Y
            #define Y(inst,id_type,id,dir,dlc,cycle) \
            ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(id!=id1)&&\
             (dir==kOpenCanRecv)&&(cycle==0))?(id):
            uint32_t id2=CAN_MSG_MATRIX 0;
            #undef Y
            FDCAN_FilterTypeDef sStdFifoFilterConfig;
            sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
            sStdFifoFilterConfig.FilterIndex=std_filt_index;
            sStdFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
            sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
            sStdFifoFilterConfig.FilterID1=id1;
            sStdFifoFilterConfig.FilterID2=id2;
            sStdFifoFilterConfig.RxBufferIndex=0;//ignored
            sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
            std_filt_index++;
        }
        else if(num_of_mcan1_recv_stdid_fifo>2)
        {
            //如果是连续的，用range 如果不是 用classic filter mask
            //range不好实现

            //先找到符合的id中都是1的位
            #define Y(inst,id_type,id,dir,dlc,cycle) \
                (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
                  (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
                rt_uint16_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
            #undef Y

            //查找符合的id中都是0的位
            #define Y(inst,id_type,id,dir,dlc,cycle) \
                (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
                  (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
                rt_uint16_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
            #undef Y

            rt_uint16_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

            #define Y(inst,id_type,id,dir,dlc,cycle) \
            ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
             (dir==kOpenCanRecv)&&(cycle==0))?(id):
            uint32_t filter_bit=CAN_MSG_MATRIX 0;
            #undef Y

            FDCAN_FilterTypeDef sStdFifoFilterConfig;
            sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
            sStdFifoFilterConfig.FilterIndex=std_filt_index;
            sStdFifoFilterConfig.FilterType=FDCAN_FILTER_MASK;
            sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
            sStdFifoFilterConfig.FilterID1=filter_bit;
            sStdFifoFilterConfig.FilterID2=mask_bit;
            sStdFifoFilterConfig.RxBufferIndex=0;//ignored
            sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
            std_filt_index++;
        }

        

        //如果extid只有两个周期为0的帧
        if(num_of_mcan1_recv_extid_fifo==2)
        {
            #define Y(inst,id_type,id,dir,dlc,cycle) \
            ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
             (dir==kOpenCanRecv)&&(cycle==0))?(id):
            uint32_t id1=CAN_MSG_MATRIX 0;
            #undef Y
            #define Y(inst,id_type,id,dir,dlc,cycle) \
            ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(id!=id1)&&\
             (dir==kOpenCanRecv)&&(cycle==0))?(id):
            uint32_t id2=CAN_MSG_MATRIX 0;
            #undef Y
            FDCAN_FilterTypeDef sExtFifoFilterConfig;
            sExtFifoFilterConfig.IdType=FDCAN_EXTENDED_ID;
            sExtFifoFilterConfig.FilterIndex=std_filt_index;
            sExtFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
            sExtFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;
            sExtFifoFilterConfig.FilterID1=id1;
            sExtFifoFilterConfig.FilterID2=id2;
            sExtFifoFilterConfig.RxBufferIndex=0;//ignored
            sExtFifoFilterConfig.IsCalibrationMsg=0;//ignored
            std_filt_index++;
        }
        


        //event类型 stdid报文放在fifo0 extid类型放在fifo1
        //合并周期为0的id的过滤器
        #define Y(inst,id_type,id,dir,dlc,cycle) \
        &((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
         (dir==kOpenCanRecv)&&(cycle==0))?id:0x7ff
        rt_uint16_t std_and_vel=0x7FF CAN_MSG_MATRIX;
        #undef Y

        if(std_and_vel==0)
        {
            //说明这些id都是连续的
        }
    }
    else if(num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>64)//有周期的帧多于64个
    {
        //周期短的先安排，
    }
}
#endif
#endif /* RT_USING_CAN */
