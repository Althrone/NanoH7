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

#include "canx.h"

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

struct stm32_mcan
{
    FDCAN_HandleTypeDef hmcan;
    struct stm32_mcan_config *config;
    struct rt_canx_device canx;
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

static struct stm32_mcan s_mcan_obj[sizeof(s_mcan_config)/sizeof(s_mcan_config[0])]={0};

static rt_err_t _inline_canx_configure(struct rt_canx_device *canx, struct canx_configure *cfg);
static rt_err_t _inline_canx_control(struct rt_canx_device *canx, int cmd, void *arg);
static int _inline_canx_recvmsg(struct rt_canx_device *canx, void *buf, rt_uint32_t boxno);
static int _inline_canx_sendmsg(struct rt_canx_device *canx, const void *buf, rt_uint32_t box_num);

static const struct rt_canx_ops s_mcan_ops=
{
    .configure= _inline_canx_configure,
    .control= _inline_canx_control,
    .sendmsg= _inline_canx_sendmsg,
    .recvmsg=_inline_canx_recvmsg,
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
////////////////////////////////接收/////////////////////////////////
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

static rt_err_t _stm32_mcan_baud_cfg(FDCAN_HandleTypeDef* pmcan,struct canx_configure *cfg);
static rt_err_t _stm32_mcan_msg_ram_cfg(FDCAN_HandleTypeDef* pmcan);
static rt_err_t _stm32_mcan_filter_cfg(FDCAN_HandleTypeDef* pmcan);

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/
#if 0
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
      #ifdef BSP_USING_MCAN1
        if(RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)//有新数据传到fifo
        {
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_RX_IND|(FDCAN_RX_FIFO0<<8));
        }
      #endif
    }  
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
      #ifdef BSP_USING_MCAN1
        if(RxFifo1ITs&FDCAN_IT_RX_FIFO1_NEW_MESSAGE)//有新数据传到fifo
        {
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_RX_IND|(FDCAN_RX_FIFO1<<8));
        }
      #endif
    }  
}

void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
      #ifdef BSP_USING_MCAN1
        //读NDAT1 NDAT2看哪些buf来了新数据
        for (size_t i = 0; i < 32; i++)
        {
            if(hfdcan->Instance->NDAT1&(1<<i))
            {
                rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_RX_IND|((FDCAN_RX_BUFFER0+i)<<8));
            }
        }
        for (size_t i = 0; i < 32; i++)
        {
            if(hfdcan->Instance->NDAT2&(1<<i))
            {
                rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_RX_IND|((FDCAN_RX_BUFFER32+i)<<8));
            }
        }
      #endif
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
        switch (ErrorStatusITs)
        {
        case FDCAN_IT_ERROR_WARNING://TEC或者REC其中一个到达进入error passive的一半
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_ERROR_WARNING);//博世mcan自己加的一个中间状态
            break;
        case FDCAN_IT_ERROR_PASSIVE://todo
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_ERROR_PASSIVE);
            break;
        case FDCAN_IT_BUS_OFF://bus off的处理
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_BUS_OFF);
            break;
        default:
            break;
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
        uint32_t ErrorCode=HAL_FDCAN_GetError(hfdcan);
        if(ErrorCode&HAL_FDCAN_ERROR_LOG_OVERFLOW)
        {
            rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_BUS_OFF);
        }
    }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
        rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_TX_DONE|(BufferIndexes<<8));
    }
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
        rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_TX_FAIL|(BufferIndexes<<8));
    }
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    struct stm32_mcan *mcan;
    RT_ASSERT(hfdcan != NULL);
    mcan = (struct stm32_uart *)hfdcan;

    if(hfdcan->Instance==FDCAN1)
    {
        //意味着可以往fifo放不超过fifo大小的can帧
        rt_hw_canx_isr(&mcan->canx,RT_CANX_EVENT_TX_DONE);//isr识别到高位为0就是fifo空的意思
    }
}
#endif
/******************************************************************************
 * private functions definition
 *****************************************************************************/

static rt_err_t _inline_canx_control(struct rt_canx_device *canx, int cmd, void *arg)
{
    struct stm32_mcan *pmcan;
    RT_ASSERT(canx != RT_NULL);
    //获取父结构体
    pmcan = rt_container_of(canx, struct stm32_mcan, canx);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT://关中断
        HAL_NVIC_DisableIRQ(pmcan->config->irq0_type);
        HAL_FDCAN_DeactivateNotification(&pmcan->hmcan,FDCAN_IT_RX_BUFFER_NEW_MESSAGE);
        HAL_FDCAN_DeactivateNotification(&pmcan->hmcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        HAL_FDCAN_DeactivateNotification(&pmcan->hmcan,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
        HAL_NVIC_DisableIRQ(pmcan->config->irq1_type);
        HAL_FDCAN_DeactivateNotification(&pmcan->hmcan,FDCAN_IT_TX_COMPLETE);
        break;
    case RT_DEVICE_CTRL_SET_INT://设置中断
        HAL_NVIC_SetPriority(pmcan->config->irq0_type, 0, 1);
        HAL_NVIC_SetPriority(pmcan->config->irq1_type, 0, 2);
        HAL_NVIC_EnableIRQ(pmcan->config->irq0_type);
        HAL_NVIC_EnableIRQ(pmcan->config->irq1_type);
        //rx新数据中断
        HAL_FDCAN_ActivateNotification(&pmcan->hmcan,FDCAN_IT_RX_BUFFER_NEW_MESSAGE,RT_NULL);
        HAL_FDCAN_ActivateNotification(&pmcan->hmcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,RT_NULL);
        HAL_FDCAN_ActivateNotification(&pmcan->hmcan,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,RT_NULL);
        //这三个应该丢去初始化
        HAL_FDCAN_ConfigInterruptLines(&pmcan->hmcan,FDCAN_IT_RX_BUFFER_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
        HAL_FDCAN_ConfigInterruptLines(&pmcan->hmcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
        HAL_FDCAN_ConfigInterruptLines(&pmcan->hmcan,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
        //tx
        // HAL_FDCAN_ActivateNotification(&pmcan->hmcan,FDCAN_IT_TX_COMPLETE,);//开了几个buf就写几个
        //丢去初始化
        HAL_FDCAN_ConfigInterruptLines(&pmcan->hmcan,FDCAN_IT_TX_COMPLETE, FDCAN_INTERRUPT_LINE1);
        break;
    default:
        break;
    }
}

static int _inline_canx_recvmsg(struct rt_canx_device *canx, void *buf, rt_uint32_t boxno)
{
    struct rt_canx_msg *pmsg=(struct rt_canx_msg*)buf;
    RT_ASSERT(canx != RT_NULL);
    struct stm32_mcan *pmcan;
    //获取父结构体
    pmcan = rt_container_of(canx, struct stm32_mcan, canx);
    FDCAN_RxHeaderTypeDef rx_header;
    if(HAL_FDCAN_GetRxMessage(&pmcan->hmcan,boxno,&rx_header,pmsg->data)!=HAL_OK)
    {
        return -RT_ERROR;
    }
    else
    {
        pmsg->id=rx_header.Identifier;
        if(rx_header.IdType==FDCAN_EXTENDED_ID)
            pmsg->ide=1;
        else pmsg->ide=0;

        pmsg->dlc=rx_header.DataLength>>16;

        if(rx_header.FDFormat==FDCAN_FD_CAN)
        {    
            pmsg->fdf=1;
            if(rx_header.BitRateSwitch==FDCAN_BRS_ON)
                pmsg->brs=1;
            else pmsg->brs=0;
            if(rx_header.ErrorStateIndicator==FDCAN_ESI_PASSIVE)
                pmsg->esi=1;//没错误发1？
            else pmsg->esi=0;
            // pmsg->rtr=0;//RRS 除了参与总线总裁外无意义
        }
        else//经典can才判断是否为远程帧
        {
            pmsg->fdf=0;
            if(rx_header.RxFrameType==FDCAN_REMOTE_FRAME)
                pmsg->rtr=1;
            else pmsg->rtr=0;
            // brs无意义
            // esi无意义
        }

        //user_data不在这里更新

        return 1;//一个帧
    }
}

static int _inline_canx_sendmsg(struct rt_canx_device *canx, const void *buf, rt_uint32_t box_num)
{
    struct rt_canx_msg *pmsg=(struct rt_canx_msg*)buf;
    RT_ASSERT(canx != RT_NULL);
    struct stm32_mcan *pmcan;
    //获取父结构体
    pmcan = rt_container_of(canx, struct stm32_mcan, canx);

    //根据boxnumber决定txheader
    FDCAN_TxHeaderTypeDef TxHeader={0};
    TxHeader.Identifier=pmsg->id;
    if(pmsg->ide)
        TxHeader.IdType=FDCAN_EXTENDED_ID;
    if(pmsg->rtr)
        TxHeader.TxFrameType=FDCAN_REMOTE_FRAME;
    TxHeader.DataLength=(uint32_t)(pmsg->dlc)<<16;
    if(pmsg->esi)
        TxHeader.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
    if(pmsg->brs)
        TxHeader.BitRateSwitch=FDCAN_BRS_ON;
    if(pmsg->fdf)
        TxHeader.FDFormat=FDCAN_FD_CAN;

    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker=0;

    if(box_num<32)
    {
        HAL_FDCAN_AddMessageToTxBuffer(&pmcan->hmcan,&TxHeader,pmsg->data,box_num);
        HAL_FDCAN_EnableTxBufferRequest(&pmcan->hmcan,box_num);
    }
    else
    {
        if(HAL_FDCAN_GetTxFifoFreeLevel(&pmcan->hmcan))
            HAL_FDCAN_AddMessageToTxFifoQ(&pmcan->hmcan,&TxHeader,pmsg->data);
    }
}

static rt_err_t _inline_canx_configure(struct rt_canx_device *canx, struct canx_configure *cfg)
{
    RT_ASSERT(canx);
	RT_ASSERT(cfg);

    struct stm32_mcan *pmcan;
    pmcan = rt_container_of(canx, struct stm32_mcan, canx);

    //配置hal部分的参数
    pmcan->hmcan.Instance=pmcan->config->Instance;//外设基地址从mcan_config的宏传入

    //通过can矩阵来判断是否有canfd帧
  #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
           (((inst==MCAN1_INDEX)&&(frame_fmt==kOpenCanFd))?1:0)|
    rt_bool_t is_fd=CAN_MSG_MATRIX 0;
  #undef Y
    //通过can矩阵来判断是否有canfd帧使用BRS
  #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
           (((inst==MCAN1_INDEX)&&(brs==1))?1:0)|
    rt_bool_t is_brs=CAN_MSG_MATRIX 0;
  #undef Y

  #ifndef RT_CANX_USING_FD
    RT_ASSERT(is_fd);
    pmcan->hmcan.Init.FrameFormat=FDCAN_FRAME_CLASSIC;
  #else
    if(is_brs)//根据can表的设置决定是否开启BRS
        pmcan->hmcan.Init.FrameFormat=FDCAN_FRAME_FD_BRS;
    else
        pmcan->hmcan.Init.FrameFormat=FDCAN_FRAME_FD_NO_BRS;
  #endif
    
    pmcan->hmcan.Init.Mode=FDCAN_MODE_NORMAL;

    pmcan->hmcan.Init.AutoRetransmission=DISABLE;
    pmcan->hmcan.Init.TransmitPause=DISABLE;
    pmcan->hmcan.Init.ProtocolException=DISABLE;

    //波特率设置
    _stm32_mcan_baud_cfg(&pmcan->hmcan,cfg);

    //msg ram设置
    _stm32_mcan_msg_ram_cfg(&pmcan->hmcan);

    //过滤器配置
    _stm32_mcan_filter_cfg(&pmcan->hmcan);

    if (HAL_FDCAN_Init(&pmcan->hmcan) != HAL_OK)
	{
		return -RT_ERROR;
	}

    HAL_FDCAN_Start(&pmcan->hmcan);
	return RT_EOK;
}

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
    //计算发送报文的dlc最大值，直接送到RxBufferSize

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>48))?1:0)|
        rt_bool_t dlc_more_than_48=CAN_MSG_MATRIX 0;
        if(dlc_more_than_48)
            return FDCAN_DATA_BYTES_64;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>32))?1:0)|
        rt_bool_t dlc_more_than_32=CAN_MSG_MATRIX 0;
        if(dlc_more_than_32)
            return FDCAN_DATA_BYTES_48;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>24))?1:0)|
        rt_bool_t dlc_more_than_24=CAN_MSG_MATRIX 0;
        if(dlc_more_than_24)
            return FDCAN_DATA_BYTES_32;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>20))?1:0)|
        rt_bool_t dlc_more_than_20=CAN_MSG_MATRIX 0;
        if(dlc_more_than_20)
            return FDCAN_DATA_BYTES_24;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>16))?1:0)|
        rt_bool_t dlc_more_than_16=CAN_MSG_MATRIX 0;
        if(dlc_more_than_16)
            return FDCAN_DATA_BYTES_20;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>12))?1:0)|
        rt_bool_t dlc_more_than_12=CAN_MSG_MATRIX 0;
        if(dlc_more_than_12)
            return FDCAN_DATA_BYTES_16;
    #undef Y

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
        (((inst==MCAN1_INDEX)&&(dir==kOpenCanRecv)&&(cycle!=0)&&(dlc>8))?1:0)|
        rt_bool_t dlc_more_than_8=CAN_MSG_MATRIX 0;
        if(dlc_more_than_8)
            return FDCAN_DATA_BYTES_12;
    #undef Y

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

static void _mcan1_get_std_event_msg_classic_filter(uint32_t* id1_filter,uint32_t* id2_mask)
{
    //先找到符合的id中都是1的位
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
               (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
        uint32_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
    #undef Y

    //查找符合的id中都是0的位
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
               (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
        uint32_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
    #undef Y

    uint32_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
              (dir==kOpenCanRecv)&&(cycle==0))?(id):
        uint32_t filter_bit=CAN_MSG_MATRIX 0;
    #undef Y

    *id1_filter=filter_bit;
    *id2_mask=mask_bit;
}

static void _mcan1_get_ext_event_msg_classic_filter(uint32_t* id1_filter,uint32_t* id2_mask)
{
    //先找到符合的id中都是1的位
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
               (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
        uint32_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
    #undef Y

    //查找符合的id中都是0的位
    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
               (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
        uint32_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
    #undef Y

    uint32_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

    #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
             ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
              (dir==kOpenCanRecv)&&(cycle==0))?(id):
        uint32_t filter_bit=CAN_MSG_MATRIX 0;
    #undef Y

    *id1_filter=filter_bit;
    *id2_mask=mask_bit;
}

void _mcan_msg_ram_auto_cfg(void)
{
    // //带周期的stdid帧和extid帧少于64个，直接安排
    // if((num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>0)&&
    //    (num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf<=64))
    // {


    //     //处理事件报文

    //     //如果stdid只有两个周期为0的帧
    //     if(num_of_mcan1_recv_stdid_fifo==2)
    //     {
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t id1=CAN_MSG_MATRIX 0;
    //         #undef Y
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&(id!=id1)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t id2=CAN_MSG_MATRIX 0;
    //         #undef Y
    //         FDCAN_FilterTypeDef sStdFifoFilterConfig;
    //         sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
    //         sStdFifoFilterConfig.FilterIndex=std_filt_index;
    //         sStdFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
    //         sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    //         sStdFifoFilterConfig.FilterID1=id1;
    //         sStdFifoFilterConfig.FilterID2=id2;
    //         sStdFifoFilterConfig.RxBufferIndex=0;//ignored
    //         sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
    //         std_filt_index++;
    //     }
    //     else if(num_of_mcan1_recv_stdid_fifo>2)
    //     {
    //         //如果是连续的，用range 如果不是 用classic filter mask
    //         //range不好实现

    //         //先找到符合的id中都是1的位
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //               (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
    //             rt_uint16_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
    //         #undef Y

    //         //查找符合的id中都是0的位
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //               (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
    //             rt_uint16_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
    //         #undef Y

    //         rt_uint16_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t filter_bit=CAN_MSG_MATRIX 0;
    //         #undef Y

    //         FDCAN_FilterTypeDef sStdFifoFilterConfig;
    //         sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
    //         sStdFifoFilterConfig.FilterIndex=std_filt_index;
    //         sStdFifoFilterConfig.FilterType=FDCAN_FILTER_MASK;
    //         sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    //         sStdFifoFilterConfig.FilterID1=filter_bit;
    //         sStdFifoFilterConfig.FilterID2=mask_bit;
    //         sStdFifoFilterConfig.RxBufferIndex=0;//ignored
    //         sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
    //         std_filt_index++;
    //     }

    //     //如果extid只有两个周期为0的帧
    //     if(num_of_mcan1_recv_extid_fifo==2)
    //     {
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t id1=CAN_MSG_MATRIX 0;
    //         #undef Y
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanExtId)&&(id!=id1)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t id2=CAN_MSG_MATRIX 0;
    //         #undef Y
    //         FDCAN_FilterTypeDef sExtFifoFilterConfig;
    //         sExtFifoFilterConfig.IdType=FDCAN_EXTENDED_ID;
    //         sExtFifoFilterConfig.FilterIndex=std_filt_index;
    //         sExtFifoFilterConfig.FilterType=FDCAN_FILTER_DUAL;
    //         sExtFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;
    //         sExtFifoFilterConfig.FilterID1=id1;
    //         sExtFifoFilterConfig.FilterID2=id2;
    //         sExtFifoFilterConfig.RxBufferIndex=0;//ignored
    //         sExtFifoFilterConfig.IsCalibrationMsg=0;//ignored
    //         std_filt_index++;
    //     }
    //     else if(num_of_mcan1_recv_stdid_fifo>2)
    //     {
    //         //如果是连续的，用range 如果不是 用classic filter mask
    //         //range不好实现

    //         //先找到符合的id中都是1的位
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //               (dir==kOpenCanRecv)&&(cycle==0))?id:0x7FF)&
    //             rt_uint16_t bit1_mask=CAN_MSG_MATRIX 0x7FF;
    //         #undef Y

    //         //查找符合的id中都是0的位
    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //             (((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //               (dir==kOpenCanRecv)&&(cycle==0))?(~id):0x7FF)&
    //             rt_uint16_t bit0_mask=CAN_MSG_MATRIX 0x7FF;
    //         #undef Y

    //         rt_uint16_t mask_bit=bit1_mask|bit0_mask;//其中为1的位，对应的filter_bit的0或者1要匹配

    //         #define Y(inst,frame_fmt,brs,id_type,id,dir,dlc,cycle) \
    //         ((inst==MCAN1_INDEX)&&(id_type==kOpenCanStdId)&&\
    //          (dir==kOpenCanRecv)&&(cycle==0))?(id):
    //         uint32_t filter_bit=CAN_MSG_MATRIX 0;
    //         #undef Y

    //         FDCAN_FilterTypeDef sStdFifoFilterConfig;
    //         sStdFifoFilterConfig.IdType=FDCAN_STANDARD_ID;
    //         sStdFifoFilterConfig.FilterIndex=std_filt_index;
    //         sStdFifoFilterConfig.FilterType=FDCAN_FILTER_MASK;
    //         sStdFifoFilterConfig.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    //         sStdFifoFilterConfig.FilterID1=filter_bit;
    //         sStdFifoFilterConfig.FilterID2=mask_bit;
    //         sStdFifoFilterConfig.RxBufferIndex=0;//ignored
    //         sStdFifoFilterConfig.IsCalibrationMsg=0;//ignored
    //         std_filt_index++;
    //     }


    // }
    // else if(num_of_mcan1_recv_stdid_buf+num_of_mcan1_recv_extid_buf>64)//有周期的帧多于64个
    // {
    //     //周期短的先安排，
    // }
}

static rt_err_t _stm32_mcan_baud_cfg(FDCAN_HandleTypeDef* pmcan,struct canx_configure *cfg)
{
    //读取时钟
    //添加其他芯片支持时，不要将uint32_t CAN_APB_CLOCK抽出来，防止没有合适的时钟但是还是通过编译了
    #if defined(SOC_SERIES_STM32H7)
    uint32_t CAN_APB_CLOCK = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    #endif

    uint32_t div=0;
    uint32_t seg1=0;
    uint32_t seg2=0;
  #ifdef RT_CANX_CALC_BITTIMING
    //计算仲裁段波分频
    uint32_t div_quotient =CAN_APB_CLOCK/cfg->nominal_baud.baud_rate;
    uint32_t div_remainder=CAN_APB_CLOCK%cfg->nominal_baud.baud_rate;
    //只有完全整除才往下跑
    RT_ASSERT(div_remainder!=0);

    div=div_quotient/100;//分频到100
    RT_ASSERT((div_quotient%100)!=0);

    //使用最小分频1，因此 num of tq=div_quotient/1
    //不考虑div_quotient是否超过SEG1+SEG2，这个应该是hal层写入寄存器的时候判断的
    uint32_t sp_quotient=div_quotient*cfg->nominal_baud.sample_point/10000;
    seg1=sp_quotient-1;//减去SyncSeg的一个tq
    seg2=div_quotient-sp_quotient;
  #else
    //按照cfg给定的参数写入
  #endif
    
    pmcan->Init.NominalPrescaler=div;
    pmcan->Init.NominalSyncJumpWidth=1;//比较严格
    pmcan->Init.NominalTimeSeg1=seg1;
    pmcan->Init.NominalTimeSeg2=seg2;
  
  #ifdef RT_CANX_USING_FD
    #ifdef RT_CANX_CALC_BITTIMING
        //todo
    #endif
  #endif
}

static rt_err_t _stm32_mcan_msg_ram_cfg(FDCAN_HandleTypeDef* pmcan)
{
    if(pmcan->Instance==FDCAN1)
    {
        pmcan->Init.MessageRAMOffset=0;
        //buf类型需要独有filter，fifo默认做成一个filter
        pmcan->Init.StdFiltersNbr=sk_mcan1_recv_cycle_stdmsg_num+1;
        pmcan->Init.ExtFiltersNbr=sk_mcan1_recv_cycle_extmsg_num+1;

        pmcan->Init.RxFifo0ElmtsNbr=sk_mcan1_recv_event_stdmsg_num;//对应一个filter
        pmcan->Init.RxFifo0ElmtSize=_mcan1_rx_std_event_msg_size();

        pmcan->Init.RxFifo1ElmtsNbr=sk_mcan1_recv_event_extmsg_num;//对应一个filter
        pmcan->Init.RxFifo1ElmtSize=_mcan1_rx_ext_event_msg_size();

        pmcan->Init.RxBuffersNbr=sk_mcan1_recv_cycle_stdmsg_num+sk_mcan1_recv_cycle_extmsg_num;
        pmcan->Init.RxBufferSize=_mcan1_rxbuf_size();

        pmcan->Init.TxEventsNbr=0;

        pmcan->Init.TxBuffersNbr=sk_mcan1_send_cycle_msg_num;
        pmcan->Init.TxFifoQueueElmtsNbr=sk_mcan1_send_event_msg_num;
        pmcan->Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;
        pmcan->Init.TxElmtSize=_mcan1_tx_size();
    }

    #if 0
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
    #endif
}

static rt_err_t _stm32_mcan_filter_cfg(FDCAN_HandleTypeDef* pmcan)
{
    FDCAN_FilterTypeDef sFilterConfig[sk_mcan1_recv_cycle_stdmsg_num+1+sk_mcan1_recv_cycle_extmsg_num+1];
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

    //计算事件报文的mask
    uint32_t std_filter;
    uint32_t std_mask;
    _mcan1_get_event_msg_classic_filter(&std_filter,&std_mask);
    sFilterConfig[i].IdType=FDCAN_STANDARD_ID;
    sFilterConfig[i].FilterIndex=std_filt_index;
    sFilterConfig[i].FilterType=FDCAN_FILTER_MASK;
    sFilterConfig[i].FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig[i].FilterID1=std_filter;
    sFilterConfig[i].FilterID2=std_mask;
    sFilterConfig[i].RxBufferIndex=rx_buf_index;
    sFilterConfig[i].IsCalibrationMsg=0;
    i++;
    std_filt_index++;
    rx_buf_index++;

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

    //计算事件报文的mask
    uint32_t ext_filter;
    uint32_t ext_mask;
    _mcan1_get_ext_event_msg_classic_filter(&ext_filter,&ext_mask);
    sFilterConfig[i].IdType=FDCAN_EXTENDED_ID;
    sFilterConfig[i].FilterIndex=ext_filt_index;
    sFilterConfig[i].FilterType=FDCAN_FILTER_MASK;
    sFilterConfig[i].FilterConfig=FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig[i].FilterID1=ext_filter;
    sFilterConfig[i].FilterID2=ext_mask;
    sFilterConfig[i].RxBufferIndex=rx_buf_index;
    sFilterConfig[i].IsCalibrationMsg=0;
    i++;
    ext_filt_index++;
    rx_buf_index++;

    for (size_t i = 0; i < sizeof(sFilterConfig); i++)
    {
        HAL_FDCAN_ConfigFilter(pmcan,&sFilterConfig[i]);
    }
}

int rt_hw_mcan_init(void)
{
    struct canx_configure config=RT_CANX_CONFIG_DEFAULT;
    for (size_t i = 0; i < (sizeof(s_mcan_obj)/sizeof(struct stm32_mcan)); i++)
    {
        s_mcan_obj[i].config=&s_mcan_config[i];//初始化mcan_config.h宏配置，主要是中断号，引脚号
        s_mcan_obj[i].canx.ops=&s_mcan_ops;
        s_mcan_obj[i].canx.config=config;
        rt_hw_canx_register(&s_mcan_obj[i].canx, s_mcan_obj[i].config->name, &s_mcan_ops, RT_NULL);
    }
}
// INIT_BOARD_EXPORT(rt_hw_mcan_init);

#endif
#endif /* RT_USING_CAN */
