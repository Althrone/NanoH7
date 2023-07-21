// #include "bmi088_sample.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_sdio.h"

static rt_thread_t USART_thread =RT_NULL;
void USART_enter(void *parameter);

// #define CAN_
// #define CAN_Pin GET_PIN(B, 14)

int main(void)
{
    // HAL_FDCAN_AddMessageToTxBuffer
    // HAL_FDCAN_EnableTxBufferRequest
    // SD_HandleTypeDef hsdio={0};
    // hsdio.Instance=SDMMC1;
    // hsdio.Init.ClockEdge=SDMMC_CLOCK_EDGE_RISING;
    // hsdio.Init.BusWide=SDMMC_BUS_WIDE_4B;
    // hsdio.Init.ClockPowerSave=SDMMC_CLOCK_POWER_SAVE_DISABLE;
    // hsdio.Init.HardwareFlowControl=SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    // hsdio.Init.ClockDiv=SDMMC_NSpeed_CLK_DIV;

    // HAL_SD_Init(&hsdio);
    // HAL_SD_CardInfoTypeDef g_sd_card_info_handle;
    // HAL_SD_GetCardInfo(&hsdio, &g_sd_card_info_handle);
    // sdcard_change();
    // double a=2.1f,b;
    // a=2.3*3.1;
    // b=a*12;
    // while(1)
    // {
    //     // rt_kprintf("LED_thread 线程被挂起!\r\n");
    // }
    // imu_thread_entry(0);

    //使能can终端电阻
    rt_pin_mode(GET_PIN(D,5),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,5),PIN_LOW);//使能终端电阻

    rt_pin_mode(GET_PIN(D,4),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,4),PIN_LOW);//使能终端电阻

    USART_thread=rt_thread_create("USART_thread",
                                USART_enter,
                                RT_NULL,
                                1024,
                                5,
                                5);
    if(USART_thread != RT_NULL) rt_thread_startup(USART_thread);
    return 0;
}

void USART_enter(void *parameter)
{
	//   rt_uint32_t count_U = 0;
    static rt_device_t can_dev;
    volatile rt_err_t ret;
    can_dev=rt_device_find("fdcan1");
    /* 以中断接收及发送模式打开 CAN 设备 */
    ret=rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    /* 设置 CAN 的工作模式为正常工作模式 */
    ret = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    // RT_CAN_MODE_LOOPBACK
    static struct rt_can_status status;    /* 获取到的 CAN 总线状态 */
    ret=rt_device_control(can_dev, RT_CAN_CMD_GET_STATUS, &status);

    struct rt_can_msg msg = {0};

    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = 0x11;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = 0x44;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
    /* 发送一帧 CAN 数据 */
    rt_size_t size=rt_device_write(can_dev, 0, &msg, sizeof(msg));
    if (size == 0)
    {
        rt_kprintf("can dev write data failed!\n");
    }
    else
    {
        rt_kprintf("can dev write data success!\n");
    }
	
    // while (1)
    // {
				rt_thread_mdelay(500);	        
				// rt_kprintf("thread usart count: %d\r\n",  count_U ++); /* 打印线程计数值输出 */
				// rt_thread_resume(&led0_thread);		
				// rt_kprintf("LED_thread 线程被解挂!\r\n");
				// if(count_U==10)
				// {
				// 		rt_thread_detach(&led0_thread);
				// 		rt_kprintf("LED_thread 线程被删除！\r\n");
				// }
        
    // }
}