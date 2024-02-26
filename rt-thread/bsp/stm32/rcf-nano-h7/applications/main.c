// #include "bmi088_sample.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "sensor.h"

#include "drv_sdio.h"

#include "dfs_fs.h"

#include "bmi088.h"

static rt_thread_t USART_thread =RT_NULL;
void USART_enter(void *parameter);
void imu_data_thrd(void *parameter);

// #define CAN_
// #define CAN_Pin GET_PIN(B, 14)
rt_sem_t sem;

static rt_err_t cb_1ms(rt_device_t dev, rt_size_t size)
{
    //信号量
    // rt_sem_t sem=rt_object_find("dsem", RT_Object_Class_Semaphore);

    rt_sem_release(sem);

    return 0;
}

int main(void)
{
    volatile rt_uint32_t sysclk=HAL_RCC_GetSysClockFreq();
    volatile rt_uint32_t coreid=DBGMCU->IDCODE;
    //定时器
    rt_uint32_t freq = 1000;
    rt_device_t tim_dev=rt_device_find("timer2");
    rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_set_rx_indicate(tim_dev, cb_1ms);
    rt_device_control(tim_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
    rt_hwtimerval_t timeout_s={
        .sec=0,
        .usec=2000,
    };
    rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));

    //创建信号量
    sem=rt_sem_create("dsem", 0, RT_IPC_FLAG_PRIO);

    char test_str[]="vcom success!\n\t";

    //usb测试
    rt_device_t dev = RT_NULL;
    dev = rt_device_find("vcom");
    rt_device_init(dev);
    rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
    // if(dev)
    // {
    //     while(1)
    //     {
    //         rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
    //         rt_device_write(dev,0,test_str,sizeof(test_str));
    //         rt_thread_mdelay(1000);
    //         // rt_device_close(dev);
    //     }
    // }
        
    // rt_uint16_t sensor_data;
    // rt_device_t dev = rt_device_find("temp_0");
    // // rt_device_init(dev);
    // rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

    // rt_device_read(dev, 0, &sensor_data, 1);

    struct rt_device_pwm *pwm_dev=(struct rt_device_pwm *)rt_device_find("pwm12");
    rt_pwm_set(pwm_dev, 1, 500000, 150000);
    // rt_pwm_set(pwm_dev, 3, 500000, 490000);
    rt_pwm_set(pwm_dev, 2, 500000, 250000);
    rt_pwm_enable(pwm_dev, 1);
    // rt_pwm_enable(pwm_dev, 3);
    rt_pwm_enable(pwm_dev, 2);

    //使能can终端电阻
    rt_pin_mode(GET_PIN(D,5),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,5),PIN_LOW);//使能终端电阻

    rt_pin_mode(GET_PIN(D,4),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,4),PIN_LOW);//使能终端电阻

    //挂载文件系统
    dfs_mount("sd0","/","elm",0,0);

    // USART_thread=rt_thread_create("USART_thread",
    //                             USART_enter,
    //                             RT_NULL,
    //                             1024,
    //                             5,
    //                             5);
    // if(USART_thread != RT_NULL) rt_thread_startup(USART_thread);
    // rt_uint32_t t1=SysTick->VAL;
    // bmi08x_get_sync_data();//42303
    // rt_uint32_t t2=SysTick->VAL;
    // rt_uint32_t l=SysTick->LOAD;
    // rt_kprintf("%d ",t1-t2);

    rt_thread_t t1=rt_thread_create("imu_get_data",imu_data_thrd,
                                    RT_NULL,1024,0,10);
    rt_thread_startup(t1);
    return 0;
}

void imu_data_thrd(void *parameter)
{
    rt_device_t dev = RT_NULL;
    dev = rt_device_find("vcom");
    char test_str[]="vcom success!\n\t";
    while (1)
    {
        // rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
        // rt_device_write(dev,0,test_str,sizeof(test_str));
        // rt_device_close(dev);
        rt_thread_mdelay(1000);

        // rt_sem_t sem=rt_object_find("dsem", RT_Object_Class_Semaphore);
        // rt_sem_take(sem, RT_WAITING_FOREVER);
        // bmi08x_get_sync_data();//42303

        // rt_thread_mdelay(1);
        // bmi08x_get_sync_data();//42303
        // rt_uint32_t t2=SysTick->VAL;
        // rt_kprintf("aaa\n\t");
    }
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
    while(1)
    {
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
}