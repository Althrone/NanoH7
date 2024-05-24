// #include "bmi088_sample.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "sensor.h"

#include "drv_sdio.h"

#include "dfs_fs.h"

#include "bmi088.h"
#include "mmc5983ma.h"

void imu_data_thrd(void *parameter);
void rc_rx_thread_entry(void *parameter);
void gps_rx_thread_entry(void *parameter);
void ins_rx_thread_entry(void *parameter);

// #define CAN_
// #define CAN_Pin GET_PIN(B, 14)

int main(void)
{
    // volatile rt_uint32_t sysclk=HAL_RCC_GetSysClockFreq();
    // volatile rt_uint32_t coreid=DBGMCU->IDCODE;
    
    rt_pin_mode(GET_PIN(D,9),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,9),PIN_LOW);

    // char test_str[]="vcom success!\n\t";

    // //usb测试
    // rt_device_t dev = RT_NULL;
    // dev = rt_device_find("vcom");
    // rt_device_init(dev);
    // rt_device_open(dev,RT_DEVICE_FLAG_RDWR);

    // if(dev)
    // {
    //     while(1)
    //     {
    //         // rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
    //         rt_device_write(dev,0,test_str,sizeof(test_str));
    //         rt_thread_mdelay(1000);
    //         // rt_device_close(dev);
    //     }
    // }

    struct rt_device_pwm *pwm_dev=(struct rt_device_pwm *)rt_device_find("pwm12");
    rt_pwm_set(pwm_dev, 1, 500000, 150000);
    // rt_pwm_set(pwm_dev, 3, 500000, 490000);
    rt_pwm_set(pwm_dev, 2, 500000, 250000);
    rt_pwm_enable(pwm_dev, 1);
    // rt_pwm_enable(pwm_dev, 3);
    rt_pwm_enable(pwm_dev, 2);

    //挂载文件系统
    // dfs_mount("sd0","/","elm",0,0);

    // rt_thread_t t=rt_thread_create("can_thread",can_thread_entry,
    //                               RT_NULL,1024,5,5);
    // if(t != RT_NULL) rt_thread_startup(t);

    // rt_thread_t t1=rt_thread_create("imu_get_data",imu_data_thrd,
    //                                 RT_NULL,1024,0,10);
    // rt_thread_startup(t1);

    //串口dma测试
    // rt_thread_t t=rt_thread_create("rc_rx_data",rc_rx_thread_entry,
    //                                 RT_NULL,1024,0,5);
    // rt_thread_startup(t);

    //gps接收线程
    // rt_thread_t t=rt_thread_create("gps_rx_data",gps_rx_thread_entry,
    //                                 RT_NULL,1024,0,5);
    // rt_thread_startup(t);

    // rt_thread_t t=rt_thread_create("ins_rx_data",ins_rx_thread_entry,
    //                                 RT_NULL,1024,0,10);
    // rt_thread_startup(t);

    //传感器获取
    struct rt_sensor_data data;
    struct rt_sensor_data temp_data;
    rt_device_t dev=rt_device_find("mag_mmc5983ma");
    rt_device_t temp_dev=rt_device_find("temp_mmc5983ma");
    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
    rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY);
    while (1)
    {
        rt_device_read(dev, 0, &data, 1);//pos不使用
        rt_device_read(temp_dev, 0, &temp_data, 1);//pos不使用
        rt_thread_mdelay(2);
    }
    
    // while(1)
    // {
    //     mmc5893ma_polling_get_mag();
    //     rt_thread_mdelay(50);
    // }
    return 0;
}

rt_sem_t tim_sem;

rt_hwtimerval_t timeout_s={
    .sec=0,
    .usec=2480,//留10us用于获取flag？
};//2480

/**
 * @brief   定时释放信号量
*/
static rt_err_t tim_cbk(rt_device_t dev, rt_size_t size)
{
    rt_pin_write(GET_PIN(D,9),PIN_HIGH);

    //等到flag置位的一瞬间才开启定时器，这样准一点？？
    while(bmi08x_wait_sync_data()!=RT_EOK);

    //开启定时器
    rt_device_write(dev, 0, &timeout_s, sizeof(timeout_s));
    rt_sem_release(tim_sem);

    rt_pin_write(GET_PIN(D,9),PIN_LOW);


    return 0;
}

/**
 * @brief   组合导航系统/板载传感器接收线程
 * @note    接受板载的IMU MGA 气压计
 */
void ins_rx_thread_entry(void *parameter)
{
    //定时器
    rt_uint32_t freq = 10000;
    rt_device_t tim_dev=rt_device_find("timer2");
    rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(tim_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    rt_hwtimer_mode_t mode = HWTIMER_MODE_ONESHOT;//HWTIMER_MODE_ONESHOT HWTIMER_MODE_PERIOD
    rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
    //创建信号量
    tim_sem=rt_sem_create("tim_sem", 0, RT_IPC_FLAG_PRIO);
    rt_device_set_rx_indicate(tim_dev, tim_cbk);

    // rt_pin_write(GET_PIN(D,9),PIN_HIGH);

    //等到flag置位的一瞬间才开启定时器，这样准一点？？
    while(bmi08x_wait_sync_data()!=RT_EOK);
    //开启定时器
    rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));

    // rt_pin_write(GET_PIN(D,9),PIN_LOW);

    //测速度
    // rt_pin_mode(GET_PIN(D,9),PIN_MODE_OUTPUT);
    // rt_pin_write(GET_PIN(D,9),PIN_LOW);
    rt_sem_take(tim_sem, RT_WAITING_FOREVER);

    while (1)
    {
        rt_pin_write(GET_PIN(D,9),PIN_HIGH);

        // rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));
        // bmi08x_get_sync_data();
        // mmc5893ma_polling_get_mag();
        // spl06_polling_get_baro();

        rt_pin_write(GET_PIN(D,9),PIN_LOW);

        rt_sem_take(tim_sem, RT_WAITING_FOREVER);

        //mag
        //baro

        // rt_pin_write(GET_PIN(D,9),PIN_HIGH);

        // //等到flag置位的一瞬间才开启定时器，这样准一点？？
        // while(bmi08x_wait_sync_data()!=RT_EOK);

        // rt_pin_write(GET_PIN(D,9),PIN_LOW);
    }
_exit:
    return;
}

void imu_data_thrd(void *parameter)
{
    rt_device_t dev = RT_NULL;
    dev = rt_device_find("vcom");
    char test_str[]="vcom success!\n\t";
    while (1)
    {
        rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
        rt_device_write(dev,0,test_str,sizeof(test_str));
        rt_device_close(dev);
        rt_thread_mdelay(1000);
    }
}

void vcom_test(void)
{
    rt_device_t dev = RT_NULL;
    dev = rt_device_find("vcom");
    char test_str[]="vcom success!\n\t";
    rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
    rt_device_write(dev,0,test_str,sizeof(test_str));
    rt_device_close(dev);
}MSH_CMD_EXPORT(vcom_test, vcom test);

void can_thread_entry(void *parameter)
{
    rt_pin_mode(GET_PIN(D,5),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,5),PIN_LOW);//使能终端电阻

    rt_pin_mode(GET_PIN(D,4),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,4),PIN_LOW);//can收发器使能

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

        rt_thread_mdelay(500);	        

    }
}

/* 消息队列控制块 */
static struct rt_messagequeue rc_rx_mq;
static char rc_rx_buffer[BSP_UART4_RX_BUFSIZE + 1];

/**
 * @brief   遥控器接收回调函数
 */
static rt_err_t rc_rx_cbk(rt_device_t dev, rt_size_t size)
{
    rt_err_t result = rt_mq_send(&rc_rx_mq, &size, sizeof(size));
    
    return result;
}

/**
 * @brief   遥控器接收线程
 */
void rc_rx_thread_entry(void *parameter)
{
    rt_device_t rc_serial = rt_device_find("uart4");
    if (!rc_serial)
    {
        return ;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
    config.baud_rate = 420000;        // 修改波特率为 9600
    config.data_bits = DATA_BITS_8;           // 数据位 8
    config.stop_bits = STOP_BITS_1;           // 停止位 1
    config.rx_bufsz     = 128;                // 修改缓冲区 rx buff size 为 128
    config.parity    = PARITY_NONE;           // 无奇偶校验位

    rt_device_control(rc_serial, RT_DEVICE_CTRL_CONFIG, &config);

    static char msg_pool[256];
    /* 初始化消息队列 */
    rt_mq_init(&rc_rx_mq, "rc_rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(rt_size_t),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(rc_serial, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(rc_serial, rc_rx_cbk);

    while (1)
    {
        rt_size_t size;
        /* 从消息队列中读取消息 */
        rt_err_t result = rt_mq_recv(&rc_rx_mq, &size, sizeof(size), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据 */
            rt_size_t rx_length = rt_device_read(rc_serial, 0, rc_rx_buffer, size);
            rc_rx_buffer[rx_length] = '\0';
        }
    }
    
}

static struct rt_messagequeue gps_rx_mq;

/**
 * @brief   遥控器接收回调函数
 */
static rt_err_t gps_rx_cbk(rt_device_t dev, rt_size_t size)
{
    rt_err_t result = rt_mq_send(&gps_rx_mq, &size, sizeof(size));
    
    return result;
}

/**
 * @brief   gps接收线程
 */
void gps_rx_thread_entry(void *parameter)
{
    rt_device_t gps_serial = rt_device_find("uart2");
    if (!gps_serial)
    {
        return ;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
    config.baud_rate = BAUD_RATE_460800;//从4800开始尝试
    config.rx_bufsz     = 128;                // 修改缓冲区 rx buff size 为 128
    rt_device_control(gps_serial, RT_DEVICE_CTRL_CONFIG, &config);

    static char msg_pool[256];
    /* 初始化消息队列 */
    rt_mq_init(&gps_rx_mq, "gps_rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(rt_size_t),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(gps_serial, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(gps_serial, gps_rx_cbk);

    while (1)
    {
        rt_size_t size;
        /* 从消息队列中读取消息 */
        rt_err_t result = rt_mq_recv(&gps_rx_mq, &size, sizeof(size), 400);
        if (result == RT_EOK)
        {
            /* 从串口读取数据 */
            rt_size_t rx_length = rt_device_read(gps_serial, 0, rc_rx_buffer, size);
            // rc_rx_buffer[rx_length] = '\0';
        }
        else if(result == -RT_ETIMEOUT)
        {
            //切换波特率
            struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
            config.baud_rate = BAUD_RATE_4800;        // 修改波特率为 9600
        }
    }
}