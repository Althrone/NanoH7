// #include "bmi088_sample.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "sensor.h"

#include "drv_sdio.h"

#include "dfs_fs.h"

#include "bmi088.h"
// #include "mmc5983ma.h"

#include "arm_math.h"

void imu_data_thrd(void *parameter);
void rc_rx_thread_entry(void *parameter);
void gps_rx_thread_entry(void *parameter);

#include <drv_config.h>

extern TIM_HandleTypeDef* extern_tim_handle;

extern DMA_HandleTypeDef hdma_tim1_ch1;

extern DMA_HandleTypeDef hdma_tim1_ch2;

extern DMA_HandleTypeDef hdma_tim1_ch3;

extern DMA_HandleTypeDef hdma_tim1_ch4;

void PWM1_CH1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&hdma_tim1_ch1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void PWM1_CH2_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&hdma_tim1_ch2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void PWM1_CH3_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&hdma_tim1_ch3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void PWM1_CH4_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&hdma_tim1_ch4);

    /* leave interrupt */
    rt_interrupt_leave();
}

int main(void)
{

    float32_t a=arm_sin_f32(PI/2);

    // rt_device_t serial = rt_device_find("uart6");

    // rt_device_open(serial, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);

    // UART_HandleTypeDef huart={
    //     .Instance=USART6,
    //     .Init={
    //         .BaudRate=19200,
    //         .WordLength=UART_WORDLENGTH_8B,
    //         .StopBits=UART_STOPBITS_1,
    //         .Parity=UART_PARITY_NONE,
    //         .Mode=UART_MODE_TX_RX,
    //         .HwFlowCtl=UART_HWCONTROL_NONE,
    //         .OverSampling=UART_OVERSAMPLING_16,
    //     }
    // };

    // HAL_LIN_Init(&huart,UART_LINBREAKDETECTLENGTH_11B);
    // HAL_LIN_SendBreak(&huart);

    

    // #include "dma_config.h"
    // DMA_HandleTypeDef pwmdma={0};
    // pwmdma.Instance=PWM1_CH1_DMA_INSTANCE;
    // pwmdma.Init.Request=PWM1_CH1_DMA_REQUEST;
    // pwmdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    // pwmdma.Init.MemInc              = DMA_MINC_ENABLE;
    // pwmdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    // pwmdma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    // pwmdma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    // pwmdma.Init.Mode                = DMA_NORMAL;
    // pwmdma.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    // pwmdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    // HAL_DMA_DeInit(&pwmdma);
    // HAL_DMA_Init(&pwmdma);
    // HAL_NVIC_SetPriority(PWM1_CH1_DMA_IRQ, 0, 0);
    // HAL_NVIC_EnableIRQ(PWM1_CH1_DMA_IRQ);
    // TIM_HandleTypeDef pwmtim={0};
    // pwmtim.Instance=TIM1;
    // pwmtim.Init.Prescaler=
    // pwmtim.Init.CounterMode=TIM_COUNTERMODE_UP;
    // rt_pin_mode(GET_PIN(E,9),PIN_MODE_OUTPUT);
    // rt_pin_write(GET_PIN(E,9),PIN_HIGH);


    // rt_uint8_t aaa[]={0x55,0x3c};
    // rt_device_write(serial, 0, &aaa, sizeof(aaa));


    // volatile rt_uint32_t sysclk=HAL_RCC_GetSysClockFreq();
    // volatile rt_uint32_t coreid=DBGMCU->IDCODE;
    
    rt_pin_mode(GET_PIN(D,9),PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(D,8),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,9),PIN_LOW);
    rt_pin_write(GET_PIN(D,8),PIN_LOW);

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

    // rt_pin_mode(GET_PIN(E,9),PIN_MODE_OUTPUT);
    // rt_pin_write(GET_PIN(E,9),PIN_HIGH);

    // struct rt_device_pwm *pwm_dev0=(struct rt_device_pwm *)rt_device_find("pwm4");
    // rt_pwm_set(pwm_dev0, 2, 500000, 1000);
    // rt_pwm_enable(pwm_dev0, 2);
    // rt_pwm_set(pwm_dev0, 3, 1000000, 1000000);
    // rt_pwm_enable(pwm_dev0, 3);
    // rt_pwm_set(pwm_dev0, 4, 500000, 150000);
    // rt_pwm_enable(pwm_dev0, 4);

    //pwm的dma
    // MX_DMA_Init();

    struct rt_device_pwm *pwm_dev1=(struct rt_device_pwm *)rt_device_find("pwm1");
    // rt_pwm_set(pwm_dev1, 1, 1000000, 500000);
    // rt_pwm_enable(pwm_dev1, 1);
    rt_pwm_set(pwm_dev1, 2, 1000000, 500000);
    // rt_pwm_enable(pwm_dev1, 2);
    // rt_pwm_set(pwm_dev1, 3, 1000000, 150000);
    // rt_pwm_enable(pwm_dev1, 3);
    // rt_pwm_set(pwm_dev1, 4, 1000000, 150000);
    // rt_pwm_enable(pwm_dev1, 4);

    rt_uint32_t data1[16]={30000,10000,20000,30000,30000,0,30000,30000,30000,30000,30000,30000,30000,30000,30000};
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_2,data1,16);

    // //能输出
    // struct rt_device_pwm *pwm_dev2=(struct rt_device_pwm *)rt_device_find("pwm15");
    // rt_pwm_set(pwm_dev2, 1, 500000, 150000);
    // rt_pwm_enable(pwm_dev2, 1);
    // rt_pwm_set(pwm_dev2, 2, 500000, 250000);
    // rt_pwm_enable(pwm_dev2, 2);

    //能输出
    // struct rt_device_pwm *pwm_dev=(struct rt_device_pwm *)rt_device_find("pwm12");
    // rt_pwm_set(pwm_dev, 1, 500000, 150000);
    // rt_pwm_set(pwm_dev, 2, 500000, 250000);
    // rt_pwm_enable(pwm_dev, 1);
    // rt_pwm_enable(pwm_dev, 2);

    //挂载文件系统
    // dfs_mount("sd0","/","elm",0,0);

    // rt_thread_t t=rt_thread_create("can_thread",can_thread_entry,
    //                               RT_NULL,1024,5,5);
    // if(t != RT_NULL) rt_thread_startup(t);

    rt_thread_t t1=rt_thread_create("imu_get_data",imu_data_thrd,
                                    RT_NULL,1024,0,10);
    rt_thread_startup(t1);

    //串口dma测试
    // rt_thread_t t=rt_thread_create("rc_rx_data",rc_rx_thread_entry,
    //                                 RT_NULL,1024,0,5);
    // rt_thread_startup(t);

    //gps接收线程
    // rt_thread_t t=rt_thread_create("gps_rx_data",gps_rx_thread_entry,
    //                                 RT_NULL,1024,0,5);
    // rt_thread_startup(t);

    return 0;
}

rt_sem_t tim_sem;

/**
 * 2300 cnt 8 这个可以很准
 * 用陀螺仪的sr更准，真的傻逼
*/
rt_hwtimerval_t timeout_s={
    .sec=0,
    .usec=2400,//留10us用于获取flag？
};//貌似最高精度是2400 在加也是计数到2.4ms，估计要调tim的参数

/**
 * @brief   定时释放信号量
*/
static rt_err_t tim_cbk(rt_device_t dev, rt_size_t size)
{
    // rt_pin_write(GET_PIN(D,9),PIN_HIGH);
    // rt_pin_write(GET_PIN(D,8),PIN_HIGH);

    rt_sem_release(tim_sem);

    // rt_pin_write(GET_PIN(D,9),PIN_LOW);
    // rt_pin_write(GET_PIN(D,8),PIN_LOW);

    return 0;
}

struct rt_sensor_data g_bmi08x_acce __attribute__((section(".test_data")));
struct rt_sensor_data g_bmi08x_gyro __attribute__((section(".test_data")));
struct rt_sensor_data g_mmc5983ma_mag __attribute__((section(".test_data")));
struct rt_sensor_data g_spl06_baro __attribute__((section(".test_data")));

void imu_data_thrd(void *parameter)
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

    rt_device_t acce_dev=rt_device_find("acce_bmi088");//加速度
    rt_device_open(acce_dev, RT_DEVICE_FLAG_RDONLY);

    rt_device_t gyro_dev=rt_device_find("gyro_bmi088");//陀螺仪
    rt_device_open(gyro_dev, RT_DEVICE_FLAG_RDONLY);

    rt_device_t mag_dev=rt_device_find("mag_mmc5983ma");//磁力计
    rt_device_open(mag_dev, RT_DEVICE_FLAG_RDONLY);

    rt_device_t baro_dev=rt_device_find("baro_spl06");//气压计
    rt_device_open(baro_dev, RT_DEVICE_FLAG_RDONLY);

    // O_WRONLY指明这是一个写入方式的打开模式；
    // O_CREAT指明如果文件不存在则创建文件
    // O_TRUNC指明如果文件存在，则把文件的长度归零，然后打开
    // int fd = open("001.csv", O_WRONLY | O_CREAT | O_TRUNC, 0);
    // close(fd);

    while(1)
    {
        Bmi08xGyroIntStat1RegUnion stat={0};
        // rt_pin_write(GET_PIN(D,9),PIN_HIGH);
        // rt_pin_write(GET_PIN(D,8),PIN_HIGH);
        do
        {
            rt_pin_write(GET_PIN(D,9),PIN_HIGH);
            rt_pin_write(GET_PIN(D,8),PIN_HIGH);
            rt_device_control(gyro_dev,RT_SENSOR_CTRL_SELF_TEST,&stat);
            rt_pin_write(GET_PIN(D,9),PIN_LOW);
            rt_pin_write(GET_PIN(D,8),PIN_LOW);
        } while (stat.B.gyro_drdy!=1);

        // rt_pin_write(GET_PIN(D,9),PIN_LOW);
        // rt_pin_write(GET_PIN(D,8),PIN_LOW);

        rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));//2us

        //先读陀螺仪，加速度有35us延迟
        rt_device_read(gyro_dev, 0, &g_bmi08x_gyro, 1);//21
        
        //考虑在这里读一下磁力计？
        // rt_hw_us_delay(35-23);
        rt_pin_write(GET_PIN(D,9),PIN_HIGH);
        rt_pin_write(GET_PIN(D,8),PIN_HIGH);
        rt_device_read(mag_dev, 0, &g_mmc5983ma_mag, 1);
        rt_pin_write(GET_PIN(D,9),PIN_LOW);
        rt_pin_write(GET_PIN(D,8),PIN_LOW);

        // rt_pin_write(GET_PIN(D,9),PIN_HIGH);
        // rt_pin_write(GET_PIN(D,8),PIN_HIGH);
        rt_device_read(acce_dev, 0, &g_bmi08x_acce, 1);
        // rt_pin_write(GET_PIN(D,9),PIN_LOW);
        // rt_pin_write(GET_PIN(D,8),PIN_LOW);

        static rt_uint8_t spl06_250ms_cnt=0;
        
        if(spl06_250ms_cnt==100)
        {
            //读取spl06的温度和气压
            rt_device_read(baro_dev, 0, &g_spl06_baro, 1);

            spl06_250ms_cnt=0;

            // write(fd, g_spl06_baro.data.baro, sizeof(g_spl06_baro.data.baro));
            // write(fd, "\r\n", sizeof("\r\n"));
        }

        spl06_250ms_cnt++;

        //解算

        rt_sem_take(tim_sem, RT_WAITING_FOREVER);
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