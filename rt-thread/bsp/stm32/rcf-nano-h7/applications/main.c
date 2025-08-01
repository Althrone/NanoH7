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

// #include "attitude.h"
#include "MahonyAHRS.h"

#include "mcan_config.h"

void imu_data_thrd(void *parameter);
void rc_rx_thread_entry(void *parameter);
void gps_rx_thread_entry(void *parameter);
void fly_log_thread_entry(void *parameter);
void can_thread_entry(void *parameter);

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

void lin_test(void)
{
    rt_device_t serial = rt_device_find("uart6");

    // rt_device_open(serial, RT_DEVICE_FLAG_ | RT_DEVICE_FLAG_TX_BLOCKING);

    UART_HandleTypeDef huart={
        .Instance=USART6,
        .Init={
            .BaudRate=19200,
            .WordLength=UART_WORDLENGTH_8B,
            .StopBits=UART_STOPBITS_1,
            .Parity=UART_PARITY_NONE,
            .Mode=UART_MODE_TX_RX,
            .HwFlowCtl=UART_HWCONTROL_NONE,
            .OverSampling=UART_OVERSAMPLING_16,
        }
    };

    HAL_LIN_Init(&huart,UART_LINBREAKDETECTLENGTH_11B);
    HAL_LIN_SendBreak(&huart);

    rt_uint8_t aaa[]={0x55,0x3c};
    rt_device_write(serial, 0, &aaa, sizeof(aaa));
}

//1 625ns 0 312.5
//250 3/4 3/8
CRC_HandleTypeDef hcrc={
    .Instance=CRC,
    .Init.DefaultPolynomialUse=DEFAULT_POLYNOMIAL_DISABLE,
    .Init.DefaultInitValueUse=DEFAULT_INIT_VALUE_DISABLE,
    .Init.GeneratingPolynomial=0xD5,
    .Init.CRCLength=CRC_POLYLENGTH_8B,
    .Init.InitValue=0x00,
    .Init.InputDataInversionMode=CRC_INPUTDATA_INVERSION_NONE,
    .Init.OutputDataInversionMode=CRC_OUTPUTDATA_INVERSION_DISABLE,
    .InputDataFormat=CRC_INPUTDATA_FORMAT_BYTES,
};
rt_uint32_t data1[16]={188,94,250,188,94,94,94,94,188,188,188,188,188,188,188};
int main(void)
{
    //dsplib测试
    float32_t a=arm_sin_f32(PI/2);

    HAL_CRC_Init(&hcrc);

    rt_pin_mode(GET_PIN(D,9),PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(D,8),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,9),PIN_LOW);
    rt_pin_write(GET_PIN(D,8),PIN_LOW);

    // rt_pin_mode(GET_PIN(E,9),PIN_MODE_OUTPUT);
    // rt_pin_write(GET_PIN(E,9),PIN_HIGH);

    struct rt_device_pwm *pwm_dev1=(struct rt_device_pwm *)rt_device_find("pwm1");
    rt_pwm_set(pwm_dev1, 1, 834, 0);
    rt_pwm_set(pwm_dev1, 2, 834, 0);//单位ns实际上是833.333333
    rt_pwm_set(pwm_dev1, 3, 834, 0);
    rt_pwm_set(pwm_dev1, 4, 834, 0);

    // 3/4为1 3/8为0
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_1,data1,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_2,data1,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_3,data1,16);
    HAL_TIM_PWM_Start_DMA(extern_tim_handle,TIM_CHANNEL_4,data1,16);

    rt_thread_t t=rt_thread_create("can_thread",can_thread_entry,
                                  RT_NULL,1024,5,5);
    if(t != RT_NULL) rt_thread_startup(t);

    rt_thread_t t1=rt_thread_create("imu_get_data",imu_data_thrd,
                                    RT_NULL,1024,0,10);
    rt_thread_startup(t1);

    //RC接收机线程
    rt_thread_t rc_t=rt_thread_create("rc_rx_data",rc_rx_thread_entry,
                                    RT_NULL,1024,1,5);
    rt_thread_startup(rc_t);

    //gps接收线程
    rt_thread_t gps_t=rt_thread_create("gps_rx_data",gps_rx_thread_entry,
                                    RT_NULL,1024,1,5);
    rt_thread_startup(gps_t);

    // 挂载文件系统
    int dfsret=dfs_mount("sd0","/","elm",0,0);
    //log线程
    // if(dfsret!=-1)
    // {
    //     rt_thread_t log_t=rt_thread_create("fly_log",fly_log_thread_entry,
    //                                        RT_NULL,1024*2,3,10);
    //     rt_thread_startup(log_t);
    // }
    

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
            
        }

        spl06_250ms_cnt++;

        // rt_kprintf("ax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\t\n\r",
        //            -g_bmi08x_gyro.data.acce.x,
        //            g_bmi08x_gyro.data.acce.y,
        //            -g_bmi08x_gyro.data.acce.z,
        //            g_bmi08x_gyro.data.gyro.x,
        //            -g_bmi08x_gyro.data.gyro.y,
        //            g_bmi08x_gyro.data.gyro.z);//输出读取到的长度

        //从传感器坐标系转至机体坐标系
        //机体坐标系 x↑ y→ z⊗
        //bmi088方向 x↓y→z⊙
        //但是手册说了If the sensor is at rest without any rotation and the force of gravity is acting contrary to the indicated directions, the output of the corresponding acceleration channel will be positive
        //所以应该是y轴取负号
        //我看了一下6050那一套代码，也是同方向取负号
        //https://blog.csdn.net/weixin_42918498/article/details/121052607

        MahonyAHRSupdateIMU(g_bmi08x_gyro.data.gyro.x*0.01745f/1000,
                            g_bmi08x_gyro.data.gyro.y*0.01745f/1000,
                            g_bmi08x_gyro.data.gyro.z*0.01745f/1000,
                            g_bmi08x_acce.data.acce.x,
                            g_bmi08x_acce.data.acce.y,
                            g_bmi08x_acce.data.acce.z);

        rt_sem_take(tim_sem, RT_WAITING_FOREVER);
    }
}

void vcom_test(void)
{
    rt_device_t dev = RT_NULL;
    dev = rt_device_find("vcom");
    char test_str[]="vcom success!";
    rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
    rt_device_write(dev,0,test_str,sizeof(test_str));
    // rt_thread_mdelay(10);
    rt_device_close(dev);//关闭之前要判断是否发完
}MSH_CMD_EXPORT(vcom_test, vcom test);

FDCAN_HandleTypeDef hfdcan={
    .Instance=FDCAN1,
    .Init.FrameFormat=FDCAN_FRAME_FD_BRS,//FDCAN_FRAME_FD_BRS,
    .Init.Mode=FDCAN_MODE_NORMAL,
    .Init.AutoRetransmission=DISABLE,
    .Init.TransmitPause=DISABLE,
    .Init.ProtocolException=DISABLE,
    //波特率1m 5m
    .Init.NominalPrescaler=5,
    .Init.NominalSyncJumpWidth=8,
    .Init.NominalTimeSeg1=15,
    .Init.NominalTimeSeg2=4,
    .Init.DataPrescaler=1,
    .Init.DataSyncJumpWidth=8,
    .Init.DataTimeSeg1=15,
    .Init.DataTimeSeg2=4,
    //msg ram
    .Init.MessageRAMOffset=0,
    .Init.StdFiltersNbr=1,
    .Init.ExtFiltersNbr=1,
    .Init.RxFifo0ElmtsNbr=1,
    .Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_64,//FDCAN_DATA_BYTES_64
    .Init.RxFifo1ElmtsNbr=1,
    .Init.RxFifo1ElmtSize=FDCAN_DATA_BYTES_64,
    .Init.RxBuffersNbr=0,
    .Init.TxEventsNbr=0,
    .Init.TxBuffersNbr=1,
    .Init.TxFifoQueueElmtsNbr=0,
    .Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION,
    .Init.TxElmtSize=FDCAN_DATA_BYTES_64,
};

FDCAN_FilterTypeDef sFilterConfig={
    .IdType=FDCAN_STANDARD_ID,
    .FilterIndex=0,
    .FilterType=FDCAN_FILTER_RANGE,
    .FilterConfig=FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1=0,
    .FilterID2=0x7ff,
    .IsCalibrationMsg=0,
};

FDCAN_TxHeaderTypeDef TxHeader={
    .Identifier=0x5aa,
    .IdType=FDCAN_STANDARD_ID,
    .TxFrameType=FDCAN_DATA_FRAME,
    .DataLength=FDCAN_DLC_BYTES_64,//FDCAN_DLC_BYTES_64
    .ErrorStateIndicator=FDCAN_ESI_ACTIVE,
    .BitRateSwitch=FDCAN_BRS_ON,//FDCAN_BRS_ON
    .FDFormat=FDCAN_FD_CAN,
    .TxEventFifoControl=FDCAN_NO_TX_EVENTS,
    .MessageMarker=0xaa,
};

void can_thread_entry(void *parameter)
{
    rt_pin_mode(GET_PIN(D,5),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,5),PIN_LOW);//使能终端电阻

    rt_pin_mode(GET_PIN(D,4),PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(D,4),PIN_LOW);//can收发器使能

    // static rt_device_t can_dev;
    // volatile rt_err_t ret;
    // can_dev=rt_device_find("fdcan1");
    // /* 以中断接收及发送模式打开 CAN 设备 */
    // ret=rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    // /* 设置 CAN 的工作模式为正常工作模式 */
    // ret = rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    // // RT_CAN_MODE_LOOPBACK
    // static struct rt_can_status status;    /* 获取到的 CAN 总线状态 */
    // ret=rt_device_control(can_dev, RT_CAN_CMD_GET_STATUS, &status);

    // struct rt_can_msg msg = {0};

    // msg.id = 0x78;              /* ID 为 0x78 */
    // msg.ide = RT_CAN_STDID;     /* 标准格式 */
    // msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    // msg.len = 8;                /* 数据长度为 8 */
    // /* 待发送的 8 字节数据 */
    // msg.data[0] = 0x00;
    // msg.data[1] = 0x11;
    // msg.data[2] = 0x22;
    // msg.data[3] = 0x33;
    // msg.data[4] = 0x44;
    // msg.data[5] = 0x55;
    // msg.data[6] = 0x66;
    // msg.data[7] = 0x77;

    // /* 发送一帧 CAN 数据 */
    // while(1)
    // {
    //     rt_size_t size=rt_device_write(can_dev, 0, &msg, sizeof(msg));
    //     if (size == 0)
    //     {
    //         rt_kprintf("can dev write data failed!\n");
    //     }
    //     else
    //     {
    //         rt_kprintf("can dev write data success!\n");
    //     }

    //     rt_thread_mdelay(500);	        

    // }

    //使用hal测试canfd功能

    HAL_FDCAN_Init(&hfdcan);
    HAL_FDCAN_ConfigFilter(&hfdcan,&sFilterConfig);
    HAL_FDCAN_Start(&hfdcan);

    while(1)
    {
        uint8_t data[64]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
        0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
        HAL_FDCAN_AddMessageToTxBuffer(&hfdcan,&TxHeader,data,FDCAN_TX_BUFFER0);
        HAL_FDCAN_EnableTxBufferRequest(&hfdcan,FDCAN_TX_BUFFER0);
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

    rt_device_control(rc_serial, RT_DEVICE_CTRL_CONFIG, &config);

    static char msg_pool[256];
    /* 初始化消息队列 */
    rt_mq_init(&rc_rx_mq, "rc_rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(rt_size_t),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(rc_serial, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_INT_TX);
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

            // rt_kprintf("rc:%d\n\r",rx_length);//输出读取到的长度
            // rt_kprintf("rc:");//在终端打印接收到的十六进制值
            // for (size_t i = 0; i < rx_length; i++)
            // {
            //     rt_kprintf("%02X",rc_rx_buffer[i]);//在终端打印接收到的十六进制值
            // }
            // rt_kprintf("\n\r");//输出读取到的长度

            //解析数据
            crsf_decode(rc_rx_buffer,rx_length);
        }
    }
    
}

static struct rt_messagequeue gps_rx_mq;
static rt_uint8_t gps_rx_buffer[BSP_UART2_RX_BUFSIZE + 1];

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
    // config.bufsz     = 128;                // 修改缓冲区 rx buff size 为 128
    rt_device_control(gps_serial, RT_DEVICE_CTRL_CONFIG, &config);

    static char msg_pool[256];
    /* 初始化消息队列 */
    rt_mq_init(&gps_rx_mq, "gps_rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(rt_size_t),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(gps_serial, RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_INT_TX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(gps_serial, gps_rx_cbk);

    while (1)
    {
        rt_size_t size;
        /* 从消息队列中读取消息 */
        rt_err_t result = rt_mq_recv(&gps_rx_mq, &size, sizeof(size), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据 */
            rt_size_t rx_length = rt_device_read(gps_serial, 0, gps_rx_buffer, size);
            
            rt_kprintf("gps:%d\n\r",rx_length);//输出读取到的长度
            // rt_kprintf("gps:");//在终端打印接收到的十六进制值
            // for (size_t i = 0; i < rx_length; i++)
            // {
            //     rt_kprintf("%02X",gps_rx_buffer[i]);//在终端打印接收到的十六进制值
            // }
            // rt_kprintf("\n\r");//在终端打印接收到的十六进制值

            //开始解析吧
            ubx_decode(gps_rx_buffer,rx_length);
        }
        else if(result == -RT_ETIMEOUT)
        {
            //切换波特率
            struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
            config.baud_rate = BAUD_RATE_4800;        // 修改波特率为 9600
        }
    }
}

#include <dfs_posix.h>

int fd;
void fly_log_thread_entry(void *parameter)
{
    //查找文件
    char file_name[]={'0','0','0','.','c','s','v','\0'};
    for (size_t i = 0; i < 1000; i++)
    {
        char ge=i%10+'0';
        char shi=i/10%10+'0';
        char bai=i/10/10%10+'0';
        file_name[0]=bai;
        file_name[1]=shi;
        file_name[2]=ge;
        fd=open(file_name,O_RDWR | O_APPEND | O_CREAT | O_EXCL,0);
        if(fd==-1)//打开失败，文件已经存在
        {
            continue;
        }
        else
        {
            char* titel="acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,baro";
            write(fd, titel, strlen(titel));
            write(fd, "\r\n", strlen("\r\n"));

            close(fd);
            break;
        }
    }
    
    while(1)
    {
        fd = open(file_name, O_RDWR | O_APPEND | O_CREAT, 0);

        char a[10]={0};
        __itoa(g_bmi08x_acce.data.acce.x,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_bmi08x_acce.data.acce.y,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_bmi08x_acce.data.acce.z,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_bmi08x_gyro.data.gyro.x,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_bmi08x_gyro.data.gyro.y,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_bmi08x_gyro.data.gyro.z,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));
        __itoa(g_spl06_baro.data.baro,a,10);
        write(fd, a, strlen(a));
        write(fd, ",", strlen(","));

        // char aaa[]="appppp\r\n";
        // write(fd, aaa, strlen(aaa));
        write(fd, "\r\n", strlen("\r\n"));
        close(fd);

        rt_thread_mdelay(500);
    }
}
