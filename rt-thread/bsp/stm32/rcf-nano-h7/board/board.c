/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-26     zylx         first version
 */

#include "board.h"

/**
 * @brief   时钟配置
 * @note    HSE=8MHz，PLL1配成480M用于核心计算
 *          PLL2配成400用于外设
 **/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct={0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct={0};
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct={0};

    /* Supply configuration update enable*/
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);//PWR_CR3 bit3

    /** Configure the main internal regulator output voltage*/
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);//VOS0更高

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState=RCC_HSE_ON;
    // RCC_OscInitStruct.HSIState=RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM=1;
    RCC_OscInitStruct.PLL.PLLN=120;
    RCC_OscInitStruct.PLL.PLLP=2;//480MHz cpu
    RCC_OscInitStruct.PLL.PLLQ=16;//60MHZ USB
    // RCC_OscInitStruct.PLL
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Error_Handler();
        //错误处理
    }

    RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|
                                RCC_CLOCKTYPE_D1PCLK1|RCC_CLOCKTYPE_PCLK1|
                                RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_D3PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        // Error_Handler();
    }

    RCC_PeriphCLKInitStruct.PeriphClockSelection=RCC_PERIPHCLK_USART1|
                                                 RCC_PERIPHCLK_USART6|
                                                 RCC_PERIPHCLK_USART2|
                                                 RCC_PERIPHCLK_USART3|
                                                 RCC_PERIPHCLK_UART4|
                                                 RCC_PERIPHCLK_UART8|
                                                 RCC_PERIPHCLK_I2C2|
                                                 RCC_PERIPHCLK_SPI1|
                                                 RCC_PERIPHCLK_FDCAN|
                                                 RCC_PERIPHCLK_SDMMC|
                                                 RCC_PERIPHCLK_USB|
                                                 RCC_PERIPHCLK_ADC|
                                                 RCC_PERIPHCLK_TIM;
    RCC_PeriphCLKInitStruct.PLL2.PLL2M=1;
    RCC_PeriphCLKInitStruct.PLL2.PLL2N=100;
    RCC_PeriphCLKInitStruct.PLL2.PLL2P=5;//80MHz ADC
    RCC_PeriphCLKInitStruct.PLL2.PLL2Q=8;//100MHz FDCAN UART
    RCC_PeriphCLKInitStruct.PLL2.PLL2R=4;//200MHz SDMMC
    RCC_PeriphCLKInitStruct.PLL3.PLL3M=1;
    RCC_PeriphCLKInitStruct.PLL3.PLL3N=100;
    RCC_PeriphCLKInitStruct.PLL3.PLL3P=4;//200MHz SPI
    RCC_PeriphCLKInitStruct.PLL3.PLL3Q=16;//50MHz USB
    RCC_PeriphCLKInitStruct.PLL3.PLL3R=8;//100MHz IIC
    RCC_PeriphCLKInitStruct.SdmmcClockSelection=RCC_SDMMCCLKSOURCE_PLL2;//250
    RCC_PeriphCLKInitStruct.Spi123ClockSelection=RCC_SPI123CLKSOURCE_PLL3;//200
    RCC_PeriphCLKInitStruct.FdcanClockSelection=RCC_FDCANCLKSOURCE_PLL2;//125
    RCC_PeriphCLKInitStruct.Usart234578ClockSelection=RCC_USART234578CLKSOURCE_PLL2;//125
    RCC_PeriphCLKInitStruct.Usart16ClockSelection=RCC_USART16CLKSOURCE_PLL2;//125
    RCC_PeriphCLKInitStruct.I2c123ClockSelection=RCC_I2C123CLKSOURCE_PLL3;//125
    RCC_PeriphCLKInitStruct.UsbClockSelection=RCC_USBCLKSOURCE_PLL;//66otg 60ULPI  48MHz???
    RCC_PeriphCLKInitStruct.AdcClockSelection=RCC_ADCCLKSOURCE_PLL2;//80
    RCC_PeriphCLKInitStruct.TIMPresSelection=RCC_TIMPRES_ACTIVATED;//
    if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
    {
        // Error_Handler();
    }

    //USB电压检测不知道有什么用
}

#ifdef RT_USING_CONSOLE

static UART_HandleTypeDef UartHandle={
    .Instance=USART1,
    .Init.BaudRate = 115200,
    .Init.WordLength = UART_WORDLENGTH_8B,
    .Init.StopBits = UART_STOPBITS_1,
    .Init.Parity = UART_PARITY_NONE,
    .Init.Mode = UART_MODE_TX_RX,
    .Init.HwFlowCtl = UART_HWCONTROL_NONE,
    .Init.OverSampling = UART_OVERSAMPLING_16,
};//用于下面两个函数的参数，估计可以用serial_v2的某些函数代替

void rt_hw_console_output(const char *str)
{
    rt_size_t i = 0, size = 0;
    char a = '\r';

    __HAL_UNLOCK(&UartHandle);

    size = rt_strlen(str);

    for (i = 0; i < size; i++)
    {
        if (*(str + i) == '\n')
        {
            HAL_UART_Transmit(&UartHandle, (uint8_t *)&a, 1, 1);
        }
        HAL_UART_Transmit(&UartHandle, (uint8_t *)(str + i), 1, 1);
    }
}
#endif

#ifdef RT_USING_FINSH
char rt_hw_console_getchar(void)
{
    /* Note: the initial value of ch must < 0 */
    int ch = -1;

    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
    {
        #if defined(STM32H750xx)
        ch = UartHandle.Instance->RDR & 0xff;//H750接收和发送寄存器是分开的
        #else
        ch = UartHandle.Instance->DR & 0xff;
        #endif
    }
    else
    {
        rt_thread_mdelay(10);
    }
    return ch;
}
#endif

// //其他片上外设初始化，假装使用MX生成的函数，然后给rtt隐式调用
// //各外设的结构体先在这里作为全局变量，因为结构体中有可能需要灵活调整的变量，不能销毁
// UART_HandleTypeDef USART1_HandleStructInst;
// UART_HandleTypeDef USART2_HandleStructInst;
// UART_HandleTypeDef USART3_HandleStructInst;
// UART_HandleTypeDef UART4_HandleStructInst;
// UART_HandleTypeDef USART6_HandleStructInst;
// UART_HandleTypeDef UART8_HandleStructInst;

/*******************************************************************************
 * 串口初始化
 * USART1   rtt命令行
 * USART2   GNSS
 * USART3   激光光流
 * UART4    RC
 * USART6   无线数传
 * UART8    有线数传
*******************************************************************************/

/**
 * ！！！！！！串口初始化用4.0的方法，只要写MSP就行了
 */

/*******************************************************************************
 * IIC初始化
 * IIC1 一个其中一个受到IMU控制
 * IIC2 GPS的磁力计的
*******************************************************************************/

// static int i2c_init(void)
// {
//     // MX_I2C1_Init();
//     // MX_I2C2_Init();
// }
// INIT_BOARD_EXPORT(i2c_init);

// /*******************************************************************************
//  * SPI初始化 受到IMU控制
// *******************************************************************************/

// static int spi_init(void)
// {
//     // MX_SPI1_Init();
// }
// INIT_BOARD_EXPORT(spi_init);

// /*******************************************************************************
//  * ADC初始化 用于检测电压电流
// *******************************************************************************/

// static int adc_init(void)
// {
//     // MX_ADC3_Init();
// }
// INIT_BOARD_EXPORT(adc_init);

// /*******************************************************************************
//  * TIM初始化，一个主pwm做四旋翼，从pwm控制舵机（火箭用），一个pwm控制led和恒温（受IMU控制）
// *******************************************************************************/

// static int tim_init(void)
// {
//     // MX_ADC3_Init();
// }
// INIT_BOARD_EXPORT(tim_init);

// /*******************************************************************************
//  * CAN初始化    控制高级电调用
// *******************************************************************************/

// static int can_init(void)
// {
//     // MX_ADC3_Init();
// }
// INIT_BOARD_EXPORT(can_init);

// /*******************************************************************************
//  * SD初始化，FDR，参数写入和调整
// *******************************************************************************/

// static int sd_init(void)
// {
//     // MX_ADC3_Init();
// }
// INIT_BOARD_EXPORT(sd_init);

// /*******************************************************************************
//  * USB初始化 FS-OTG模式，用于从机OTA，从机u盘，主机接键盘，主机接u盘
// *******************************************************************************/

// static int usb_init(void)
// {
//     // MX_ADC3_Init();
// }
// INIT_BOARD_EXPORT(usb_init);