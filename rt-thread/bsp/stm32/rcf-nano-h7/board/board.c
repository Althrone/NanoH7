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

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;//大时钟要加在这里，不然就会出现usb的问题（HSI48）
    RCC_OscInitStruct.HSEState=RCC_HSE_ON;
    // RCC_OscInitStruct.HSIState=RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State=RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM=1;
    RCC_OscInitStruct.PLL.PLLN=150;
    RCC_OscInitStruct.PLL.PLLP=2;//480MHz cpu
    RCC_OscInitStruct.PLL.PLLQ=16;//60MHZ USB
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
    RCC_PeriphCLKInitStruct.PLL3.PLL3N=120;//960
    RCC_PeriphCLKInitStruct.PLL3.PLL3P=10;//96MHz SPI
    RCC_PeriphCLKInitStruct.PLL3.PLL3Q=16;//60MHz USB//usb不用这个
    RCC_PeriphCLKInitStruct.PLL3.PLL3R=8;//120MHz IIC
    RCC_PeriphCLKInitStruct.SdmmcClockSelection=RCC_SDMMCCLKSOURCE_PLL2;//250
    RCC_PeriphCLKInitStruct.Spi123ClockSelection=RCC_SPI123CLKSOURCE_PLL3;//32
    RCC_PeriphCLKInitStruct.FdcanClockSelection=RCC_FDCANCLKSOURCE_PLL2;//最高125
    RCC_PeriphCLKInitStruct.Usart234578ClockSelection=RCC_USART234578CLKSOURCE_PLL2;//最高125
    RCC_PeriphCLKInitStruct.Usart16ClockSelection=RCC_USART16CLKSOURCE_PLL2;//125
    RCC_PeriphCLKInitStruct.I2c123ClockSelection=RCC_I2C123CLKSOURCE_PLL3;//125
    RCC_PeriphCLKInitStruct.UsbClockSelection=RCC_USBCLKSOURCE_HSI48;//66otg 60ULPI  48MHz???
    RCC_PeriphCLKInitStruct.AdcClockSelection=RCC_ADCCLKSOURCE_PLL2;//80
    RCC_PeriphCLKInitStruct.TIMPresSelection=RCC_TIMPRES_ACTIVATED;//
    if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
    {
        // Error_Handler();
    }

    //USB电压检测不知道有什么用
    HAL_PWREx_EnableUSBVoltageDetector();
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

#if (0)

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

#else

#include "SEGGER_RTT.h"
void rt_hw_console_output(const char *str)
{
    rt_size_t i = 0, size = 0;

    size = rt_strlen(str);
    for (i = 0; i < size; i++)
    {
        if (*(str + i) == '\n')
        {
           break;
        }
    }
    SEGGER_RTT_printf(0,"%s",str);
}
#endif

#endif

#ifdef RT_USING_FINSH

#if (0)

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
#else
char rt_hw_console_getchar(void)
{
    int ch = -1;
    char tempbuffer[2];
    uint8_t NumBytes=0;

    NumBytes = SEGGER_RTT_Read(0, &tempbuffer[0], 1);
    if(NumBytes==1)
    {
        ch=tempbuffer[0];
    }
    return ch;
}
#endif
#endif