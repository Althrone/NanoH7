/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-01-08     bernard      first version.
 * 2014-07-12     bernard      Add workqueue implementation.
 */

#ifndef __RT_DEVICE_H__
#define __RT_DEVICE_H__

#include <rtthread.h>

#include "ipc/ringbuffer.h"
#include "ipc/completion.h"
#include "ipc/dataqueue.h"
#include "ipc/workqueue.h"
#include "ipc/waitqueue.h"
#include "ipc/pipe.h"
#include "ipc/poll.h"
#include "ipc/ringblk_buf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RT_DEVICE(device)            ((rt_device_t)device)

#ifdef RT_USING_RTC
#include "drivers/rtc.h"
#ifdef RT_USING_ALARM
#include "drivers/alarm.h"
#endif
#endif /* RT_USING_RTC */

#ifdef RT_USING_SPI
#include "drivers/spi.h"
#endif /* RT_USING_SPI */

#ifdef RT_USING_MTD_NOR
#include "drivers/mtd_nor.h"
#endif /* RT_USING_MTD_NOR */

#ifdef RT_USING_MTD_NAND
#include "drivers/mtd_nand.h"
#endif /* RT_USING_MTD_NAND */

#ifdef RT_USING_USB_DEVICE
#include "drivers/usb_device.h"
#endif /* RT_USING_USB_DEVICE */

#ifdef RT_USING_USB_HOST
#include "drivers/usb_host.h"
#endif /* RT_USING_USB_HOST */

#ifdef RT_USING_SERIAL
#ifdef RT_USING_SERIAL_V2
#include "drivers/serial_v2.h"
#else
#include "drivers/serial.h"
#endif
#endif /* RT_USING_SERIAL */

#ifdef RT_USING_I2C
#include "drivers/i2c.h"
#include "drivers/i2c_dev.h"

#ifdef RT_USING_I2C_BITOPS
#include "drivers/i2c-bit-ops.h"
#endif /* RT_USING_I2C_BITOPS */
#endif /* RT_USING_I2C */

#ifdef RT_USING_PHY
#include "drivers/phy.h"
#include "drivers/phy_mdio.h"
#endif /* RT_USING_PHY */

#ifdef RT_USING_SDIO
#include "drivers/mmcsd_core.h"
#include "drivers/sd.h"
#include "drivers/sdio.h"
#endif /* RT_USING_SDIO */


#ifdef RT_USING_WDT
#include "drivers/watchdog.h"
#endif /* RT_USING_WDT */

#ifdef RT_USING_PIN
#include "drivers/pin.h"
#endif /* RT_USING_PIN */

#ifdef RT_USING_SENSOR
#include "drivers/sensor.h"
#endif /* RT_USING_SENSOR */

#ifdef RT_USING_CAN
#include "drivers/can.h"
#endif /* RT_USING_CAN */

#ifdef RT_USING_CANX
#include "drivers/canx.h"
#endif /* RT_USING_CANX */

#ifdef RT_USING_LIN
#include "drivers/lin.h"
#endif /* RT_USING_LIN */

#ifdef RT_USING_HWTIMER
#include "drivers/hwtimer.h"
#endif /* RT_USING_HWTIMER */

#ifdef RT_USING_AUDIO
#include "drivers/audio.h"
#endif /* RT_USING_AUDIO */

#ifdef RT_USING_CPUTIME
#include "drivers/cputime.h"
#endif /* RT_USING_CPUTIME */

#ifdef RT_USING_ADC
#include "drivers/adc.h"
#endif /* RT_USING_ADC */

#ifdef RT_USING_DAC
#include "drivers/dac.h"
#endif /* RT_USING_DAC */

#ifdef RT_USING_PWM
#include "drivers/rt_drv_pwm.h"
#endif /* RT_USING_PWM */

#ifdef RT_USING_PM
#include "drivers/pm.h"
#endif /* RT_USING_PM */

#ifdef RT_USING_WIFI
#include "drivers/wlan.h"
#endif /* RT_USING_WIFI */

#ifdef MTD_USING_NOR
#include "drivers/mtdnor.h"
#endif /* MTD_USING_NOR */

#ifdef MTD_USING_NAND
#include "drivers/mtdnand.h"
#endif /* MTD_USING_NAND */

#ifdef RT_USING_HWCRYPTO
#include "drivers/crypto.h"
#endif /* RT_USING_HWCRYPTO */

#ifdef RT_USING_PULSE_ENCODER
#include "drivers/pulse_encoder.h"
#endif /* RT_USING_PULSE_ENCODER */

#ifdef RT_USING_INPUT_CAPTURE
#include "drivers/rt_inputcapture.h"
#endif /* RT_USING_INPUT_CAPTURE */

#ifdef __cplusplus
}
#endif

#endif /* __RT_DEVICE_H__ */
