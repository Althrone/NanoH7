#ifndef __MMC5983MA_H__
#define __MMC5983MA_H__

#include <rtthread.h>
#include "sensor.h"
// #include "drv_gpio.h"
// #include "drv_spi.h"

typedef enum 
{
    X_OUT0_REG      = 0x00,
    X_OUT1_REG      = 0x01,
    Y_OUT0_REG      = 0x02,
    Y_OUT1_REG      = 0x03,
    Z_OUT0_REG      = 0x04,
    Z_OUT1_REG      = 0x05,
    XYZ_OUT2_REG    = 0x06,
    T_OUT_REG       = 0x07,
    STAT_REG        = 0x08,
    CTLR_0_REG      = 0x09,
    CTLR_1_REG      = 0x0A,
    CTLR_2_REG      = 0x0B,
    CTLR_3_REG      = 0x0C,
    ID_REG          = 0x2F
}MMC5893MARegEnum;

rt_err_t mmc5893ma_get_id(struct rt_sensor_device *sensor, void *args);

#endif /* __HMC5883L_H__ */