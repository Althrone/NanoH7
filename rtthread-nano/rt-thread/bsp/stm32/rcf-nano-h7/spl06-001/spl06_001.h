#ifndef __SPL06_001_H__
#define __SPL06_001_H__

#include <rtthread.h>
#include "sensor.h"
#include <stdint.h>

typedef enum 
{
    PSR_B2_REG      = 0x00,
    PSR_B1_REG      = 0x01,
    PSR_B0_REG      = 0x02,

    TMP_B2_REG      = 0x03,
    TMP_B1_REG      = 0x04,
    TMP_B0_REG      = 0x05,

    PRS_CFG_REG     = 0x06,
    TMP_CFG_REG     = 0x07,
    MEAS_CFG_REG    = 0x08,
    CFG_REG_REG     = 0x09,
    INT_STS_REG     = 0x0A,
    FIFO_STS_REG    = 0x0B,
    RESET_REG       = 0x0C,
    ID_REG          = 0x0D,

    COEF_REG        = 0x10,
}spl06_001_reg_list_t;

enum spl06_001_intf {
/*! I2C interface */
SPL06_001_I2C_INTF,
/*! SPI interface */
SPL06_001_SPI_INTF
};



#endif /* __SPL06_001_H__ */