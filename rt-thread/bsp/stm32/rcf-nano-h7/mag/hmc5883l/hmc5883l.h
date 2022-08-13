#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include <rtthread.h>
#include "sensor.h"
#include <stdint.h>

//由于HMC5883L的IIC地址只有一个，所以一条iic总线只能有一个
#define HMC5883L_ADDR 0x1E

typedef enum 
{
    CFG_A_REG   = 0x00,
    CFG_B_REG   = 0x01,
    MODE_REG    = 0x02,
    X_MSB_REG   = 0x03,
    X_LSB_REG   = 0x04,
    Z_MSB_REG   = 0x05,
    Z_LSB_REG   = 0x06,
    Y_MSB_REG   = 0x07,
    Y_LSB_REG   = 0x08,
    STAT_REG    = 0x09,
    ID_A_REG    = 0x0A,
    ID_B_REG    = 0x0B,
    ID_C_REG    = 0x0C
}hmc5883l_reg_list_t;

enum hmc5883l_intf {
    HMC5883L_I2C_INTF   /*! I2C interface */
};

struct hmc5883l_cfg 
{
    uint8_t power;      /*! power mode */
    uint8_t range;      /*! range */
    uint8_t bw;         /*! bandwidth */
    uint8_t odr;        /*! output data rate */
};

/* HMC5883L device structure */
struct hmc5883l_dev
{
    uint32_t chip_id;               /*! 实际上是ASCII码 */
    rt_base_t mag_id;               /*! device Id in I2C mode */
    rt_device_t mag_bus;            /*! Device of mag bus*/
    enum hmc5883l_intf intf;        /*! 0 - I2C */
    struct hmc5883l_cfg mag_cfg;    /*! Structure to configure mag sensor */
    const uint8_t *config_file_ptr; /*! Config stream data buffer address will be assigned */
    uint8_t read_write_len;         /*! Max read/write length */
};

struct hmc5883l_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

struct hmc5883l_data
{
    float x;
    float y;
    float z;
};

// int rt_hw_hmc5883l_init(const char *name, struct rt_sensor_config *cfg);
rt_size_t hmc5883l_get_mag(struct hmc5883l_dev *dev, struct hmc5883l_data *buf);
rt_err_t hmc5883l_mag_get_id(struct rt_sensor_device *sensor, void *args);
rt_size_t hmc5883l_get_raw_data(struct rt_sensor_device *sensor,struct hmc5883l_3axes *data);

#endif /* __HMC5883L_H__ */