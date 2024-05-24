#ifndef __MMC5983MA_H__
#define __MMC5983MA_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"
// #include "drv_gpio.h"
// #include "drv_spi.h"

typedef enum 
{
    MMC5983MA_X_OUT_0_ADDR      = 0x00,
    MMC5983MA_X_OUT_1_ADDR      = 0x01,
    MMC5983MA_Y_OUT_0_ADDR      = 0x02,
    MMC5983MA_Y_OUT_1_ADDR      = 0x03,
    MMC5983MA_Z_OUT_0_ADDR      = 0x04,
    MMC5983MA_Z_OUT_1_ADDR      = 0x05,
    MMC5983MA_XYZ_OUT_2_ADDR    = 0x06,
    MMC5983MA_T_OUT_ADDR        = 0x07,
    MMC5983MA_SR_ADDR           = 0x08,
    MMC5983MA_CR0_ADDR          = 0x09,
    MMC5983MA_CR1_ADDR          = 0x0A,
    MMC5983MA_CR2_ADDR          = 0x0B,
    MMC5983MA_CR3_ADDR          = 0x0C,
    MMC5983MA_PID1_ADDR         = 0x2F
}Mmc5893maRegAddrEnum;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t         :2;
        volatile rt_uint8_t Zout    :2;
        volatile rt_uint8_t Yout    :2;
        volatile rt_uint8_t Xout    :2;
    }B;
}Mmc5893maXyzOut2RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Meas_M_Done :1;
        volatile rt_uint8_t Meas_T_Done :1;
        volatile rt_uint8_t             :2;
        volatile rt_uint8_t OTP_Rd_Done :1;
        volatile rt_uint8_t             :3;
    }B;
}Mmc5893maStatRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t TM_M                :1;
        volatile rt_uint8_t TM_T                :1;
        volatile rt_uint8_t INT_meas_done_en    :1;
        volatile rt_uint8_t Set                 :1;
        volatile rt_uint8_t Reset               :1;
        volatile rt_uint8_t Auto_SR_en          :1;
        volatile rt_uint8_t OTP_Read            :1;
        volatile rt_uint8_t                     :1;
    }B;
}Mmc5893maCtrl0RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t BW          :2;
        volatile rt_uint8_t X_inhibit   :1;
        volatile rt_uint8_t YZ_inhibit  :2;
        volatile rt_uint8_t             :2;
        volatile rt_uint8_t SW_RST      :1;
    }B;
}Mmc5893maCtrl1RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Cm_freq     :3;
        volatile rt_uint8_t Cmm_en      :1;
        volatile rt_uint8_t Prd_set     :3;
        volatile rt_uint8_t En_prd_set  :1;
    }B;
}Mmc5893maCtrl2RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t         :1;
        volatile rt_uint8_t St_enp  :1;
        volatile rt_uint8_t St_enm  :1;
        volatile rt_uint8_t         :3;
        volatile rt_uint8_t Spi_3w  :1;
        volatile rt_uint8_t         :1;
    }B;
}Mmc5893maCtrl3RegUnion;

void mmc5893ma_init(rt_sensor_t sensor);
void mmc5893ma_reset(rt_sensor_t sensor);

rt_err_t mmc5893ma_get_id(struct rt_sensor_device *sensor, void *args);
rt_err_t mmc5893ma_set_odr(struct rt_sensor_device *sensor, void *args);

rt_err_t _mmc5893ma_mag_polling_get_data(struct rt_sensor_device *sensor, struct rt_sensor_data *sensor_data, rt_size_t len);

#endif /* __HMC5883L_H__ */