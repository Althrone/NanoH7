/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file       bmi088.h
 * @date       24 Aug 2018
 * @version    1.2.0
 *
 */
/*! \file bmi088.h
 \brief Sensor Driver for BMI088 family of sensors */
#ifndef BMI088_H_
#define BMI088_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* header files */
#include "bmi08x_defs.h"
#if BMI08X_FEATURE_BMI088 == 1
/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/

/**********************************************************************************/
/* function prototype declarations */
/*!
 *  @brief This API is the entry point for bmi088 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 *
 *  @param[in,out] dev  : Structure instance of bmi08x_dev.
 *
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_init(struct bmi08x_dev *dev);

/*!
 *  @brief This API uploads the bmi088 config file onto the device.
 *
 *  @param[in,out] dev  : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_apply_config_file(struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable the data synchronization
 *  feature.
 *
 *  @param[in] sync_cfg : configure sync feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_data_synchronization(struct bmi08x_data_sync_cfg sync_cfg, struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 *
 *  @param[in] anymotion_cfg : configure anymotion feature
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_configure_anymotion(struct bmi08x_anymotion_cfg anymotion_cfg, const struct bmi08x_dev *dev);
/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[out] gyro   : Structure pointer to store gyro  data
 *  @param[in]  dev    : Structure instance of bmi08x_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_get_synchronized_data(struct bmi08x_sensor_data *accel, struct bmi08x_sensor_data *gyro,
		const struct bmi08x_dev *dev);
/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi08x_int_cfg
 *  structure instance.
 *
 *  @param[in] int_config : Structure instance of accel bmi08x_int_cfg.
 *  @param[in] dev         : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi088_set_data_sync_int_config(const struct bmi08x_int_cfg *int_config, const struct bmi08x_dev *dev);

#endif
#ifdef __cplusplus
}
#endif

#endif /* BMI088_H_ */

/** @}*/



/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2023 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\imu\bmi088\bmi088.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_
#define NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "sensor.h"
// #include "drv_gpio.h"
// #include "drv_spi.h"

/******************************************************************************
 * macros
 *****************************************************************************/

#define BMI08x_SOFTRESET_CMD    0xB6

#define BMI08x_ACC_PWR_CTRL_ACC_EN_ACC_OFF  0x00
#define BMI08x_ACC_PWR_CTRL_ACC_EN_ACC_ON   0x01

#define BMI08x_ACC_PWR_CONF_PWR_SAVE_MODE_APS_OFF   0x00
#define BMI08x_ACC_PWR_CONF_PWR_SAVE_MODE_APS_ON    0x01

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum{
    BMI08x_ACC_CHIP_ID_ADDR     = 0x00,
    BMI08x_ACC_ERR_REG_ADDR     = 0x02,
    BMI08x_ACC_STATUS_ADDR      = 0x03,
    BMI08x_DATA_0_ADDR          = 0x0A,
    BMI08x_DATA_1_ADDR          = 0x0B,
    BMI08x_DATA_2_ADDR          = 0x0C,
    BMI08x_DATA_3_ADDR          = 0x0D,
    BMI08x_DATA_4_ADDR          = 0x0E,
    BMI08x_DATA_5_ADDR          = 0x0F,
    BMI08x_DATA_6_ADDR          = 0x10,
    BMI08x_DATA_7_ADDR          = 0x11,
    BMI08x_ACC_X_LSB_ADDR       = 0x12,
    BMI08x_ACC_X_MSB_ADDR       = 0x13,
    BMI08x_ACC_Y_LSB_ADDR       = 0x14,
    BMI08x_ACC_Y_MSB_ADDR       = 0x15,
    BMI08x_ACC_Z_LSB_ADDR       = 0x16,
    BMI08x_ACC_Z_MSB_ADDR       = 0x17,
    BMI08x_SENSORTIME_0_ADDR    = 0x18,
    BMI08x_SENSORTIME_1_ADDR    = 0x19,
    BMI08x_SENSORTIME_2_ADDR    = 0x1A,
    BMI08x_EVENT_ADDR           = 0x1B,
    BMI08x_ACC_INT_STAT_0_ADDR  = 0x1C,
    BMI08x_ACC_INT_STAT_1_ADDR  = 0x1D,
    BMI08x_GP_0_ADDR            = 0x1E,
    BMI08x_TEMP_MSB_ADDR        = 0x22,
    BMI08x_TEMP_LSB_ADDR        = 0x23,
    BMI08x_FIFO_LENGTH_0_ADDR   = 0x24,
    BMI08x_FIFO_LENGTH_1_ADDR   = 0x25,
    BMI08x_FIFO_DATA_ADDR       = 0x26,
    BMI08x_GP_4_ADDR            = 0x27,
    BMI08x_ORIENT_HIGH_OUT_ADDR = 0x29,
    BMI08x_INTERNAL_STATUS_ADDR = 0x2A,
    BMI08x_ACC_CONF_ADDR        = 0x40,
    BMI08x_ACC_RANGE_ADDR       = 0x41,
    BMI08x_AUX_CONF_ADDR        = 0x44,
    BMI08x_FIFO_DOWNS_ADDR      = 0x45,
    BMI08x_FIFO_WTM_0_ADDR      = 0x46,
    BMI08x_FIFO_WTM_1_ADDR      = 0x47,
    BMI08x_FIFO_CONFIG_0_ADDR   = 0x48,
    BMI08x_FIFO_CONFIG_1_ADDR   = 0x49,
    BMI08x_AUX_DEV_ID_ADDR      = 0x4B,
    BMI08x_AUX_IF_CONF_ADDR     = 0x4C,
    BMI08x_AUX_RD_ADDR_ADDR     = 0x4D,
    BMI08x_AUX_WR_ADDR_ADDR     = 0x4E,
    BMI08x_AUX_WR_DATA_ADDR     = 0x4F,
    BMI08x_INT1_IO_CTRL_ADDR    = 0x53,
    BMI08x_INT2_IO_CTRL_ADDR    = 0x54,
    BMI08x_INT_LATCH_ADDR       = 0x55,
    BMI08x_INT1_MAP_ADDR        = 0x56,
    BMI08x_INT2_MAP_ADDR        = 0x57,
    BMI08x_INT_MAP_DATA_ADDR    = 0x58,
    BMI08x_INIT_CTRL_ADDR       = 0x59,
    BMI08x_FEATURES_LSB_ADDR    = 0x5B,
    BMI08x_FEATURES_MSB_ADDR    = 0x5C,
    BMI08x_FEATURES_IN_ADDR     = 0x5E,
    BMI08x_INTERNAL_ERROR_ADDR  = 0x5F,
    BMI08x_NVM_CONF_ADDR        = 0x6A,
    BMI08x_IF_CONF_ADDR         = 0x6B,
    BMI08x_ACC_SELF_TEST_ADDR   = 0x6D,
    BMI08x_NV_CONF_ADDR         = 0x70,
    BMI08x_OFFSET_0_ADDR        = 0x71,
    BMI08x_OFFSET_1_ADDR        = 0x72,
    BMI08x_OFFSET_2_ADDR        = 0x73,
    BMI08x_ACC_PWR_CONF_ADDR    = 0x7C,
    BMI08x_ACC_PWR_CTRL_ADDR    = 0x7D,
    BMI08x_ACC_SOFTRESET_ADDR   = 0x7E
}Bmi08xAccRegAddrEnum;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t         :2;
        volatile rt_uint8_t acc_en  :1;
        volatile rt_uint8_t         :5;
    }B;
}Bmi08xAccPwrCtrlRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t pwr_save_mode   :1;
        volatile rt_uint8_t                 :7;
    }B;
}Bmi08xAccPwrConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t spi_en      :1;
        volatile rt_uint8_t i2c_wdt_sel :1;
        volatile rt_uint8_t i2c_wdt_en  :1;
        volatile rt_uint8_t acc_off_en  :1;
        volatile rt_uint8_t             :4;
    }B;
}Bmi08xNvConfRegUnion;//操作这个寄存器貌似不需要nvm_prog_en

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t acc_self_test_en    :1;
        volatile rt_uint8_t                     :1;
        volatile rt_uint8_t acc_self_test_sign  :1;
        volatile rt_uint8_t acc_self_test_amp   :1;
        volatile rt_uint8_t                     :4;
    }B;
}Bmi08xAccSelfTestRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t spi3    :1;
        volatile rt_uint8_t         :3;
        volatile rt_uint8_t if_mode :1;//居然tm可以开磁力计
        volatile rt_uint8_t         :3;
    }B;
}Bmi08xIfConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t nvm_prog_en :1;
        volatile rt_uint8_t             :6;
    }B;
}Bmi08xNvmConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t int_err_1   :1;
        volatile rt_uint8_t int_err_2   :1;
        volatile rt_uint8_t             :5;
    }B;
}Bmi08xInternalErrorRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t int1_ffull  :1;
        volatile rt_uint8_t int1_fwm    :1;
        volatile rt_uint8_t int1_drdy   :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t int2_ffull  :1;
        volatile rt_uint8_t int2_fwm    :1;
        volatile rt_uint8_t int2_drdy   :1;
        volatile rt_uint8_t             :1;
    }B;
}Bmi08xIntMapDataRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Data_sync_out   :1;
        volatile rt_uint8_t any_motion_out  :1;
        volatile rt_uint8_t high_g_out      :1;
        volatile rt_uint8_t low_g_out       :1;
        volatile rt_uint8_t orientation_out :1;
        volatile rt_uint8_t no_motion_out   :1;
        volatile rt_uint8_t                 :1;
        volatile rt_uint8_t error_int_out   :1;
    }B;
}Bmi08xInt2MapRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Data_sync_out   :1;
        volatile rt_uint8_t any_motion_out  :1;
        volatile rt_uint8_t high_g_out      :1;
        volatile rt_uint8_t low_g_out       :1;
        volatile rt_uint8_t orientation_out :1;
        volatile rt_uint8_t no_motion_out   :1;
        volatile rt_uint8_t                 :1;
        volatile rt_uint8_t error_int_out   :1;
    }B;
}Bmi08xInt1MapRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t int_latch   :1;
        volatile rt_uint8_t             :1;
    }B;
}Bmi08xIntLatchRegUnion;//锁存，不知道有啥用

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t edge_ctrl   :1;
        volatile rt_uint8_t lvl         :1;
        volatile rt_uint8_t od          :1;
        volatile rt_uint8_t output_en   :1;
        volatile rt_uint8_t input_en    :1;
        volatile rt_uint8_t             :3;
    }B;
}Bmi08xInt2IoCtrlRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t edge_ctrl   :1;
        volatile rt_uint8_t lvl         :1;
        volatile rt_uint8_t od          :1;
        volatile rt_uint8_t output_en   :1;
        volatile rt_uint8_t input_en    :1;
        volatile rt_uint8_t             :3;
    }B;
}Bmi08xInt1IoCtrlRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t aux_rd_burst    :2;
        volatile rt_uint8_t                 :5;
        volatile rt_uint8_t aux_manual_en   :1;
    }B;
}Bmi08xAuxIfConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t                 :1;
        volatile rt_uint8_t i2c_device_addr :7;
    }B;
}Bmi08xAuxDevIdRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t                 :2;
        volatile rt_uint8_t fifo_tag_int2_en:1;
        volatile rt_uint8_t fifo_tag_int1_en:1;
        volatile rt_uint8_t fifo_header_en  :1;
        volatile rt_uint8_t fifo_aux_en     :1;
        volatile rt_uint8_t fifo_acc_en     :1;
        volatile rt_uint8_t                 :1;
    }B;
}Bmi08xFifoConfig1RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t fifo_stop_on_full   :1;
        volatile rt_uint8_t fifo_time_en        :1;
        volatile rt_uint8_t                     :6;
    }B;
}Bmi08xFifoConfig0RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t                     :4;
        volatile rt_uint8_t acc_fifo_downs      :3;
        volatile rt_uint8_t acc_fifo_filt_data  :1;
    }B;
}Bmi08xFifoDownsRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t aux_odr     :4;
        volatile rt_uint8_t aux_offset  :4;
    }B;
}Bmi08xAuxConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t aux_range   :2;
        volatile rt_uint8_t             :6;
    }B;
}Bmi08xAccRangeRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t acc_odr         :4;
        volatile rt_uint8_t acc_bwp         :3;
        volatile rt_uint8_t acc_pref_mode   :1;
    }B;
}Bmi08xAccConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t message         :5;
        volatile rt_uint8_t axes_remap_error:1;
        volatile rt_uint8_t                 :2;
    }B;
}Bmi08xInternalStatusRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t orientation_portrait_landspace  :2;
        volatile rt_uint8_t orientation_faceup_down         :1;
        volatile rt_uint8_t high_g_detect_x                 :1;
        volatile rt_uint8_t high_g_detect_y                 :1;
        volatile rt_uint8_t high_g_detect_z                 :1;
        volatile rt_uint8_t high_g_detect_sign              :1;
    }B;
}Bmi08xOrientHighgOutRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t ffull_int   :1;
        volatile rt_uint8_t fwm_int     :1;
        volatile rt_uint8_t             :5;
        volatile rt_uint8_t acc_drdy_int:1;
    }B;
}Bmi08xAccIntStat1RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Data_sync_out   :1;
        volatile rt_uint8_t any_motion_out  :1;
        volatile rt_uint8_t high_g_out      :1;
        volatile rt_uint8_t low_g_out       :1;
        volatile rt_uint8_t orientation_out :1;
        volatile rt_uint8_t no_motion_out   :1;
        volatile rt_uint8_t                 :1;
        volatile rt_uint8_t error_int_out   :1;
    }B;
}Bmi08xAccIntStat0RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t por_detected:1;
        volatile rt_uint8_t             :1;
    }B;
}Bmi08xEventRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t             :2;
        volatile rt_uint8_t aux_map_op  :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t cmd_rdy     :1;
        volatile rt_uint8_t drdy_aux    :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t drdy_acc    :1;
    }B;
}Bmi08xAccStatusRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t fatal_err   :2;
        volatile rt_uint8_t cmd_err     :1;
        volatile rt_uint8_t error_code  :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t fifo_err    :1;
        volatile rt_uint8_t aux_err     :1;
    }B;
}Bmi08xAccErrRegRegUnion;

typedef enum{
    BMI08x_GYRO_CHIP_ID_ADDR        = 0x00,
    BMI08x_RATE_X_LSB_ADDR          = 0x02,
    BMI08x_RATE_X_MSB_ADDR          = 0x03,
    BMI08x_RATE_Y_LSB_ADDR          = 0x04,
    BMI08x_RATE_Y_MSB_ADDR          = 0x05,
    BMI08x_RATE_Z_LSB_ADDR          = 0x06,
    BMI08x_RATE_Z_MSB_ADDR          = 0x07,
    BMI08x_GYRO_INT_STAT_1_ADDR     = 0x0A,
    BMI08x_FIFO_STATUS_ADDR         = 0x0E,
    BMI08x_GYRO_RANGE_ADDR          = 0x0F,
    BMI08x_GYRO_BANDWIDTH_ADDR      = 0x10,
    BMI08x_GYRO_LPM1_ADDR           = 0x11,
    BMI08x_GYRO_SOFTRESET_ADDR      = 0x14,
    BMI08x_GYRO_INT_CTRL_ADDR       = 0x15,
    BMI08x_INT3_INT4_IO_CONF_ADDR   = 0x16,
    BMI08x_INT3_INT4_IO_MAP_ADDR    = 0x18,
    BMI08x_FIFO_WM_EN_ADDR          = 0x1E,
    BMI08x_FIFO_EXT_INT_S           = 0x34,
    BMI08x_GYRO_SELF_TEST_ADDR      = 0x3C,
    BMI08x_GYRO_FIFO_CONFIG_0_ADDR  = 0x3D,
    BMI08x_GYRO_FIFO_CONFIG_1_ADDR  = 0x3E,
    BMI08x_GYRO_FIFO_DATA_ADDR      = 0x3F
}Bmi08xGyroRegAddrEnum;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t fifo_water_mark_level_trigger_retain:7;
        volatile rt_uint8_t                                     :1;
    }B;
}Bmi08xGyrFifoConfig0RegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t trig_bist   :1;
        volatile rt_uint8_t bist_rdy    :1;
        volatile rt_uint8_t bist_fail   :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t rate_ok     :1;
        volatile rt_uint8_t             :3;
    }B;
}Bmi08xGyroSelfTestRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t                 :4;
        volatile rt_uint8_t ext_fifo_s_sel  :1;
        volatile rt_uint8_t ext_fifo_s_en   :1;
        volatile rt_uint8_t                 :2;
    }B;
}Bmi08xFifoExtIntSRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Int3_data   :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t Int3_fifo   :1;
        volatile rt_uint8_t             :2;
        volatile rt_uint8_t Int4_fifo   :1;
        volatile rt_uint8_t             :1;
        volatile rt_uint8_t Int4_data   :1;
    }B;
}Bmi08xInt3Int4IoMapRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t Int3_lvl:1;
        volatile rt_uint8_t Int3_od :1;
        volatile rt_uint8_t Int4_lvl:1;
        volatile rt_uint8_t Int4_od :1;
        volatile rt_uint8_t         :4;
    }B;
}Bmi08xInt3Int4IoConfRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t         :1;
        volatile rt_uint8_t fifo_en :1;
        volatile rt_uint8_t data_en :4;
    }B;
}Bmi08xGyroIntCtrlRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t fifo_frame_counter  :7;
        volatile rt_uint8_t fifo_overrun        :1;
    }B;
}Bmi08xFifoStatusRegUnion;

typedef union
{
    volatile rt_uint8_t r;
    struct
    {
        volatile rt_uint8_t             :4;
        volatile rt_uint8_t fifo_int    :1;
        volatile rt_uint8_t data_en     :2;
        volatile rt_uint8_t gyro_drdy   :1;
    }B;
}Bmi08xGyroIntStat1RegUnion;

typedef enum 
{
    ACC_CHIP_ID     = 0x00,
    ACC_ERR_REG     = 0x02,
    ACC_STATUS      = 0x03,
    ACC_X_LSB       = 0x12,
    ACC_X_MSB       = 0x13,
    ACC_Y_LSB       = 0x14,
    ACC_Y_MSB       = 0x15,
    ACC_Z_LSB       = 0x16,
    ACC_Z_MSB       = 0x17,
    SENSORTIME_0    = 0x18,
    SENSORTIME_1    = 0x19,
    SENSORTIME_2    = 0x1A,
    ACC_INT_STAT_1  = 0x1D,
    TEMP_MSB        = 0x22,
    TEMP_LSB        = 0x23,
    ACC_CONF        = 0x40,
    ACC_RANGE       = 0x41,
    FIFO_CONFIG_1   = 0x49,
    INT1_IO_CTRL    = 0x53,
    INT2_IO_CTRL    = 0x54,
    INT_MAP_DATA    = 0x58,
    ACC_SELF_TEST   = 0x6D,
    ACC_PWR_CONF    = 0x7C,
    ACC_PWR_CTRL    = 0x7D,
    ACC_SOFTRESET   = 0x7E
}BMI088AccRegEnum;

typedef enum
{
    kBmi088AccRange3G,
    kBmi088AccRange6G,
    kBmi088AccRange12G,
    kBmi088AccRange24G,
}Bmi088AccRangeEnum;

typedef enum 
{
    GYRO_CHIP_ID        = 0x00,
    RATE_X_LSB          = 0x02,
    RATE_X_MSB          = 0x03,
    RATE_Y_LSB          = 0x04,
    RATE_Y_MSB          = 0x05,
    RATE_Z_LSB          = 0x06,
    RATE_Z_MSB          = 0x07,
    GYRO_INT_STAT_1     = 0x0A,
    GYRO_RANGE          = 0x0F,
    GYRO_BANDWIDTH      = 0x10,
    GYRO_LPM1           = 0x11,
    GYRO_SOFTRESET      = 0x14,
    GYRO_INT_CTRL       = 0x15,
    INT3_INT4_IO_CONF   = 0x16,
    INT3_INT4_IO_MAP    = 0x18,
    GYRO_SELF_TEST      = 0x3C
}BMI088GyroRegEnum;

/******************************************************************************
 * pubilc functions
 *****************************************************************************/

rt_err_t bmi088_acce_init(rt_sensor_t sensor);
rt_err_t bmi08x_acce_reset(rt_sensor_t sensor);

rt_err_t bmi088_acce_get_id(struct rt_sensor_device *sensor, void *args);
rt_err_t bmi088_acce_set_range(struct rt_sensor_device *sensor, void *args);
rt_err_t bmi088_acce_set_odr(struct rt_sensor_device *sensor, void *args);

rt_err_t bmi088_gyro_init(rt_sensor_t sensor);
rt_err_t bmi08x_gyro_reset(rt_sensor_t sensor);

rt_err_t bmi088_gyro_get_id(struct rt_sensor_device *sensor, void *args);

void bmi088_temp_init(void);
rt_size_t bmi088_temp_polling_get_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len);

rt_err_t bmi08x_load_config_file(rt_sensor_t sensor, const rt_uint8_t* file_ptr,rt_size_t file_size);
rt_err_t bmi08x_config_feature(rt_sensor_t sensor, rt_uint8_t feature_addr, rt_uint16_t* feature_cfg, rt_size_t feature_len);
rt_err_t bmi08x_get_sync_data(void);


#ifdef __cplusplus
}
#endif

#endif /* NANOH7_RT_THREAD_BSP_STM32_RCF_NANO_H7_IMU_BMI088_BMI088_H_ */