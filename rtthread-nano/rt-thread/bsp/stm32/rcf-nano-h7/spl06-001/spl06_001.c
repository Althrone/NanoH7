#include "spl06_001.h"

rt_err_t _spl06_001_get_id(struct rt_sensor_device *sensor, void *args)
{
    
}

static rt_size_t _spl06_001_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{

}

static rt_err_t _spl06_001_control(struct rt_sensor_device *sensor, int cmd, void *args)//args是32位(指针都是4个字节)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        // *(spl06_001_reg_list_t *)args = ID_REG;
        result = _spl06_001_get_id(sensor, args);
        break;
    }

}

static struct rt_sensor_ops sensor_ops =
{
    _spl06_001_fetch_data, 
    _spl06_001_control
};

int rt_hw_spl06_001_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_temp = RT_NULL, sensor_baro = RT_NULL;

    sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_temp == RT_NULL)
        return -1;

    sensor_temp->info.type          = RT_SENSOR_CLASS_BARO;
    sensor_temp->info.vendor        = RT_SENSOR_VENDOR_GOERTEK;
    sensor_temp->info.model         = "spl06-001_temp";
    sensor_temp->info.unit          = RT_SENSOR_UNIT_PA;
    sensor_temp->info.intf_type     = RT_SENSOR_INTF_SPI;//需要实现单总线控制
    sensor_temp->info.range_max     = 85;
    sensor_temp->info.range_min     = -40;
    sensor_temp->info.period_min    = 5;
    // sensor->info.fifo_max           = 0;

    rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
    sensor_temp->ops = &sensor_ops;
}